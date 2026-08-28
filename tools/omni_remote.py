"""Keyboard remote control for the four-wheel omni chassis.

Hold W/A/S/D for translation, E/R for clockwise/counter-clockwise rotation.
Releasing a key sends an immediate stop command. Keycaps can also be
clicked with the mouse (press-and-hold) to drive without the keyboard.
"""

import argparse
import sys
import tkinter as tk
from tkinter import ttk

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Missing dependency: pyserial", file=sys.stderr)
    print("Install it with: python -m pip install pyserial", file=sys.stderr)
    raise SystemExit(1)


COMMAND_KEYS = {"w", "a", "s", "d", "e", "r"}
SPEED_STEP = 20

# ---- 调色板（现代浅色） ----
BG          = "#eef2f7"
CARD_FILL   = "#f8fafc"
CARD_BORDER = "#e2e8f0"
KEY_FILL    = "#ffffff"
KEY_BORDER  = "#cbd5e1"
KEY_TEXT    = "#334155"
KEY_SUB     = "#64748b"
ACT_FILL    = "#3b82f6"
ACT_BORDER  = "#2563eb"
ACT_TEXT    = "#ffffff"
ACT_SUB     = "#dbeafe"
SHADOW      = "#dde3ea"
ACCENT      = "#2563eb"
OK_COLOR    = "#16a34a"
ERR_COLOR   = "#dc2626"
FONT        = "Microsoft YaHei UI"

# key -> (大字, 小标签)
KEY_INFO = {
    "w": ("W", "前进"),
    "a": ("A", "左移"),
    "s": ("S", "后退"),
    "d": ("D", "右移"),
    "e": ("E", "顺时针"),
    "r": ("R", "逆时针"),
}


class OmniRemote:
    def __init__(self, port: str, baudrate: int, speed: int) -> None:
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.speed = speed
        self.pressed = set()
        self.last_command = "x"       # 串口去重用
        self.display_command = "x"    # 界面高亮用
        self._mouse_key = None

        self.key_shapes = {}
        self.key_letters = {}
        self.key_subs = {}

        self.window = tk.Tk()
        self.window.title("全向底盘遥控")
        self.window.geometry("620x500")
        self.window.minsize(540, 440)
        self.window.configure(bg=BG)

        self._build_topbar()
        self._build_pad()
        self._build_speedbar()

        self.window.bind("<KeyPress>", self.on_key_press)
        self.window.bind("<KeyRelease>", self.on_key_release)
        self.window.bind("<FocusOut>", lambda _e: self.stop())
        self.window.protocol("WM_DELETE_WINDOW", self.close)
        self.window.focus_force()
        self.refresh_ports()

    # ---------------------------------------------------------------- UI 构建
    def _make_button(self, parent, text, command, primary=True):
        if primary:
            bg, fg, abg = ACCENT, "#ffffff", "#1d4ed8"
        else:
            bg, fg, abg = "#e2e8f0", "#334155", "#cbd5e1"
        return tk.Button(parent, text=text, command=command,
                         bg=bg, fg=fg, activebackground=abg, activeforeground=fg,
                         relief="flat", bd=0, padx=16, pady=5, cursor="hand2",
                         font=(FONT, 10, "bold"), takefocus=False)

    def _build_topbar(self):
        bar = tk.Frame(self.window, bg=BG)
        bar.pack(fill="x", padx=18, pady=(16, 8))

        tk.Label(bar, text="串口", bg=BG, fg="#475569",
                 font=(FONT, 10, "bold")).pack(side="left")
        self.port_box = ttk.Combobox(bar, state="readonly", width=26)
        self.port_box.pack(side="left", padx=(8, 8))

        self.scan_button = self._make_button(bar, "扫描", self._on_scan, primary=False)
        self.scan_button.pack(side="left")
        self.connect_button = self._make_button(bar, "连接", self._on_connect, primary=True)
        self.connect_button.pack(side="left", padx=(8, 0))

        self.status = tk.Label(bar, text="未连接", bg=BG, fg=ERR_COLOR,
                               font=(FONT, 10, "bold"))
        self.status.pack(side="right")

    def _build_pad(self):
        self.canvas = tk.Canvas(self.window, bg=BG, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=18, pady=4)
        self.canvas.bind("<Configure>", lambda _e: self._draw_controls())
        self.canvas.bind("<ButtonRelease-1>", self._mouse_release)

    def _build_speedbar(self):
        bar = tk.Frame(self.window, bg=BG)
        bar.pack(fill="x", padx=18, pady=(4, 16))

        self._make_button(bar, "－", lambda: self._speed_click(-1),
                          primary=False).pack(side="left")
        self.speed_label = tk.Label(bar, text=f"{self.speed} RPM", bg=BG, fg="#0f172a",
                                    font=(FONT, 13, "bold"), width=9)
        self.speed_label.pack(side="left", padx=6)
        self._make_button(bar, "＋", lambda: self._speed_click(1),
                          primary=False).pack(side="left")

        tk.Label(bar, text="按住 W/A/S/D 移动 · E/R 旋转 · 松开停止", bg=BG,
                 fg="#94a3b8", font=(FONT, 9)).pack(side="right")

    # ---------------------------------------------------------------- 绘制
    def _round_rect(self, x1, y1, x2, y2, r, **kw):
        pts = [
            x1 + r, y1, x2 - r, y1, x2, y1,
            x2, y1 + r, x2, y2 - r, x2, y2,
            x2 - r, y2, x1 + r, y2, x1, y2,
            x1, y2 - r, x1, y1 + r, x1, y1,
        ]
        return self.canvas.create_polygon(pts, smooth=True, **kw)

    def _draw_key(self, cx, cy, size, key):
        c = self.canvas
        letter, sub = KEY_INFO[key]
        h = size / 2
        r = size * 0.24
        tag = f"k_{key}"
        self._round_rect(cx - h + 3, cy - h + 4, cx + h + 3, cy + h + 4, r,
                         fill=SHADOW, outline="")
        shape = self._round_rect(cx - h, cy - h, cx + h, cy + h, r,
                                 fill=KEY_FILL, outline=KEY_BORDER, width=2, tags=(tag,))
        letter_id = c.create_text(cx, cy - size * 0.12, text=letter, fill=KEY_TEXT,
                                  font=(FONT, int(size * 0.34), "bold"), tags=(tag,))
        sub_id = c.create_text(cx, cy + size * 0.27, text=sub, fill=KEY_SUB,
                               font=(FONT, max(8, int(size * 0.15))), tags=(tag,))
        c.tag_bind(tag, "<ButtonPress-1>", lambda _e, k=key: self._mouse_press(k))
        self.key_shapes[key] = shape
        self.key_letters[key] = letter_id
        self.key_subs[key] = sub_id

    def _draw_controls(self):
        c = self.canvas
        c.delete("all")
        self.key_shapes.clear()
        self.key_letters.clear()
        self.key_subs.clear()

        w = c.winfo_width()
        h = c.winfo_height()
        if w < 20 or h < 20:
            return

        self._round_rect(6, 6, w - 6, h - 6, 20,
                         fill=CARD_FILL, outline=CARD_BORDER, width=2)
        c.create_text(w // 2, 30, text="方向控制", font=(FONT, 13, "bold"), fill="#475569")

        size = int(min((w - 70) / 6.2, (h - 96) / 3.0, 96))
        size = max(52, size)
        gap = int(size * 0.18)
        step = size + gap
        cy = (h + 48) // 2
        wasd_cx = int(w * 0.38)
        rot_cx = int(w * 0.80)

        self._draw_key(wasd_cx, cy - step // 2, size, "w")
        self._draw_key(wasd_cx - step, cy + step // 2, size, "a")
        self._draw_key(wasd_cx, cy + step // 2, size, "s")
        self._draw_key(wasd_cx + step, cy + step // 2, size, "d")
        self._draw_key(rot_cx, cy - (size + gap) // 2, size, "e")
        self._draw_key(rot_cx, cy + (size + gap) // 2, size, "r")

        self._update_indicator(self.display_command)

    def _update_indicator(self, command: str) -> None:
        for key, shape in self.key_shapes.items():
            on = key == command
            self.canvas.itemconfig(shape,
                                   fill=ACT_FILL if on else KEY_FILL,
                                   outline=ACT_BORDER if on else KEY_BORDER)
            self.canvas.itemconfig(self.key_letters[key],
                                   fill=ACT_TEXT if on else KEY_TEXT)
            self.canvas.itemconfig(self.key_subs[key],
                                   fill=ACT_SUB if on else KEY_SUB)

    def _highlight(self, command: str) -> None:
        self.display_command = command
        if self.key_shapes:
            self._update_indicator(command)

    def _update_speed_text(self) -> None:
        self.speed_label.config(text=f"{self.speed} RPM")

    # ---------------------------------------------------------------- 串口
    def refresh_ports(self) -> None:
        ports = serial.tools.list_ports.comports()
        values = [f"{port.device} | {port.description}" for port in ports]
        self.port_box["values"] = values
        if values:
            self.port_box.current(0)
            self.port = values[0].split(" | ", 1)[0]
            self.status.config(text=f"发现 {len(values)} 个串口", fg="#7c3aed")
        else:
            self.port_box.set("未发现串口")
            self.status.config(text="未发现串口", fg=ERR_COLOR)

    def toggle_connection(self) -> None:
        if self.serial and self.serial.is_open:
            self.serial.write(b"x")
            self.serial.close()
            self.serial = None
            self.connect_button.config(text="连接")
            self.status.config(text="未连接", fg=ERR_COLOR)
            return
        selected = self.port_box.get()
        if " | " in selected:
            self.port = selected.split(" | ", 1)[0]
        if not self.port or self.port == "未发现串口":
            self.status.config(text="请先选择串口", fg=ERR_COLOR)
            return
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0)
            self.connect_button.config(text="断开")
            self.status.config(text=f"已连接 {self.port}", fg=OK_COLOR)
            self.last_command = "x"
        except serial.SerialException as error:
            self.serial = None
            self.status.config(text=f"连接失败: {error}", fg=ERR_COLOR)

    def send(self, command: str) -> None:
        if self.serial and self.serial.is_open and command != self.last_command:
            self.serial.write(command.encode("ascii"))
            self.serial.flush()
            self.last_command = command

    # ---------------------------------------------------------------- 输入
    def on_key_press(self, event: tk.Event) -> None:
        key = event.keysym.lower()
        if key in COMMAND_KEYS:
            self.pressed.add(key)
            self.send(key)
            self._highlight(key)
        elif key in ("plus", "equal", "kp_add"):
            self.send_speed_adjust(1)
        elif key in ("minus", "underscore", "kp_subtract"):
            self.send_speed_adjust(-1)

    def on_key_release(self, event: tk.Event) -> None:
        key = event.keysym.lower()
        if key in COMMAND_KEYS:
            self.pressed.discard(key)
            nxt = next(iter(self.pressed)) if self.pressed else "x"
            self.send(nxt)
            self._highlight(nxt)

    def _mouse_press(self, key: str) -> None:
        self._mouse_key = key
        self.pressed.add(key)
        self.send(key)
        self._highlight(key)
        self.window.focus_set()

    def _mouse_release(self, _event=None) -> None:
        if self._mouse_key is None:
            return
        self.pressed.discard(self._mouse_key)
        self._mouse_key = None
        nxt = next(iter(self.pressed)) if self.pressed else "x"
        self.send(nxt)
        self._highlight(nxt)

    def _on_scan(self) -> None:
        self.refresh_ports()
        self.window.focus_set()

    def _on_connect(self) -> None:
        self.toggle_connection()
        self.window.focus_set()

    def _speed_click(self, direction: int) -> None:
        self.send_speed_adjust(direction)
        self.window.focus_set()

    def send_speed_adjust(self, direction: int) -> None:
        if not self.serial or not self.serial.is_open:
            return
        # 固件用换行做终止符解析 +/-，这里发 "+\r\n" / "-\r\n"
        self.serial.write(("+\r\n" if direction > 0 else "-\r\n").encode("ascii"))
        self.serial.flush()
        self.speed = max(20, min(1000, self.speed + direction * SPEED_STEP))
        self._update_speed_text()

    def stop(self) -> None:
        self.pressed.clear()
        self._mouse_key = None
        self.send("x")
        self._highlight("x")

    def close(self) -> None:
        if self.serial and self.serial.is_open:
            self.serial.write(b"x")
            self.serial.flush()
            self.serial.close()
        self.window.destroy()

    def run(self) -> None:
        try:
            self.window.mainloop()
        finally:
            if self.serial and self.serial.is_open:
                self.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="WASD/E/R omni chassis remote")
    parser.add_argument("--port", default="", help="optional initial serial port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--speed", type=int, default=100, help="displayed motor speed")
    parser.add_argument("--list", action="store_true", help="list available serial ports")
    args = parser.parse_args()
    if args.list:
        ports = serial.tools.list_ports.comports()
        if not ports:
            print("No serial ports found. Connect the USB-TTL adapter or development board.")
        for port in ports:
            print(f"{port.device}: {port.description}")
        return
    OmniRemote(args.port, args.baud, args.speed).run()


if __name__ == "__main__":
    main()
