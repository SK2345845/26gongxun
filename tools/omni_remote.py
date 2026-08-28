"""Keyboard remote control for the four-wheel omni chassis.

Hold W/A/S/D for translation, E/R for clockwise/counter-clockwise rotation.
Releasing a key sends an immediate stop command. Keycaps can also be
clicked with the mouse (press-and-hold) to drive without the keyboard.
"""

import argparse
import queue
import sys
import threading
import time
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
HEARTBEAT_MS = 200       # 心跳周期：周期性重发当前指令，单字节丢失可自愈
RX_POLL_MS = 100         # 主线程轮询接收队列的周期

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

        # ---- 接收线程 / 心跳 / 日志相关状态 ----
        self.rx_queue = queue.Queue()
        self.reader_thread = None
        self._heartbeat_job = None
        self._rx_poll_job = None
        self._closing = False
        self._rx_partial = ""    # 接收数据的半行缓冲
        self._log_seq = 0        # 日志行颜色标签序号

        self.window = tk.Tk()
        self.window.title("全向底盘遥控")
        self.window.geometry("620x660")
        self.window.minsize(540, 600)
        self.window.configure(bg=BG)

        self._build_topbar()
        self._build_pad()
        self._build_log()
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

        self.tx_label = tk.Label(bar, text="TX: -", bg=BG, fg=KEY_SUB,
                                 font=(FONT, 10, "bold"))
        self.tx_label.pack(side="right", padx=(0, 16))

    def _build_log(self):
        """MCU 回显日志区：后台线程读串口，主线程定时刷新显示。"""
        bar = tk.Frame(self.window, bg=BG)
        bar.pack(fill="x", padx=18, pady=(2, 0))
        tk.Label(bar, text="MCU 回显（[REMOTE] cmd=w = 单片机已收到并执行）",
                 bg=BG, fg="#64748b", font=(FONT, 9)).pack(side="left")
        tk.Button(bar, text="清空", command=self._clear_log,
                  bg="#e2e8f0", fg="#334155", activebackground="#cbd5e1",
                  relief="flat", bd=0, padx=10, cursor="hand2",
                  font=(FONT, 9)).pack(side="right")

        self.log_text = tk.Text(self.window, height=8, state="disabled",
                                bg="#0f172a", fg="#cbd5e1", insertbackground="#cbd5e1",
                                relief="flat", padx=10, pady=6,
                                font=("Consolas", 9))
        self.log_text.pack(fill="x", padx=18, pady=(2, 4))

    def _clear_log(self):
        self.log_text.config(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.config(state="disabled")

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
            self._stop_heartbeat()
            try:
                self.serial.write(b"x")
                self.serial.flush()
            except (serial.SerialException, OSError):
                pass
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
            # timeout=0.1：接收线程 read 最多阻塞 100ms，既不丢收也不空转
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.connect_button.config(text="断开")
            self.status.config(text=f"已连接 {self.port}", fg=OK_COLOR)
            self.last_command = "x"
            self._rx_partial = ""
            self._log_system(f"已连接 {self.port} @ {self.baudrate}")
            # 启动接收线程 + 心跳重发 + 主线程接收轮询
            self.reader_thread = threading.Thread(
                target=self._reader_loop, daemon=True)
            self.reader_thread.start()
            self._start_heartbeat()
            self._start_rx_poll()
        except serial.SerialException as error:
            self.serial = None
            self.status.config(text=f"连接失败: {error}", fg=ERR_COLOR)

    def send(self, command: str) -> None:
        if self.serial and self.serial.is_open and command != self.last_command:
            try:
                self.serial.write(command.encode("ascii"))
                self.serial.flush()
            except (serial.SerialException, OSError) as error:
                self._on_rx_error(f"发送失败: {error}")
                return
            self.last_command = command
            self.tx_label.config(text=f"TX: {command}", fg=ACCENT)

    # -------------------------------------------------- 心跳重发（可靠性核心）
    def _start_heartbeat(self) -> None:
        if self._heartbeat_job is None:
            self._heartbeat_job = self.window.after(HEARTBEAT_MS, self._heartbeat)

    def _stop_heartbeat(self) -> None:
        if self._heartbeat_job is not None:
            self.window.after_cancel(self._heartbeat_job)
            self._heartbeat_job = None

    def _heartbeat(self) -> None:
        """每 200ms 重发当前指令。协议是幂等的：收到重复 w 只是重设同一速度。
        这样即使某个字节在串口上丢了（STM32 RX 只有单字节缓冲），
        200ms 后的下一次心跳就会自愈，不会出现'按了没反应还不知道'。"""
        self._heartbeat_job = None
        if self._closing or not self.serial or not self.serial.is_open:
            return
        try:
            self.serial.write(self.last_command.encode("ascii"))
            self.serial.flush()
        except (serial.SerialException, OSError) as error:
            self._on_rx_error(f"心跳发送失败: {error}")
            return
        self._heartbeat_job = self.window.after(HEARTBEAT_MS, self._heartbeat)

    # -------------------------------------------------- 串口接收（线程 + 队列）
    def _reader_loop(self) -> None:
        """后台线程：只负责读串口塞队列，绝不碰 tkinter。"""
        ser = self.serial
        try:
            while self.serial is ser and ser.is_open and not self._closing:
                data = ser.read(256)
                if data:
                    self.rx_queue.put(data)
        except (serial.SerialException, OSError) as error:
            self.rx_queue.put(("__RX_ERROR__", str(error)))

    def _start_rx_poll(self) -> None:
        if self._rx_poll_job is None:
            self._rx_poll_job = self.window.after(RX_POLL_MS, self._poll_rx)

    def _poll_rx(self) -> None:
        """主线程：把队列里的数据搬到日志区（tkinter 只允许主线程操作控件）。"""
        try:
            while True:
                item = self.rx_queue.get_nowait()
                if isinstance(item, tuple):
                    self._on_rx_error(f"串口读失败: {item[1]}")
                    continue
                self._feed_rx(item.decode("utf-8", errors="replace"))
        except queue.Empty:
            pass
        if not self._closing:
            self._rx_poll_job = self.window.after(RX_POLL_MS, self._poll_rx)
        else:
            self._rx_poll_job = None

    def _feed_rx(self, text: str) -> None:
        self._rx_partial += text
        while "\n" in self._rx_partial:
            line, self._rx_partial = self._rx_partial.split("\n", 1)
            line = line.strip("\r").rstrip()
            if line:
                self._log_line(line)

    def _on_rx_error(self, message: str) -> None:
        self._log_system(f"[错误] {message}")
        self.status.config(text=f"串口异常: {message}", fg=ERR_COLOR)
        if self.serial and self.serial.is_open:
            try:
                self.serial.close()
            except (serial.SerialException, OSError):
                pass
        self.serial = None
        self.connect_button.config(text="连接")

    # -------------------------------------------------- 日志显示
    def _log_line(self, text: str, color: str = "#cbd5e1") -> None:
        stamp = time.strftime("%H:%M:%S")
        tag = f"c{self._log_seq}"
        self._log_seq += 1
        self.log_text.config(state="normal")
        self.log_text.tag_config(tag, foreground=color)
        self.log_text.insert("end", f"{stamp}  {text}\n", tag)
        # 只保留最近 200 行
        total = int(self.log_text.index("end-1c").split(".")[0])
        if total > 200:
            self.log_text.delete("1.0", f"{total - 200}.0")
        self.log_text.see("end")
        self.log_text.config(state="disabled")

    def _log_system(self, text: str) -> None:
        self._log_line(text, color="#fbbf24")

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
        # 速度值本地即时更新（未连接串口也可见），连接状态下再同步发送给固件
        self.speed = max(20, min(1000, self.speed + direction * SPEED_STEP))
        self._update_speed_text()
        if not self.serial or not self.serial.is_open:
            return
        # 固件用换行做终止符解析 +/-，这里发 "+\r\n" / "-\r\n"
        try:
            self.serial.write(("+\r\n" if direction > 0 else "-\r\n").encode("ascii"))
            self.serial.flush()
        except (serial.SerialException, OSError) as error:
            self._on_rx_error(f"发送失败: {error}")

    def stop(self) -> None:
        self.pressed.clear()
        self._mouse_key = None
        self.send("x")
        self._highlight("x")

    def close(self) -> None:
        if self._closing:
            return
        self._closing = True
        self._stop_heartbeat()
        if self._rx_poll_job is not None:
            self.window.after_cancel(self._rx_poll_job)
            self._rx_poll_job = None
        if self.serial and self.serial.is_open:
            try:
                self.serial.write(b"x")
                self.serial.flush()
            except (serial.SerialException, OSError):
                pass
            self.serial.close()
        try:
            self.window.destroy()
        except tk.TclError:
            pass

    def run(self) -> None:
        try:
            self.window.mainloop()
        finally:
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
