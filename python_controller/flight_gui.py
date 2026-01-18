import errno
import socket
import struct
import time
import tkinter as tk
from tkinter import messagebox
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText


CRTP_PORT_SETPOINT = 0x03
CRTP_PORT_LOG = 0x05

CRTP_CH_TOC = 0
CRTP_CH_CONTROL = 1
CRTP_CH_LOG = 2

CMD_GET_ITEM_V2 = 2
CMD_GET_INFO_V2 = 3

CONTROL_CREATE_BLOCK_V2 = 6
CONTROL_APPEND_BLOCK_V2 = 7
CONTROL_DELETE_BLOCK = 2
CONTROL_START_BLOCK = 3
CONTROL_STOP_BLOCK = 4

LOG_FLOAT = 7

UDP_DIAG_PING = b"CFPING"
UDP_DIAG_PONG = b"CFPONG"


def crtp_header(port, channel):
    return ((port & 0x0F) << 4) | (channel & 0x0F)


def build_udp_packet(payload):
    checksum = sum(payload) & 0xFF
    return payload + bytes([checksum])


def parse_udp_packet(data):
    if len(data) < 2:
        return None
    payload = data[:-1]
    checksum = data[-1]
    if (sum(payload) & 0xFF) != checksum:
        return None
    return payload


def build_setpoint_packet(roll_deg, pitch_deg, yaw_deg, thrust):
    payload = struct.pack("<fffH", roll_deg, pitch_deg, yaw_deg, int(thrust))
    header = crtp_header(CRTP_PORT_SETPOINT, 0)
    return build_udp_packet(bytes([header]) + payload)


def build_log_packet(channel, payload):
    header = crtp_header(CRTP_PORT_LOG, channel)
    return build_udp_packet(bytes([header]) + payload)


def vbat_to_percent(vbat):
    if vbat is None:
        return None
    if vbat <= 3.30:
        return 0
    if vbat >= 4.20:
        return 100
    return int((vbat - 3.30) / 0.90 * 100)


class LogClient:
    def __init__(self, send_cb, log_cb, telemetry_cb, status_cb):
        self.send_cb = send_cb
        self.log_cb = log_cb
        self.telemetry_cb = telemetry_cb
        self.status_cb = status_cb

        self.enabled = False
        self.state = "idle"
        self.toc_count = 0
        self.next_id = 0
        self.last_cmd_packet = None
        self.last_cmd_time = 0.0
        self.retry_interval = 1.0

        self.block_id = 1
        self.period_ms = 100
        self.pending_create_after_delete = False

        self.targets = [
            "pm.vbat",
            "stabilizer.roll",
            "stabilizer.pitch",
            "stabilizer.yaw",
        ]
        self.order = list(self.targets)
        self.var_ids = {}

    def start(self):
        self.enabled = True
        self.state = "toc_info"
        self.var_ids.clear()
        self.pending_create_after_delete = False
        self.status_cb("telemetry: requesting TOC")
        self._send_toc_info()

    def stop(self):
        if not self.enabled:
            return
        self.enabled = False
        self.state = "idle"
        self.pending_create_after_delete = False
        self._send_control(bytes([CONTROL_STOP_BLOCK, self.block_id, 0]))
        self.status_cb("telemetry: stopped")

    def tick(self):
        if not self.enabled or not self.last_cmd_packet:
            return
        if (time.time() - self.last_cmd_time) >= self.retry_interval:
            self.send_cb(self.last_cmd_packet)
            self.last_cmd_time = time.time()

    def handle_packet(self, channel, data):
        if not self.enabled:
            return
        if channel == CRTP_CH_TOC:
            self._handle_toc(data)
        elif channel == CRTP_CH_CONTROL:
            self._handle_control(data)
        elif channel == CRTP_CH_LOG:
            self._handle_log(data)

    def _send_toc_info(self):
        payload = bytes([CMD_GET_INFO_V2])
        self._send_toc(payload)

    def _send_toc_item(self, log_id):
        payload = struct.pack("<BH", CMD_GET_ITEM_V2, log_id)
        self._send_toc(payload)

    def _send_toc(self, payload):
        packet = build_log_packet(CRTP_CH_TOC, payload)
        self.send_cb(packet)
        self.last_cmd_packet = packet
        self.last_cmd_time = time.time()

    def _send_control(self, payload):
        packet = build_log_packet(CRTP_CH_CONTROL, payload)
        self.send_cb(packet)
        self.last_cmd_packet = packet
        self.last_cmd_time = time.time()

    def _handle_toc(self, data):
        if not data:
            return
        cmd = data[0]
        if cmd == CMD_GET_INFO_V2:
            if len(data) < 9:
                return
            self.toc_count = data[1] | (data[2] << 8)
            self.next_id = 0
            self.state = "toc_item"
            self.status_cb(f"telemetry: toc entries={self.toc_count}")
            self._send_toc_item(self.next_id)
            return
        if cmd != CMD_GET_ITEM_V2:
            return
        if len(data) < 5:
            return
        log_id = data[1] | (data[2] << 8)
        names = data[4:]
        parts = names.split(b"\x00")
        if len(parts) < 2:
            return
        group = parts[0].decode("ascii", "ignore")
        name = parts[1].decode("ascii", "ignore")
        full_name = f"{group}.{name}" if group else name
        if full_name in self.targets:
            self.var_ids[full_name] = log_id

        if all(key in self.var_ids for key in self.targets):
            self._send_create_block()
            return

        next_id = log_id + 1
        if next_id < self.toc_count:
            self._send_toc_item(next_id)
        else:
            missing = [key for key in self.targets if key not in self.var_ids]
            self.status_cb(f"telemetry: missing {', '.join(missing)}")
            self.enabled = False

    def _send_create_block(self):
        payload = bytes([CONTROL_CREATE_BLOCK_V2, self.block_id])
        for key in self.order:
            var_id = self.var_ids.get(key)
            if var_id is None:
                self.status_cb("telemetry: missing variables")
                self.enabled = False
                return
            payload += struct.pack("<BH", LOG_FLOAT, var_id)
        self.state = "create_block"
        self._send_control(payload)

    def _send_delete_block(self):
        payload = bytes([CONTROL_DELETE_BLOCK, self.block_id, 0])
        self._send_control(payload)

    def _send_start_block(self):
        period_ticks = max(1, int(self.period_ms / 10))
        payload = bytes([CONTROL_START_BLOCK, self.block_id, period_ticks])
        self.state = "start_block"
        self._send_control(payload)

    def _handle_control(self, data):
        if len(data) < 3:
            return
        cmd = data[0]
        block_id = data[1]
        ret = data[2]
        if block_id != self.block_id:
            return

        if cmd == CONTROL_CREATE_BLOCK_V2:
            if ret == 0:
                self.status_cb("telemetry: block created")
                self._send_start_block()
            elif ret == errno.EEXIST:
                self.status_cb("telemetry: block exists, deleting")
                self.pending_create_after_delete = True
                self._send_delete_block()
            else:
                self.status_cb(f"telemetry: create failed ({ret})")
                self.enabled = False
        elif cmd == CONTROL_DELETE_BLOCK:
            if self.pending_create_after_delete:
                self.pending_create_after_delete = False
                self._send_create_block()
        elif cmd == CONTROL_START_BLOCK:
            if ret == 0:
                self.state = "running"
                self.last_cmd_packet = None
                self.status_cb("telemetry: running")
            else:
                self.status_cb(f"telemetry: start failed ({ret})")
                self.enabled = False

    def _handle_log(self, data):
        if self.state != "running":
            return
        if len(data) < 4:
            return
        if data[0] != self.block_id:
            return
        count = len(self.order)
        expected = 4 + (4 * count)
        if len(data) < expected:
            return
        values = struct.unpack_from("<" + ("f" * count), data, 4)
        telemetry = dict(zip(self.order, values))
        self.telemetry_cb(telemetry)


class FlightGui:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP-Drone Flight GUI")
        self.root.resizable(False, False)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

        self.streaming = False
        self.awaiting_ping = None
        self.last_telemetry_time = None
        self.pending_motion_job = None
        self.pending_ramp_job = None

        self.ip_var = tk.StringVar(value="192.168.43.42")
        self.port_var = tk.StringVar(value="2390")
        self.roll_var = tk.StringVar(value="0.0")
        self.pitch_var = tk.StringVar(value="0.0")
        self.yaw_var = tk.StringVar(value="0.0")
        self.rate_var = tk.StringVar(value="50")
        self.thrust_var = tk.IntVar(value=0)
        self.ramp_time_var = tk.StringVar(value="2.0")
        self.move_pitch_var = tk.StringVar(value="5.0")
        self.move_time_var = tk.StringVar(value="1.0")

        self.battery_var = tk.StringVar(value="n/a")
        self.attitude_var = tk.StringVar(value="roll=0.0 pitch=0.0 yaw=0.0")
        self.telemetry_status_var = tk.StringVar(value="telemetry: idle")

        self._build_ui()

        self.log_client = LogClient(
            self._send_packet,
            self._log,
            self._on_telemetry,
            self._set_telemetry_status,
        )

        self.root.after(20, self._poll_rx)
        self.root.after(250, self._telemetry_age_tick)

    def _build_ui(self):
        pad = {"padx": 6, "pady": 4}
        main = ttk.Frame(self.root)
        main.grid(row=0, column=0, sticky="nsew")

        conn = ttk.LabelFrame(main, text="Connection")
        conn.grid(row=0, column=0, sticky="ew", **pad)
        ttk.Label(conn, text="IP").grid(row=0, column=0, sticky="w", **pad)
        ttk.Entry(conn, width=16, textvariable=self.ip_var).grid(row=0, column=1, **pad)
        ttk.Label(conn, text="Port").grid(row=0, column=2, sticky="w", **pad)
        ttk.Entry(conn, width=8, textvariable=self.port_var).grid(row=0, column=3, **pad)
        ttk.Button(conn, text="Ping", command=self.ping_link).grid(row=0, column=4, **pad)
        self.link_status = ttk.Label(conn, text="Link: unknown")
        self.link_status.grid(row=0, column=5, sticky="w", **pad)

        tel = ttk.LabelFrame(main, text="Telemetry")
        tel.grid(row=1, column=0, sticky="ew", **pad)
        ttk.Button(tel, text="Start", command=self.start_telemetry).grid(row=0, column=0, **pad)
        ttk.Button(tel, text="Stop", command=self.stop_telemetry).grid(row=0, column=1, **pad)
        ttk.Label(tel, textvariable=self.telemetry_status_var).grid(row=0, column=2, columnspan=3, sticky="w", **pad)
        ttk.Label(tel, text="Battery").grid(row=1, column=0, sticky="w", **pad)
        ttk.Label(tel, textvariable=self.battery_var).grid(row=1, column=1, columnspan=2, sticky="w", **pad)
        ttk.Label(tel, text="Attitude").grid(row=2, column=0, sticky="w", **pad)
        ttk.Label(tel, textvariable=self.attitude_var).grid(row=2, column=1, columnspan=3, sticky="w", **pad)

        sp = ttk.LabelFrame(main, text="Setpoint (deg, thrust)")
        sp.grid(row=2, column=0, sticky="ew", **pad)
        ttk.Label(sp, text="Roll").grid(row=0, column=0, sticky="w", **pad)
        ttk.Entry(sp, width=10, textvariable=self.roll_var).grid(row=0, column=1, **pad)
        ttk.Label(sp, text="Pitch").grid(row=0, column=2, sticky="w", **pad)
        ttk.Entry(sp, width=10, textvariable=self.pitch_var).grid(row=0, column=3, **pad)
        ttk.Label(sp, text="Yaw").grid(row=0, column=4, sticky="w", **pad)
        ttk.Entry(sp, width=10, textvariable=self.yaw_var).grid(row=0, column=5, **pad)

        ttk.Label(sp, text="Thrust").grid(row=1, column=0, sticky="w", **pad)
        thrust_scale = ttk.Scale(
            sp, from_=0, to=60000, orient="horizontal",
            command=self._on_thrust_scale
        )
        thrust_scale.set(self.thrust_var.get())
        thrust_scale.grid(row=1, column=1, columnspan=4, sticky="ew", **pad)
        self.thrust_entry = ttk.Entry(sp, width=8, textvariable=self.thrust_var)
        self.thrust_entry.grid(row=1, column=5, **pad)

        rate = ttk.LabelFrame(main, text="Streaming")
        rate.grid(row=3, column=0, sticky="ew", **pad)
        ttk.Label(rate, text="Rate (Hz)").grid(row=0, column=0, sticky="w", **pad)
        ttk.Entry(rate, width=8, textvariable=self.rate_var).grid(row=0, column=1, **pad)
        ttk.Button(rate, text="Send Once", command=self.send_once).grid(row=0, column=2, **pad)
        ttk.Button(rate, text="Start Stream", command=self.start_stream).grid(row=0, column=3, **pad)
        ttk.Button(rate, text="Stop Stream", command=self.stop_stream).grid(row=0, column=4, **pad)

        actions = ttk.LabelFrame(main, text="Flight Actions")
        actions.grid(row=4, column=0, sticky="ew", **pad)
        ttk.Button(actions, text="Unlock (thrust=0)", command=self.send_unlock).grid(row=0, column=0, **pad)
        ttk.Button(actions, text="E-Stop", command=self.send_estop).grid(row=0, column=1, **pad)
        ttk.Button(actions, text="Takeoff", command=self.takeoff).grid(row=0, column=2, **pad)
        ttk.Button(actions, text="Land", command=self.land).grid(row=0, column=3, **pad)
        ttk.Button(actions, text="Forward", command=lambda: self.move_pitch(1)).grid(row=1, column=0, **pad)
        ttk.Button(actions, text="Backward", command=lambda: self.move_pitch(-1)).grid(row=1, column=1, **pad)
        ttk.Button(actions, text="Stop Motion", command=self.stop_motion).grid(row=1, column=2, **pad)
        ttk.Label(actions, text="Move pitch (deg)").grid(row=2, column=0, sticky="w", **pad)
        ttk.Entry(actions, width=8, textvariable=self.move_pitch_var).grid(row=2, column=1, **pad)
        ttk.Label(actions, text="Move time (s)").grid(row=2, column=2, sticky="w", **pad)
        ttk.Entry(actions, width=8, textvariable=self.move_time_var).grid(row=2, column=3, **pad)
        ttk.Label(actions, text="Ramp time (s)").grid(row=3, column=0, sticky="w", **pad)
        ttk.Entry(actions, width=8, textvariable=self.ramp_time_var).grid(row=3, column=1, **pad)

        log = ttk.LabelFrame(main, text="Log")
        log.grid(row=5, column=0, sticky="ew", **pad)
        self.log_box = ScrolledText(log, width=70, height=8, state="disabled")
        self.log_box.grid(row=0, column=0, **pad)

        note = ("Notes: unlock once (thrust=0), then ramp thrust carefully.\n"
                "Remove props or secure the drone for testing.")
        ttk.Label(main, text=note).grid(row=6, column=0, sticky="w", **pad)

        sp.columnconfigure(1, weight=1)
        sp.columnconfigure(2, weight=1)
        sp.columnconfigure(3, weight=1)
        sp.columnconfigure(4, weight=1)
        sp.columnconfigure(5, weight=1)

    def _log(self, msg):
        ts = time.strftime("%H:%M:%S")
        self.log_box.configure(state="normal")
        self.log_box.insert("end", f"[{ts}] {msg}\n")
        self.log_box.see("end")
        self.log_box.configure(state="disabled")

    def _set_telemetry_status(self, msg):
        self.telemetry_status_var.set(msg)
        self._log(msg)

    def _on_thrust_scale(self, value):
        try:
            self.thrust_var.set(int(float(value)))
        except ValueError:
            pass

    def _get_target(self):
        ip = self.ip_var.get().strip()
        try:
            port = int(self.port_var.get().strip())
        except ValueError:
            raise ValueError("invalid port")
        return ip, port

    def _get_setpoint(self):
        try:
            roll = float(self.roll_var.get().strip())
            pitch = float(self.pitch_var.get().strip())
            yaw = float(self.yaw_var.get().strip())
            thrust = int(self.thrust_var.get())
        except ValueError:
            raise ValueError("invalid setpoint")
        thrust = max(0, min(60000, thrust))
        return roll, pitch, yaw, thrust

    def _send_packet(self, payload):
        ip, port = self._get_target()
        self.sock.sendto(payload, (ip, port))

    def _send_setpoint(self, roll, pitch, yaw, thrust):
        packet = build_setpoint_packet(roll, pitch, yaw, thrust)
        self._send_packet(packet)

    def send_once(self):
        try:
            roll, pitch, yaw, thrust = self._get_setpoint()
            self._send_setpoint(roll, pitch, yaw, thrust)
            self._log(f"send roll={roll:.2f} pitch={pitch:.2f} yaw={yaw:.2f} thrust={thrust}")
        except Exception as exc:
            messagebox.showerror("Send failed", str(exc))

    def send_unlock(self):
        try:
            roll, pitch, yaw, _ = self._get_setpoint()
            self._send_setpoint(roll, pitch, yaw, 0)
            self._log("unlock packet sent (thrust=0)")
        except Exception as exc:
            messagebox.showerror("Unlock failed", str(exc))

    def send_estop(self):
        self.stop_stream()
        try:
            self._send_setpoint(0.0, 0.0, 0.0, 0)
            self._log("e-stop sent")
        except Exception as exc:
            messagebox.showerror("E-stop failed", str(exc))

    def start_stream(self):
        if self.streaming:
            return
        self.streaming = True
        self._log("streaming started")
        self._stream_tick()

    def stop_stream(self):
        if self.streaming:
            self.streaming = False
            self._log("streaming stopped")

    def _stream_tick(self):
        if not self.streaming:
            return
        try:
            rate_hz = float(self.rate_var.get().strip())
            if rate_hz <= 0:
                raise ValueError("rate must be > 0")
            roll, pitch, yaw, thrust = self._get_setpoint()
            self._send_setpoint(roll, pitch, yaw, thrust)
        except Exception as exc:
            self.streaming = False
            messagebox.showerror("Stream error", str(exc))
            return

        interval_ms = max(10, int(1000.0 / rate_hz))
        self.root.after(interval_ms, self._stream_tick)

    def _ensure_streaming(self):
        if not self.streaming:
            self.start_stream()

    def takeoff(self):
        try:
            ramp_time = float(self.ramp_time_var.get().strip())
            _, _, _, target = self._get_setpoint()
        except Exception as exc:
            messagebox.showerror("Takeoff failed", str(exc))
            return
        self.roll_var.set("0.0")
        self.pitch_var.set("0.0")
        self.yaw_var.set("0.0")
        self._ensure_streaming()
        self._ramp_thrust(0, target, ramp_time)
        self._log(f"takeoff ramp to {target} over {ramp_time:.1f}s")

    def land(self):
        try:
            ramp_time = float(self.ramp_time_var.get().strip())
            _, _, _, current = self._get_setpoint()
        except Exception as exc:
            messagebox.showerror("Land failed", str(exc))
            return
        self._ensure_streaming()
        self._ramp_thrust(current, 0, ramp_time)
        self._log(f"land ramp to 0 over {ramp_time:.1f}s")

    def _ramp_thrust(self, start, end, duration):
        if self.pending_ramp_job:
            self.root.after_cancel(self.pending_ramp_job)
            self.pending_ramp_job = None
        steps = max(1, int(duration * 50))
        step = (end - start) / float(steps)

        def _step(i=0):
            value = start + (step * i)
            self.thrust_var.set(int(value))
            if i < steps:
                self.pending_ramp_job = self.root.after(20, _step, i + 1)
            else:
                self.pending_ramp_job = None

        _step(0)

    def move_pitch(self, direction):
        try:
            move_pitch = float(self.move_pitch_var.get().strip())
            move_time = float(self.move_time_var.get().strip())
        except ValueError as exc:
            messagebox.showerror("Move failed", str(exc))
            return
        if self.pending_motion_job:
            self.root.after_cancel(self.pending_motion_job)
            self.pending_motion_job = None
        self._ensure_streaming()
        self.roll_var.set("0.0")
        self.yaw_var.set("0.0")
        self.pitch_var.set(f"{direction * move_pitch:.2f}")
        self._log(f"move pitch {direction * move_pitch:.2f} deg for {move_time:.1f}s")

        def _stop():
            self.pitch_var.set("0.0")
            self.pending_motion_job = None

        self.pending_motion_job = self.root.after(int(move_time * 1000), _stop)

    def stop_motion(self):
        if self.pending_motion_job:
            self.root.after_cancel(self.pending_motion_job)
            self.pending_motion_job = None
        self.roll_var.set("0.0")
        self.pitch_var.set("0.0")
        self.yaw_var.set("0.0")
        self._log("motion stopped")

    def ping_link(self):
        try:
            packet = build_udp_packet(UDP_DIAG_PING)
            self._send_packet(packet)
        except Exception as exc:
            messagebox.showerror("Ping failed", str(exc))
            return
        self.awaiting_ping = time.time()
        self.link_status.configure(text="Link: waiting")

    def start_telemetry(self):
        try:
            self.log_client.start()
        except Exception as exc:
            messagebox.showerror("Telemetry start failed", str(exc))

    def stop_telemetry(self):
        self.log_client.stop()

    def _on_telemetry(self, telemetry):
        self.last_telemetry_time = time.time()
        vbat = telemetry.get("pm.vbat")
        roll = telemetry.get("stabilizer.roll")
        pitch = telemetry.get("stabilizer.pitch")
        yaw = telemetry.get("stabilizer.yaw")
        percent = vbat_to_percent(vbat)
        if vbat is None:
            self.battery_var.set("n/a")
        elif percent is None:
            self.battery_var.set(f"{vbat:.2f} V")
        else:
            self.battery_var.set(f"{vbat:.2f} V ({percent}%)")
        if roll is not None and pitch is not None and yaw is not None:
            self.attitude_var.set(f"roll={roll:.2f} pitch={pitch:.2f} yaw={yaw:.2f}")

    def _telemetry_age_tick(self):
        if self.last_telemetry_time:
            age = time.time() - self.last_telemetry_time
            if age > 2.0 and self.log_client.state == "running":
                self.telemetry_status_var.set("telemetry: stale")
        self.root.after(250, self._telemetry_age_tick)

    def _poll_rx(self):
        while True:
            try:
                data, _addr = self.sock.recvfrom(512)
            except BlockingIOError:
                break
            except OSError:
                break
            payload = parse_udp_packet(data)
            if not payload:
                continue
            if payload == UDP_DIAG_PONG:
                self.link_status.configure(text="Link: OK")
                self._log("ping ok")
                self.awaiting_ping = None
                continue

            header = payload[0]
            port = (header >> 4) & 0x0F
            channel = header & 0x0F
            body = payload[1:]
            if port == CRTP_PORT_LOG:
                self.log_client.handle_packet(channel, body)

        if self.awaiting_ping and (time.time() - self.awaiting_ping) > 0.6:
            self.link_status.configure(text="Link: timeout")
            self._log("ping timeout")
            self.awaiting_ping = None

        self.log_client.tick()
        self.root.after(20, self._poll_rx)


def main():
    root = tk.Tk()
    app = FlightGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()
