#!/usr/bin/env python3
"""
BER (Bearcats Electric Racing) - EV4 Serial CAN Dashboard
=========================================================

Reads CAN frames streamed over USB serial from the ESP32
(EV4_CAN_Serial_Forwarder.ino), decodes them against the
EV4_Vehicle_Bus.dbc, and shows a live Tkinter dashboard.

The DBC is the single source of truth: the signal panels are
built dynamically from it, so editing the .dbc and restarting
the app is all that's needed when the bus layout changes.

Serial line format produced by the ESP32:
    <ID_HEX>#<DATA_HEX>[x]\\n      (trailing 'x' = extended/29-bit id)

Usage:
    pip install -r requirements.txt
    python ev4_dashboard.py
    # optional: python ev4_dashboard.py --port COM5 --baud 921600

Pick the serial port in the dropdown and click Connect.
"""

import argparse
import csv
import datetime
import os
import struct
import sys
import threading
import time
import queue
import tkinter as tk
from tkinter import ttk

try:
    import serial
    import serial.tools.list_ports as list_ports
except ImportError:
    sys.exit("pyserial is required:  pip install -r requirements.txt")

try:
    import cantools
except ImportError:
    sys.exit("cantools is required:  pip install -r requirements.txt")


HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_BAUD = 921600
STALE_AFTER = 1.0          # seconds without a frame -> message marked stale

# DBC sources, decoded together.  Each (filename, prefix); the prefix keeps
# signals/messages unique when two buses reuse the same names (the two
# inverters share every message + signal name, only their CAN IDs differ).
# Frame IDs must not collide across files (they don't for EV4).
DBC_SOURCES = [
    ("EV4_Vehicle_Bus.dbc", ""),      # main vehicle bus -> no prefix
    ("Inverter_1.dbc",      "INV1"),
    ("Inverter_2.dbc",      "INV2"),
]

# Friendly bus names by prefix (used for tab labels and the lookup table).
TAB_TITLES = {"": "Vehicle", "INV1": "Inverter 1", "INV2": "Inverter 2"}

LOOKUP_MAX = 150          # cap rows rendered per search

# Signals promoted to the big "driver" strip at the top.  Each entry is
# (label, qualified_signal_name, unit, format).  A signal is "qualified" with
# its source prefix, e.g. INV1_INV_Motor_Speed.  Missing signals are skipped.
KEY_SIGNALS = [
    ("SOC",     "BMS_SOC",              "%",   "{:.0f}"),
    ("POWER",   "PowerDraw_kW",         "kW",  "{:.0f}"),
    ("PACK",    "BMS_Pack_Voltage",     "V",   "{:.0f}"),
    ("MAX T",   "BMS_Max_Cell_Temp",    "C",   "{:.0f}"),
    ("APPS",    "APPS_Pct",             "%",   "{:.0f}"),
    ("TORQUE",  "Torque_Cmd",           "Nm",  "{:.0f}"),
    ("M1 RPM",  "INV1_INV_Motor_Speed", "rpm", "{:.0f}"),
    ("M2 RPM",  "INV2_INV_Motor_Speed", "rpm", "{:.0f}"),
]


def clean_unit(unit):
    """Cascadia DBC units look like 'temperature:C' / 'angular_speed:rpm'.
    Keep only the part after the colon for display."""
    if unit and ":" in unit:
        return unit.split(":", 1)[1]
    return unit or ""


def qualify(prefix, name):
    """Globally-unique signal key: 'INV1_INV_Motor_Speed', or just the raw
    name for the unprefixed vehicle bus."""
    return f"{prefix}_{name}" if prefix else name


def panel_key(prefix, msg):
    return f"{prefix} {msg.name}" if prefix else msg.name


class MsgInfo:
    """Everything needed to decode a frame and route it into the UI."""
    __slots__ = ("message", "prefix", "panel_key")

    def __init__(self, message, prefix):
        self.message = message
        self.prefix = prefix
        self.panel_key = panel_key(prefix, message)

# Colors
BG       = "#0d0d10"
PANEL    = "#16161c"
FG       = "#e6e6e6"
GREY     = "#8a8a92"
ACCENT   = "#ff2d2d"
GREEN    = "#27d17c"
YELLOW   = "#ffd23f"
CYAN     = "#36c5d6"


def is_fault_signal(name: str) -> bool:
    n = name.lower()
    return "fault" in n or n.endswith("_fault") or "critical" in n


# ── Cascadia Motion (PM100) inverter write helpers ───────────────────
# Calibration / command parameter addresses (CAN Protocol v5.9 §2.3.3-2.3.4).
PARAM_RESOLVER_DELAY_CMD   = 11    # live Resolver PWM Delay (PM Gen3 only)
PARAM_GAMMA_ADJUST_CMD     = 12    # live Gamma Adjust, degrees x10
PARAM_FAULT_CLEAR          = 20    # write 0 to clear faults
PARAM_RESOLVER_DELAY_EEP   = 151   # save Resolver PWM Delay to EEPROM
PARAM_GAMMA_ADJUST_EEP     = 152   # save Gamma Adjust to EEPROM, degrees x10


def _clamp_s16(v):
    return max(-32768, min(32767, int(v)))


def _frame_hex(data: bytes) -> str:
    return data.hex().upper()


def build_command_frame(torque_nm=0.0, speed_rpm=0, forward=True, enable=False,
                        discharge=False, speed_mode=False, torque_limit_nm=0.0):
    """0x_C0 Command Message. torque/limit are N·m (x10 on the wire),
    speed is RPM. Byte 5 packs enable/discharge/speed-mode bits."""
    b = bytearray(8)
    struct.pack_into("<h", b, 0, _clamp_s16(round(torque_nm * 10)))
    struct.pack_into("<h", b, 2, _clamp_s16(round(speed_rpm)))
    b[4] = 1 if forward else 0
    b[5] = ((1 if enable else 0)
            | ((1 if discharge else 0) << 1)
            | ((1 if speed_mode else 0) << 2))
    struct.pack_into("<h", b, 6, _clamp_s16(round(torque_limit_nm * 10)))
    return bytes(b)


def build_param_write(address, value, signed=True):
    """0x_C1 Read/Write Parameter Command, write. Data goes in bytes 4-5."""
    b = bytearray(8)
    struct.pack_into("<H", b, 0, address & 0xFFFF)
    b[2] = 1                                   # 1 = write
    fmt = "<h" if signed else "<H"
    struct.pack_into(fmt, b, 4, _clamp_s16(value) if signed else (int(value) & 0xFFFF))
    return bytes(b)


def build_param_read(address):
    """0x_C1 Read/Write Parameter Command, read."""
    b = bytearray(8)
    struct.pack_into("<H", b, 0, address & 0xFFFF)
    b[2] = 0                                   # 0 = read
    return bytes(b)


class SerialReader(threading.Thread):
    """Background thread: reads lines, decodes frames, pushes results to a queue."""

    def __init__(self, port, baud, frame_map, out_queue, status_queue):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.frame_map = frame_map
        self.out_queue = out_queue
        self.status_queue = status_queue
        self._stop = threading.Event()
        self.ser = None
        self._write_lock = threading.Lock()

    def stop(self):
        self._stop.set()

    def send_line(self, text):
        """Thread-safe write of one ASCII command line to the ESP32."""
        ser = self.ser
        if ser is None:
            return False
        try:
            with self._write_lock:
                ser.write((text + "\n").encode("ascii"))
            return True
        except Exception:
            return False

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.2)
        except Exception as e:
            self.status_queue.put(("error", f"Could not open {self.port}: {e}"))
            return
        self.ser = ser

        self.status_queue.put(("connected", self.port))
        buf = bytearray()
        try:
            while not self._stop.is_set():
                chunk = ser.read(512)
                if chunk:
                    buf.extend(chunk)
                    while b"\n" in buf:
                        line, _, rest = buf.partition(b"\n")
                        buf = bytearray(rest)
                        self._handle_line(line)
        except Exception as e:
            self.status_queue.put(("error", f"Serial read error: {e}"))
        finally:
            self.ser = None
            try:
                ser.close()
            except Exception:
                pass
            self.status_queue.put(("disconnected", self.port))

    def _handle_line(self, raw: bytes):
        try:
            line = raw.decode("ascii", "ignore").strip()
        except Exception:
            return
        if not line or line.startswith("#") or "#" not in line:
            return
        id_part, _, data_part = line.partition("#")
        if data_part.endswith("x") or data_part.endswith("X"):
            data_part = data_part[:-1]
        try:
            frame_id = int(id_part, 16)
            data = bytes.fromhex(data_part) if data_part else b""
        except ValueError:
            return

        info = self.frame_map.get(frame_id)
        if info is None:
            self.out_queue.put(("unknown", frame_id, None))
            return
        msg = info.message

        # Pad/truncate to the expected length so decode never crashes on a
        # short frame.
        if len(data) < msg.length:
            data = data + b"\x00" * (msg.length - len(data))
        elif len(data) > msg.length:
            data = data[: msg.length]

        try:
            decoded = msg.decode(data, decode_choices=False, scaling=True)
        except Exception:
            return
        # Qualify signal names with the source prefix so the two inverters
        # (identical signal names) never overwrite each other.
        qual = {qualify(info.prefix, k): v for k, v in decoded.items()}
        self.out_queue.put(("frame", info.panel_key, qual))


class DemoReader(threading.Thread):
    """Generates plausible values for EVERY message/signal on every bus so the
    whole UI (all tabs) can be exercised with no hardware.  Activated with
    --demo."""

    def __init__(self, ordered_msgs, out_queue, status_queue):
        super().__init__(daemon=True)
        self.ordered_msgs = ordered_msgs      # list of (prefix, message)
        self.out_queue = out_queue
        self.status_queue = status_queue
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    @staticmethod
    def _sig_value(sig, t):
        import math
        import random
        name = sig.name
        if is_fault_signal(name):
            return 1 if random.random() < 0.01 else 0     # occasional red blip
        if sig.length == 1:                               # boolean-ish
            return 1 if (int(t) + (hash(name) % 7)) % 7 == 0 else 0
        lo = sig.minimum if sig.minimum is not None else 0.0
        hi = sig.maximum if sig.maximum is not None else 0.0
        if hi <= lo:                                       # no usable range
            lo, hi = 0.0, 100.0
        phase = (hash(name) % 100) / 100.0 * 2 * math.pi   # desync the waves
        wave = (math.sin(t + phase) + 1) / 2               # 0..1
        val = lo + (hi - lo) * wave
        return round(val, 3)

    def run(self):
        self.status_queue.put(("connected", "DEMO"))
        t = 0.0
        while not self._stop.is_set():
            t += 0.1
            for prefix, msg in self.ordered_msgs:
                decoded = {qualify(prefix, s.name): self._sig_value(s, t)
                           for s in msg.signals}
                self.out_queue.put(("frame", panel_key(prefix, msg), decoded))
            time.sleep(0.1)
        self.status_queue.put(("disconnected", "DEMO"))


class Dashboard(tk.Tk):
    def __init__(self, sources, port, baud, demo=False):
        super().__init__()
        # sources: list of (prefix, cantools.Database)
        self.sources = sources
        self.vehicle_db = sources[0][1]
        self.baud = baud
        self.demo = demo
        self.reader = None

        # Build the frame_id -> MsgInfo routing table and an ordered list of
        # (prefix, message) for UI + CSV column order.
        self.frame_map = {}
        self.ordered_msgs = []
        for prefix, db in sources:
            # Keep real frames (incl. 29-bit extended, max 0x1FFFFFFF); the
            # VECTOR__INDEPENDENT_SIG_MSG placeholder (0xC0000000) is excluded.
            msgs = sorted((m for m in db.messages if m.frame_id <= 0x1FFFFFFF),
                          key=lambda m: m.frame_id)
            for m in msgs:
                if m.frame_id not in self.frame_map:   # first source wins on clash
                    self.frame_map[m.frame_id] = MsgInfo(m, prefix)
                self.ordered_msgs.append((prefix, m))

        # Inverter write targets: command + parameter message IDs per inverter,
        # looked up from the DBC by message name (so they track the DBC).
        self.inverters = {}
        for prefix, db in sources:
            if not prefix:
                continue
            try:
                cmd = db.get_message_by_name("M192_Command_Message")
                par = db.get_message_by_name("M193_Read_Write_Param_Command")
            except KeyError:
                continue
            self.inverters[prefix] = {"command_id": cmd.frame_id,
                                      "param_id": par.frame_id}

        # Transmit / heartbeat state (set by the Control tab).
        self.tx_active = False          # heartbeat running?
        self.tx_command = None          # 8-byte command frame being repeated
        self.tx_command_id = None       # CAN id for the heartbeat

        # Searchable catalog of every signal (for the Lookup tab).
        self.catalog = []
        for prefix, m in self.ordered_msgs:
            bus = TAB_TITLES.get(prefix, prefix or "Vehicle")
            for s in m.signals:
                comment = (s.comment or "").replace("\n", " ").strip()
                if s.choices:
                    enum = ", ".join(f"{k}={v}" for k, v in list(s.choices.items())[:10])
                    comment = (comment + "  |  " if comment else "") + "states: " + enum
                key = qualify(prefix, s.name)
                self.catalog.append({
                    "key": key, "name": s.name, "bus": bus, "prefix": prefix,
                    "msg": m.name, "frame_id": m.frame_id,
                    "unit": clean_unit(s.unit),
                    "scale": s.scale, "offset": s.offset,
                    "min": s.minimum, "max": s.maximum,
                    "comment": comment,
                    "blob": f"{key} {s.name} {bus} {m.name} {comment}".lower(),
                })
        self.lookup_value_labels = {}

        self.out_queue = queue.Queue()
        self.status_queue = queue.Queue()

        self.values = {}          # signal_name -> latest value
        self.msg_last_rx = {}     # message_name -> timestamp
        self.frame_count = 0
        self.unknown_count = 0
        self._fps_window = []     # timestamps for frames/sec

        self.value_labels = {}    # signal_name -> tk Label
        self.key_labels = {}      # signal_name -> tk Label
        self.panel_titles = {}    # message_name -> LabelFrame

        # CSV logging.  Column order = qualified signals grouped by message.
        self.log_signals = [qualify(prefix, s.name)
                            for prefix, m in self.ordered_msgs for s in m.signals]
        self.csv_file = None
        self.csv_writer = None
        self.log_rows = 0
        self.log_start = 0.0

        self.title("BER EV4 - Vehicle Bus Dashboard")
        self.configure(bg=BG)
        self.geometry("1280x820")
        self._build_ui(port)

        self.after(100, self._poll)

    # ---- UI construction -------------------------------------------------
    def _build_ui(self, port):
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        # --- connection bar ---
        bar = tk.Frame(self, bg=PANEL)
        bar.pack(side=tk.TOP, fill=tk.X)

        tk.Label(bar, text="Port", bg=PANEL, fg=GREY).pack(side=tk.LEFT, padx=(10, 4), pady=8)
        self.port_var = tk.StringVar(value=port or "")
        self.port_combo = ttk.Combobox(bar, textvariable=self.port_var, width=22, values=self._ports())
        self.port_combo.pack(side=tk.LEFT, padx=4)

        tk.Button(bar, text="Refresh", command=self._refresh_ports,
                  bg=PANEL, fg=FG, relief=tk.FLAT).pack(side=tk.LEFT, padx=4)

        tk.Label(bar, text="Baud", bg=PANEL, fg=GREY).pack(side=tk.LEFT, padx=(12, 4))
        self.baud_var = tk.StringVar(value=str(self.baud))
        tk.Entry(bar, textvariable=self.baud_var, width=8, bg=BG, fg=FG,
                 insertbackground=FG, relief=tk.FLAT).pack(side=tk.LEFT, padx=4)

        self.connect_btn = tk.Button(bar, text="Connect", command=self._toggle_connect,
                                     bg=GREEN, fg="#06210f", relief=tk.FLAT, width=10)
        self.connect_btn.pack(side=tk.LEFT, padx=10)

        self.status_lbl = tk.Label(bar, text="disconnected", bg=PANEL, fg=GREY)
        self.status_lbl.pack(side=tk.LEFT, padx=10)

        self.fps_lbl = tk.Label(bar, text="0 fps", bg=PANEL, fg=GREY)
        self.fps_lbl.pack(side=tk.RIGHT, padx=14)

        self.log_lbl = tk.Label(bar, text="", bg=PANEL, fg=GREY)
        self.log_lbl.pack(side=tk.RIGHT, padx=4)
        self.log_btn = tk.Button(bar, text="Log CSV", command=self._toggle_log,
                                 bg=PANEL, fg=FG, relief=tk.FLAT, width=10)
        self.log_btn.pack(side=tk.RIGHT, padx=6)

        # --- key metric strip ---
        strip = tk.Frame(self, bg=BG)
        strip.pack(side=tk.TOP, fill=tk.X, pady=(8, 4))
        all_sig_names = set(self.log_signals)
        col = 0
        for label, sig, unit, fmt in KEY_SIGNALS:
            if sig not in all_sig_names:
                continue
            cell = tk.Frame(strip, bg=PANEL, bd=0)
            cell.grid(row=0, column=col, padx=6, pady=2, sticky="nsew")
            strip.grid_columnconfigure(col, weight=1)
            tk.Label(cell, text=label, bg=PANEL, fg=GREY,
                     font=("Segoe UI", 11)).pack(anchor="w", padx=12, pady=(8, 0))
            val = tk.Label(cell, text="--", bg=PANEL, fg=FG,
                           font=("Consolas", 24, "bold"))
            val.pack(anchor="w", padx=12)
            tk.Label(cell, text=unit, bg=PANEL, fg=GREY,
                     font=("Segoe UI", 9)).pack(anchor="w", padx=12, pady=(0, 8))
            self.key_labels[sig] = (val, fmt)
            col += 1

        # --- tabbed pages: one per DBC source (Vehicle / Inverter 1 / 2) ---
        style.configure("TNotebook", background=BG, borderwidth=0)
        style.configure("TNotebook.Tab", background=PANEL, foreground=GREY,
                        padding=(16, 6), font=("Segoe UI", 10, "bold"))
        style.map("TNotebook.Tab",
                  background=[("selected", BG)], foreground=[("selected", FG)])

        nb = ttk.Notebook(self)
        nb.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8, pady=(4, 8))

        for prefix, _db in self.sources:
            msgs = [(p, m) for (p, m) in self.ordered_msgs if p == prefix]
            page = tk.Frame(nb, bg=BG)
            nb.add(page, text=TAB_TITLES.get(prefix, prefix or "Vehicle"))
            inner = self._make_scrollable(page)
            self._build_message_panels(inner, msgs)

        self._build_lookup_tab(nb)
        if self.inverters:
            self._build_control_tab(nb)

    def _make_scrollable(self, parent):
        """Create a vertically-scrollable frame inside parent; return the inner
        frame to populate.  Mouse wheel works while the pointer is over it."""
        canvas = tk.Canvas(parent, bg=BG, highlightthickness=0)
        vsb = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        inner = tk.Frame(canvas, bg=BG)
        inner.bind("<Configure>",
                   lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=inner, anchor="nw")
        canvas.configure(yscrollcommand=vsb.set)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        vsb.pack(side=tk.RIGHT, fill=tk.Y)
        # Route the wheel to whichever page the pointer is currently over.
        canvas.bind("<Enter>", lambda e: canvas.bind_all(
            "<MouseWheel>", lambda ev: canvas.yview_scroll(int(-ev.delta / 120), "units")))
        canvas.bind("<Leave>", lambda e: canvas.unbind_all("<MouseWheel>"))
        return inner

    # Color the panel border by source so the buses are easy to tell apart.
    SRC_COLOR = {"": CYAN, "INV1": "#7ee081", "INV2": "#e0c97e"}

    def _build_message_panels(self, parent, msgs):
        ncols = 3
        for i, (prefix, msg) in enumerate(msgs):
            tag = f"{prefix}  " if prefix else ""
            color = self.SRC_COLOR.get(prefix, CYAN)
            frame = tk.LabelFrame(parent,
                                  text=f"  {tag}{msg.name}  (0x{msg.frame_id:X})  ",
                                  bg=PANEL, fg=color, bd=1, relief=tk.SOLID,
                                  font=("Segoe UI", 10, "bold"), labelanchor="nw")
            frame.grid(row=i // ncols, column=i % ncols, padx=6, pady=6, sticky="nsew")
            parent.grid_columnconfigure(i % ncols, weight=1)
            self.panel_titles[panel_key(prefix, msg)] = (frame, color)

            for r, sig in enumerate(msg.signals):
                tk.Label(frame, text=sig.name, bg=PANEL, fg=GREY, anchor="w",
                         font=("Consolas", 9)).grid(row=r, column=0, sticky="w",
                                                    padx=(8, 6), pady=1)
                u = clean_unit(sig.unit)
                val = tk.Label(frame, text="--" + (f" {u}" if u else ""), bg=PANEL,
                               fg=FG, anchor="e", font=("Consolas", 9, "bold"), width=12)
                val.grid(row=r, column=1, sticky="e", padx=(6, 8), pady=1)
                self.value_labels[qualify(prefix, sig.name)] = (val, u)

    # ---- lookup / search tab --------------------------------------------
    def _build_lookup_tab(self, nb):
        page = tk.Frame(nb, bg=BG)
        nb.add(page, text="\U0001F50D Lookup")

        top = tk.Frame(page, bg=BG)
        top.pack(side=tk.TOP, fill=tk.X, padx=10, pady=10)
        tk.Label(top, text="Search signal:", bg=BG, fg=GREY,
                 font=("Segoe UI", 11)).pack(side=tk.LEFT)
        self.search_var = tk.StringVar()
        entry = tk.Entry(top, textvariable=self.search_var, bg=PANEL, fg=FG,
                         insertbackground=FG, relief=tk.FLAT,
                         font=("Consolas", 13), width=42)
        entry.pack(side=tk.LEFT, padx=10, ipady=3)
        entry.bind("<KeyRelease>", lambda e: self._do_search())
        tk.Button(top, text="Clear", command=lambda: (self.search_var.set(""), self._do_search()),
                  bg=PANEL, fg=FG, relief=tk.FLAT).pack(side=tk.LEFT, padx=4)
        self.search_count = tk.Label(top, text="", bg=BG, fg=GREY, font=("Segoe UI", 10))
        self.search_count.pack(side=tk.LEFT, padx=14)

        inner = self._make_scrollable(page)
        self.lookup_results = tk.Frame(inner, bg=BG)
        self.lookup_results.pack(fill=tk.BOTH, expand=True)
        self._do_search()

    def _do_search(self):
        for w in self.lookup_results.winfo_children():
            w.destroy()
        self.lookup_value_labels = {}

        q = self.search_var.get().strip().lower()
        if not q:
            self.search_count.config(text=f"{len(self.catalog)} signals — type to filter")
            tk.Label(self.lookup_results,
                     text="Start typing part of a signal name, message, bus, or description…",
                     bg=BG, fg=GREY, font=("Segoe UI", 11)).grid(row=0, column=0, padx=12, pady=20)
            return

        terms = q.split()
        matches = [c for c in self.catalog if all(t in c["blob"] for t in terms)]
        shown = matches[:LOOKUP_MAX]
        extra = f"  (showing first {LOOKUP_MAX})" if len(matches) > LOOKUP_MAX else ""
        self.search_count.config(text=f"{len(matches)} match"
                                 f"{'' if len(matches) == 1 else 'es'}{extra}")

        headers = ["Signal", "Bus", "Message", "Value", "Scale / Range", "Description"]
        widths = [30, 11, 26, 14, 22, 60]
        for col, (h, w) in enumerate(zip(headers, widths)):
            tk.Label(self.lookup_results, text=h, bg=BG, fg=CYAN, anchor="w",
                     font=("Segoe UI", 9, "bold"), width=w).grid(
                         row=0, column=col, sticky="w", padx=6, pady=(0, 4))

        for r, c in enumerate(shown, start=1):
            color = self.SRC_COLOR.get(c["prefix"], CYAN)
            tk.Label(self.lookup_results, text=c["name"], bg=BG, fg=FG, anchor="w",
                     font=("Consolas", 9, "bold")).grid(row=r, column=0, sticky="w", padx=6)
            tk.Label(self.lookup_results, text=c["bus"], bg=BG, fg=color, anchor="w",
                     font=("Consolas", 9)).grid(row=r, column=1, sticky="w", padx=6)
            tk.Label(self.lookup_results, text=f"{c['msg']} (0x{c['frame_id']:X})",
                     bg=BG, fg=GREY, anchor="w",
                     font=("Consolas", 9)).grid(row=r, column=2, sticky="w", padx=6)
            val = tk.Label(self.lookup_results, text="--", bg=BG, fg=FG, anchor="w",
                           font=("Consolas", 9, "bold"))
            val.grid(row=r, column=3, sticky="w", padx=6)
            self.lookup_value_labels[c["key"]] = (val, c["unit"])

            sr = f"×{c['scale']}"
            if c["offset"]:
                sr += f" {'+' if c['offset'] > 0 else ''}{c['offset']}"
            if c["min"] is not None and c["max"] is not None and c["max"] > c["min"]:
                sr += f"  [{c['min']}..{c['max']}]"
            tk.Label(self.lookup_results, text=sr, bg=BG, fg=GREY, anchor="w",
                     font=("Consolas", 8)).grid(row=r, column=4, sticky="w", padx=6)
            tk.Label(self.lookup_results, text=c["comment"], bg=BG, fg=GREY, anchor="w",
                     font=("Segoe UI", 8), justify="left", wraplength=460).grid(
                         row=r, column=5, sticky="w", padx=6)

        self._refresh_values()   # fill in current values right away

    # ---- control / write tab --------------------------------------------
    def _build_control_tab(self, nb):
        page = tk.Frame(nb, bg=BG)
        nb.add(page, text="⚙ Control")
        outer = self._make_scrollable(page)

        # Safety banner
        warn = tk.Frame(outer, bg="#3a0d0d")
        warn.grid(row=0, column=0, columnspan=2, sticky="ew", padx=8, pady=(8, 6))
        tk.Label(warn, bg="#3a0d0d", fg="#ff8a8a", justify="left",
                 font=("Segoe UI", 9, "bold"),
                 text=("⚠  WRITES TO LIVE INVERTERS — SPINS THE MOTOR.  "
                       "Drive wheels OFF the ground, area clear, physical e-stop ready.\n"
                       "STOP button disables the inverter and halts the heartbeat. "
                       "Closing the app or unplugging USB also safely stops the motor."
                       )).pack(anchor="w", padx=10, pady=8)

        # Inverter selector + torque cap
        sel = tk.Frame(outer, bg=BG)
        sel.grid(row=1, column=0, columnspan=2, sticky="w", padx=8, pady=4)
        tk.Label(sel, text="Inverter:", bg=BG, fg=GREY,
                 font=("Segoe UI", 11)).pack(side=tk.LEFT)
        self.ctl_inv = tk.StringVar(value=sorted(self.inverters)[0])
        for pfx in sorted(self.inverters):
            tk.Radiobutton(sel, text=TAB_TITLES.get(pfx, pfx), value=pfx,
                           variable=self.ctl_inv, bg=BG, fg=FG, selectcolor=PANEL,
                           activebackground=BG, activeforeground=FG,
                           command=self._ctl_inv_changed).pack(side=tk.LEFT, padx=6)
        tk.Label(sel, text="     Torque cap (Nm):", bg=BG, fg=GREY).pack(side=tk.LEFT)
        self.ctl_cap = tk.StringVar(value="30")
        tk.Entry(sel, textvariable=self.ctl_cap, width=6, bg=PANEL, fg=FG,
                 insertbackground=FG, relief=tk.FLAT).pack(side=tk.LEFT, padx=4)

        # ---- Command panel ----
        cmd = tk.LabelFrame(outer, text="  Command (motor)  ", bg=PANEL, fg=CYAN,
                            font=("Segoe UI", 10, "bold"), bd=1, relief=tk.SOLID)
        cmd.grid(row=2, column=0, sticky="nsew", padx=8, pady=6)

        self.ctl_armed = False
        self.ctl_enabled = False
        self.ctl_forward = tk.BooleanVar(value=True)
        self.ctl_torque = tk.StringVar(value="0")

        r = 0
        self.arm_btn = tk.Button(cmd, text="ARM (start heartbeat, disabled)",
                                 command=self._ctl_arm, bg="#33415c", fg=FG, relief=tk.FLAT)
        self.arm_btn.grid(row=r, column=0, columnspan=2, sticky="ew", padx=8, pady=(8, 4))
        r += 1
        self.enable_btn = tk.Button(cmd, text="ENABLE", command=self._ctl_enable,
                                    bg=PANEL, fg=GREY, relief=tk.FLAT, state=tk.DISABLED)
        self.enable_btn.grid(row=r, column=0, sticky="ew", padx=8, pady=2)
        self.disable_btn = tk.Button(cmd, text="DISABLE", command=self._ctl_disable,
                                     bg=PANEL, fg=FG, relief=tk.FLAT, state=tk.DISABLED)
        self.disable_btn.grid(row=r, column=1, sticky="ew", padx=8, pady=2)
        r += 1
        tk.Label(cmd, text="Direction:", bg=PANEL, fg=GREY).grid(row=r, column=0, sticky="w", padx=8)
        dirf = tk.Frame(cmd, bg=PANEL); dirf.grid(row=r, column=1, sticky="w")
        tk.Radiobutton(dirf, text="Forward", value=True, variable=self.ctl_forward, bg=PANEL,
                       fg=FG, selectcolor=BG).pack(side=tk.LEFT)
        tk.Radiobutton(dirf, text="Reverse", value=False, variable=self.ctl_forward, bg=PANEL,
                       fg=FG, selectcolor=BG).pack(side=tk.LEFT)
        r += 1
        tk.Label(cmd, text="Torque cmd (Nm):", bg=PANEL, fg=GREY).grid(row=r, column=0, sticky="w", padx=8, pady=4)
        tk.Entry(cmd, textvariable=self.ctl_torque, width=8, bg=BG, fg=FG,
                 insertbackground=FG, relief=tk.FLAT).grid(row=r, column=1, sticky="w", pady=4)
        r += 1
        self.estop_btn = tk.Button(cmd, text="⏹  STOP (disable + halt heartbeat)",
                                   command=self._ctl_estop, bg=ACCENT, fg="white",
                                   font=("Segoe UI", 10, "bold"), relief=tk.FLAT)
        self.estop_btn.grid(row=r, column=0, columnspan=2, sticky="ew", padx=8, pady=(6, 8))
        r += 1
        self.ctl_status = tk.Label(cmd, text="idle", bg=PANEL, fg=GREY, anchor="w")
        self.ctl_status.grid(row=r, column=0, columnspan=2, sticky="ew", padx=8, pady=(0, 8))

        # ---- Parameter panel ----
        par = tk.LabelFrame(outer, text="  Parameter read / write  ", bg=PANEL, fg=CYAN,
                            font=("Segoe UI", 10, "bold"), bd=1, relief=tk.SOLID)
        par.grid(row=2, column=1, sticky="nsew", padx=8, pady=6)
        self.par_addr = tk.StringVar(value="12")
        self.par_val = tk.StringVar(value="0")
        tk.Label(par, text="Address:", bg=PANEL, fg=GREY).grid(row=0, column=0, sticky="w", padx=8, pady=4)
        tk.Entry(par, textvariable=self.par_addr, width=8, bg=BG, fg=FG,
                 insertbackground=FG, relief=tk.FLAT).grid(row=0, column=1, sticky="w")
        tk.Label(par, text="Value:", bg=PANEL, fg=GREY).grid(row=1, column=0, sticky="w", padx=8, pady=4)
        tk.Entry(par, textvariable=self.par_val, width=8, bg=BG, fg=FG,
                 insertbackground=FG, relief=tk.FLAT).grid(row=1, column=1, sticky="w")
        bf = tk.Frame(par, bg=PANEL); bf.grid(row=2, column=0, columnspan=2, sticky="ew", padx=8, pady=4)
        tk.Button(bf, text="Read", command=self._par_read, bg="#33415c", fg=FG,
                  relief=tk.FLAT, width=8).pack(side=tk.LEFT, padx=2)
        tk.Button(bf, text="Write", command=self._par_write, bg="#5c4633", fg=FG,
                  relief=tk.FLAT, width=8).pack(side=tk.LEFT, padx=2)
        self.par_status = tk.Label(par, text="", bg=PANEL, fg=GREY, anchor="w", wraplength=300, justify="left")
        self.par_status.grid(row=3, column=0, columnspan=2, sticky="ew", padx=8, pady=4)

        # calibration quick-actions
        qa = tk.Frame(par, bg=PANEL); qa.grid(row=4, column=0, columnspan=2, sticky="w", padx=8, pady=(8, 8))
        tk.Label(qa, text="Quick:", bg=PANEL, fg=GREY).grid(row=0, column=0, sticky="w")
        tk.Button(qa, text="Fault Clear", command=self._par_fault_clear,
                  bg=PANEL, fg=FG, relief=tk.FLAT).grid(row=0, column=1, padx=2, pady=2)
        tk.Button(qa, text="Set Gamma (live)", command=lambda: self._fill_param(PARAM_GAMMA_ADJUST_CMD),
                  bg=PANEL, fg=FG, relief=tk.FLAT).grid(row=0, column=2, padx=2, pady=2)
        tk.Button(qa, text="Save Gamma→EEPROM", command=lambda: self._fill_param(PARAM_GAMMA_ADJUST_EEP),
                  bg=PANEL, fg=FG, relief=tk.FLAT).grid(row=1, column=1, padx=2, pady=2)
        tk.Button(qa, text="Set Resolver Delay (live)", command=lambda: self._fill_param(PARAM_RESOLVER_DELAY_CMD),
                  bg=PANEL, fg=FG, relief=tk.FLAT).grid(row=1, column=2, padx=2, pady=2)
        tk.Button(qa, text="Save Resolver Delay→EEPROM", command=lambda: self._fill_param(PARAM_RESOLVER_DELAY_EEP),
                  bg=PANEL, fg=FG, relief=tk.FLAT).grid(row=2, column=2, padx=2, pady=2)

        # ---- Live calibration readouts ----
        cal = tk.LabelFrame(outer, text="  Live calibration readouts (selected inverter)  ",
                            bg=PANEL, fg=CYAN, font=("Segoe UI", 10, "bold"), bd=1, relief=tk.SOLID)
        cal.grid(row=3, column=0, columnspan=2, sticky="ew", padx=8, pady=6)
        self.cal_labels = {}
        readouts = [("Delta Resolver (deg)  → target +90 fwd / -90 rev", "INV_Delta_Resolver_Filtered"),
                    ("Motor Speed (rpm)", "INV_Motor_Speed"),
                    ("Motor Angle Electrical (deg)", "INV_Motor_Angle_Electrical"),
                    ("Last param Read response (Data_Response)", "INV_Data_Response")]
        for i, (label, sig) in enumerate(readouts):
            tk.Label(cal, text=label, bg=PANEL, fg=GREY, anchor="w",
                     font=("Consolas", 10)).grid(row=i, column=0, sticky="w", padx=8, pady=2)
            v = tk.Label(cal, text="--", bg=PANEL, fg=FG, anchor="e",
                         font=("Consolas", 13, "bold"), width=12)
            v.grid(row=i, column=1, sticky="e", padx=8, pady=2)
            if sig:
                self.cal_labels[sig] = v

        outer.grid_columnconfigure(0, weight=1)
        outer.grid_columnconfigure(1, weight=1)

    # ---- control helpers ----
    def _ctl_cmd_id(self):
        return self.inverters[self.ctl_inv.get()]["command_id"]

    def _ctl_param_id(self):
        return self.inverters[self.ctl_inv.get()]["param_id"]

    def _ctl_torque_capped(self):
        try:
            cap = abs(float(self.ctl_cap.get()))
        except ValueError:
            cap = 30.0
        try:
            t = float(self.ctl_torque.get())
        except ValueError:
            t = 0.0
        return max(-cap, min(cap, t)), cap

    def _ctl_update_command(self):
        """Rebuild the heartbeat command frame from the current UI state."""
        torque, cap = self._ctl_torque_capped()
        if not self.ctl_enabled:
            torque = 0.0
        self.tx_command_id = self._ctl_cmd_id()
        self.tx_command = build_command_frame(
            torque_nm=torque, forward=self.ctl_forward.get(),
            enable=self.ctl_enabled, torque_limit_nm=cap)

    def _ctl_arm(self):
        if self.reader is None or self.demo:
            self.ctl_status.config(text="connect to a real ESP32 first", fg=YELLOW)
            return
        # Lockout release: heartbeat begins with a DISABLE command.
        self.ctl_armed = True
        self.ctl_enabled = False
        self.tx_active = True
        self._ctl_update_command()
        self.enable_btn.config(state=tk.NORMAL, bg=GREEN, fg="#06210f")
        self.disable_btn.config(state=tk.NORMAL)
        self.ctl_status.config(text="armed — heartbeat running, inverter DISABLED", fg=YELLOW)

    def _ctl_enable(self):
        if not self.ctl_armed:
            return
        self.ctl_enabled = True
        self._ctl_update_command()
        self.ctl_status.config(text="ENABLED — motor live", fg=ACCENT)

    def _ctl_disable(self):
        self.ctl_enabled = False
        self._ctl_update_command()
        self.ctl_status.config(text="armed — inverter DISABLED", fg=YELLOW)

    def _ctl_estop(self):
        self.ctl_enabled = False
        self.ctl_armed = False
        # send a final disable, then stop the heartbeat
        if self.reader is not None and not self.demo:
            self._send_oneshot(self._ctl_cmd_id(), build_command_frame(enable=False))
            self.reader.send_line("S")
        self.tx_active = False
        self.tx_command = None
        self.enable_btn.config(state=tk.DISABLED, bg=PANEL, fg=GREY)
        self.disable_btn.config(state=tk.DISABLED)
        self.ctl_status.config(text="STOPPED — heartbeat halted", fg=GREY)

    def _ctl_inv_changed(self):
        if self.tx_active:        # don't keep commanding a different inverter
            self._ctl_estop()

    # ---- parameter helpers ----
    def _fill_param(self, addr):
        self.par_addr.set(str(addr))

    def _par_write(self):
        if self.reader is None or self.demo:
            self.par_status.config(text="connect to a real ESP32 first", fg=YELLOW)
            return
        try:
            addr = int(self.par_addr.get(), 0)
            val = int(float(self.par_val.get()))
        except ValueError:
            self.par_status.config(text="bad address/value", fg=YELLOW)
            return
        self._send_oneshot(self._ctl_param_id(), build_param_write(addr, val, signed=True))
        self.par_status.config(text=f"wrote {val} to addr {addr} on {self.ctl_inv.get()}", fg=GREEN)

    def _par_read(self):
        if self.reader is None or self.demo:
            self.par_status.config(text="connect to a real ESP32 first", fg=YELLOW)
            return
        try:
            addr = int(self.par_addr.get(), 0)
        except ValueError:
            self.par_status.config(text="bad address", fg=YELLOW)
            return
        self._send_oneshot(self._ctl_param_id(), build_param_read(addr))
        self.par_status.config(text=f"read addr {addr} — see Data_Response in panels", fg=CYAN)

    def _par_fault_clear(self):
        if self.reader is None or self.demo:
            self.par_status.config(text="connect to a real ESP32 first", fg=YELLOW)
            return
        self._send_oneshot(self._ctl_param_id(), build_param_write(PARAM_FAULT_CLEAR, 0, signed=False))
        self.par_status.config(text=f"fault clear sent to {self.ctl_inv.get()}", fg=GREEN)

    # ---- ports -----------------------------------------------------------
    def _ports(self):
        return [p.device for p in list_ports.comports()]

    def _refresh_ports(self):
        self.port_combo["values"] = self._ports()

    # ---- connection ------------------------------------------------------
    def _toggle_connect(self):
        if self.reader and self.reader.is_alive():
            if getattr(self, "tx_active", False):
                self._ctl_estop()       # never leave the motor commanded
            self.reader.stop()
            self.reader = None
            self.connect_btn.config(text="Connect", bg=GREEN, fg="#06210f")
            self.status_lbl.config(text="disconnecting...", fg=GREY)
            return

        if self.demo:
            self.reader = DemoReader(self.ordered_msgs, self.out_queue, self.status_queue)
            self.reader.start()
            self.connect_btn.config(text="Disconnect", bg=ACCENT, fg="white")
            self.status_lbl.config(text="DEMO mode", fg=YELLOW)
            return

        port = self.port_var.get().strip()
        if not port:
            self.status_lbl.config(text="pick a port", fg=YELLOW)
            return
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            self.status_lbl.config(text="bad baud", fg=YELLOW)
            return

        self.reader = SerialReader(port, baud, self.frame_map, self.out_queue, self.status_queue)
        self.reader.start()
        self.connect_btn.config(text="Disconnect", bg=ACCENT, fg="white")
        self.status_lbl.config(text=f"opening {port}...", fg=YELLOW)

    # ---- main poll loop --------------------------------------------------
    def _poll(self):
        # drain status messages
        while True:
            try:
                kind, payload = self.status_queue.get_nowait()
            except queue.Empty:
                break
            if kind == "connected":
                self.status_lbl.config(text=f"connected {payload}", fg=GREEN)
            elif kind == "disconnected":
                self.status_lbl.config(text="disconnected", fg=GREY)
                self.connect_btn.config(text="Connect", bg=GREEN, fg="#06210f")
            elif kind == "error":
                self.status_lbl.config(text=payload, fg=ACCENT)
                self.connect_btn.config(text="Connect", bg=GREEN, fg="#06210f")

        # drain decoded frames
        now = time.time()
        got = False
        while True:
            try:
                item = self.out_queue.get_nowait()
            except queue.Empty:
                break
            got = True
            if item[0] == "unknown":
                self.unknown_count += 1
                self._fps_window.append(now)
                continue
            _, msg_name, decoded = item
            self.msg_last_rx[msg_name] = now
            self.frame_count += 1
            self._fps_window.append(now)
            for name, value in decoded.items():
                self.values[name] = value
            if self.csv_writer is not None:
                self._write_log_row(now, msg_name)

        if got:
            self._refresh_values()

        # frames/sec over a 1s sliding window
        self._fps_window = [t for t in self._fps_window if now - t <= 1.0]
        self.fps_lbl.config(text=f"{len(self._fps_window)} fps  |  unk {self.unknown_count}")
        if self.csv_writer is not None:
            self.log_lbl.config(text=f"{os.path.basename(self.log_path)}  ({self.log_rows} rows)",
                                fg=GREEN)

        self._mark_stale(now)
        self._service_heartbeat()
        self.after(100, self._poll)

    # ---- inverter transmit / heartbeat ----------------------------------
    def _service_heartbeat(self):
        """Refresh the ESP32's repeating command frame (~10 Hz). The ESP32
        re-sends it on CAN every 20 ms and stops if we go silent."""
        if not self.tx_active:
            return
        if self.reader is None or not getattr(self.reader, "is_alive", lambda: False)():
            return
        self._ctl_update_command()      # reflect live torque/direction edits
        if self.tx_command is not None:
            self.reader.send_line(f"H {self.tx_command_id:X} {_frame_hex(self.tx_command)}")

    def _send_oneshot(self, can_id, data):
        """Send a single CAN frame (parameter write/read, fault clear)."""
        if self.reader is None or self.demo:
            return False
        return self.reader.send_line(f"O {can_id:X} {_frame_hex(data)}")

    # ---- CSV logging -----------------------------------------------------
    def _toggle_log(self):
        if self.csv_writer is not None:
            self._stop_log()
            return
        os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(HERE, "logs", f"ev4_log_{stamp}.csv")
        try:
            self.csv_file = open(path, "w", newline="", encoding="utf-8")
        except OSError as e:
            self.log_lbl.config(text=f"log error: {e}", fg=ACCENT)
            return
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["datetime", "elapsed_s", "trigger_msg"] + self.log_signals)
        self.log_rows = 0
        self.log_start = time.time()
        self.log_path = path
        self.log_btn.config(text="Stop Log", bg=ACCENT, fg="white")
        self.log_lbl.config(text=os.path.basename(path), fg=GREEN)

    def _stop_log(self):
        if self.csv_file is not None:
            try:
                self.csv_file.close()
            except OSError:
                pass
        self.csv_file = None
        self.csv_writer = None
        self.log_btn.config(text="Log CSV", bg=PANEL, fg=FG)
        self.log_lbl.config(text=f"saved {self.log_rows} rows", fg=GREY)

    def _write_log_row(self, now, msg_name):
        row = [datetime.datetime.now().isoformat(timespec="milliseconds"),
               f"{now - self.log_start:.3f}", msg_name]
        for s in self.log_signals:
            v = self.values.get(s, "")
            row.append(float(v) if isinstance(v, bool) else v)
        try:
            self.csv_writer.writerow(row)
            self.log_rows += 1
        except (OSError, ValueError):
            self._stop_log()

    def _fmt(self, value, unit):
        try:
            f = float(value)
        except (TypeError, ValueError):
            return f"{value}{(' ' + unit) if unit else ''}"
        if f == int(f):
            txt = f"{int(f)}"
        else:
            txt = f"{f:.2f}"
        return txt + (f" {unit}" if unit else "")

    def _refresh_values(self):
        for name, (lbl, unit) in self.value_labels.items():
            if name not in self.values:
                continue
            value = self.values[name]
            lbl.config(text=self._fmt(value, unit))
            if is_fault_signal(name):
                try:
                    active = float(value) != 0
                except (TypeError, ValueError):
                    active = bool(value)
                lbl.config(fg=ACCENT if active else GREEN)

        for sig, (lbl, fmt) in self.key_labels.items():
            if sig in self.values:
                try:
                    lbl.config(text=fmt.format(float(self.values[sig])))
                except (TypeError, ValueError):
                    lbl.config(text=str(self.values[sig]))

        # live values in the Lookup tab
        for name, (lbl, unit) in self.lookup_value_labels.items():
            if name not in self.values:
                continue
            value = self.values[name]
            lbl.config(text=self._fmt(value, unit))
            if is_fault_signal(name):
                try:
                    active = float(value) != 0
                except (TypeError, ValueError):
                    active = bool(value)
                lbl.config(fg=ACCENT if active else GREEN)

        # live calibration readouts on the Control tab (selected inverter)
        cal = getattr(self, "cal_labels", None)
        if cal:
            pfx = self.ctl_inv.get()
            for base, lbl in cal.items():
                key = qualify(pfx, base)
                if key in self.values:
                    lbl.config(text=self._fmt(self.values[key], ""))

    def _mark_stale(self, now):
        for name, (frame, base_color) in self.panel_titles.items():
            last = self.msg_last_rx.get(name)
            if last is None:
                frame.config(fg=GREY)               # never seen
            elif now - last > STALE_AFTER:
                frame.config(fg=YELLOW)             # data going stale
            else:
                frame.config(fg=base_color)         # live (source color)

    def on_close(self):
        if getattr(self, "tx_active", False):
            self._ctl_estop()           # disable motor + stop heartbeat
        if self.csv_writer is not None:
            self._stop_log()
        if self.reader:
            self.reader.stop()
        self.destroy()


def load_sources():
    """Load every DBC in DBC_SOURCES. Returns list of (prefix, Database).
    Missing/broken files are warned about and skipped rather than fatal."""
    sources = []
    for fname, prefix in DBC_SOURCES:
        path = os.path.join(HERE, fname)
        if not os.path.exists(path):
            print(f"WARN: DBC not found, skipping: {path}")
            continue
        try:
            sources.append((prefix, cantools.database.load_file(path)))
            print(f"loaded {fname} as '{prefix or 'vehicle'}'")
        except Exception as e:
            print(f"WARN: failed to load {fname}: {e}")
    if not sources:
        sys.exit("No DBC files could be loaded.")
    return sources


def main():
    ap = argparse.ArgumentParser(description="BER EV4 serial CAN dashboard")
    ap.add_argument("--port", help="serial port (e.g. COM5). If omitted, pick in the UI.")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--demo", action="store_true",
                    help="generate fake data instead of reading serial (no hardware needed)")
    args = ap.parse_args()

    sources = load_sources()
    app = Dashboard(sources, args.port, args.baud, demo=args.demo)
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    if args.port or args.demo:
        app.after(300, app._toggle_connect)   # auto-connect / auto-start demo
    app.mainloop()


if __name__ == "__main__":
    main()
