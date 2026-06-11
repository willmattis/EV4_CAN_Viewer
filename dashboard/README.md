# EV4 Serial CAN Dashboard

Live PC dashboard for the BER EV4 Vehicle Bus. An ESP32 (with an MCP2515)
sniffs the CAN bus and streams every frame over USB serial; this Python app
decodes the frames against `EV4_Vehicle_Bus.dbc` and displays them live. It can
also **write** to the Cascadia inverters (command + parameter messages) for
motor calibration.

```
  Vehicle Bus (CAN 500k) ◄─► MCP2515 ◄─► ESP32 ◄─USB serial─► ev4_dashboard.py
                                         (bridge.ino)          (this app)
```

## 1. Flash the ESP32

Open `../firmware/EV4_CAN_Serial_Forwarder/EV4_CAN_Serial_Forwarder.ino` in the
Arduino IDE, select your ESP32 board, and upload. It needs the **mcp_can**
library (same one the EV3 dash uses). Wiring matches the dash board:

```
MCP2515 (HSPI):  SCK->14  SI->13  SO->12  CS->15  INT->4
Bus: 500 kbps, 8 MHz crystal
```

The board prints one line per CAN frame at **921600 baud**:

```
7#FF802C4500000000      <- id 0x7, 8 data bytes (hex)
4#0000C8001A00          <- id 0x4
```

(`#`-prefixed lines are status/comments and are ignored by the dashboard.)

## 2. Run the dashboard

Use a virtual environment so the packages stay isolated from your system Python.

**Windows (PowerShell)** — first time only:

```powershell
cd dashboard
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

Then run it (every time):

```powershell
.\.venv\Scripts\python.exe ev4_dashboard.py
```

If you'd rather "activate" the venv first (so plain `python` works), run
`.\.venv\Scripts\Activate.ps1` once per terminal — then just `python ev4_dashboard.py`.
If activation is blocked, run `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned` once.

<details><summary>macOS / Linux</summary>

```sh
cd dashboard
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python ev4_dashboard.py
```
</details>

Pick the ESP32's COM port from the dropdown and click **Connect**. Or skip the
UI step:

```sh
python ev4_dashboard.py --port COM5
```

Options: `--baud` (default 921600), `--dbc <path>` (default the bundled
`EV4_Vehicle_Bus.dbc`).

### Try it with no hardware

```sh
python ev4_dashboard.py --demo
```

Generates plausible fake telemetry for **every** message on all three buses
(values sweep through each signal's range, with occasional fault blips), so all
tabs and panels populate. Handy for checking the layout without the car.
Demo values are synthetic and can read out-of-range — they only prove the UI
works, not real behavior.

## What you see

- **Top strip** – big readouts for SOC, power, pack voltage, max cell temp,
  APPS %, and torque command.
- **Tabs** – the panels are split into pages: **Vehicle**, **Inverter 1**,
  **Inverter 2**, and **🔍 Lookup**. The top strip and connection bar stay
  visible on every tab.
- **🔍 Lookup tab** – type part of a signal name (or message, bus, or
  description) to instantly filter all 356 signals. Each result shows its bus,
  message + CAN ID, **live value**, scale/offset/range, and the DBC description
  (including enum/state meanings). Multi-word search is AND (e.g. `inv2 temp`).
- **Panels** – one per CAN message, every signal decoded with its DBC units.
  Fault signals turn **red** when set, **green** when clear. A panel title goes
  **yellow** if that message stops arriving (>1 s stale), **grey** if never seen.
- **Top-right** – frames/sec, count of unknown IDs (frames not in the DBC), and
  the **Log CSV** button.
- **⚙ Control tab** – write to the Cascadia inverters (command + parameter
  messages) for motor calibration. See "Writing to the inverters" below.

## Writing to the inverters (Control tab)

> ⚠️ **This spins the motor.** Drive wheels off the ground, area clear, physical
> e-stop within reach. Demo mode cannot transmit — you must be connected to a
> real ESP32.

The ESP32 firmware is a two-way bridge. The PC sends ASCII command lines and the
ESP32 puts them on CAN:

| Line | Meaning |
|------|---------|
| `H <idHex> <dataHex>` | set the repeating **heartbeat** frame (the inverter Command message). The ESP32 re-sends it every 20 ms. |
| `S` | stop the heartbeat |
| `O <idHex> <dataHex>` | send one frame once (parameter writes, fault clear) |

**Heartbeat + deadman safety:** the inverter faults if the Command message stops
for >1 s, so the ESP32 auto-repeats it. If the PC goes silent for >500 ms (USB
unplugged, app crash), the ESP32 drops the heartbeat and the inverter disables
the motor via its own CAN timeout. The **STOP** button, disconnecting, and
closing the app all send a Disable and halt the heartbeat.

**Command panel** (`0xC0` INV1 / `0xF0` INV2): ARM (starts the heartbeat with the
inverter disabled — this also releases the enable lockout), then ENABLE, set
direction and a capped torque command, DISABLE, or STOP.

**Parameter panel** (`0xC1` INV1 / `0xF1` INV2): read/write any parameter by
address, plus quick buttons for the calibration parameters and Fault Clear.

## Motor calibration (Cascadia resolver / gamma)

Full procedure from Cascadia's *Resolver Calibration Process*. Do this once per
inverter before ever running the motor. **Gamma Adjust applies to all inverter
generations; Resolver PWM Delay only applies to PM Gen3** (skip it on Gen5/CM).

1. **Set up safely.** Wheels off the ground. Motor Type EEPROM already set for
   your motor. Connect the dashboard to the ESP32 and pick the inverter on the
   Control tab.
2. **Clear faults** with the Fault Clear quick button.
3. *(PM Gen3 only)* **Resolver PWM Delay:** with the motor still, watch
   `INV_*` resolver signals while writing addr **11** (try values around 1100)
   to maximize the cosine reading, then save to EEPROM addr **151**.
4. **Verify direction:** slowly hand-spin the motor forward and confirm
   `Motor Angle Electrical` increases and `Motor Speed` reads positive. If it
   decreases / goes negative, the resolver wiring is reversed.
5. **Gamma Adjust:** spin the motor to ~¼–⅓ of base speed (≈1000 rpm) using a
   **small torque command** in torque mode, then DISABLE so it coasts. While
   coasting (inverter disabled), read **Delta Resolver (deg)** on the Control
   tab. Goal: **+90° forward** (or −90° reverse), held steady within ±0.7°.
6. **Adjust:** write Gamma Adjust (addr **12**, degrees) to drive Delta Resolver
   toward 90°. *Increasing gamma decreases delta resolver.* Example: delta reads
   82.8°, you need +7.2°, current gamma is 2.9° → new gamma = 2.9 − 7.2 = −4.3°.
   Re-spin, re-read, repeat until delta = 90° ±0.7°.
7. **Save** the final gamma to EEPROM (addr **152**), power-cycle the inverter,
   and re-verify.

If the motor won't spin at any gamma value, the resolver direction doesn't match
the motor phase order — swap both SIN with both COS, or swap two motor phases.

## Logging to CSV

Click **Log CSV** to start recording; click **Stop Log** to finish (it also
stops automatically when you close the app). Files are written to
`dashboard/logs/ev4_log_<date>_<time>.csv`.

Each received frame writes one row:

| datetime | elapsed_s | trigger_msg | APPS_Pct | BMS_SOC | ... |
|----------|-----------|-------------|----------|---------|-----|
| ISO timestamp | seconds since log start | which message arrived | every signal in the DBC |

Every signal gets its own column (the latest value is repeated each row, so any
column is a complete time series). `trigger_msg` tells you which message caused
that row. A cell is blank until that signal has been seen at least once. Opens
directly in Excel, Google Sheets, MATLAB, or pandas (`pd.read_csv`).

## DBC files (multiple buses)

The dashboard decodes against **three** DBCs at once, listed in `DBC_SOURCES`
near the top of `ev4_dashboard.py`:

| File | Prefix | CAN IDs |
|------|--------|---------|
| `EV4_Vehicle_Bus.dbc` | *(none)* | 2–7 |
| `Inverter_1.dbc` | `INV1` | 160–514 |
| `Inverter_2.dbc` | `INV2` | 208–562 |

The two inverters reuse the **same message and signal names** (e.g. both have
`INV_Motor_Speed`), so each source gets a prefix. In the UI, inverter panels are
titled `INV1 M165_...` / `INV2 M165_...` and colored differently; in the CSV the
columns are `INV1_INV_Motor_Speed`, `INV2_INV_Motor_Speed`, etc. The vehicle bus
keeps its plain names. Frame IDs don't overlap between the three files.

To add another bus, drop the `.dbc` next to the script and add a
`("file.dbc", "PREFIX")` line to `DBC_SOURCES`.

### IMD (Bender iso165C)

The vehicle-bus DBC includes the IMD message at the **29-bit extended** ID
`0x18FF01F4` (`IMD_Info`). `IMD_R_iso` (bytes 0–1, uint16, **kΩ**) is the
insulation resistance — that scaling is from the documented iso165C protocol.
Bytes 2–7 are added as raw `IMD_Status_Byte2..7` placeholders; replace them with
real status/flag signals once verified against your iso165C manual or firmware.
Extended IDs are handled automatically by both the firmware (tags them with a
trailing `x`) and the dashboard.

## Keeping it in sync with the bus

The DBCs are the single source of truth. When the team edits a bus layout,
replace the corresponding `.dbc` here (masters live in the team's
`EV4_Software/CAN/` folder) and restart the app — panels rebuild automatically.
No code changes needed for new/changed signals.

> Note: the EV3 dash firmware referenced a message ID `0x8` (speed / TS voltage)
> that isn't in any of these DBCs, so it won't decode until it's added. Any
> unknown ID is counted (top-right) but otherwise ignored.
