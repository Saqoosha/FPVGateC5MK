# Direct USB to RotorHazard

Firmware `1.2.0` and later exposes eight native RotorHazard serial
nodes over the C5's USB port. It needs no S3, Wi-Fi connection, FPVGate plugin
or split-timer/Sensor Tuning PR. Software tests pass and native USB discovery
has been verified on a Waveshare C5-Zero and a Seeed XIAO ESP32-C5, and the
XIAO has timed flown heats on a Raspberry Pi 5 alongside RX5808 nodes (see
Checks). Two or more pilots flying at once, and timing accuracy against video,
still need validation.

## Setup

1. Download the firmware for your board from the release, or build from
   `multipilot` with `pio run -e c5zero` or `pio run -e xiaoc5`. Follow
   [release/README.md](../release/README.md) to flash it. The same image supports
   both direct USB and the existing S3/UART connection; no mode-specific flash
   is needed.
2. Connect the C5's native USB port directly to the computer running RH, using
   a data cable. Fit the correct antenna. An S3 must not also control the receiver.
3. In RH, open **Advanced Settings > Hardware > Serial Ports**. Add its port,
   such as `COM5` on Windows or `/dev/ttyACM0` on Linux, and restart RH. A Linux
   `/dev/serial/by-id/` path is preferable when available because port numbers
   can change. The host account needs permission to open the port.
4. RH discovers eight nodes and applies its frequency profile. For Raceband,
   select R1-R8, assign each pilot to the matching seat, and calibrate EnterAt
   and ExitAt in the normal Sensor Tuning panel.
5. Start a race in RH. C5MK detects crossings on the device; RH records laps,
   displays them, announces times and saves results.

This is native serial compatibility, not OS-wide automatic port registration.
The port must be configured once. After unplugging, reconnect the board and
restart RH if its serial connection does not recover. RH's discovery and
reconnection behaviour is unchanged.

Close the C5 web dashboard, `mp.py`, flashing tools and serial monitors first:
only one application can own the port. If the FPVGate plugin is also installed,
do not configure a network gate as additional timing hardware unless you
actually want its extra race seats. The direct C5 does not use the plugin.

## Firmware behaviour

- After a reset the C5 sends nothing on USB until the host sends a byte; the
  dashboard and `mp.py` send a command first, and `mp.py console` stays quiet
  until something is typed. On Linux, opening the port resets the C5 (the
  kernel raises DTR and RTS, then the serial library lowers DTR first), and RH
  reads nothing for 2 s before its revision request. Output sent in that time
  was held by the host's USB serial driver, which clearing the port's input
  does not empty, so RH read scanner data as its revision code, failed, reopened
  the port and reset the C5 again. From 1.2.2 the scanner waits, scanner
  output still queued when RH takes USB is dropped, and the bootloader logs
  only warnings, leaving a few ROM lines that RH clears.
- Boot still opens the existing USB scanner. RH's revision request switches
  USB to its standard binary serial protocol, with eight independent seats. From
  1.2.2 it reports node API 36, RH's newest, so RH no longer shows "Node firmware
  update is available"; 1.2.1 and earlier report 35.
- Discovery starts at R1-R8; RH then applies its saved profile. Turning a seat
  off in RH (RH writes 1111, or 0) stops the C5 scanning it; 1.2.1 and earlier
  kept scanning it. Supported frequencies remain 5180-5917 MHz. After each
  frequency write RH runs API 36's register test, which passes when the seat is
  on a supported frequency.
- RSSI is the existing C5 node-filter output divided by four, clamped to RH's
  valid range 1-254, then smoothed as RH's own nodes do: samples are averaged
  per millisecond and those values go through a running median of the last
  255 ms. In flight the per-capture level drops 10-20 dB for a few ms at a time
  (multipath, the quad's attitude), so without it a pass crossed ExitAt and
  EnterAt several times: on one recorded flight the gap between the lowest
  gate peak and the highest level elsewhere on the track went from 14 to 29
  RH levels (about 3.5 to 7 dB). Crossing detection, the RSSI RH shows and its history all use the
  smoothed level, so a pass is timed about 128 ms late, the same for every
  pass. `node smooth <ms>` (0 to 255, 0 = off) changes the window until the C5
  restarts. Calibrate this mode rather than assuming thresholds from a
  different receiver transfer.
- EnterAt/ExitAt hysteresis and peak-plateau timing run on every successful
  capture. Pass timestamps use the plateau midpoint on the C5 clock, reported
  as milliseconds since the pass. USB polling delay is not added to that age;
  RH applies its own round-trip correction.
- Pass counters wrap at 256, as in RH's node protocol. Only the latest pass per
  seat is retained between host reads; this is not an unbounded event queue.
- RH polls each receiver a few times a second and its live graph adds a point
  about every 200 ms, while the C5 samples each seat about 1000 times a second.
  From 1.2.2 the RSSI the C5 reports to RH is the highest sample of about the
  last 250 ms, so every pass reaches the live graph at its real height; before,
  it was the latest sample, and a short pass could fall between graph points.
  RH's Capture averages this value, so captured levels come out a few counts
  higher than with 1.2.1 (on the safe side). Crossing detection still uses
  every sample.
- For RH's RSSI history (Marshal), the C5 keeps turning points as RH's own
  nodes do (`RssiNode.cpp`): a peak where the smoothed level turns from rising
  to falling, a nadir the reverse, each with its first and last sample times.
  Each waits in a one-entry queue; a poll takes the older of the two, a queued
  peak is replaced by a higher one or merged with an equal one (nadirs: lower,
  equal), and a poll with nothing queued sends level 0, which RH skips.
  Reporting a level at every poll, as 1.2.3 did, put mid-plateau points into
  the history; RH folds a third equal value into the previous one, so the
  plateau's real start was lost and Marshal's recalculated pass moved by up to
  58 ms on a real heat. With turning points only, recalculated and live pass
  times agree within 1 ms (a replay of a recorded flight at 80-200 ms poll
  intervals, and a five-lap heat). The full 1 kHz trace is not transferred.
- After any frequency change no crossing starts for 100 ms, and NodePeak and
  NodeNadir ignore those samples: the scan restarts with empty filters, and a
  lone settling sample could otherwise blip the crossing flag or count a pass.
- A VTX powering on or changing channel jumps from the noise floor past
  EnterAt in a few ms, and often lights up neighbouring channels for 10-80 ms
  on the way. A quad flying in takes far longer to rise that far. So a
  crossing that starts within 30 ms of leaving the floor, after at least 100 ms
  at the floor (within 16 levels of the seat's nadir), is hidden from RH for
  90 ms and dropped if it ends sooner. One that lasts becomes a normal
  crossing, timed from its peak as usual. A dropped jump is left out of RH's
  RSSI, history and NodePeak too, so its graph doesn't show it; one that lasts
  appears up to 90 ms late, at its own time. The same goes for a jump that
  stays under EnterAt: anything that rises twice the margin off the floor (or
  to EnterAt, if that is lower) that fast and falls back within 90 ms is left
  out.
  `node reject [rise hold quiet margin]` on the console changes the four
  values (`node reject 0` turns it off) until the C5 restarts. On a bench
  recording on 2026-10-16 this dropped 116 of 128 VTX start-up and
  channel-change bursts and no passes.
- The crossing flag RH reads stays set for 250 ms after a pass ends. RH reads
  it about every 100 ms and a fast pass can be over EnterAt for less, so
  without this most fast passes never showed as crossings on its graph. Lap
  counts and times are unchanged, and RH's force-end (race start and stop)
  clears it at once.
- Invalid checksums, unsupported frequencies and invalid seat indexes do not
  change settings. Thresholds are 1-254; detection is suspended while EnterAt
  is not higher than ExitAt. Failed captures do not produce timing samples.
- Crossings are cancelled after a sample gap over 100 ms. Stale reads report
  RSSI 1 with no crossing rather than showing the last reading as live.
- USB does not take the radio from an already active S3/UART session. While RH
  owns it, UART commands are ignored and framed scanner output is suppressed.
  From 1.2.2 RH keeps USB until the console asks for it back, however long it
  stays quiet: RH only starts polling once a browser connects, and 1.2.1 and
  earlier released USB after 5 s of silence and could not be reclaimed by RH,
  which then failed with "Error setting WRITE_CURNODE_INDEX" or "Unable to fetch
  revision code". A console command line (`scan on`, `status`, `node`, `help`,
  `regs`, `pilots`, `gain` or `timer`, as sent by the dashboard or `mp.py`) ends
  the RH session and runs. Power-cycle before switching an
  actively S3-controlled receiver to standalone USB.
- Thresholds are held in RAM; RH supplies calibration from its saved profile.
  C5 receiver gain remains the existing stored node gain. USB firmware updating
  through RH's STM32 updater is not advertised; use the normal C5 flash tool.

## Checks

From 1.2.2-rc.3, an unfinished or overflowed USB console line no longer blocks
RH's revision request. Earlier builds could report "Unable to fetch revision
code" repeatedly even after RH restarted. Linux PTY integration tests reproduce
this failure with the original parser and verify discovery after partial input,
overflow and tty echo with the fix. These tests do not validate physical Pi USB
hardware; see [TESTING.md](TESTING.md#linux-serial-integration-no-pi-or-c5-required).

That was not the only cause on a Raspberry Pi. With 1.2.2-rc.3 on a C5-Zero, RH
4.4.0 on a Pi 4 (Raspberry Pi OS Bookworm) still failed discovery: each
connection attempt reset the C5, and the scanner output it then sent arrived
ahead of the revision reply. 1.2.2 on the same setup was found at the first
attempt in all seven server starts (as 1.2.2-rc.4 and as the release image),
with eight seats and no communication errors; see [TESTING.md](TESTING.md#raspberry-pi-discovery).

Run the portable C++ tests with `bash test/run.sh` from `multipilot`.
For integration tests, install RH's dependencies, set `RH_SOURCE` to a standard
RH checkout (4.5.0-beta.1 was tested), and run:

```text
python -m unittest discover -s test -p test_rh_usb.py
```

The integration test feeds RH's actual `serial_node.discover()` and
`RHInterface` through a simulated serial port backed by the compiled C++
endpoint. It checks eight-seat discovery, firmware identification, independent
tuning, disabling a seat, calibration capture and peak-timestamp lap recording.
No live RH server, serial port or gate is contacted.

Local validation on 2026-10-13: 160 USB protocol checks, four integration tests
against stock RH 4.5.0-beta.1, 60 meter checks, 1201 node/UART checks and 48
reference-host checks passed. Both `c5zero` and `xiaoc5` development and 1.2.0
release firmware built successfully; their SRAM guard checks passed.

The Waveshare C5-Zero was subsequently flashed with all written
sections hash-verified. Boot checks reported live captures, and stock RH's
actual serial client discovered API 35 firmware `1.2.0-dev.1`, eight slots,
R1-R8 frequencies and live RSSI 28-31. A local RH server was then switched to
this USB receiver, preserving the R6 pilot assignment and two-minute heat.
The tester reported it working. This does not establish RF rate, gate-pass
accuracy against video, USB reconnection or XIAO behaviour.

Flight checks on 2026-11-03: a XIAO ESP32-C5 (ESP32-C5HR8) on a Raspberry Pi 5
with RH 4.4.0 and 4.5.0, discovered at the first attempt on every server start.
One quad flew Raceband/Fatshark F1 laps of about 13-18 s through one gate,
timed at the same time by the C5 and by a NuclearHazard board (STM32 +
RX5808) on the same Pi, both as RH nodes on the same frequencies.

- The C5 needs shielding like the RX5808s it replaces. With its antenna out
  in the open it heard the quad all round the track: the off-gate level
  (median) was 80, its highest 134, against gate peaks of 134-146, so no
  EnterAt separated passes from the rest of the lap. In the timer's metal case
  (half open) the off-gate median fell to 44-49 with the gate peaks unchanged.
- With the case and the 255 ms median, a heat RH timed as 5 laps on the C5,
  with no extra or missed passes; the RX5808 node on the same frequency counted
  one extra pass (a peak at exactly its EnterAt). Lowest gate peak against the
  highest off-gate level: C5 137 / 101, RX5808 124 / 104 (each on its own
  scale). Pass times of the two receivers differed by -317 to +364 ms per
  pass, varying from pass to pass, so lap times differed by up to 0.7 s; which
  receiver is nearer the truth needs video.
- Marshal's recalculated passes matched the C5's own within 1 ms on a real
  heat (up to 58 ms before history was sent as turning points).
- Neighbouring Raceband/Fatshark channels (R2, F1, R4, R5) did not see each
  other: with F1 on at the gate, R4 stayed at its floor (34.7 against 35.3 with
  the VTX off), although F1 sits 9 MHz from R4's image.
- Scan rate does not change what RH sees. With 8, 4 and 1 seats on (about 1040,
  2060 and 7850 Hz per seat), the reported floor and a steady VTX level moved
  less than the same setup drifted over a few minutes, in no consistent
  direction. Turning seats off in RH needs no new thresholds for this reason.
- The integration tests below also pass against the UDGP fork of RH 4.4.0 on
  the Pi.

Before deploying, use a spare C5 to check discovery, all eight graphs,
frequency/threshold readback, deliberate gate passes, unplug/replug and
switching back to the original S3/UART connection. Compare recorded pass times
against video before making accuracy claims.

## Reference

[Widscan58's RH setup](https://github.com/CapnBry/widscan58/blob/main/docs/RotorHazard.md)
demonstrates this native serial-node approach. C5MK's endpoint is independently
implemented from RotorHazard's public serial-client command IDs, checksums,
byte order and API 35 data layouts; no Widscan58 firmware code was copied.
