# Direct USB to RotorHazard

Firmware `1.2.0` and later exposes eight native RotorHazard serial
nodes over the C5's USB port. It needs no S3, Wi-Fi connection, FPVGate plugin
or split-timer/Sensor Tuning PR. Software tests pass and native USB discovery
has been verified on a Waveshare C5-Zero and a Seeed XIAO ESP32-C5, with
timing tested with one quad (see the field test below). Multi-pilot RF
detection and timing accuracy against video still need validation.

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

The crossing merge and peak-history preservation described below are new in
**1.2.4-rc.1** and are not in stable 1.2.3. Same-frequency sample sharing is
also included in this RC. See [release notes](../release/RELEASE_NOTES.md)
for the direct-USB accuracy comparison and hardware-validation limits.

The running median is not in 1.2.4-rc.1. Firmware built from the branch that
adds it (called "the median build" below) reports the same version, so check
with `node detect`: the median build answers `median 255 ms` and `merge 0 ms
(off)`, where 1.2.4-rc.1 answers `merge 300 ms`. Check before RH connects: a
console command ends RH's USB session. The release notes describe 1.2.4-rc.1's
defaults.

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
  valid range 1-254, then passed through a running median when one is set
  (the median build starts with 255 ms; see below). Calibrate this mode
  rather than assuming thresholds from a different receiver transfer.
- EnterAt/ExitAt hysteresis and peak-plateau timing run on every successful
  capture (with the median on, on each millisecond's median). Pass
  timestamps use the plateau midpoint on the C5 clock, reported
  as milliseconds since the pass. USB polling delay is not added to that age;
  RH applies its own round-trip correction.
- Pass counters wrap at 256, as in RH's node protocol. Only the latest pass per
  seat is retained between host reads; this is not an unbounded event queue.
- RH polls each receiver a few times a second and its live graph adds a point
  about every 200 ms, while the C5 samples each seat about 1030 times a second
  with eight seats on. From 1.2.2 the RSSI the C5 reports to RH is the highest
  level of about the last 250 ms, so every pass reaches the live graph at the
  height detection saw; before, it was the latest sample, and a short pass
  could fall between graph points. With the median on, that level is the
  median's, so passes show lower than their raw peak and ones too short for
  the median don't show at all. RH's Capture averages this value, so captured
  levels came out a few counts higher than with 1.2.1 (measured without the
  median). Crossing detection still uses every sample (every millisecond's
  median with the median on).
- For RH's RSSI history (Marshal), each poll also reports one extremum of the
  samples since the previous poll (median values with the median on), at its
  own sample times: the peak if it rose
  further from the extremum reported last than the nadir fell, otherwise the
  nadir. The full 1 kHz trace is not transferred. During a crossing, a poll
  whose samples reach the pass peak always reports that peak, so the history
  holds the first and last samples at the peak level. Marshal's
  recalculation times a lap from those, as the live lap is, so with the same
  EnterAt/ExitAt the two agree to the millisecond on every pass both find.
  They can still find different passes: Marshal cannot apply a merge, and
  history is one point per poll (see the replay table below). Before this
  the peak's poll often reported a multipath dip instead and recalculated
  laps were tens of ms off (up to 142 ms with slow polls in a simulation
  against RH 4.4.0's own client).
- From `1.2.4-rc.1`, seats on the same frequency share one capture, filtered
  value and timestamp per scan cycle, in USB and UART modes. Thresholds and
  crossing detection remain independent per seat.
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
  channel-change bursts and no passes. That was measured without the median;
  with it, bursts under about 128 ms are gone before the rejection sees them.
- The median build starts with a 255 ms running median and no merge
  (`node detect 0 0 255`). Samples are averaged per millisecond and RH gets
  the median of the last 255 of those millisecond values, like the median of
  the last 255 samples (at up to 1 kHz) in RH's own nodes. The window is 255 ms
  while each seat is sampled at least once per millisecond (eight seats on get
  about 1030 samples a second each, but jitter and failed captures leave the
  odd millisecond empty); a millisecond without a sample adds no value,
  so missed captures stretch it. Detection, RH's RSSI and its history all use
  the median, so Marshal's recalculation sees the same level as live
  detection; a merge window is invisible to Marshal, which is why it is off
  here.
  The median trails the signal by half its window (about 128 ms), so, like
  RH's nodes, which time the median at the middle of its window, the median
  build moves the pass and history times it sends RH back by that much: on
  the recorded flight, passes moved from 141 ms to 14 ms after a centred
  reference, with live and recalculated laps still within 1 ms. The C5's own
  clock is not moved, so the 100 ms gap and stale checks and the 250 ms RSSI
  hold are as before. Unlike RH's nodes, which report nothing until the window
  is full, the median build starts from one value. The window restarts when
  RH starts a session, when the seat's own frequency changes, for every seat
  when a settle ends (about 100 ms after any frequency change), after a gap of
  over 100 ms, and when `node detect` changes its length. It then grows from a
  single value, so for the first few ms it filters almost nothing (a burst of
  a few ms right after a restart can still count as a pass, as it would
  without the median) and reaches full strength only after 255 ms. With a
  full window, a pass above EnterAt for less than about 128 ms, and a VTX
  burst that short, never reaches detection; the VTX-jump rejection above
  sees median output, not raw samples.
  A recorded 2 kHz flight (four passes, one quad) was replayed through each
  version, polled like RH, with RH's history rebuild and Marshal
  recalculation. The table lists the EnterAt values (ExitAt 15 below) that
  timed every pass with no extras, and how many EnterAt values made the
  recalculation disagree with live. The replay used the raw capture and a
  harness outside this repository; `multipilot/tools/rh_db_report.py
  --replay` replays RH's saved history, one point per poll, which is too
  sparse to put through a median:

  | Version | EnterAt timing all 4 passes | EnterAt 60-125 (14 values) where recalc disagreed with live |
  |---|---|---|
  | 1.2.3 | none | all |
  | 1.2.4-rc.1 (merge 300) | 120, 125 | 13 |
  | + median 255, merge 300 | 90, 105-120 | 6 |
  | + median 255, merge 0 | 105-120 | 2 |

  The median removes the multipath dips inside a pass and also short peaks
  elsewhere on the track, which a merge window cannot, so it widens the range
  of working thresholds.
- The median build was flown outdoors on 2026-11-07 with one quad on F1, the
  C5 beside an RX5808 node on the same RH: five races, 42 passes. RH read both
  receivers in races 1-3. In races 4-5 it read only the RX5808 while the C5
  recorded raw samples, replayed through the firmware's detection afterwards.

  | | RX5808 | C5 |
  |---|---|---|
  | Passes timed | 41 of 42 | 42 of 42 |
  | Extra laps | 0 | 0 |
  | Lowest pass peak, races 1-3 | 102 | 119 |
  | Highest level away from passes, races 1-3 | 95 | 84 |

  The RSSI scales differ, so compare each column with itself: the gap between
  lowest peak and highest other level, divided by the full range, was 0.15-0.29
  on the RX5808 and 0.29-0.51 on the C5. The RX5808's miss was a peak of 102
  under EnterAt 107; at 100/90 it timed the other 35. The C5 stayed at 105/90.
  Neither receiver can be called more accurate, as there was no reference. In
  races 1-3 the C5 was earlier on 20 of 22 passes, by 31 ms on average, and
  lap times differed by 16-23 ms on average per race, 70 ms at most. In races
  4-5 the two clocks were matched only through the host's wall clock, so only
  the spread counts: one pass in race 4 differed by 106 ms and a lap by 138 ms.
  Race 2's pass differences fell into two clusters, near 0 and near -60 ms. No
  other race showed that, and the cause is unknown. It was not peak splitting:
  race 5's split peaks differed from the RX5808 by no more than other passes.

  In the raw samples a pass rises for about a second, usually dips 4-13 dB
  just before the peak, has a peak 43-170 ms wide at -6 dB and then falls
  steeply. The peak is narrower than the window, so the median tops out 16-59
  levels (4-15 dB) under it: 118-151 against 164-187 raw. No pass was lost, but
  the lowest was only 13 above EnterAt 105. A faster course may need a shorter
  median. Replaying other lengths gave these EnterAt values (ExitAt 15 below)
  timing every pass with no extras, and recalculated laps within 1 ms of live:

  | Median | Field, races 4 and 5 | Indoor flight above |
  |---|---|---|
  | 255 ms | 90-115 | 105-120 |
  | 191 ms | 95, 105-120 | 110-120 |
  | 127 ms | 105-140 | 120 |
  | 95 ms | 105-145 | 120 |
  | 63 ms | 110-150 | 120, 125 |

  Shorter medians suit the field and not the indoor flight, whose dips are
  deeper, so 255 ms stays the default. The indoor reference times came from
  the 255 ms replay, which favours it. With no median, no EnterAt timed the
  field races without extras. The dip is no use as a timing mark: it came
  98-392 ms before the fall. Timing from the fall itself was inconclusive (17
  laps, no reference).
- With a merge set (1.2.4-rc.1's default, 300 ms; off in the median build), a
  pass ends only once the level has stayed under ExitAt for that long. A quad
  passing a gate in a small room dips under ExitAt and back (multipath); back
  over EnterAt within the merge is the same pass, timed at its highest
  sample, as FPVGate's own lap detection does. Rising only to between ExitAt
  and EnterAt doesn't rejoin it. The lap count reaches RH the merge time after
  the pass, still timed at the peak. A pass already waiting out the merge when
  the samples stop still counts; RH's force-end drops it, as it drops any
  crossing. In a whoop hall with 1-unit ExitAt hysteresis, each pass produced
  up to five extra crossings without it.
  `node detect [merge smooth median]` on the console changes these until the
  C5 restarts (with no values it only reports them): `node detect 300 0 255`
  adds the merge to the median, and `node detect 300 0 0` is 1.2.4-rc.1's
  behaviour. `smooth` (ms, off by default) averages the level over about that
  long before crossing detection and before RH sees it. That steadies a noisy
  trace but lowers passes shorter than it on RH's graph, so try it per venue.
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

Before deploying, use a spare C5 to check discovery, all eight graphs,
frequency/threshold readback, deliberate gate passes, unplug/replug and
switching back to the original S3/UART connection. Compare recorded pass times
against video before making accuracy claims.

## Reference

[Widscan58's RH setup](https://github.com/CapnBry/widscan58/blob/main/docs/RotorHazard.md)
demonstrates this native serial-node approach. C5MK's endpoint is independently
implemented from RotorHazard's public serial-client command IDs, checksums,
byte order and API 35 data layouts; no Widscan58 firmware code was copied.
