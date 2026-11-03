#include "rh_usb.h"

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

static int checks = 0, fails = 0;
#define CHECK(c) do { ++checks; if (!(c)) { ++fails; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)

struct Port : rhusb::Port {
    std::vector<uint8_t> bytes;
    std::vector<uint16_t> slots;
    unsigned changes = 0;
    void write(const uint8_t* data, size_t size) override { bytes.insert(bytes.end(), data, data + size); }
    void frequencies(const uint16_t* mhz) override {
        slots.assign(mhz, mhz + rhusb::kSeats);
        ++changes;
    }
};

std::vector<uint8_t> read(rhusb::Core& core, Port& port, uint8_t command, uint32_t now = 1000) {
    port.bytes.clear();
    core.feed(command, now);
    CHECK(!port.bytes.empty());
    if (port.bytes.empty()) return {};
    uint8_t sum = 0;
    for (size_t i = 0; i + 1 < port.bytes.size(); ++i) sum += port.bytes[i];
    CHECK(sum == port.bytes.back());
    return {port.bytes.begin(), port.bytes.end() - 1};
}

void write(rhusb::Core& core, uint8_t command, std::initializer_list<uint8_t> data, uint32_t now = 1000) {
    core.feed(command, now);
    uint8_t sum = 0;
    for (uint8_t value : data) { core.feed(value, now); sum += value; }
    core.feed(sum, now);
}

void tests() {
    Port port;
    rhusb::Core core(port);
    core.setIdentity("1.1.0", "xiaoc5");
    core.begin(0);
    CHECK(read(core, port, 0x22) == std::vector<uint8_t>({0x25, 36}));
    CHECK(read(core, port, 0x39) == std::vector<uint8_t>({8}));
    CHECK(read(core, port, 0x3D).size() == 16);
    CHECK(read(core, port, 0x40).size() == 16);
    CHECK(read(core, port, 0x3E).size() == 16);
    CHECK(read(core, port, 0x3F).size() == 16);
    CHECK(read(core, port, 0x11) == std::vector<uint8_t>({0, 0}));
    CHECK(read(core, port, 0x33, 0x12345678) == std::vector<uint8_t>({0x12, 0x34, 0x56, 0x78}));
    for (uint8_t i = 0; i < 8; ++i) {
        write(core, 0x7A, {i});
        CHECK(read(core, port, 0x3A)[0] == i);
        CHECK(read(core, port, 0x3C)[0] == i);
        uint16_t expected[] = {5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917};
        auto frequency = read(core, port, 0x03);
        CHECK((frequency[0] * 256 + frequency[1]) == expected[i]);
    }
    write(core, 0x7A, {8});
    CHECK(read(core, port, 0x3A)[0] == 7);
    write(core, 0x7A, {5});
    write(core, 0x71, {100});
    write(core, 0x72, {80});
    CHECK(read(core, port, 0x31)[0] == 100);
    CHECK(read(core, port, 0x32)[0] == 80);
    core.sample(5, 200, 1000);
    core.sample(5, 440, 1001);
    auto pass = read(core, port, 0x0D, 1002);
    auto extremum = read(core, port, 0x0E, 1002);
    CHECK(pass.size() == 8 && extremum.size() == 8);
    CHECK((extremum[0] & 1) == 1);
    core.sample(5, 600, 1003);
    core.sample(5, 600, 1005);
    core.sample(5, 600, 1007);
    core.sample(5, 300, 1008);
    auto stats = read(core, port, 0x05, 1010);
    CHECK(stats.size() == 16);
    CHECK(stats[0] == 1 && stats[1] == 0 && stats[2] == 5);  // plateau middle at 1005
    // RSSI shown: the highest sample of the last ~250 ms (600 / 4), not the last (300 / 4).
    // The pass ended at 1008 but still shows as crossing for RH's slower reads.
    CHECK(stats[3] == 150 && stats[5] == 150 && (stats[8] & 1));
    write(core, 0x78, {0});
    // Split reads are latched, even if samples or a new pass arrive in between.
    read(core, port, 0x0D, 1010);
    core.sample(5, 800, 1011);
    CHECK(!(read(core, port, 0x0E, 1012)[0] & 1));
    write(core, 0x78, {0});
    CHECK(!(read(core, port, 0x05, 1012)[8] & 1));
    // Failed writes, unknown commands and partial frames do not modify settings.
    core.feed(0x71, 1020); core.feed(150, 1020); core.feed(149, 1020);
    CHECK(read(core, port, 0x31, 1020)[0] == 100);
    core.feed(0x51, 1020); core.feed(0xFF, 1020);
    CHECK(read(core, port, 0x22, 1120).size() == 2);
    write(core, 0x51, {0x17, 0x38});  // 5944 unsupported: previous R6 remains
    CHECK(read(core, port, 0x03) == std::vector<uint8_t>({0x16, 0xD3}));
    CHECK(read(core, port, 0x04)[0] == 1);  // API 36 register test: tuned
    write(core, 0x51, {0, 0});
    CHECK(port.slots[5] == 0 && port.slots[6] == 5880);
    CHECK(read(core, port, 0x04)[0] == 0);  // off
    // RH turns a seat off with its "power down" 1111 and reads it back.
    write(core, 0x51, {0x16, 0xD3});
    CHECK(port.slots[5] == 5843);
    write(core, 0x51, {0x04, 0x57});
    CHECK(port.slots[5] == 0 && port.slots[6] == 5880);
    CHECK(read(core, port, 0x03) == std::vector<uint8_t>({0x04, 0x57}));
    CHECK(read(core, port, 0x04)[0] == 0);
    core.sample(5, 600, 1020);  // an off seat takes no samples
    CHECK(read(core, port, 0x05, 1020)[3] == 1);
    write(core, 0x51, {0, 0});
    core.sample(5, 1023, 1200);
    CHECK(read(core, port, 0x05, 1200)[3] == 1);
    write(core, 0x51, {0x16, 0xD3});
    core.sample(5, 1023, 1201);
    CHECK(read(core, port, 0x05, 1201)[3] == 254);
    // A missing capture run cannot turn the old crossing into a lap.
    core.sample(5, 200, 1400);
    CHECK(read(core, port, 0x05, 1400)[0] == 1);
    core.sample(5, 600, 1500);
    CHECK(read(core, port, 0x05, 1601)[3] == 1);
    CHECK(!(read(core, port, 0x05, 1601)[8] & 1));
    CHECK(!core.expired(6600));
    CHECK(core.expired(6601));
    read(core, port, 0x22, 0xFFFFFF00);
    CHECK(!core.expired(0x100));  // millisecond rollover
    core.begin(0xFFFFFF00);
    write(core, 0x7A, {0}, 0xFFFFFF00);
    core.sample(0, 600, 0xFFFFFFFE);
    core.sample(0, 600, 2);
    core.sample(0, 100, 3);
    stats = read(core, port, 0x05, 10);
    CHECK(stats[0] == 1 && stats[2] == 10);  // plateau middle wraps to zero
    // Lap counters wrap without losing a pass, as expected by RH's API.
    core.begin(0);
    for (unsigned i = 0; i < 256; ++i) {  // after the start-up settle time
        core.sample(0, 600, rhusb::kSettleMs + i * 2);
        core.sample(0, 100, rhusb::kSettleMs + i * 2 + 1);
    }
    CHECK(read(core, port, 0x05, rhusb::kSettleMs + 512)[0] == 0);
}

// RSSI history: RH polls a few times a second and each poll takes one
// turning point, the older of the queued peak and nadir, as RH's own nodes do.
void historyTests() {
    Port port;
    rhusb::Core core(port);
    core.begin(0);
    rhusb::Reject off;
    off.riseMs = 0;  // short peaks straight off the floor (see rejectTests)
    core.setReject(off);
    write(core, 0x7A, {2});
    write(core, 0x71, {200});  // no crossings here: history only
    write(core, 0x72, {190});
    auto poll = [&](uint32_t now) { read(core, port, 0x0D, now); return read(core, port, 0x0E, now); };
    // flags, pass nadir, node nadir, level, ms since first (16 bit), duration (16 bit)
    auto level = [](const std::vector<uint8_t>& e) { return e[3]; };
    auto since = [](const std::vector<uint8_t>& e) { return e[4] * 256 + e[5]; };
    auto held = [](const std::vector<uint8_t>& e) { return e[6] * 256 + e[7]; };
    for (uint32_t t = 0; t < 50; ++t) core.sample(2, 120, t);  // floor: 30
    auto e = poll(50);
    CHECK(level(e) == 0);  // no turning point yet: nothing for RH's history (0 is skipped)
    // A 3 ms pass at 70 ms: the floor before it is a nadir, the pass a peak.
    for (uint32_t t = 50; t < 100; ++t) core.sample(2, t >= 70 && t < 73 ? 440 : 120, t);
    e = poll(100);
    CHECK(!(e[0] & 2) && level(e) == 30 && since(e) == 100 && held(e) == 69);  // 30 at 0..69, older
    // A 1 ms dip at 120 ms; the pass waits its turn and keeps its times.
    for (uint32_t t = 100; t < 150; ++t) core.sample(2, t == 120 ? 100 : 120, t);
    e = poll(150);
    CHECK((e[0] & 2) && level(e) == 110 && since(e) == 80 && held(e) == 2);  // 440 at 70..72
    for (uint32_t t = 150; t < 200; ++t) core.sample(2, 160, t);
    e = poll(200);
    CHECK(!(e[0] & 2) && level(e) == 25 && since(e) == 80 && held(e) == 0);  // the dip
    // Still rising (to 40 at 150 ms), so no turning point: nothing.
    e = poll(250);
    CHECK(level(e) == 0);
    // READ_LAP_STATS carries the same history in bytes 8-15.
    for (uint32_t t = 250; t < 300; ++t) core.sample(2, t == 260 ? 600 : 160, t);
    auto stats = read(core, port, 0x05, 300);
    CHECK((stats[8] & 2) && stats[11] == 150 && stats[3] == 150);  // shown: the 260 ms peak

    // Between two polls a peak and then a deeper fall: both reach RH, in order.
    // (Before, one extremum per poll was kept and the other dropped.)
    for (uint32_t t = 300; t < 400; ++t) core.sample(2, t >= 310 && t < 313 ? 440 : t >= 330 && t < 340 ? 40 : 160, t);
    e = poll(400);
    CHECK(!(e[0] & 2) && level(e) == 40 && since(e) == 139 && held(e) == 48);  // the floor at 261..309
    e = poll(450);
    CHECK((e[0] & 2) && level(e) == 110 && since(e) == 140 && held(e) == 2);  // 440 at 310..312
    e = poll(500);
    CHECK(!(e[0] & 2) && level(e) == 10 && since(e) == 170 && held(e) == 9);  // 40 at 330..339

    // Two equal peaks while one is queued merge, so the plateau spans both.
    for (uint32_t t = 500; t < 600; ++t)
        core.sample(2, (t >= 510 && t < 513) || (t >= 530 && t < 533) ? 440 : t == 560 ? 200 : 160, t);
    std::vector<std::vector<uint8_t>> seen;
    for (uint32_t now = 600; now <= 800; now += 50) seen.push_back(poll(now));
    bool merged = false;
    for (auto& h : seen) merged |= (h[0] & 2) && level(h) == 110 && held(h) == 22;  // 510..532
    CHECK(merged);

    // A poll in the middle of a pass's top plateau sends nothing; the whole
    // plateau follows once it ends. (A mid-plateau level made RH fold the
    // plateau's real first time away, and Marshal's recalc moved the pass.)
    for (uint32_t t = 800; t < 900; ++t) core.sample(2, 120, t);
    while (level(poll(900)) != 0) {}
    for (uint32_t t = 900; t < 1000; ++t) core.sample(2, t < 945 ? 120 : 520, t);  // 130 from 945
    e = poll(1000);
    CHECK(!(e[0] & 2));  // the floor before it at most, never 130
    for (uint32_t t = 1000; t < 1100; ++t) core.sample(2, t < 1002 ? 520 : 120, t);  // ends at 1001
    std::vector<std::vector<uint8_t>> after;
    for (uint32_t now = 1100; now <= 1200; now += 50) after.push_back(poll(now));
    bool plateau = false;
    for (size_t i = 0; i < after.size(); ++i)
        plateau |= (after[i][0] & 2) && level(after[i]) == 130 && since(after[i]) == 1100 + 50 * i - 945 &&
                   held(after[i]) == 56;  // 945..1001
    CHECK(plateau);
}

// The RSSI RH shows (its live graph, a point every ~200 ms) holds each peak
// for ~250 ms, and a frequency change can't start a crossing while settling.
void shownAndSettleTests() {
    Port port;
    rhusb::Core core(port);
    core.begin(0);
    rhusb::Reject off;
    off.riseMs = 0;  // short peaks straight off the floor (see rejectTests)
    core.setReject(off);
    write(core, 0x7A, {1}, 0);
    write(core, 0x71, {100}, 0);
    write(core, 0x72, {80}, 0);
    // Samples up to 'now' (a 3 ms peak at 500 ms, under EnterAt), then the RSSI shown at 'now'.
    uint32_t sampled = 200;
    auto shown = [&](uint32_t now) {
        for (; sampled <= now; ++sampled) core.sample(1, sampled >= 500 && sampled < 503 ? 380 : 120, sampled);
        return read(core, port, 0x05, now)[3];
    };
    // The peak shows from then until ~250 ms later, then the floor.
    CHECK(shown(450) == 30);
    CHECK(shown(510) == 95);
    CHECK(shown(700) == 95);
    CHECK(shown(760) == 30);
    CHECK(shown(999) == 30);
    // Settling: a lone high sample right after a frequency change isn't a pass.
    uint8_t laps = read(core, port, 0x05, 1000)[0];
    write(core, 0x51, {0x16, 0x47}, 1000);  // seat 1 to 5703 (any change settles all seats)
    core.sample(1, 600, 1010);
    CHECK(!(read(core, port, 0x05, 1010)[8] & 1));
    core.sample(1, 120, 1011);
    CHECK(read(core, port, 0x05, 1011)[0] == laps);
    CHECK(read(core, port, 0x23, 1011)[0] == 1);  // nor a node peak (reset by the change)
    // After it, a pass counts as usual.
    core.sample(1, 600, 1120);
    core.sample(1, 120, 1121);
    CHECK(read(core, port, 0x05, 1121)[0] == (uint8_t)(laps + 1));
}

// Smoothing: a pass whose level drops for a few ms at a time is one lap, a
// short burst is none, and several samples in one millisecond are averaged.
void smoothTests() {
    auto run = [](unsigned smoothMs, uint32_t burstMs, unsigned* lapsOut, uint16_t* ageOut) {
        Port port;
        rhusb::Core core(port);
        core.begin(0);
        rhusb::Reject off;
        off.riseMs = 0;  // short peaks straight off the floor (see rejectTests)
        core.setReject(off);
        core.setSmooth(smoothMs);
        write(core, 0x7A, {3}, 0);
        write(core, 0x71, {100}, 0);
        write(core, 0x72, {85}, 0);
        // Floor 40; from 1000 ms a signal at 140 that drops to 40 for 8 ms in every 40.
        for (uint32_t t = 200; t < 2500; ++t) {
            bool on = t >= 1000 && t < 1000 + burstMs && (t - 1000) % 40 >= 8;
            core.sample(3, on ? 560 : 160, t);
        }
        auto stats = read(core, port, 0x05, 2500);
        *lapsOut = stats[0];
        *ageOut = (uint16_t)(stats[1] * 256 + stats[2]);
        return core.smooth();
    };
    unsigned laps;
    uint16_t age;
    run(0, 600, &laps, &age);
    CHECK(laps > 10);  // unsmoothed: every dip ends the crossing
    CHECK(run(255, 600, &laps, &age) == 255);
    CHECK(laps == 1);
    // Timed at the middle of the smoothed plateau: ~1300 ms, ~128 ms late.
    CHECK(age >= 2500 - 1300 - 128 - 40 && age <= 2500 - 1300 - 128 + 40);
    run(255, 100, &laps, &age);
    CHECK(laps == 0);  // a 100 ms burst never reaches the median
    CHECK(run(1000, 600, &laps, &age) == rhusb::kMaxSmoothMs);

    // Two samples in one millisecond count as their mean.
    Port port;
    rhusb::Core core(port);
    core.begin(0);
    core.setSmooth(1);
    write(core, 0x7A, {4}, 0);
    for (uint32_t t = 200; t < 300; ++t) {
        core.sample(4, 400, t);
        core.sample(4, 480, t);
    }
    CHECK(read(core, port, 0x05, 300)[3] == 110);  // (100 + 120) / 2
}

void rejectTests() {
    Port port;
    rhusb::Core core(port);
    core.begin(0);
    write(core, 0x7A, {1}, 0);
    write(core, 0x71, {100}, 0);
    write(core, 0x72, {80}, 0);
    // One sample a ms; the floor is level 30 (raw 120), a VTX nearby 150.
    uint32_t t = 200;
    auto run = [&](uint32_t ms, uint16_t raw) { for (uint32_t end = t + ms; t < end; ++t) core.sample(1, raw, t); };
    auto laps = [&]() { return read(core, port, 0x05, t)[0]; };
    auto flagged = [&]() { return (read(core, port, 0x05, t)[8] & 1) != 0; };
    run(300, 120);
    uint8_t before = laps();
    // A VTX powering on: floor, then straight past EnterAt for 60 ms. RH never
    // sees a crossing and no lap counts.
    run(30, 600);
    CHECK(!flagged());
    CHECK(read(core, port, 0x05, t)[3] == 30);  // nor on its graph
    run(30, 600);
    run(30, 120);
    auto after = read(core, port, 0x05, t);
    CHECK(after[3] == 30 && after[4] == 30 && after[11] == 0);  // RSSI, node peak, no history point
    run(270, 120);
    CHECK(laps() == before);
    CHECK(read(core, port, 0x05, t)[5] == 1);  // pass peak untouched
    // Nor is a jump that stays under EnterAt (level 95 here).
    run(40, 380);
    CHECK(read(core, port, 0x05, t)[3] == 30);
    run(10, 120);
    CHECK(read(core, port, 0x05, t)[3] == 30);
    run(300, 120);
    // The same jump that stays: shown from 90 ms, then a lap timed at its peak.
    run(80, 600);
    CHECK(!flagged());
    run(20, 600);
    CHECK(flagged());
    CHECK(read(core, port, 0x05, t)[3] == 150);  // shown once it lasts
    run(100, 520);
    run(300, 120);
    CHECK(laps() == (uint8_t)(before + 1));
    CHECK(read(core, port, 0x05, t)[5] == 150);
    // A fast pass rises over ~50 ms from the floor to EnterAt: counted
    // however short it is.
    for (int i = 0; i < 120; ++i) core.sample(1, (uint16_t)(120 + i * 4), t++);
    run(10, 600);
    run(300, 120);
    CHECK(laps() == (uint8_t)(before + 2));
    // A jump without 100 ms of floor before it (a pass after a dip) counts.
    run(60, 360);
    run(50, 120);
    run(10, 600);
    run(300, 120);
    CHECK(laps() == (uint8_t)(before + 3));
    // A 20 ms pass shows as crossing for 250 ms after it, for RH's ~100 ms
    // reads; RH's force-end clears that at once.
    run(300, 300);
    run(20, 600);
    run(100, 300);
    CHECK(flagged());
    run(160, 300);
    CHECK(!flagged());
    run(20, 600);
    run(10, 300);
    write(core, 0x78, {1}, t);
    CHECK(!flagged());
    CHECK(laps() == (uint8_t)(before + 5));
    // Rise 0 turns the rejection off.
    rhusb::Reject off;
    off.riseMs = 0;
    core.setReject(off);
    run(10, 600);
    run(300, 120);
    CHECK(laps() == (uint8_t)(before + 6));
}

// EnterAt set just above the floor: a small jump past it is still a jump.
void lowEnterTests() {
    Port port;
    rhusb::Core core(port);
    core.begin(0);
    write(core, 0x7A, {0}, 0);
    write(core, 0x71, {45}, 0);
    write(core, 0x72, {40}, 0);
    uint32_t t = 200;
    auto run = [&](uint32_t ms, uint16_t raw) { for (uint32_t end = t + ms; t < end; ++t) core.sample(0, raw, t); };
    run(300, 120);  // floor 30
    uint8_t before = read(core, port, 0x05, t)[0];
    run(40, 220);   // 55: 25 over the floor, 10 over EnterAt
    CHECK(read(core, port, 0x05, t)[3] == 30);
    run(300, 120);
    CHECK(read(core, port, 0x05, t)[0] == before);
    // The same level reached over 100 ms is a pass.
    for (int i = 0; i < 100; ++i) core.sample(0, (uint16_t)(120 + i), t++);
    run(300, 220);
    run(300, 120);
    CHECK(read(core, port, 0x05, t)[0] == (uint8_t)(before + 1));
}

// Line-based simulator for Python tests driving RH's actual serial-node client.
int bridge() {
    Port port;
    rhusb::Core core(port);
    core.setIdentity("usb-dev", "C5MK");
    core.begin(0);
    std::string line;
    while (std::getline(std::cin, line)) {
        std::istringstream input(line);
        std::string command;
        uint32_t now;
        input >> command >> now;
        port.bytes.clear();
        if (command == "feed") {
            unsigned value;
            while (input >> std::hex >> value) core.feed((uint8_t)value, now);
        } else if (command == "sample") {
            unsigned seat, value;
            input >> seat >> value;
            core.sample(seat, (uint16_t)value, now);
        }
        for (uint8_t value : port.bytes) printf("%02x", value);
        printf("\n");
        fflush(stdout);
    }
    return 0;
}

int main(int argc, char** argv) {
    if (argc > 1 && !strcmp(argv[1], "--bridge")) return bridge();
    tests();
    historyTests();
    shownAndSettleTests();
    smoothTests();
    rejectTests();
    lowEnterTests();
    printf("RH USB: %d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
