#include "landed_logic.h"
#include "landed_packet.h"

#include <cassert>
#include <cmath>
#include <cstdio>

using namespace landed;

namespace {

void test_pad_idle_rate_gate()
{
    // Pad Idle: locally generated packets go once per 3 s, whatever their rate.
    assert(radio_period_ms(kStatePadIdle, 1000, false) == 3000);
    // CAN-relayed packets keep their normal rate (kPadIdleThrottlesCanRelayed false).
    static_assert(!kPadIdleThrottlesCanRelayed, "test assumes the default");
    assert(radio_period_ms(kStatePadIdle, 1000, true) == 1000);
    // Every other state: normal period.
    for (std::uint8_t s = 1; s <= 4; ++s) assert(radio_period_ms(s, 1000, false) == 1000);
    // A slower normal period is never shortened.
    assert(radio_period_ms(kStatePadIdle, 5000, false) == 5000);

    // Immediate return to the full rate: a packet sent 1.2 s ago is not due in
    // Pad Idle but is due the moment the state is Boost (no waiting out 3 s).
    std::uint32_t last = 10'000, now = 11'200;
    assert(!period_elapsed(now, last, radio_period_ms(kStatePadIdle, 1000, false)));
    assert(period_elapsed(now, last, radio_period_ms(1, 1000, false)));
    // 3 s pacing in Pad Idle, tick wrap safe.
    last = 0xFFFFFF00u;
    assert(!period_elapsed(last + 2999, last, 3000));
    assert(period_elapsed(last + 3000, last, 3000));
}

void test_utc_clock()
{
    UtcClock c;
    assert(!c.valid());
    // Fix at local 100 s, UTC 12:00:20.000.
    const std::uint32_t noon = 12u * 3600u * 1000u;
    c.observe(100'000, noon + 20'000);
    assert(c.valid());
    // Next minute boundary: UTC 12:01:00 = 40 s later.
    assert(c.minute_boundary_at_or_after(100'000) == 140'000);
    assert(c.minute_boundary_at_or_after(140'000) == 140'000);   // at or after
    assert(c.minute_boundary_at_or_after(140'001) == 200'000);
    // Day wrap: 23:59:50 -> boundary at 00:00:00 is 10 s away.
    UtcClock w;
    w.observe(0, 86'400'000u - 10'000u);
    assert(w.minute_boundary_at_or_after(0) == 10'000);
    assert(std::fabs(w.utc_ms_at(15'000) - 5'000.0) < 1e-6);

    // Rate learning: the local clock runs 1% slow (600 s of UTC pass in
    // 594 s of local time). The second anchor corrects the model.
    UtcClock r;
    r.observe(0, noon);
    r.observe(594'000, noon + 600'000);
    assert(std::fabs(r.rate() - 600.0 / 594.0) < 1e-6);
    // 60 s of UTC later is 59.4 s of local time.
    const std::uint64_t b = r.minute_boundary_at_or_after(594'000 + 1);
    // utc at anchor is 12:10:00 -> next boundary 12:11:00, 60 s of UTC away.
    assert(b >= 594'000 + 59'399 && b <= 594'000 + 59'402);
    // A wild jump is not learned as a rate.
    UtcClock j;
    j.observe(0, noon);
    j.observe(400'000, noon + 800'000);
    assert(j.rate() == 1.0);
    // Anchors closer than the minimum span do not change the rate.
    UtcClock k;
    k.observe(0, noon);
    k.observe(10'000, noon + 10'100);
    assert(k.rate() == 1.0);
}

void test_tx_schedule_utc()
{
    UtcClock c;
    TxSchedule s;
    s.begin(1'000);
    // First packet: immediately.
    assert(s.due(c, 1'000));
    s.on_sent(1'000);
    // UTC known: fix at local 5 s says 12:00:40.
    const std::uint32_t t0 = 12u * 3600u * 1000u;
    c.observe(5'000, t0 + 40'000);
    // Next boundary at least 30 s after the last packet (31 s): UTC 12:01:06
    // is past 12:01:00, so the next one is 12:02:00 = local 85 s.
    assert(s.next_tx_ms(c, 2'000) == 85'000);
    assert(!s.due(c, 84'999));
    assert(s.due(c, 85'000));
    s.on_sent(85'100);
    // Never twice for one boundary: the next one is the grid point a minute on.
    assert(s.next_tx_ms(c, 85'200) == 145'000);
    // A late wake-up (3 s late) still sends once, then resumes the grid.
    s.on_sent(148'000);
    assert(s.next_tx_ms(c, 148'000) == 205'000);
}

void test_tx_schedule_free_running()
{
    UtcClock none;
    TxSchedule s;
    s.begin(0);
    s.on_fix(10'000);          // last fix at 10 s
    s.on_sent(12'000);         // first packet, a bit after
    // Free-running: 10 s + n * 60 s, at least 30 s after the last packet.
    assert(s.next_tx_ms(none, 12'000) == 70'000);
    s.on_sent(70'000);
    assert(s.next_tx_ms(none, 70'000) == 130'000);
    // A new fix re-anchors the timer.
    s.on_fix(100'000);
    assert(s.next_tx_ms(none, 100'000) == 160'000);
    // Never closer than the minimum gap, even if the anchor says so.
    s.on_sent(159'000);
    assert(s.next_tx_ms(none, 159'000) >= 159'000 + kLandedTxMinGapMs);
    assert(s.next_tx_ms(none, 159'000) == 220'000);
}

void test_gps_duty()
{
    GpsDuty d;
    bool ok = true;
    d.begin(1'000);
    assert(d.phase() == GpsDuty::Phase::Active);
    assert(d.next_event_ms() == 1'000 + kGpsFixTimeoutMs);
    assert(d.poll(2'000, false, ok) == GpsDuty::Action::None);
    // Fix: sleep, result ok, next wake 10 minutes later.
    assert(d.poll(5'000, true, ok) == GpsDuty::Action::Sleep && ok);
    assert(d.phase() == GpsDuty::Phase::Asleep);
    assert(d.next_event_ms() == 5'000 + kGpsWakeIntervalMs);
    assert(d.poll(5'000 + kGpsWakeIntervalMs - 1, false, ok) == GpsDuty::Action::None);
    assert(d.poll(5'000 + kGpsWakeIntervalMs, false, ok) == GpsDuty::Action::Wake);
    const std::uint64_t wake = 5'000 + kGpsWakeIntervalMs;
    assert(d.next_event_ms() == wake + kGpsFixTimeoutMs);
    // No fix before the timeout: sleep again, not ok (stale), 10 min from now.
    assert(d.poll(wake + kGpsFixTimeoutMs - 1, false, ok) == GpsDuty::Action::None);
    assert(d.poll(wake + kGpsFixTimeoutMs, false, ok) == GpsDuty::Action::Sleep && !ok);
    assert(d.next_event_ms() == wake + kGpsFixTimeoutMs + kGpsWakeIntervalMs);
    // Failure never stops the cycle: it wakes again.
    assert(d.poll(wake + kGpsFixTimeoutMs + kGpsWakeIntervalMs, false, ok) == GpsDuty::Action::Wake);
}

void test_ping_limiter()
{
    PingLimiter p;
    assert(p.allow(1'000));
    assert(!p.allow(1'001));
    assert(!p.allow(1'000 + kPingMinIntervalMs - 1));
    assert(p.allow(1'000 + kPingMinIntervalMs));
    assert(!p.allow(1'000 + kPingMinIntervalMs + 1));
}

void test_sleep_ms()
{
    assert(sleep_ms(0, 100'000, 150, 5) == kMaxSleepChunkMs);
    assert(sleep_ms(0, 1'000, 150, 5) == 850);
    assert(sleep_ms(0, 100, 150, 5) == 5);     // due now: spin once
    assert(sleep_ms(500, 400, 150, 5) == 5);   // overdue
}

void test_packet()
{
    Position pos;
    pos.valid = true;
    pos.latitude_deg = -33.8688197;
    pos.longitude_deg = 151.2092955;
    pos.altitude_m = 123.5F;
    pos.satellites = 9;
    pos.stale = true;
    pos.fix_age_s = 0x01020304;
    const std::uint8_t status[2] = {0x81, 0x1C};   // Landed + GPS bit + ...
    std::uint8_t f[IRIS_PACKET_LANDED_POSITION_FRAME_LENGTH + 2];
    std::memset(f, 0xEE, sizeof(f));
    const std::size_t n = encode_position(f, status, 0xA1B2C3D4u, pos, 0x1D14, true, true);
    assert(n == 28 && n == IRIS_PACKET_LANDED_POSITION_FRAME_LENGTH);
    assert(f[0] == 0x09 && f[27] == 0x0A && f[28] == 0xEE);   // no overrun
    const std::uint8_t *p = f + 1;
    assert(p[0] == 0x81 && p[1] == 0x1C);
    assert(p[2] == 0xA1 && p[5] == 0xD4);
    const auto get32 = [&](int o) {
        return (std::uint32_t(p[o]) << 24) | (std::uint32_t(p[o + 1]) << 16) |
               (std::uint32_t(p[o + 2]) << 8) | p[o + 3];
    };
    assert(static_cast<std::int32_t>(get32(6)) == -338688197);
    assert(static_cast<std::int32_t>(get32(10)) == 1512092955);
    std::uint32_t bits = get32(14);
    float alt;
    std::memcpy(&alt, &bits, 4);
    assert(alt == 123.5F);
    assert(get32(18) == 0x01020304);
    assert(p[22] == (IRIS_LANDED_FLAG_POSITION_VALID | IRIS_LANDED_FLAG_STALE |
                     IRIS_LANDED_FLAG_PING_REPLY | IRIS_LANDED_FLAG_UTC_ALIGNED));
    assert(p[23] == 9);
    assert(p[24] == 0x1D && p[25] == 0x14);

    // No fix yet: zero position, age "none", flag clear.
    Position none;
    encode_position(f, status, 5, none, 0, false, false);
    assert(get32(6) == 0 && get32(10) == 0 && get32(18) == 0xFFFFFFFFu && p[22] == 0);
    // Extreme coordinates saturate instead of wrapping.
    assert(coordinate_counts(1000.0) == 2147483647);
    assert(coordinate_counts(-1000.0) == -2147483647 - 1);
}

}  // namespace

int main()
{
    test_pad_idle_rate_gate();
    test_utc_clock();
    test_tx_schedule_utc();
    test_tx_schedule_free_running();
    test_gps_duty();
    test_ping_limiter();
    test_sleep_ms();
    test_packet();
    std::puts("landed_logic_test passed");
    return 0;
}
