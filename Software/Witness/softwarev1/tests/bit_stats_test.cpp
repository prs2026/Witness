#include <cmath>
#include <cstdio>

#include "bit_stats.h"

using namespace bit;
static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL line %d: %s\n", __LINE__, #c); ++failures; } } while (0)
static bool near(const float a, const float b, const float tol) { return std::fabs(a - b) <= tol; }

int main()
{
    // RateMeter: 960 Hz counter over 2 s windows.
    {
        RateMeter m;
        CHECK(!m.update(0, 0));
        CHECK(!m.update(960, 1000));            // window not complete
        CHECK(m.update(1920, 2000));
        CHECK(near(m.rate_hz(), 960.0F, 0.5F));
        // counter wrap
        RateMeter w;
        w.update(0xFFFFFF00U, 0);
        w.update(0xFFFFFF00U + 1920U, 2000);
        CHECK(near(w.rate_hz(), 960.0F, 0.5F));
    }
    // GrowthFlag holds for 10 s after the last growth.
    {
        GrowthFlag g;
        CHECK(!g.update(5, 0));
        CHECK(!g.update(5, 1000));
        CHECK(g.update(6, 2000));
        CHECK(g.update(6, 11'999));
        CHECK(!g.update(6, 12'000));
        CHECK(g.update(7, 13'000));
    }
    // StillWindow: gravity on +X, quiet.
    {
        StillWindow w;
        float low[3], gyro[3], high[3];
        for (int i = 0; i < 120; ++i) {
            const float n = (i % 2 ? 1.0F : -1.0F);
            low[0] = 1000.0F + n; low[1] = n; low[2] = -n;
            gyro[0] = 200.0F + n * 10.0F; gyro[1] = 0.0F; gyro[2] = 0.0F;   // mdps
            high[0] = 1020.0F; high[1] = 0.0F; high[2] = 0.0F;
            w.add(low, gyro, high, true, static_cast<std::uint32_t>(i * 10));
        }
        const StillStats s = w.latest(1200);
        CHECK(s.valid);
        CHECK(near(s.accel_mag_g, 1.0F, 0.001F));
        CHECK(near(s.axial_g, 1.0F, 0.001F));
        CHECK(near(s.accel_sd_mg, 1.0F, 0.1F));
        CHECK(near(s.gyro_mean_dps, 0.2F, 0.01F));
        CHECK(s.high_g_valid && near(s.high_g_mag_g, 1.02F, 0.001F));
        CHECK(!w.latest(1200 + 4000).valid);        // stale
    }
    // StillWindow: nose down reads -1 g on +X.
    {
        StillWindow w;
        float low[3] = {-1000.0F, 0, 0}, gyro[3] = {0, 0, 0}, high[3] = {-1000.0F, 0, 0};
        for (int i = 0; i < 120; ++i) w.add(low, gyro, high, true, static_cast<std::uint32_t>(i * 10));
        CHECK(near(w.latest(1200).axial_g, -1.0F, 0.001F));
        CHECK(near(w.latest(1200).accel_mag_g, 1.0F, 0.001F));
    }
    // StillWindow: too few samples -> no window.
    {
        StillWindow w;
        float v[3] = {1000.0F, 0, 0};
        for (int i = 0; i < 5; ++i) w.add(v, v, v, true, static_cast<std::uint32_t>(i * 300));
        CHECK(!w.latest(2000).valid);
    }
    // BaroWindow: noise, rate and the value-change rate.
    {
        BaroWindow b;
        std::int32_t p = 100000;
        for (int i = 0; i < 300; ++i) {
            if (i % 2 == 0) p = 100000 + ((i / 2) % 7) - 3;   // changes every 20 ms = 50 Hz
            b.add(p, 2000, 100.0F + 0.5F * static_cast<float>(i) * 0.01F, static_cast<std::uint32_t>(i * 10));
        }
        const BaroStats s = b.latest(3000);
        CHECK(s.valid);
        CHECK(s.sd_pa > 1.0F && s.sd_pa < 5.0F);
        CHECK(near(s.rate_mps, 0.5F, 0.02F));
        CHECK(near(s.sample_rate_hz, 50.0F, 3.0F));
        CHECK(!s.frozen);
    }
    // BaroWindow: a frozen output is flagged after 100 identical polls.
    {
        BaroWindow b;
        for (int i = 0; i < 300; ++i) b.add(101325, 2000, 0.0F, static_cast<std::uint32_t>(i * 10));
        CHECK(b.latest(3000).frozen);
        BaroWindow c;
        for (int i = 0; i < 300; ++i) c.add(101325 + (i % 5), 2000, 0.0F, static_cast<std::uint32_t>(i * 10));
        CHECK(!c.latest(3000).frozen);
    }
    // ResetHistory ring.
    {
        ResetHistory h{};
        CHECK(!h.valid());
        h.clear();
        CHECK(h.valid() && h.count == 0 && h.fault_count() == 0);
        for (int i = 0; i < 10; ++i) h.push(i % 2 ? ResetClass::Panic : ResetClass::Software, 100U + i);
        CHECK(h.count == ResetHistory::kDepth);
        CHECK(h.fault_count() == 4);
        h.push(ResetClass::PowerOn, 0);
        CHECK(h.valid());
        ResetHistory junk{};
        junk.magic = ResetHistory::kMagic; junk.count = 200; junk.head = 3;   // RTC contents corrupted
        CHECK(!junk.valid() && junk.fault_count() == 0);
        junk.push(ResetClass::Panic, 1);                                      // recovers by clearing
        CHECK(junk.valid() && junk.count == 1);
    }

    if (failures == 0) std::printf("bit_stats_test: all passed\n");
    return failures == 0 ? 0 : 1;
}
