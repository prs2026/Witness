// Host test for main/gps_trust.h.
// g++ -std=c++20 -Imain tests/gps_trust_test.cpp -o build_host/gps_trust_test.exe
#include "gps_trust.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {

int failures = 0;

void check(bool ok, const char *what)
{
    if (!ok) {
        std::printf("FAIL: %s\n", what);
        ++failures;
    }
}

constexpr double kDt = 0.04;   // 25 Hz fixes

// Feed fixes for seconds: GPS on a trajectory gps(t), filter velocity vf(t).
template <typename Gps, typename Vel>
float run(position::GpsTrust &trust, double &t, double seconds, Gps gps, Vel vf,
          std::uint8_t sats = 12, float speed = 100.0F)
{
    const position::GpsTrustConfig c{};
    float w = 0.0F;
    const double end = t + seconds;
    for (; t < end; t += kDt) w = trust.on_fix(t, gps(t), vf(t), 5.0F, speed, sats, c);
    return w;
}

}  // namespace

int main()
{
    // Consistent receiver: ramps from restart to 1 in about fast_ramp_s.
    {
        position::GpsTrust trust;
        double t = 0.0;
        const float w1 = run(trust, t, 1.5, [](double s) { return float(100.0 * s); },
                             [](double) { return 100.0F; });
        const float w2 = run(trust, t, 3.0, [](double s) { return float(100.0 * s); },
                             [](double) { return 100.0F; });
        check(w1 < 0.7F, "ramp is gradual");
        check(w2 == 1.0F, "consistent receiver reaches full trust");
    }
    // Lagging receiver after reacquisition: 550 m/s implied against 460 in the
    // filter. With few satellites it stays capped; with many, only the slow
    // path applies, so large corrections (weight >= 0.5) wait about 9 s.
    {
        position::GpsTrust trust;
        double t = 0.0;
        const float w = run(trust, t, 20.0,
                            [](double s) { return float(550.0 * s - 5.0 * s * s); },
                            [](double s) { return float(460.0 - 10.0 * s); }, 5, 460.0F);
        check(w <= 0.1F, "lagging low-satellite receiver stays capped");
        position::GpsTrust many;
        t = 0.0;
        const float w8 = run(many, t, 8.0,
                             [](double s) { return float(550.0 * s - 5.0 * s * s); },
                             [](double s) { return float(460.0 - 10.0 * s); }, 12, 460.0F);
        check(w8 < 0.5F, "lagging receiver cannot force large corrections for 8 s");
    }
    // Filter off in velocity (+60 m/s) but receiver tracks the motion: the
    // acceleration check lets trust ramp slowly, so GPS can pull it back.
    {
        position::GpsTrust trust;
        double t = 0.0;
        const float w3 = run(trust, t, 3.0, [](double s) { return float(200.0 * s - 5.0 * s * s); },
                             [](double s) { return float(260.0 - 10.0 * s); });
        const float w12 = run(trust, t, 17.0, [](double s) { return float(200.0 * s - 5.0 * s * s); },
                              [](double s) { return float(260.0 - 10.0 * s); });
        check(w3 < 0.5F, "slow ramp when only acceleration agrees");
        check(w12 > 0.7F, "slow ramp reaches high trust within 20 s");
    }
    // Few satellites cap the weight.
    {
        position::GpsTrust trust;
        double t = 0.0;
        const float w = run(trust, t, 6.0, [](double s) { return float(100.0 * s); },
                            [](double) { return 100.0F; }, 4);
        check(std::fabs(w - 0.1F) < 1e-6F, "low satellites cap at 0.1");
    }
    // A fix gap restarts trust.
    {
        position::GpsTrust trust;
        double t = 0.0;
        run(trust, t, 5.0, [](double s) { return float(100.0 * s); }, [](double) { return 100.0F; });
        t += 3.0;   // 3 s without fixes
        const float w = run(trust, t, 0.2, [](double s) { return float(100.0 * s); },
                            [](double) { return 100.0F; });
        check(w <= 0.06F, "gap restarts trust");
        check(trust.restarts() == 1U, "restart counted");
    }
    if (failures == 0) std::printf("gps_trust_test: all passed\n");
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
