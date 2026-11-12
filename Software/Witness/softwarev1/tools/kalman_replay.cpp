// Offline replay of Witness's position filter on a recorded HITL run.
//
// Runs the firmware's own accel_filter (low/high-g selection) and
// position::Filter on an event file made by HITL/kalman_replay.py, so filter
// changes and tuning can be tried in seconds instead of a 10-minute HITL run.
// The attitude comes from the event file (truth or Witness's logged attitude);
// the attitude estimator itself is not replayed.
//
// Build (host, from softwarev1/):
//   g++ -std=c++17 -O2 -I main tools/kalman_replay.cpp main/position_filter.cpp \
//       main/accel_filter.cpp -o kalman_replay
// Run:
//   kalman_replay events.txt out.csv [key=value ...]
// Keys: any position::Config field listed in kConfigKeys, plus
//   highg_offset=0|1   estimate the high-g zero offset on the pad (default 1)
//
// Event lines (time in sim seconds, sorted):
//   I t low_x low_y low_z high_x high_y high_z qx qy qz qw   (mg, body->ENU quaternion)
//   B t pressure_pa
//   G t lat lon alt_msl sats hdop vdop                        (hdop/vdop: nan if unknown)
//   S t phase                                                  (0 pad .. 4 landed)
// Output: CSV at 100 Hz: t,phase,e,n,u,ve,vn,vu,alt_msl,baro_used,gps_used,gps_rejects

#include "accel_filter.h"
#include "position_filter.h"
#include "pressure_altitude.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace {

struct Key {
    const char *name;
    float position::Config::*f;
    std::uint32_t position::Config::*u;
};

const Key kConfigKeys[] = {
    {"accel_noise_mps2", &position::Config::accel_noise_mps2, nullptr},
    {"accel_noise_fraction", &position::Config::accel_noise_fraction, nullptr},
    {"baro_pressure_sigma_pa", &position::Config::baro_pressure_sigma_pa, nullptr},
    {"baro_min_sigma_m", &position::Config::baro_min_sigma_m, nullptr},
    {"gps_horizontal_uere_m", &position::Config::gps_horizontal_uere_m, nullptr},
    {"gps_vertical_uere_m", &position::Config::gps_vertical_uere_m, nullptr},
    {"gps_min_sigma_m", &position::Config::gps_min_sigma_m, nullptr},
    {"gps_max_sigma_m", &position::Config::gps_max_sigma_m, nullptr},
    {"gps_lag_speed_s", &position::Config::gps_lag_speed_s, nullptr},
    {"gps_lag_accel_s2", &position::Config::gps_lag_accel_s2, nullptr},
    {"gps_stress_tau_s", &position::Config::gps_stress_tau_s, nullptr},
    {"gps_latency_s", &position::Config::gps_latency_s, nullptr},
    {"baro_scale_sigma", &position::Config::baro_scale_sigma, nullptr},
    {"baro_scale_rate", &position::Config::baro_scale_rate, nullptr},
    {"gate_sigma", &position::Config::gate_sigma, nullptr},
    {"max_consecutive_rejects", nullptr, &position::Config::max_consecutive_rejects},
    {"no_imu_accel_noise_mps2", &position::Config::no_imu_accel_noise_mps2, nullptr},
};

bool set_key(position::Config &c, const char *key, const char *value)
{
    for (const Key &k : kConfigKeys) {
        if (std::strcmp(k.name, key) != 0) continue;
        if (k.f) c.*(k.f) = static_cast<float>(std::atof(value));
        else c.*(k.u) = static_cast<std::uint32_t>(std::atol(value));
        return true;
    }
    return false;
}

void rotate(const float q[4], const float v[3], float out[3])
{
    // q = (x, y, z, w), body -> ENU: v' = v + 2w (u x v) + 2 u x (u x v)
    const float x = q[0], y = q[1], z = q[2], w = q[3];
    const float tx = 2.0F * (y * v[2] - z * v[1]);
    const float ty = 2.0F * (z * v[0] - x * v[2]);
    const float tz = 2.0F * (x * v[1] - y * v[0]);
    out[0] = v[0] + w * tx + (y * tz - z * ty);
    out[1] = v[1] + w * ty + (z * tx - x * tz);
    out[2] = v[2] + w * tz + (x * ty - y * tx);
}

}  // namespace

int main(int argc, char **argv)
{
    if (argc < 3) {
        std::fprintf(stderr, "usage: kalman_replay events.txt out.csv [key=value ...]\n");
        return 2;
    }
    position::Config config;
    bool highg_offset = true;
    for (int i = 3; i < argc; ++i) {
        std::string arg = argv[i];
        const auto eq = arg.find('=');
        if (eq == std::string::npos) {
            std::fprintf(stderr, "bad argument %s\n", argv[i]);
            return 2;
        }
        const std::string key = arg.substr(0, eq), value = arg.substr(eq + 1);
        if (key == "highg_offset") highg_offset = std::atoi(value.c_str()) != 0;
        else if (!set_key(config, key.c_str(), value.c_str())) {
            std::fprintf(stderr, "unknown key %s\n", key.c_str());
            return 2;
        }
    }
    std::FILE *in = std::fopen(argv[1], "r");
    std::FILE *out = std::fopen(argv[2], "w");
    if (!in || !out) {
        std::fprintf(stderr, "cannot open files\n");
        return 2;
    }
    std::fprintf(out, "t,phase,e,n,u,ve,vn,vu,alt_msl,baro_used,gps_used,gps_rejects,sigma_u,sigma_vu\n");

    position::Filter filter(config);
    accel_filter::FilteredAccel accel;
    accel.configure(100.0F, 960.0F);   // as sensors.cpp: kFilteredAccelCutoffHz at the IMU rate
    accel_filter::HighGCalibration offset;
    position::Phase phase = position::Phase::PadIdle;
    double last_imu = -1.0, next_update = 0.0, next_out = 0.0;
    float pressure = NAN;
    bool baro_pending = false;
    char line[512];
    while (std::fgets(line, sizeof line, in)) {
        const char kind = line[0];
        double t = 0.0;
        if (kind == 'S') {
            int ph = 0;
            std::sscanf(line + 2, "%lf %d", &t, &ph);
            phase = static_cast<position::Phase>(ph);
        } else if (kind == 'B') {
            float p = 0.0F;
            std::sscanf(line + 2, "%lf %f", &t, &p);
            pressure = p;
            baro_pending = true;
        } else if (kind == 'G') {
            double lat = 0.0, lon = 0.0;
            float alt = 0.0F, hdop = NAN, vdop = NAN;
            int sats = 0;
            char hs[32] = {0}, vs[32] = {0};
            std::sscanf(line + 2, "%lf %lf %lf %f %d %31s %31s", &t, &lat, &lon, &alt, &sats, hs, vs);
            hdop = std::strcmp(hs, "nan") == 0 ? NAN : static_cast<float>(std::atof(hs));
            vdop = std::strcmp(vs, "nan") == 0 ? NAN : static_cast<float>(std::atof(vs));
            if (filter.initialized())
                filter.update_gps(lat, lon, alt, static_cast<std::uint8_t>(sats), phase, hdop, vdop);
        } else if (kind == 'I') {
            float low[3], high[3], q[4];
            std::sscanf(line + 2, "%lf %f %f %f %f %f %f %f %f %f %f", &t, &low[0], &low[1], &low[2],
                        &high[0], &high[1], &high[2], &q[0], &q[1], &q[2], &q[3]);
            if (highg_offset) {
                offset.observe(low, high, phase == position::Phase::PadIdle);
                offset.apply(high);
            }
            float mg[3];
            accel.step(low, high, mg);
            constexpr float kMgToMps2 = position::kGravity / 1000.0F;
            const float body[3] = {mg[0] * kMgToMps2, mg[1] * kMgToMps2, mg[2] * kMgToMps2};
            float world[3];
            rotate(q, body, world);
            if (last_imu >= 0.0) filter.predict(world, static_cast<float>(t - last_imu), phase);
            last_imu = t;
            if (t >= next_update) {
                next_update = t + 1.0 / 25.0;
                if (baro_pending && filter.initialized()) {
                    baro_pending = false;
                    float alt_m = 0.0F;
                    const auto centi = static_cast<std::int32_t>(std::lround(pressure));
                    if (pressure_altitude_meters(centi, alt_m))
                        filter.update_baro(alt_m, pressure, phase);
                }
                filter.update_constraints(phase);
                filter.begin_update_cycle();
            }
            if (t >= next_out) {
                next_out = t + 0.01;
                float p[3], v[3];
                filter.position(p);
                filter.velocity(v);
                std::fprintf(out, "%.4f,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%lu,%lu,%lu,%.3f,%.3f\n", t,
                             static_cast<int>(phase), p[0], p[1], p[2], v[0], v[1], v[2],
                             filter.altitude_msl_m(), static_cast<unsigned long>(filter.baro_updates()),
                             static_cast<unsigned long>(filter.gps_updates()),
                             static_cast<unsigned long>(filter.gps_rejects()),
                             std::sqrt(filter.up().P00), std::sqrt(filter.up().P11));
            }
        }
    }
    std::fclose(in);
    std::fclose(out);
    std::fprintf(stderr, "highg_cal scale %.4f %.4f %.4f offset_mg %.1f %.1f %.1f baro_scale %.4f +- %.4f\n",
                 offset.scale(0), offset.scale(1), offset.scale(2), offset.offset_mg(0),
                 offset.offset_mg(1), offset.offset_mg(2), filter.baro_scale(),
                 filter.baro_scale_sigma());
    return 0;
}
