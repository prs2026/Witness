// Host test for imu_calibration::sanitize (stored-calibration validation).
#include <cmath>
#include <cstdio>
#include <limits>

#include "imu_calibration_store.h"

using imu_calibration::Calibration;
using imu_calibration::sanitize;
static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL line %d: %s\n", __LINE__, #c); ++failures; } } while (0)

static Calibration good()
{
    Calibration c;
    c.gyro_bias_mdps[0] = 150.0F; c.gyro_bias_mdps[1] = -300.0F; c.gyro_bias_mdps[2] = 40.0F;
    c.accel_offset_mg[0] = 12.0F; c.accel_offset_mg[1] = -20.0F; c.accel_offset_mg[2] = 35.0F;
    c.accel_scale[0] = 1.002F; c.accel_scale[1] = 0.998F; c.accel_scale[2] = 1.01F;
    c.high_g_offset_mg[0] = 300.0F;
    c.high_g_scale[0] = 1.05F;
    c.gyro_valid = true;
    c.accel_valid = true;
    return c;
}

int main()
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    {
        Calibration c = good();
        CHECK(sanitize(c) == 0);
        CHECK(c.gyro_valid && c.accel_valid);
        CHECK(c.accel_scale[2] == 1.01F);
    }
    {   // scale 0 -> accelerometer section replaced, gyro kept
        Calibration c = good();
        c.accel_scale[1] = 0.0F;
        CHECK(sanitize(c) == 1);
        CHECK(!c.accel_valid && c.gyro_valid);
        CHECK(c.accel_scale[0] == 1.0F && c.accel_scale[1] == 1.0F && c.accel_offset_mg[2] == 0.0F);
        CHECK(c.gyro_bias_mdps[1] == -300.0F);
    }
    {   // NaN / inf in each field class
        Calibration c = good(); c.accel_scale[0] = nan; CHECK(sanitize(c) == 1 && !c.accel_valid);
        c = good(); c.high_g_scale[2] = inf; CHECK(sanitize(c) == 1 && !c.accel_valid);
        c = good(); c.accel_offset_mg[0] = nan; CHECK(sanitize(c) == 1);
        c = good(); c.high_g_offset_mg[1] = -inf; CHECK(sanitize(c) == 1);
        c = good(); c.gyro_bias_mdps[2] = nan; CHECK(sanitize(c) == 1 && !c.gyro_valid && c.accel_valid);
        CHECK(c.gyro_bias_mdps[0] == 0.0F);
    }
    {   // range edges: 0.8..1.25 inclusive ok, outside rejected
        Calibration c = good(); c.accel_scale[0] = 0.8F; c.high_g_scale[0] = 1.25F;
        CHECK(sanitize(c) == 0);
        c = good(); c.accel_scale[0] = 0.79F; CHECK(sanitize(c) == 1);
        c = good(); c.high_g_scale[0] = 1.26F; CHECK(sanitize(c) == 1);
        c = good(); c.accel_offset_mg[0] = 501.0F; CHECK(sanitize(c) == 1);
        c = good(); c.gyro_bias_mdps[0] = 30001.0F; CHECK(sanitize(c) == 1);
        c = good(); c.gyro_bias_mdps[0] = 20000.0F; CHECK(sanitize(c) == 0);  // solver max accepted
    }
    {   // both sections bad
        Calibration c = good(); c.gyro_bias_mdps[0] = nan; c.accel_scale[0] = 0.0F;
        CHECK(sanitize(c) == 2 && !c.gyro_valid && !c.accel_valid);
    }
    {   // default (uncalibrated) record passes
        Calibration c;
        CHECK(sanitize(c) == 0);
    }
    std::printf(failures ? "imu_calibration_store: %d failure(s)\n" : "imu_calibration_store: all passed\n", failures);
    return failures != 0;
}
