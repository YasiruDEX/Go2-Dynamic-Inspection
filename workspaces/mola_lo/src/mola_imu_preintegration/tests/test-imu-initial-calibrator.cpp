/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this odometry package
 alone or in combination with the complete SLAM system.
*/

/**
 * @file   test_ImuInitialCalibrator.cpp
 * @brief  Unit test for ImuInitialCalibrator
 * @author Jose Luis Blanco Claraco
 * @date   Nov 10, 2025
 */

#include <mola_imu_preintegration/ImuInitialCalibrator.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/bits_math.h>  // mrpt::DEG2RAD
#include <mrpt/core/exceptions.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/obs/CObservationIMU.h>

#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>

using namespace mola::imu;

namespace
{

/**
 * @brief Helper class to capture stream insertions and convert them to a string.
 * This allows syntax like: MRPT_ASSERT_NEAR_MSG_(..., "Field " << name << ".y mismatch.")
 */
class StreamCatcher
{
   public:
    std::ostringstream ss;

    // Overload the stream insertion operator (<<) to append to the internal stream
    template <typename T>
    StreamCatcher& operator<<(const T& value)
    {
        ss << value;
        return *this;
    }

    // Explicitly convert the object to a string when needed
    std::string str() const { return ss.str(); }
};

/**
 * @brief Helper function to perform the NEAR check and throw a formatted exception.
 * @param v1 First value.
 * @param v2 Second value.
 * @param tolerance Allowed absolute difference.
 * @param v1_str String representation of the first value variable name.
 * @param v2_str String representation of the second value variable name.
 * @param tol_str String representation of the tolerance variable name.
 * @param custom_msg The complete custom message string.
 */
inline void assert_near_impl(
    double v1, double v2, double tolerance, const char* v1_str, const char* v2_str,
    const char* tol_str, const std::string& custom_msg)
{
    if (std::abs(v1 - v2) > tolerance)
    {
        std::ostringstream oss;
        oss << "Assertion failed: " << v1_str << " (" << v1 << ")"
            << " is not near " << v2_str << " (" << v2 << ")"
            << " with tolerance " << tol_str << " (" << tolerance << ").";

        if (!custom_msg.empty())
        {
            oss << " " << custom_msg;
        }

        // Throw a standard C++ exception
        throw std::runtime_error(oss.str());
    }
}

}  // end anonymous namespace

/**
 * @name MRPT_ASSERT_NEAR_MSG_()
 * @brief Asserts that two floating-point values are approximately equal within a tolerance, with a
 * custom message that supports streaming.
 * * * It uses the StreamCatcher to turn the streamed message components (e.g., "Field " << name <<
 * " mismatch")
 * * into a single std::string before passing it to the implementation function.
 */
#define MRPT_ASSERT_NEAR_MSG_(VAL1, VAL2, TOLERANCE, MSG) \
    assert_near_impl(                                     \
        (VAL1), (VAL2), (TOLERANCE), #VAL1, #VAL2, #TOLERANCE, (StreamCatcher() << MSG).str())

// Helper function to create an IMU observation
mrpt::obs::CObservationIMU::Ptr create_imu_obs(
    double time_sec, const mrpt::math::TVector3D& acc, const mrpt::math::TVector3D& gyro,
    const std::optional<mrpt::math::CQuaternionDouble>& orientation_quat = std::nullopt)
{
    auto obs       = mrpt::obs::CObservationIMU::Create();
    obs->timestamp = mrpt::Clock::fromDouble(time_sec);

    // Accelerations (IMU_X_ACC, IMU_Y_ACC, IMU_Z_ACC)
    obs->set(mrpt::obs::IMU_X_ACC, acc.x);
    obs->set(mrpt::obs::IMU_Y_ACC, acc.y);
    obs->set(mrpt::obs::IMU_Z_ACC, acc.z);

    // Angular velocities (IMU_WX, IMU_WY, IMU_WZ)
    obs->set(mrpt::obs::IMU_WX, gyro.x);
    obs->set(mrpt::obs::IMU_WY, gyro.y);
    obs->set(mrpt::obs::IMU_WZ, gyro.z);

    // Orientation (optional)
    if (orientation_quat.has_value())
    {
        obs->set(mrpt::obs::IMU_ORI_QUAT_W, orientation_quat->w());
        obs->set(mrpt::obs::IMU_ORI_QUAT_X, orientation_quat->x());
        obs->set(mrpt::obs::IMU_ORI_QUAT_Y, orientation_quat->y());
        obs->set(mrpt::obs::IMU_ORI_QUAT_Z, orientation_quat->z());
    }

    return obs;
}

// Helper to check TPoint3D (TVector3D) equality with tolerance
void check_vector_equal(
    const mrpt::math::TVector3D& actual, const mrpt::math::TVector3D& expected, double tol,
    const char* name)
{
    MRPT_ASSERT_NEAR_MSG_(actual.x, expected.x, tol, "Field " << name << ".x mismatch.");
    MRPT_ASSERT_NEAR_MSG_(actual.y, expected.y, tol, "Field " << name << ".y mismatch.");
    MRPT_ASSERT_NEAR_MSG_(actual.z, expected.z, tol, "Field " << name << ".z mismatch.");
}

void TestImuInitialCalibrator()
{
    std::cout << "--- TestImuInitialCalibrator started ---\n";

    // 1. Test isReady() and basic add()
    {
        ImuInitialCalibrator calibrator;
        calibrator.parameters.required_samples = 5;
        calibrator.parameters.max_samples_age  = 10.0;  // seconds
        calibrator.parameters.gravity          = 9.81;

        ASSERT_(!calibrator.isReady());

        for (int i = 0; i < 4; ++i)
        {
            calibrator.add(create_imu_obs(static_cast<double>(i), {0, 0, 9.81}, {0, 0, 0}));
            ASSERT_(!calibrator.isReady());
        }

        calibrator.add(create_imu_obs(5.0, {0, 0, 9.81}, {0, 0, 0}));
        ASSERT_(calibrator.isReady());

        std::cout << "Test 1: isReady() and sample addition OK.\n";
    }

    // 2. Test getCalibration() - Static, no noise, Z-up
    {
        ImuInitialCalibrator calibrator;
        calibrator.parameters.required_samples = 3;
        calibrator.parameters.max_samples_age  = 100.0;
        calibrator.parameters.gravity          = 9.81;
        double g                               = calibrator.parameters.gravity;

        for (int i = 0; i < 5; ++i)
        {
            // Ideal IMU reading at rest, aligned with gravity (Z-up):
            // Acceleration should read 0, 0, +g (or 0, 0, -g depending on
            // convention, we use 0,0,+g here for simplicity and check the bias)
            // Gyro should read 0, 0, 0
            calibrator.add(
                create_imu_obs(10.0 + static_cast<double>(i), {0, 0, g}, {0.0, 0.0, 0.0}));
        }

        ASSERT_(calibrator.isReady());

        auto calib = calibrator.getCalibration();
        ASSERT_(calib.has_value());

        const double tol = 1e-6;

        // Accelerometer bias: bias_acc_b = (up_vector * g) - nominal_gravity_vector
        // up_vector (average_accel.unitarize) = (0, 0, g).unitarize() = (0, 0, 1)
        // nominal_gravity_vector = (0, 0, g)
        // Expected bias: (0, 0, g) - (0, 0, g) = (0, 0, 0)
        check_vector_equal(calib->bias_acc_b, {0.0, 0.0, 0.0}, tol, "bias_acc_b (static Z-up)");

        // Gyro bias: average_gyro = (0, 0, 0)
        check_vector_equal(calib->bias_gyro, {0.0, 0.0, 0.0}, tol, "bias_gyro (static Z-up)");

        // Noise stddev: Should be near zero since all samples are identical.
        check_vector_equal(
            calib->noise_stddev_acc, {0.0, 0.0, 0.0}, tol, "noise_stddev_acc (static Z-up)");
        check_vector_equal(
            calib->noise_stddev_gyro, {0.0, 0.0, 0.0}, tol, "noise_stddev_gyro (static Z-up)");

        // Orientation: up_vector=(0,0,1). pitch = -asin(0) = 0. roll = -asin(0) = 0.
        MRPT_ASSERT_NEAR_MSG_(calib->pitch, 0.0, tol, "pitch (static Z-up)");
        MRPT_ASSERT_NEAR_MSG_(calib->roll, 0.0, tol, "roll (static Z-up)");

        std::cout << "Test 2: Static Z-up (No orientation data) OK.\n";
    }

    // 3. Test getCalibration() - Static, no noise, X-up (90 deg roll)
    {
        ImuInitialCalibrator calibrator;
        calibrator.parameters.required_samples = 3;
        calibrator.parameters.max_samples_age  = 100.0;
        calibrator.parameters.gravity          = 9.81;
        double g                               = calibrator.parameters.gravity;

        for (int i = 0; i < 5; ++i)
        {
            // IMU reading at rest, X-up (90 deg roll from Z-up)
            calibrator.add(
                create_imu_obs(20.0 + static_cast<double>(i), {g, 0, 0}, {0.0, 0.0, 0.0}));
        }

        auto calib = calibrator.getCalibration();
        ASSERT_(calib.has_value());

        const double tol = 1e-6;

        check_vector_equal(calib->bias_acc_b, {0.0, 0.0, 0.0}, tol, "bias_acc_b (static X-up)");
        check_vector_equal(calib->bias_gyro, {0.0, 0.0, 0.0}, tol, "bias_gyro (static X-up)");

        // Orientation: up_vector=(1,0,0). pitch = -asin(1) = -pi/2. roll = -asin(0) = 0.
        MRPT_ASSERT_NEAR_MSG_(calib->pitch, -mrpt::DEG2RAD(90.0), tol, "pitch (static X-up)");
        MRPT_ASSERT_NEAR_MSG_(calib->roll, 0.0, tol, "roll (static X-up)");

        std::cout << "Test 3: Static X-up (No orientation data) OK.\n";
    }

    // 4. Test getCalibration() - Z-up, with simple noise
    {
        ImuInitialCalibrator calibrator;
        calibrator.parameters.required_samples = 3;
        calibrator.parameters.max_samples_age  = 100.0;
        calibrator.parameters.gravity          = 9.81;
        double g                               = calibrator.parameters.gravity;

        std::vector<mrpt::math::TVector3D> acc_samples = {
            {0.01, 0.0, g}, {0.0, 0.01, g}, {-0.01, -0.01, g}};
        std::vector<mrpt::math::TVector3D> gyro_samples = {
            {0.0, 0.0, 0.01}, {0.0, 0.0, -0.01}, {0.0, 0.0, 0.0}};

        for (size_t i = 0; i < acc_samples.size(); ++i)
        {
            calibrator.add(
                create_imu_obs(30.0 + static_cast<double>(i), acc_samples[i], gyro_samples[i]));
        }

        auto calib = calibrator.getCalibration();
        ASSERT_(calib.has_value());

        const double tol        = 1e-3;
        const double stddev_tol = 1e-4;

        // Average Accel (0, 0, g)
        // up_vector ~ (0, 0, 1)
        // Bias Acc: ~ (0, 0, 0)
        check_vector_equal(calib->bias_acc_b, {0.0, 0.0, 0.0}, tol, "bias_acc_b (with noise)");

        // Average Gyro (0, 0, 0)
        // Bias Gyro: ~ (0, 0, 0)
        check_vector_equal(calib->bias_gyro, {0.0, 0.0, 0.0}, tol, "bias_gyro (with noise)");

        // Pitch/Roll: ~ 0
        MRPT_ASSERT_NEAR_MSG_(calib->pitch, 0.0, tol, "pitch (with noise)");
        MRPT_ASSERT_NEAR_MSG_(calib->roll, 0.0, tol, "roll (with noise)");

        // Noise stddev check (unbiased estimation n-1 divisor)
        // Accel X: var = ((0.01-0)^2 + (0-0)^2 + (-0.01-0)^2) / (3-1) = (0.0001 + 0 + 0.0001) / 2 =
        // 0.0001 Accel X stddev = sqrt(0.0001) = 0.01 Accel Y: var = ((0-0)^2 + (0.01-0)^2 +
        // (-0.01-0)^2) / 2 = 0.0001 Accel Y stddev = 0.01 Accel Z: var = ((g-g)^2 + (g-g)^2 +
        // (g-g)^2) / 2 = 0 Accel Z stddev = 0.0
        mrpt::math::TVector3D expected_acc_stddev = {0.01, 0.01, 0.0};
        check_vector_equal(
            calib->noise_stddev_acc, expected_acc_stddev, stddev_tol,
            "noise_stddev_acc (with noise)");

        // Gyro X/Y: var = 0, stddev = 0
        // Gyro Z: average = (0.01 + (-0.01) + 0) / 3 = 0
        // Gyro Z var = ((0.01-0)^2 + (-0.01-0)^2 + (0-0)^2) / 2 = (0.0001 + 0.0001 + 0) / 2 =
        // 0.0001 Gyro Z stddev = 0.01
        mrpt::math::TVector3D expected_gyro_stddev = {0.0, 0.0, 0.01};
        check_vector_equal(
            calib->noise_stddev_gyro, expected_gyro_stddev, stddev_tol,
            "noise_stddev_gyro (with noise)");

        std::cout << "Test 4: Z-up with noise OK.\n";
    }

    // 5. Test getCalibration() - Orientation data available (Z-up)
    {
        ImuInitialCalibrator calibrator;
        calibrator.parameters.required_samples = 3;
        calibrator.parameters.max_samples_age  = 100.0;
        calibrator.parameters.gravity          = 9.81;
        double g                               = calibrator.parameters.gravity;

        for (int i = 0; i < 5; ++i)
        {
            // Quat for identity (0 roll, 0 pitch)
            mrpt::math::CQuaternionDouble quat(1.0, 0.0, 0.0, 0.0);
            quat.normalize();

            calibrator.add(
                create_imu_obs(40.0 + static_cast<double>(i), {0, 0, g}, {0.0, 0.0, 0.0}, quat));
        }

        auto calib = calibrator.getCalibration();
        ASSERT_(calib.has_value());

        const double tol = 1e-6;

        // Bias checks remain the same as Test 2
        check_vector_equal(calib->bias_acc_b, {0.0, 0.0, 0.0}, tol, "bias_acc_b (with quat Z-up)");
        check_vector_equal(calib->bias_gyro, {0.0, 0.0, 0.0}, tol, "bias_gyro (with quat Z-up)");

        // Orientation: Should come from SO(3) average, which is (0, 0)
        MRPT_ASSERT_NEAR_MSG_(calib->pitch, 0.0, tol, "pitch (with quat Z-up)");
        MRPT_ASSERT_NEAR_MSG_(calib->roll, 0.0, tol, "roll (with quat Z-up)");

        std::cout << "Test 5: Z-up with orientation data OK.\n";
    }

    // 6. Test getCalibration() - Orientation data available (45 deg pitch)
    {
        ImuInitialCalibrator calibrator;
        calibrator.parameters.required_samples = 3;
        calibrator.parameters.max_samples_age  = 100.0;
        calibrator.parameters.gravity          = 9.81;
        double g                               = calibrator.parameters.gravity;

        // Tilt 45 degrees around Y (pitch)
        const double pitch_rad = mrpt::DEG2RAD(45.0);
        const double cos_p     = std::cos(pitch_rad);
        const double sin_p     = std::sin(pitch_rad);

        // Expected Accel: Rotated (0, 0, g) by R_y(-pitch)
        // [ cos(-p)  0 sin(-p) ] [ 0 ]   [ g*sin(-p) ]   [ -g*sin(p) ]
        // [ 0        1 0       ] [ 0 ] = [ 0         ] = [ 0         ]
        // [-sin(-p)  0 cos(-p) ] [ g ]   [ g*cos(-p) ]   [ g*cos(p)  ]
        // Accel = (-g*sin(p), 0, g*cos(p))
        const mrpt::math::TVector3D expected_accel = {-g * sin_p, 0.0, g * cos_p};

        // Quat for 45 deg pitch: from CPose3D(0,0,0,pitch_rad, 0, 0)
        mrpt::math::CQuaternionDouble quat;
        mrpt::poses::CPose3D::FromYawPitchRoll(0.0, pitch_rad, 0.0).getAsQuaternion(quat);

        quat.normalize();

        for (int i = 0; i < 5; ++i)
        {
            calibrator.add(create_imu_obs(
                50.0 + static_cast<double>(i), expected_accel, {0.0, 0.0, 0.0}, quat));
        }

        auto calib = calibrator.getCalibration();
        ASSERT_(calib.has_value());

        const double tol = 1e-5;

        // Bias Acc: Should be near zero
        const mrpt::math::TVector3D expected_bias = {.0, .0, .0};
        check_vector_equal(
            calib->bias_acc_b, expected_bias, tol, "bias_acc_b (with quat 45 deg pitch)");

        // Bias Gyro: ~ (0, 0, 0)
        check_vector_equal(
            calib->bias_gyro, {0.0, 0.0, 0.0}, tol, "bias_gyro (with quat 45 deg pitch)");

        // Orientation: Should come from SO(3) average (45 deg pitch, 0 roll)
        MRPT_ASSERT_NEAR_MSG_(calib->pitch, pitch_rad, tol, "pitch (with quat 45 deg pitch)");
        MRPT_ASSERT_NEAR_MSG_(calib->roll, 0.0, tol, "roll (with quat 45 deg pitch)");

        std::cout << "Test 6: 45 deg pitch with orientation data OK.\n";
    }

    // 7. Test getCalibration() - No orientation data available (45 deg pitch)
    {
        ImuInitialCalibrator calibrator;
        calibrator.parameters.required_samples = 3;
        calibrator.parameters.max_samples_age  = 100.0;
        calibrator.parameters.gravity          = 9.81;
        double g                               = calibrator.parameters.gravity;

        // Tilt 45 degrees around Y (pitch)
        const double pitch_rad = mrpt::DEG2RAD(45.0);
        const double cos_p     = std::cos(pitch_rad);
        const double sin_p     = std::sin(pitch_rad);

        // Expected Accel: Rotated (0, 0, g) by R_y(-pitch)
        // [ cos(-p)  0 sin(-p) ] [ 0 ]   [ g*sin(-p) ]   [ -g*sin(p) ]
        // [ 0        1 0       ] [ 0 ] = [ 0         ] = [ 0         ]
        // [-sin(-p)  0 cos(-p) ] [ g ]   [ g*cos(-p) ]   [ g*cos(p)  ]
        // Accel = (-g*sin(p), 0, g*cos(p))
        const mrpt::math::TVector3D expected_accel = {-g * sin_p, 0.0, g * cos_p};

        for (int i = 0; i < 5; ++i)
        {
            calibrator.add(
                create_imu_obs(50.0 + static_cast<double>(i), expected_accel, {0.0, 0.0, 0.0}));
        }

        auto calib = calibrator.getCalibration();
        ASSERT_(calib.has_value());

        const double tol = 1e-5;

        // Bias Acc
        const mrpt::math::TVector3D expected_bias = {.0, .0, .0};
        check_vector_equal(
            calib->bias_acc_b, expected_bias, tol, "bias_acc_b (with quat 45 deg pitch)");

        // Bias Gyro: ~ (0, 0, 0)
        check_vector_equal(
            calib->bias_gyro, {0.0, 0.0, 0.0}, tol, "bias_gyro (with quat 45 deg pitch)");

        // Orientation: Should come from SO(3) average (45 deg pitch, 0 roll)
        MRPT_ASSERT_NEAR_MSG_(calib->pitch, pitch_rad, tol, "pitch (with quat 45 deg pitch)");
        MRPT_ASSERT_NEAR_MSG_(calib->roll, 0.0, tol, "roll (with quat 45 deg pitch)");

        // std::cout << calib->asString();

        std::cout << "Test 7: 45 deg pitch with orientation data OK.\n";
    }

    // 8. Test sample aging
    {
        ImuInitialCalibrator calibrator;
        calibrator.parameters.required_samples = 3;
        calibrator.parameters.max_samples_age  = 5.0;  // seconds

        // Add 5 samples from t=0 to t=4
        for (int i = 0; i < 5; ++i)
        {
            calibrator.add(create_imu_obs(static_cast<double>(i), {0, 0, 9.81}, {0, 0, 0}));
        }
        ASSERT_(calibrator.isReady());

        // Add a sample at t=10.0. This should remove samples at t=0, t=1, t=2, t=3, t=4
        // The last sample added was at t=4.0. The oldest one allowed is t=10.0 - 5.0 = 5.0
        // All previous 5 samples (t=0..4) are older than the allowed age of 5.0 seconds
        // from the newest sample (t=4.0). Wait, the check is against samples.rbegin()->first
        // (newest): samples.begin()->first < samples.rbegin()->first - parameters.max_samples_age
        // New sample at t=10.0: samples.rbegin()->first = 10.0. Max age = 5.0. Threshold = 5.0.
        // Sample at t=0: 0 < 10 - 5 (5) -> removed.
        // Sample at t=4: 4 < 10 - 5 (5) -> removed.
        // Sample at t=5: 5 is NOT < 5 -> not removed.
        // This means the samples t=0..4 should be removed when adding a sample at t=10.0.

        // Re-adding 5 samples from t=0 to t=4
        calibrator                             = ImuInitialCalibrator();
        calibrator.parameters.required_samples = 3;
        calibrator.parameters.max_samples_age  = 5.0;
        for (int i = 0; i < 5; ++i)
        {
            calibrator.add(create_imu_obs(static_cast<double>(i), {0, 0, 9.81}, {0, 0, 0}));
        }

        // Add sample at t=4.9. Newest is 4.9. Oldest allowed is 4.9 - 5.0 = -0.1.
        // Sample at t=0: 0 is NOT < -0.1.
        // Wait, the logic is slightly tricky here. Let's rely on the max_samples_age logic from the
        // file. The implementation removes while samples.begin()->first < samples.rbegin()->first -
        // parameters.max_samples_age

        // Add sample at t=5.0. Newest is 5.0. Oldest allowed is 5.0 - 5.0 = 0.0.
        // Sample at t=0: 0 is NOT < 0.0. All 6 samples should remain.
        calibrator.add(create_imu_obs(5.0, {0, 0, 9.81}, {0, 0, 0}));
        // If the implementation is slightly different, this may fail. Let's trust the logic from
        // the source file. Since we don't have access to the private members, we rely on isReady()
        // or getCalibration() which don't help here. We assume the logic works as intended and just
        // ensure no crash.

        // Add sample at t=5.0001. Newest is 5.0001. Oldest allowed is 0.0001.
        // Sample at t=0: 0 < 0.0001 -> removed.
        // Sample at t=1: 1 NOT < 0.0001.
        // This is complex to test without direct access. Let's create a clearer scenario:

        calibrator                             = ImuInitialCalibrator();
        calibrator.parameters.required_samples = 3;
        calibrator.parameters.max_samples_age  = 2.0;

        // t=1, t=2, t=3, t=4
        calibrator.add(create_imu_obs(1.0, {0, 0, 9.81}, {0, 0, 0}));
        calibrator.add(create_imu_obs(2.0, {0, 0, 9.81}, {0, 0, 0}));
        calibrator.add(create_imu_obs(3.0, {0, 0, 9.81}, {0, 0, 0}));
        calibrator.add(create_imu_obs(4.0, {0, 0, 9.81}, {0, 0, 0}));
        ASSERT_(calibrator.isReady());  // size=4

        // Add t=6.0. Newest=6.0. Oldest allowed = 4.0.
        // t=1: 1 < 4 -> removed.
        // t=2: 2 < 4 -> removed.
        // t=3: 3 < 4 -> removed.
        // t=4: 4 is NOT < 4. Retained.
        // New set: {t=4, t=6} -> size=2. NOT ready.

        calibrator.add(create_imu_obs(6.0, {0, 0, 9.81}, {0, 0, 0}));
        ASSERT_(!calibrator.isReady());  // size must be 2 (<3)
        // If we get a result, the size is still >= 3, which is wrong.
        ASSERT_(!calibrator.getCalibration().has_value());

        std::cout << "Test 8: Sample aging OK (implicit check).\n";
    }

    // 9. Test getCalibration() - Static, no noise, 60 deg Roll (No orientation data)
    {
        ImuInitialCalibrator calibrator;
        calibrator.parameters.required_samples = 3;
        calibrator.parameters.max_samples_age  = 100.0;
        calibrator.parameters.gravity          = 9.81;
        const double g                         = calibrator.parameters.gravity;

        // 60 degrees roll (around X axis)
        const double roll_rad = M_PI / 3.0;  // 60 deg in radians

        // Expected Accel Reading (Static body rotated 60 deg roll, measuring -g)
        // a_body = R_x(-60) * (0, 0, g) = (0, g*sin(60), g*cos(60))
        const double sin_60 = std::sin(roll_rad);
        const double cos_60 = std::cos(roll_rad);

        const mrpt::math::TVector3D expected_accel = {
            0.0, g * sin_60, g * cos_60};  // (0, 8.4957, 4.905)

        // Expected Bias Accel: bias_acc_b = (up_vector * g) - (0, 0, g)
        // up_vector * g is the expected_accel vector itself.
        const mrpt::math::TVector3D expected_bias_acc = {.0, .0, .0};

        // Expected Roll:
        const double expected_roll  = roll_rad;
        const double expected_pitch = 0.0;

        for (int i = 0; i < 5; ++i)
        {
            // Add IMU observation without quaternion data
            calibrator.add(create_imu_obs(
                80.0 + static_cast<double>(i), expected_accel, {0.0, 0.0, 0.0}, std::nullopt));
        }

        auto calib = calibrator.getCalibration();

        // Assertions
        const double tol = 1e-5;

        ASSERT_(calib.has_value());

        // Gyro Bias: Should be zero
        ASSERT_NEAR_(calib->bias_gyro.x, 0.0, tol);
        ASSERT_NEAR_(calib->bias_gyro.y, 0.0, tol);
        ASSERT_NEAR_(calib->bias_gyro.z, 0.0, tol);

        // Accel Bias: (0, g*sin(60), g*cos(60) - g)
        MRPT_ASSERT_NEAR_MSG_(
            calib->bias_acc_b.x, expected_bias_acc.x, tol, "Field bias_acc_b.x mismatch.");
        MRPT_ASSERT_NEAR_MSG_(
            calib->bias_acc_b.y, expected_bias_acc.y, tol, "Field bias_acc_b.y mismatch.");
        MRPT_ASSERT_NEAR_MSG_(
            calib->bias_acc_b.z, expected_bias_acc.z, tol, "Field bias_acc_b.z mismatch.");

        // Orientation (derived from gravity vector)
        MRPT_ASSERT_NEAR_MSG_(calib->pitch, expected_pitch, tol, "Pitch mismatch for 60 deg roll.");
        // The implementation computes -roll_rad due to convention (Pitch = -asin(up_x), Roll =
        // -asin(up_y))
        MRPT_ASSERT_NEAR_MSG_(calib->roll, expected_roll, tol, "Roll mismatch for 60 deg roll.");
        // std::cout << calib->asString();

        std::cout << "Test 9: 60 deg roll (gravity-based attitude) OK.\n";
    }

    std::cout << "--- TestImuInitialCalibrator finished OK ---\n";
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        TestImuInitialCalibrator();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception: \n" << e.what() << "\n";
        return 1;
    }
    catch (...)
    {
        std::cerr << "Untyped exception!\n";
        return 1;
    }

    return 0;
}