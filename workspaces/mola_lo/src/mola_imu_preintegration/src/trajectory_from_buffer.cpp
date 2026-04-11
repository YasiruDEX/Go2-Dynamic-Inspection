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
 * @file   trajectory_from_buffer.cpp
 * @brief  Reconstruct a trajectory from a LocalVelocityBuffer
 * @author Jose Luis Blanco Claraco
 * @date   Sep 5, 2025
 */

#include <mola_imu_preintegration/trajectory_from_buffer.h>
#include <mrpt/containers/find_closest.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/poses/Lie/SO.h>

#include <Eigen/Dense>
#include <type_traits>

using namespace mola::imu;

namespace
{
template <bool SourceIsConst = true>
void execute_integration(
    Trajectory&                           t,
    const std::function<void(
        std::conditional_t<SourceIsConst, const TrajectoryPoint&, TrajectoryPoint&> p0,
        TrajectoryPoint& p1, double dt)>& update,
    const double                          start_time = 0.0)
{
    for (auto it = t.find(start_time); it != t.end(); it++)
    {
        auto it_next = std::next(it);
        if (it_next == t.end())
        {
            break;
        }
        auto&        p_prev = it->second;
        auto&        p_this = it_next->second;
        const double dt     = it_next->first - it->first;

        update(p_prev, p_this, dt);
    }
    for (auto it = std::make_reverse_iterator(std::next(t.find(start_time))); it != t.rend(); ++it)
    {
        auto it_next = std::next(it);  // reverse_iterator trick
        if (it_next == t.rend())
        {
            break;
        }

        auto&        p_prev = it->second;
        auto&        p_this = it_next->second;
        const double dt     = it_next->first - it->first;

        update(p_prev, p_this, dt);
    }
};

}  // namespace

std::string TrajectoryPoint::asString() const
{
    using mrpt::format;

    auto vecToStr = [](const mrpt::math::TVector3D& x)
    { return mrpt::format("[%.3f %.3f %.3f]", x[0], x[1], x[2]); };

    auto optVecToStr = [&](const std::optional<mrpt::math::TVector3D>& x)
    { return x ? vecToStr(*x) : std::string{"<none>"}; };

    auto matToStr = [](const mrpt::math::CMatrixDouble33& M)
    {
        mrpt::poses::CPose3D pp;
        pp.setRotationMatrix(M);
        return mrpt::format(
            "(ypr)=(%.02f,%.02f,%.02f) [deg]", mrpt::RAD2DEG(pp.yaw()), mrpt::RAD2DEG(pp.pitch()),
            mrpt::RAD2DEG(pp.roll()));
    };

    auto optMatToStr = [&](const std::optional<mrpt::math::CMatrixDouble33>& M)
    { return M ? matToStr(*M) : std::string{"<none>"}; };

    std::ostringstream oss;
    oss << "TrajectoryPoint{"
        << "\n  pose = " << pose.asString()  //
        << "\n  R_ga = " << optMatToStr(R_ga)  //
        << "\n  v    = " << optVecToStr(v)  //
        << "\n  w_b  = " << optVecToStr(w_b)  //
        << "\n  a_b  = " << optVecToStr(a_b)  //
        << "\n  ac_b = " << optVecToStr(ac_b) << "\n}";
    //<< "\n  alpha= " << vecToStr(alpha_b)  // Not used much
    //<< "\n  j_b  = " << vecToStr(j_b)   // Not used much
    return oss.str();
}

std::string mola::imu::trajectory_as_string(const Trajectory& traj)
{
    std::ostringstream oss;
    oss << "Trajectory with " << traj.size() << " frames:\n";
    for (const auto& [t, p] : traj)
    {
        oss << mrpt::format(" t=%.04f: %s\n", t, p.asString().c_str());
    }
    return oss.str();
}

Trajectory mola::imu::trajectory_from_buffer(
    const LocalVelocityBuffer::SampleHistory& samples, const ImuIntegrationParams& imu_params,
    bool use_higher_order)
{
    Trajectory t;

    const auto& bias_acc  = imu_params.bias_acc;
    const auto& bias_gyro = imu_params.bias_gyro;

    const Eigen::Vector3d gravity(
        imu_params.gravity_vector.x, imu_params.gravity_vector.y, imu_params.gravity_vector.z);

    // 1) Build the list of all timestamps that we will reconstruct:
    // {0, t_IMU}
    t[0] = TrajectoryPoint(mrpt::math::CMatrixDouble33::Identity(), {.0, .0, .0});

    // 2) Fill (ω,a) from IMU:
    for (const auto& [stamp, sample] : samples.by_time)
    {
        if (sample.w_b.has_value() && sample.a_b.has_value())
        {
            t[stamp].w_b = *sample.w_b - bias_gyro;
            t[stamp].a_b = *sample.a_b - bias_acc;
        }
    }

    // and assign the closest IMU reading to t=0:
    t[0].w_b = mrpt::containers::find_closest(samples.by_type.w_b, 0.0)->second - bias_gyro;
    t[0].a_b = mrpt::containers::find_closest(samples.by_type.a_b, 0.0)->second - bias_acc;

    // 3) Copy the latest gravity-aligned hints on global orientations:
    const double last_sample_rel_time = samples.by_time.rbegin()->first;
    const auto closest_q = mrpt::containers::find_closest(samples.by_type.q, last_sample_rel_time);
    ASSERTMSG_(
        closest_q,
        "At least one entry with gravity-aligned orientation is needed for IMU integration");
    const double stamp_first_R_ga = closest_q->first;
    t[closest_q->first].R_ga      = closest_q->second;

    // 3b) and copy the closest velocity from the given samples:
    const auto closest_v_b =
        mrpt::containers::find_closest(samples.by_type.v_b, last_sample_rel_time);
    ASSERTMSG_(closest_v_b, "At least one entry with velocity is needed for IMU integration");
    const double stamp_first_v_b = closest_v_b->first;
    t[closest_v_b->first].v      = closest_v_b->second;

    // and assign the closest IMU reading to all frames:
    for (auto& [stamp, tp] : t)
    {
        if (!tp.w_b.has_value())
        {
            tp.w_b = mrpt::containers::find_closest(samples.by_type.w_b, stamp)->second - bias_gyro;
        }
        if (!tp.a_b.has_value())
        {
            tp.a_b = mrpt::containers::find_closest(samples.by_type.a_b, stamp)->second - bias_acc;
        }
    }

    // 7) (only for higher-order) Estimate alpha:
    if (use_higher_order)
    {
        // Estimate jerk & alpha:
        execute_integration<false>(
            t,
            [](TrajectoryPoint& p0, TrajectoryPoint& p1, double dt)
            {
                ASSERT_(p0.w_b && p1.w_b);
                p0.alpha_b = p1.alpha_b = (*p1.w_b - *p0.w_b) / dt;
            });
    }

    // 4) Integrate R forward / backwards in time:
    if (use_higher_order)
    {
        execute_integration<>(
            t,
            [](const TrajectoryPoint& p0, TrajectoryPoint& p1, double dt)
            {
                ASSERT_(p0.w_b.has_value());
                const mrpt::math::TVector3D  w     = *p0.w_b;
                const mrpt::math::TVector3D& alpha = p0.alpha_b;

                const double dt2 = dt * dt;

                p1.pose.setRotationMatrix(
                    p0.pose.getRotationMatrix() *
                    mrpt::poses::Lie::SO<3>::exp(
                        (w * dt + alpha * 0.5 * dt2)
                            .asVector<mrpt::math::CVectorFixedDouble<3>>()));
            });
    }
    else
    {
        execute_integration<>(
            t,
            [](const TrajectoryPoint& p0, TrajectoryPoint& p1, double dt)
            {
                ASSERT_(p0.w_b.has_value());
                const mrpt::math::TVector3D w = *p0.w_b;

                p1.pose.setRotationMatrix(
                    p0.pose.getRotationMatrix() *
                    mrpt::poses::Lie::SO<3>::exp(
                        (w * dt).asVector<mrpt::math::CVectorFixedDouble<3>>()));
            });
    }

    // 5) Integrate R_gravity_aligned:
    execute_integration<>(
        t,
        [](const TrajectoryPoint& p0, TrajectoryPoint& p1, [[maybe_unused]] double dt)
        {
            p1.R_ga = (*p0.R_ga) *
                      ((p0.pose.getRotationMatrix()).inverse() * (p1.pose.getRotationMatrix()));
        },
        stamp_first_R_ga);

    // 6) convert acceleration:
    // proper acceleration in the body frame => coordinate acceleration in the body frame
    for (auto& [stamp, p] : t)
    {
        ASSERT_(p.a_b);
        const auto gravity_b = p.R_ga->transpose() * gravity;

        ASSERT_(p.a_b);
        p.ac_b = *p.a_b + mrpt::math::TVector3D(gravity_b.x(), gravity_b.y(), gravity_b.z());
    }

    // 7) (only for higher-order) Estimate jerk:
    if (use_higher_order)
    {
        // Estimate jerk & alpha:
        execute_integration<false>(
            t,
            [](TrajectoryPoint& p0, TrajectoryPoint& p1, double dt)
            {
                ASSERT_(p0.w_b && p1.w_b);
                ASSERT_(p0.ac_b && p1.ac_b);
                p0.j_b = p1.j_b = (*p1.ac_b - *p0.ac_b) / dt;
            });
    }

    // 8) Integrate v:
    if (use_higher_order)
    {
        execute_integration<>(
            t,
            [](const TrajectoryPoint& p0, TrajectoryPoint& p1, [[maybe_unused]] double dt)
            {
                ASSERT_(p0.v.has_value());
                const double dt2 = dt * dt;
                p1.v = *p0.v + p0.pose.rotateVector(p0.ac_b.value() * dt + p0.j_b * dt2 * 0.5);
            },
            stamp_first_v_b);
    }
    else
    {
        execute_integration<>(
            t,
            [](const TrajectoryPoint& p0, TrajectoryPoint& p1, [[maybe_unused]] double dt)
            {
                ASSERT_(p0.v.has_value());
                p1.v = *p0.v + p0.pose.rotateVector(p0.ac_b.value() * dt);
            },
            stamp_first_v_b);
    }

    // 9) Integrate p:
    if (use_higher_order)
    {
        execute_integration<>(
            t,
            [](const TrajectoryPoint& p0, TrajectoryPoint& p1, [[maybe_unused]] double dt)
            {
                ASSERT_(p0.v.has_value());
                const double dt2 = dt * dt;
                const double dt3 = dt2 * dt;

                const auto pt =  //
                    p0.pose.translation() +  //
                    p0.v.value() * dt +
                    p0.pose.rotateVector(
                        p0.ac_b.value() * dt2 * 0.5 +  //
                        p0.j_b * dt3 * (1.0 / 6.0));
                p1.pose.x(pt.x);
                p1.pose.y(pt.y);
                p1.pose.z(pt.z);
            });
    }
    else
    {
        execute_integration<>(
            t,
            [](const TrajectoryPoint& p0, TrajectoryPoint& p1, [[maybe_unused]] double dt)
            {
                ASSERT_(p0.v.has_value());
                const double dt2 = dt * dt;

                const auto pt = p0.pose.translation() + p0.v.value() * dt +
                                p0.pose.rotateVector(p0.ac_b.value() * dt2 * 0.5);
                p1.pose.x(pt.x);
                p1.pose.y(pt.y);
                p1.pose.z(pt.z);
            });
    }

#if 0
    // Debug print
    for (auto& [stamp, p] : t)
    {
        std::cout << "t=" << stamp << " " << p.asString() << "\n";
    }
#endif

    return t;
}
