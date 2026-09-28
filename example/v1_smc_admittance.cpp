/**
 * Realtime Cartesian sliding-mode admittance control in the Z direction.
 *
 * This example keeps the motion framework and state machine from v1:
 *   stage 0: move to the pre-contact pose using a quintic polynomial
 *   stage 1: approach the surface slowly until contact is detected
 *   stage 2: move along Y while regulating Z contact force
 *
 * Only the stage-2 force controller is changed to:
 *   Fd -> force error -> SMC -> admittance -> desired Z position
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/utility.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>

using namespace flexiv;

namespace {

std::ofstream log_file;
constexpr std::uint64_t kLogInterval = 10;

constexpr std::size_t kLoopFreq = 1000;
constexpr double kDt = 0.001;

// Desired contact force [N].
constexpr double kDesiredForceZ = 10.0;

// Sliding-mode surface and boundary-layer parameters.
// s_f = d(e_f)/dt + lambda_f * e_f
// u_smc = -K_f * sat(s_f / phi)
constexpr double kLambdaF = 20.0;
constexpr double kSmcGainF = 2.0;
constexpr double kBoundaryLayerPhi = 50.0;

// Desired admittance parameters.
// M_d * ddot(x_r) + B_d * dot(x_r) + K_d * (x_r - x_0)
//     = F_d - F_e + u_smc
constexpr double kMd = 1.0;
constexpr double kBd = 200.0;
constexpr double kKd = 0;//5

// Change this to -1.0 if the measured contact force has the opposite sign.
constexpr double kForceSignZ = 1.0;

constexpr double kExtForceThreshold = 30.0;
constexpr double kApproachSpeedZ = 0.002;
constexpr double kMaxApproachDistanceZ = 0.2;
constexpr double kForceReachBand = 0.5;
constexpr double kHardStopForceZ = 25.0;

std::atomic<bool> g_stop {false};

double Saturation(double value)
{
    return std::clamp(value, -1.0, 1.0);
}

} // namespace

enum class ControlStage
{
    MoveToPreContact = 0,
    SlowApproach = 1,
    ForceControl = 2
};

void QuinticPolynomial(
    std::array<std::array<double, 6>, rdk::kPoseSize>& coeffs,
    const std::array<double, rdk::kPoseSize>& init_pos,
    const std::array<double, rdk::kPoseSize>& final_pos,
    double duration)
{
    for (std::size_t i = 0; i < rdk::kPoseSize; ++i) {
        coeffs[i][0] = init_pos[i];
        coeffs[i][1] = 0.0;
        coeffs[i][2] = 0.0;
        coeffs[i][3] = 10.0 * (final_pos[i] - init_pos[i]) / std::pow(duration, 3);
        coeffs[i][4] = -15.0 * (final_pos[i] - init_pos[i]) / std::pow(duration, 4);
        coeffs[i][5] = 6.0 * (final_pos[i] - init_pos[i]) / std::pow(duration, 5);
    }
}

/**
 * One-dimensional sliding-mode admittance controller.
 *
 * x_r is a displacement in the positive pressing direction. In this example,
 * pressing means decreasing world-frame Z, so the caller sends
 * target_z = contact_z - x_r to the robot position loop.
 */
class SlidingModeAdmittanceZ
{
public:
    double Update(double desired_force, double measured_force)
    {
        // e_f(k) = F_e(k) - F_d(k)
        force_error_ = measured_force - desired_force;

        // dot(e_f)(k) = (e_f(k) - e_f(k-1)) / T_s
        force_error_rate_ = (force_error_ - previous_force_error_) / kDt;

        // s_f(k) = dot(e_f)(k) + lambda_f * e_f(k)
        sliding_surface_ = force_error_rate_ + kLambdaF * force_error_;

        // u_smc(k) = -K_f * sat(s_f(k) / phi)
        smc_output_ = -kSmcGainF * Saturation(sliding_surface_ / kBoundaryLayerPhi);

        // ddot(x_r)(k) = [F_d - F_e + u_smc - B_d*dot(x_r)
        //                    - K_d*(x_r-x_0)] / M_d
        acceleration_ = (desired_force - measured_force + smc_output_
                            - kBd * velocity_ - kKd * (position_ - equilibrium_position_))
            / kMd;

        // Semi-implicit Euler integration, as specified by the control law.
        velocity_ += kDt * acceleration_;
        position_ += kDt * velocity_;

        previous_force_error_ = force_error_;
        return position_;
    }

    void Reset(double desired_force, double measured_force)
    {
        equilibrium_position_ = 0.0;
        position_ = equilibrium_position_;
        velocity_ = 0.0;
        acceleration_ = 0.0;

        force_error_ = measured_force - desired_force;
        previous_force_error_ = force_error_;
        force_error_rate_ = 0.0;
        sliding_surface_ = kLambdaF * force_error_;
        smc_output_ = 0.0;
    }

    double position() const { return position_; }
    double velocity() const { return velocity_; }
    double acceleration() const { return acceleration_; }
    double force_error() const { return force_error_; }
    double force_error_rate() const { return force_error_rate_; }
    double sliding_surface() const { return sliding_surface_; }
    double smc_output() const { return smc_output_; }

private:
    double equilibrium_position_ = 0.0;
    double position_ = 0.0;
    double velocity_ = 0.0;
    double acceleration_ = 0.0;

    double previous_force_error_ = 0.0;
    double force_error_ = 0.0;
    double force_error_rate_ = 0.0;
    double sliding_surface_ = 0.0;
    double smc_output_ = 0.0;
};

void PrintHelp()
{
    std::cout << "Usage: ./v1_smc_admittance [robot_sn] [--hold] [--collision]"
              << std::endl;
}

void PeriodicTask(
    rdk::Robot& robot,
    const std::array<double, rdk::kPoseSize>& init_pose,
    bool enable_hold,
    bool enable_collision)
{
    static std::uint64_t loop_counter = 0;
    static bool initialized = false;
    static ControlStage stage = ControlStage::MoveToPreContact;
    static SlidingModeAdmittanceZ admittance_z;

    static std::array<std::array<double, 6>, rdk::kPoseSize> coeffs;
    static std::array<double, rdk::kPoseSize> final_pose;

    static double approach_start_z = 0.0;
    static double stage1_start_time = 0.0;
    static double last_pose_y = 0.0;

    constexpr double kMoveTime = 5.0;

    try {
        if (robot.fault()) {
            throw std::runtime_error("Robot fault occurred");
        }

        const double time = loop_counter * kDt;
        auto target_pose = init_pose;

        if (!initialized) {
            final_pose = init_pose;
            final_pose[2] = 0.00;
            final_pose[1] = init_pose[1] - 0.19;

            QuinticPolynomial(coeffs, init_pose, final_pose, kMoveTime);

            initialized = true;
            stage = ControlStage::MoveToPreContact;
            spdlog::info("Stage 0: move to pre-contact pose");
        }

        if (enable_hold) {
            robot.StreamCartesianMotionForce(init_pose);
            ++loop_counter;
            return;
        }

        const double raw_force_z = robot.states().ext_wrench_in_world[2];
        const double measured_force_z = kForceSignZ * raw_force_z;
        const double force_error = measured_force_z - kDesiredForceZ;

        switch (stage) {
        case ControlStage::MoveToPreContact:
        {
            const double t = std::min(time, kMoveTime);

            for (std::size_t i = 0; i < rdk::kPoseSize; ++i) {
                target_pose[i] = coeffs[i][0] + coeffs[i][1] * t
                    + coeffs[i][2] * std::pow(t, 2)
                    + coeffs[i][3] * std::pow(t, 3)
                    + coeffs[i][4] * std::pow(t, 4)
                    + coeffs[i][5] * std::pow(t, 5);
            }

            if (time >= kMoveTime) {
                target_pose = final_pose;
                approach_start_z = final_pose[2];
                stage1_start_time = time;
                stage = ControlStage::SlowApproach;
                spdlog::info("Stage 0 finished, enter Stage 1: slow approach");
            }
            break;
        }

        case ControlStage::SlowApproach:
        {
            const double approach_time = time - stage1_start_time;
            const double approach_distance
                = std::min(kApproachSpeedZ * approach_time, kMaxApproachDistanceZ);

            target_pose = final_pose;
            last_pose_y = final_pose[1];
            target_pose[2] = approach_start_z - approach_distance;

            const bool target_force_reached
                = measured_force_z >= kDesiredForceZ - kForceReachBand;
            const bool max_approach_reached
                = approach_distance >= kMaxApproachDistanceZ;

            if (target_force_reached || max_approach_reached) {
                final_pose = target_pose;
                admittance_z.Reset(kDesiredForceZ, measured_force_z);
                stage = ControlStage::ForceControl;

                if (target_force_reached) {
                    spdlog::info(
                        "Target force reached. Enter Stage 2. Fz: {:.3f} N",
                        measured_force_z);
                } else {
                    spdlog::warn(
                        "Max approach distance reached. Enter Stage 2. Fz: {:.3f} N",
                        measured_force_z);
                }
            }

            if (loop_counter % kLoopFreq == 0) {
                spdlog::info(
                    "Stage 1 | Fz: {:.3f} N, Ferr: {:.3f} N, approach: {:.6f} m, "
                    "target_z: {:.6f}",
                    measured_force_z,
                    force_error,
                    approach_distance,
                    target_pose[2]);
            }
            break;
        }

        case ControlStage::ForceControl:
        {
            target_pose = final_pose;

            constexpr double kYSpeed = 0.005;
            last_pose_y += kYSpeed * kDt;
            target_pose[1] = last_pose_y;

            const double displacement_z
                = admittance_z.Update(kDesiredForceZ, measured_force_z);

            // The controller coordinate is positive into the surface, whereas
            // the v1 setup presses by decreasing world-frame Z.
            target_pose[2] = final_pose[2] - displacement_z;

            if (loop_counter % kLoopFreq == 0) {
                spdlog::info(
                    "Stage 2 | Fz: {:.3f} N, Ferr: {:.3f} N, dFerr: {:.3f} N/s, "
                    "s: {:.3f}, u_smc: {:.3f} N, vel_z: {:.6f}, x_r: {:.6f}, "
                    "target_z: {:.6f}",
                    measured_force_z,
                    admittance_z.force_error(),
                    admittance_z.force_error_rate(),
                    admittance_z.sliding_surface(),
                    admittance_z.smc_output(),
                    admittance_z.velocity(),
                    admittance_z.position(),
                    target_pose[2]);
            }
            break;
        }
        }

        if (log_file.is_open() && loop_counter % kLogInterval == 0) {
            log_file << time << ',' << measured_force_z << ',' << kDesiredForceZ << ','
                     << admittance_z.force_error() << ','
                     << admittance_z.force_error_rate() << ','
                     << admittance_z.sliding_surface() << ',' << admittance_z.smc_output()
                     << ',' << admittance_z.acceleration() << ',' << admittance_z.velocity()
                     << ',' << admittance_z.position() << ',' << target_pose[2] << '\n';
        }

        if (std::fabs(measured_force_z) > kHardStopForceZ) {
            robot.Stop();
            spdlog::error(
                "Hard stop: Fz too large. Fz: {:.3f} N, stage: {}",
                measured_force_z,
                static_cast<int>(stage));
            g_stop = true;
            return;
        }

        robot.StreamCartesianMotionForce(target_pose);

        if (enable_collision) {
            const double fx = robot.states().ext_wrench_in_world[0];
            const double fy = robot.states().ext_wrench_in_world[1];
            const double fz = robot.states().ext_wrench_in_world[2];
            const double force_norm = std::sqrt(fx * fx + fy * fy + fz * fz);

            if (force_norm > kExtForceThreshold) {
                robot.Stop();
                spdlog::warn("Large external force detected, robot stopped");
                g_stop = true;
                return;
            }
        }

        ++loop_counter;
    } catch (const std::exception& e) {
        spdlog::error(e.what());
        g_stop = true;
    }
}

int main(int argc, char* argv[])
{
    if (argc < 2 || rdk::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }

    const std::string robot_sn = argv[1];

    spdlog::info(
        ">>> Tutorial description <<<\n"
        "Stage 0: move to pre-contact pose by quintic polynomial.\n"
        "Stage 1: slow approach until target force is reached.\n"
        "Stage 2: start Z-direction sliding-mode admittance force control.\n");

    const bool enable_hold = rdk::utility::ProgramArgsExist(argc, argv, "--hold");
    if (enable_hold) {
        spdlog::info("Robot holding current TCP pose");
    } else {
        spdlog::info("Robot will move down, approach contact, then start force control");
    }

    const bool enable_collision
        = rdk::utility::ProgramArgsExist(argc, argv, "--collision");
    if (enable_collision) {
        spdlog::info("Collision detection enabled");
    } else {
        spdlog::info("Collision detection disabled");
    }

    try {
        rdk::Robot robot(robot_sn);

        if (robot.fault()) {
            spdlog::warn("Fault occurred on the connected robot, trying to clear ...");
            if (!robot.ClearFault()) {
                spdlog::error("Fault cannot be cleared, exiting ...");
                return 1;
            }
            spdlog::info("Fault on the connected robot is cleared");
        }

        spdlog::info("Enabling robot ...");
        robot.Enable();
        while (!robot.operational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        spdlog::info("Robot is now operational");

        spdlog::info("Moving to home pose");
        robot.SwitchMode(rdk::Mode::NRT_PLAN_EXECUTION);
        robot.ExecutePlan("PLAN-Home");
        while (robot.busy()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        robot.SwitchMode(rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        spdlog::warn(
            "Zeroing force/torque sensors, make sure nothing is in contact with the robot");
        robot.ExecutePrimitive(
            "ZeroFTSensor", std::map<std::string, rdk::FlexivDataTypes> {});
        while (!std::get<int>(robot.primitive_states()["terminated"])) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        spdlog::info("Sensor zeroing complete");

        robot.SwitchMode(rdk::Mode::RT_CARTESIAN_MOTION_FORCE);

        // Keep all axes in motion control. The external force loop changes
        // target_pose[2], exactly as in v1.
        robot.SetForceControlAxis(
            std::array<bool, rdk::kCartDoF> {false, false, false, false, false, false});

        auto init_pose = robot.states().tcp_pose;
        rdk::Scheduler scheduler;
        scheduler.AddTask(
            std::bind(
                PeriodicTask,
                std::ref(robot),
                std::ref(init_pose),
                enable_hold,
                enable_collision),
            "HP periodic",
            1,
            scheduler.max_priority());

        log_file.open("force_control_smc_admittance_log.csv");
        log_file << "time,Fz,Fd,force_error,force_error_rate,sliding_surface,u_smc,"
                    "acceleration,velocity,x_r,target_z\n";

        scheduler.Start();
        while (!g_stop) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        scheduler.Stop();

        if (log_file.is_open()) {
            log_file.close();
        }
    } catch (const std::exception& e) {
        spdlog::error(e.what());
        return 1;
    }

    return 0;
}
