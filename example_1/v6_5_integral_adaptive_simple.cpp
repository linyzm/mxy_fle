/**
 * Z 向力误差积分自适应控制
 * Stage 0: 五次多项式移动到预接触点
 * Stage 1: Z 向慢速下降直到接触
 * Stage 2: Z 向恒力控制 + Y 向匀速运动
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/utility.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <stdexcept>
#include <thread>

using namespace flexiv;

namespace {
constexpr size_t kLoopFreq = 1000;
constexpr double kDt = 0.001;

// 目标力与自适应积分参数
constexpr double kDesiredForceZ = 5.0;
constexpr double kLambdaZ = 0.0002;
constexpr double kMaxOffsetZ = 0.005;
constexpr double kMaxForceInt = 30.0;
constexpr double kForceSignZ = 1.0;

// 默认：Z 减小 = 下压，Z 增大 = 上抬。方向反了就改成 +1.0。
constexpr double kZControlDirection = -1.0;

// Stage 1 接触参数
constexpr double kApproachSpeedZ = 0.007;
constexpr double kMaxApproachDistanceZ = 0.2;
constexpr double kForceReachBand = 0.5;

// Stage 2 Y 向运动参数
constexpr double kYSpeed = 0.005;

// 力滤波参数
constexpr double kForceFilterAlpha = 0.90;

// 碰撞检测阈值
constexpr double kExtForceThreshold = 30.0;
constexpr double kExtTorqueThreshold = 5.0;

// 记录数据
constexpr int kLogInterval = 10;
std::ofstream log_file;

std::atomic<bool> g_stop_sched = false;
}

enum class ControlStage { MoveToPreContact, SlowApproach, ForceControl };

void PrintHelp()
{
    std::cout << "Usage: ./program [robot_sn] [--hold] [--collision]" << std::endl;
}

void QuinticPolynomial(
    std::array<std::array<double, 6>, rdk::kPoseSize>& coeffs,
    const std::array<double, rdk::kPoseSize>& start,
    const std::array<double, rdk::kPoseSize>& goal,
    double T)
{
    const double T3 = std::pow(T, 3);
    const double T4 = std::pow(T, 4);
    const double T5 = std::pow(T, 5);

    for (size_t i = 0; i < rdk::kPoseSize; ++i) {
        const double dp = goal[i] - start[i];
        coeffs[i] = {start[i], 0.0, 0.0, 10.0 * dp / T3, -15.0 * dp / T4, 6.0 * dp / T5};
    }
}

std::array<double, rdk::kPoseSize> EvalQuintic(
    const std::array<std::array<double, 6>, rdk::kPoseSize>& coeffs,
    double t)
{
    std::array<double, rdk::kPoseSize> pose {};
    for (size_t i = 0; i < rdk::kPoseSize; ++i) {
        pose[i] = coeffs[i][0]
                + coeffs[i][1] * t
                + coeffs[i][2] * std::pow(t, 2)
                + coeffs[i][3] * std::pow(t, 3)
                + coeffs[i][4] * std::pow(t, 4)
                + coeffs[i][5] * std::pow(t, 5);
    }
    return pose;
}

class ZIntegralForceController {
public:
    double Update(double desired_force, double measured_force)
    {
        force_error_ = desired_force - measured_force;
        force_integral_ += force_error_ * kDt;
        force_integral_ = std::clamp(force_integral_, -kMaxForceInt, kMaxForceInt);

        offset_ = kLambdaZ * force_integral_;
        offset_ = std::clamp(offset_, -kMaxOffsetZ, kMaxOffsetZ);
        return offset_;
    }

    void Reset()
    {
        force_error_ = 0.0;
        force_integral_ = 0.0;
        offset_ = 0.0;
    }

    double force_error() const { return force_error_; }
    double force_integral() const { return force_integral_; }
    double offset() const { return offset_; }

private:
    double force_error_ = 0.0;
    double force_integral_ = 0.0;
    double offset_ = 0.0;
};

bool CollisionDetected(const rdk::Robot& robot)
{
    Eigen::Vector3d ext_force = {
        robot.states().ext_wrench_in_world[0],
        robot.states().ext_wrench_in_world[1],
        robot.states().ext_wrench_in_world[2]
    };

    if (ext_force.norm() > kExtForceThreshold) {
        return true;
    }

    for (const auto& tau : robot.states().tau_ext) {
        if (std::fabs(tau) > kExtTorqueThreshold) {
            return true;
        }
    }

    return false;
}

void PeriodicTask(
    rdk::Robot& robot,
    const std::array<double, rdk::kPoseSize>& init_pose,
    bool enable_hold,
    bool enable_collision)
{
    static uint64_t loop_counter = 0;
    static bool initialized = false;
    static bool filter_initialized = false;

    static ControlStage stage = ControlStage::MoveToPreContact;
    static ZIntegralForceController force_ctrl;
    static std::array<std::array<double, 6>, rdk::kPoseSize> coeffs;
    static std::array<double, rdk::kPoseSize> final_pose;

    static double filtered_force_z = 0.0;
    static double approach_start_z = 0.0;
    static double stage1_start_time = 0.0;
    static double current_y = 0.0;

    try {
        if (robot.fault()) {
            throw std::runtime_error("Robot fault occurred");
        }

        const double time = loop_counter * kDt;
        auto target_pose = init_pose;

        if (!initialized) {
            final_pose = init_pose;
            final_pose[2] = 0.01;
            final_pose[1] = init_pose[1] - 0.19;
            current_y = final_pose[1];

            QuinticPolynomial(coeffs, init_pose, final_pose, kMoveTime);
            initialized = true;
            spdlog::info("Stage 0: move to pre-contact pose");
        }

        if (enable_hold) {
            robot.StreamCartesianMotionForce(init_pose);
            ++loop_counter;
            return;
        }

        const double measured_force_z = kForceSignZ * robot.states().ext_wrench_in_world[2];
        if (!filter_initialized) {
            filtered_force_z = measured_force_z;
            filter_initialized = true;
        } else {
            filtered_force_z = kForceFilterAlpha * filtered_force_z
                             + (1.0 - kForceFilterAlpha) * measured_force_z;
        }

        const double force_error = kDesiredForceZ - filtered_force_z;

        if (log_file.is_open() && loop_counter % kLogInterval == 0) {
            log_file << time << ","
                     << measured_force_z << ","
                     << filtered_force_z << ","
                     << force_error << ","
                     << force_ctrl.offset() << "\n";
        }

        switch (stage) {
        case ControlStage::MoveToPreContact:
            target_pose = EvalQuintic(coeffs, std::min(time, kMoveTime));

            if (time >= kMoveTime) {
                target_pose = final_pose;
                approach_start_z = final_pose[2];
                stage1_start_time = time;
                stage = ControlStage::SlowApproach;
                spdlog::info("Stage 0 finished, enter Stage 1");
            }
            break;

        case ControlStage::SlowApproach: {
            const double approach_time = time - stage1_start_time;
            const double approach_distance = std::min(
                kApproachSpeedZ * approach_time, kMaxApproachDistanceZ);

            target_pose = final_pose;
            target_pose[2] = approach_start_z - approach_distance;
            current_y = target_pose[1];

            const bool force_reached = filtered_force_z >= kDesiredForceZ - kForceReachBand;
            const bool distance_reached = approach_distance >= kMaxApproachDistanceZ;

            if (force_reached || distance_reached) {
                final_pose = target_pose;
                force_ctrl.Reset();
                stage = ControlStage::ForceControl;
                spdlog::info("Enter Stage 2 | Fz: {:.3f} N", filtered_force_z);
            }

            if (loop_counter % kLoopFreq == 0) {
                spdlog::info(
                    "Stage 1 | Fz: {:.3f}, Ferr: {:.3f}, approach: {:.6f}, target_z: {:.6f}",
                    filtered_force_z, force_error, approach_distance, target_pose[2]);
            }
            break;
        }

        case ControlStage::ForceControl: {
            target_pose = final_pose;

            current_y += kYSpeed * kDt;
            target_pose[1] = current_y;

            const double offset_z = force_ctrl.Update(kDesiredForceZ, filtered_force_z);
            target_pose[2] = final_pose[2] + kZControlDirection * offset_z;

            if (loop_counter % kLoopFreq == 0) {
                spdlog::info(
                    "Stage 2 | Fz: {:.3f}, Ferr: {:.3f}, int: {:.6f}, offset_z: {:.6f}, target_z: {:.6f}, target_y: {:.6f}",
                    filtered_force_z,
                    force_ctrl.force_error(),
                    force_ctrl.force_integral(),
                    force_ctrl.offset(),
                    target_pose[2],
                    target_pose[1]);
            }
            break;
        }
        }

        if (enable_collision && CollisionDetected(robot)) {
            robot.Stop();
            spdlog::warn("Collision detected, stopping robot ...");
            g_stop_sched = true;
            return;
        }

        robot.StreamCartesianMotionForce(target_pose);
        ++loop_counter;

    } catch (const std::exception& e) {
        spdlog::error(e.what());
        g_stop_sched = true;
    }
}

int main(int argc, char* argv[])
{
    if (argc < 2 || rdk::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }

    const std::string robot_sn = argv[1];
    const bool enable_hold = rdk::utility::ProgramArgsExist(argc, argv, "--hold");
    const bool enable_collision = rdk::utility::ProgramArgsExist(argc, argv, "--collision");

    try {
        rdk::Robot robot(robot_sn);

        if (robot.fault()) {
            spdlog::warn("Fault occurred, trying to clear ...");
            if (!robot.ClearFault()) {
                spdlog::error("Fault cannot be cleared, exiting ...");
                return 1;
            }
        }

        spdlog::info("Enabling robot ...");
        robot.Enable();
        while (!robot.operational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        spdlog::info("Moving to home pose");
        robot.SwitchMode(rdk::Mode::NRT_PLAN_EXECUTION);
        robot.ExecutePlan("PLAN-Home");
        while (robot.busy()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        robot.SwitchMode(rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        spdlog::warn("Zeroing force/torque sensors, make sure nothing is in contact");
        robot.ExecutePrimitive("ZeroFTSensor", std::map<std::string, rdk::FlexivDataTypes> {});
        while (!std::get<int>(robot.primitive_states()["terminated"])) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        robot.SwitchMode(rdk::Mode::RT_CARTESIAN_MOTION_FORCE);
        robot.SetForceControlAxis(
            std::array<bool, rdk::kCartDoF> {false, false, false, false, false, false});

        const auto init_pose = robot.states().tcp_pose;

        log_file.open("force_control_log.csv");
        log_file << "time,Fz_raw,Fz_filtered,Ferr,offset_z\n";

        rdk::Scheduler scheduler;
        scheduler.AddTask(
            std::bind(PeriodicTask, std::ref(robot), std::ref(init_pose), enable_hold, enable_collision),
            "HP periodic",
            1,
            scheduler.max_priority());

        scheduler.Start();
        while (!g_stop_sched) {
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
