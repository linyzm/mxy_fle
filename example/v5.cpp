/**
 * Realtime Cartesian PI Admittance Control in Z direction
 *
 * stage 0: move down slightly by quintic polynomial
 * stage 1: continue slow downward approach until target force is reached
 * stage 2: start Z-direction admittance force control
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/utility.hpp>
#include <spdlog/spdlog.h>

#include <array>
#include <vector>
#include <cmath>
#include <thread>
#include <atomic>
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <functional>

using namespace flexiv;

namespace {
constexpr size_t kLoopFreq = 1000;
constexpr double kDt = 0.001;

// ===================== 导纳控制参数 =====================
// 调试阶段先用 5 N，确认方向和稳定性后再逐步加到 10 N / 20 N
constexpr double kDesiredForceZ = 1.0;

// 保守导纳参数，避免一进入力控就饱和
constexpr double kM = 10.0;
constexpr double kD = 800.0;
constexpr double kKp = 0.5;
constexpr double kKi = 1.0;

// ===================== 导纳安全限幅 =====================
constexpr double kMaxOffsetZ = 0.05;       // 最大位置修正 2 cm
constexpr double kMaxVelocityZ = 0.005;    // 最大修正速度 5 mm/s
constexpr double kMaxForceInt = 10.0;      // 积分限幅

// ===================== 力方向修正 =====================
// 如果发现力越大机器人越往里压，把 1.0 改成 -1.0
constexpr double kForceSignZ = 1.0;

// ===================== 碰撞检测阈值 =====================
constexpr double kExtForceThreshold = 30.0;

// ===================== Stage 1 慢速接触参数 =====================
constexpr double kApproachSpeedZ = 0.001;       // 继续下降速度 0.5 mm/s
constexpr double kMaxApproachDistanceZ = 0.05;  // 最多继续下降 5 mm
constexpr double kForceReachBand = 0.5;          // Fz >= 目标力 - 0.5 N 就进入 stage 2

// ===================== 硬保护 =====================
// 即使没开 --collision，只要 Z 向力过大也立即停
constexpr double kHardStopForceZ = 25.0;

// 停止实时任务标志
std::atomic<bool> g_stop = {false};
}

/**
 * @brief 生成五次多项式轨迹系数
 *
 * p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
 */
void QuinticPolynomial(
    std::array<std::array<double, 6>, rdk::kPoseSize>& coeffs,
    const std::array<double, rdk::kPoseSize>& init_pos,
    const std::array<double, rdk::kPoseSize>& final_pos,
    double T)
{
    for (size_t i = 0; i < rdk::kPoseSize; i++) {
        coeffs[i][0] = init_pos[i];
        coeffs[i][1] = 0.0;
        coeffs[i][2] = 0.0;
        coeffs[i][3] = 10.0 * (final_pos[i] - init_pos[i]) / std::pow(T, 3);
        coeffs[i][4] = -15.0 * (final_pos[i] - init_pos[i]) / std::pow(T, 4);
        coeffs[i][5] = 6.0 * (final_pos[i] - init_pos[i]) / std::pow(T, 5);
    }
}

/**
 * @brief Z 向 PI 导纳控制器
 *
 * ddXc = (Kp * force_error + Ki * force_error_int - D * dXc) / M
 * dXc  = dXc + dt * ddXc
 * Xc   = Xc + dt * dXc
 */
class PIAdmittanceZ
{
public:
    double Update(double desired_force, double measured_force)
    {
        double force_error = measured_force - desired_force;

        force_error_int_ += force_error * kDt;
        force_error_int_ = std::clamp(force_error_int_, -kMaxForceInt, kMaxForceInt);

        double acc = (kKp * force_error + kKi * force_error_int_ - kD * velocity_) / kM;

        velocity_ += acc * kDt;
        velocity_ = std::clamp(velocity_, -kMaxVelocityZ, kMaxVelocityZ);

        offset_ += velocity_ * kDt;
        offset_ = std::clamp(offset_, -kMaxOffsetZ, kMaxOffsetZ);

        last_force_error_ = force_error;
        last_acc_ = acc;

        return offset_;
    }

    void Reset()
    {
        force_error_int_ = 0.0;
        velocity_ = 0.0;
        offset_ = 0.0;
        last_force_error_ = 0.0;
        last_acc_ = 0.0;
    }

    double offset() const { return offset_; }
    double velocity() const { return velocity_; }
    double force_error() const { return last_force_error_; }
    double force_error_int() const { return force_error_int_; }
    double acc() const { return last_acc_; }

private:
    double force_error_int_ = 0.0;
    double velocity_ = 0.0;
    double offset_ = 0.0;

    double last_force_error_ = 0.0;
    double last_acc_ = 0.0;
};

void PrintHelp()
{
    std::cout << "Usage: ./program [robot_sn] [--hold] [--collision]" << std::endl;
}

/**
 * @brief 实时周期任务
 *
 * stage 0: 先用五次多项式下降 5 mm
 * stage 1: 继续慢速下降，达到目标力附近立刻进入导纳控制
 * stage 2: Z 向 PI 导纳恒力控制
 */
void PeriodicTask(
    rdk::Robot& robot,
    const std::array<double, rdk::kPoseSize>& init_pose,
    bool enable_hold,
    bool enable_collision)
{
    static uint64_t loop_counter = 0;
    static PIAdmittanceZ admittance_z;

    // ===================== switch-case 状态 =====================
    static bool initialized = false;
    static int stage = 0;

    // stage 1 慢速下降相关状态
    static double approach_start_z = 0.0;
    static double stage1_start_time = 0.0;

    // ===================== 轨迹相关变量 =====================
    static std::array<std::array<double, 6>, rdk::kPoseSize> coeffs;
    static std::array<double, rdk::kPoseSize> final_pose;

    // stage 0：五次多项式下降时间
    constexpr double kMoveTime = 5.0;

    try {
        if (robot.fault()) {
            throw std::runtime_error("Robot fault occurred");
        }

        double time = loop_counter * kDt;

        // 默认目标位姿
        auto target_pose = init_pose;

        // 只初始化一次轨迹
        if (!initialized) {
            final_pose = init_pose;

            // 先相对当前位置下降 5 mm

            final_pose[2] = 0.02;

            QuinticPolynomial(coeffs, init_pose, final_pose, kMoveTime);

            stage = 0;
            initialized = true;

            spdlog::info("Stage 0: move down 5 mm first");
        }

        if (enable_hold) {
            target_pose = init_pose;
        } else {
            switch (stage) {
            case 0:
            {
                // ==================================================
                // stage 0：先用五次多项式下降 5 mm，不启用力控
                // ==================================================
                double t = std::min(time, kMoveTime);

                for (size_t i = 0; i < rdk::kPoseSize; i++) {
                    target_pose[i]
                        = coeffs[i][0]
                        + coeffs[i][1] * t
                        + coeffs[i][2] * std::pow(t, 2)
                        + coeffs[i][3] * std::pow(t, 3)
                        + coeffs[i][4] * std::pow(t, 4)
                        + coeffs[i][5] * std::pow(t, 5);
                }

                if (time >= kMoveTime) {
                    target_pose = final_pose;

                    stage = 1;
                    approach_start_z = final_pose[2];
                    stage1_start_time = time;

                    spdlog::info("Stage 0 finished, enter Stage 1: slow approach");
                }
            }
            break;

            case 1:
            {
                // ==================================================
                // stage 1：继续慢速下降
                // 只要接触力达到目标附近，立即进入 stage 2
                // ==================================================
                target_pose = final_pose;

                double raw_force_z = robot.states().ext_wrench_in_world[2];
                double measured_force_z = kForceSignZ * raw_force_z;
                double force_error = measured_force_z - kDesiredForceZ;

                double approach_time = time - stage1_start_time;
                double approach_distance = kApproachSpeedZ * approach_time;
                approach_distance = std::min(approach_distance, kMaxApproachDistanceZ);

                // 默认认为：
                // Z 减小 = 下降 / 下压
                // Z 增大 = 退让
                //
                // 如果实际机器人 Z 增大才是下降，把这里的 - 改成 +
                target_pose[2] = approach_start_z - approach_distance;

                // 达到目标力附近，立刻进入导纳力控
                // 不再等待“稳定 100 个周期”，否则容易继续下降过头
                if (measured_force_z >= kDesiredForceZ - kForceReachBand) {
                    final_pose = target_pose;
                    admittance_z.Reset();
                    stage = 2;

                    spdlog::info(
                        "Target force reached. Enter Stage 2. Fz: {:.3f} N",
                        measured_force_z);
                }

                // 如果最大接近距离用完，也进入 stage 2，避免无限下降
                if (approach_distance >= kMaxApproachDistanceZ) {
                    final_pose = target_pose;
                    admittance_z.Reset();
                    stage = 2;

                    spdlog::warn(
                        "Max approach distance reached. Enter Stage 2. Fz: {:.3f} N",
                        measured_force_z);
                }

                // 硬保护：力过大立即停止
                if (std::fabs(measured_force_z) > kHardStopForceZ) {
                    robot.Stop();
                    spdlog::error(
                        "Hard stop: Fz too large in Stage 1. Fz: {:.3f} N",
                        measured_force_z);
                    g_stop = true;
                }

                if (loop_counter % kLoopFreq == 0) {
                    spdlog::info(
                        "Stage 1 | Fz: {:.3f} N, Ferr: {:.3f} N, approach: {:.6f} m, target_z: {:.6f}",
                        measured_force_z,
                        force_error,
                        approach_distance,
                        target_pose[2]);
                }
            }
            break;

            case 2:
            {
                // ==================================================
                // stage 2：Z 向 PI 导纳恒力控制
                // ==================================================
                target_pose = final_pose;

                double raw_force_z = robot.states().ext_wrench_in_world[2];
                double measured_force_z = kForceSignZ * raw_force_z;

                double offset_z = admittance_z.Update(kDesiredForceZ, measured_force_z);

                // 默认认为：
                // Fz 大于目标时 offset_z 为正，target_pose[2] 增大，机器人退让
                target_pose[2] = final_pose[2] + offset_z;

                // 硬保护：力过大立即停止
                if (std::fabs(measured_force_z) > kHardStopForceZ) {
                    robot.Stop();
                    spdlog::error(
                        "Hard stop: Fz too large in Stage 2. Fz: {:.3f} N, offset_z: {:.6f}",
                        measured_force_z,
                        offset_z);
                    g_stop = true;
                }

                if (loop_counter % kLoopFreq == 0) {
                    spdlog::info(
                        "Stage 2 | Fz: {:.3f} N, Ferr: {:.3f} N, "
                        "Ferr_int: {:.3f}, vel_z: {:.6f}, offset_z: {:.6f}, target_z: {:.6f}",
                        measured_force_z,
                        admittance_z.force_error(),
                        admittance_z.force_error_int(),
                        admittance_z.velocity(),
                        admittance_z.offset(),
                        target_pose[2]);
                }
            }
            break;

            default:
            {
                target_pose = final_pose;
                stage = 2;
                spdlog::warn("Unknown stage, switch to Stage 2");
            }
            break;
            }
        }

        // 发送目标位姿
        robot.StreamCartesianMotionForce(target_pose);

        // 额外碰撞保护，只有 --collision 打开时生效
        if (enable_collision) {
            double fx = robot.states().ext_wrench_in_world[0];
            double fy = robot.states().ext_wrench_in_world[1];
            double fz = robot.states().ext_wrench_in_world[2];

            double force_norm = std::sqrt(fx * fx + fy * fy + fz * fz);

            if (force_norm > kExtForceThreshold) {
                robot.Stop();
                spdlog::warn("Large external force detected, robot stopped");
                g_stop = true;
            }
        }

        loop_counter++;

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

    std::string robot_sn = argv[1];

    spdlog::info(
        ">>> Tutorial description <<<\n"
        "Stage 0: move down 5 mm by quintic polynomial.\n"
        "Stage 1: slow approach until target force reached.\n"
        "Stage 2: start Z-direction PI admittance force control.\n");

    bool enable_hold = false;
    if (rdk::utility::ProgramArgsExist(argc, argv, "--hold")) {
        spdlog::info("Robot holding current TCP pose");
        enable_hold = true;
    } else {
        spdlog::info("Robot will move down, approach contact, then start force control");
    }

    bool enable_collision = false;
    if (rdk::utility::ProgramArgsExist(argc, argv, "--collision")) {
        spdlog::info("Collision detection enabled");
        enable_collision = true;
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

        robot.ExecutePrimitive("ZeroFTSensor", std::map<std::string, rdk::FlexivDataTypes> {});

        while (!std::get<int>(robot.primitive_states()["terminated"])) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        spdlog::info("Sensor zeroing complete");

        robot.SwitchMode(rdk::Mode::RT_CARTESIAN_MOTION_FORCE);

        // 全部轴设置为运动控制
        // 这里不是用机器人内置力控轴，而是外层修改 target_pose[2]
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

        scheduler.Start();

        while (!g_stop) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        scheduler.Stop();

    } catch (const std::exception& e) {
        spdlog::error(e.what());
        return 1;
    }

    return 0;
}