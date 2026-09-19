/**
 * Realtime Cartesian PI Admittance Control in Z direction
 *
 * stage 0: move to pre-contact pose by quintic polynomial
 * stage 1: continue slow downward approach until target force is reached
 * stage 2: start Z-direction admittance force control
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/utility.hpp>
#include <spdlog/spdlog.h>
#include <fstream>
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
//导出数据
std::ofstream log_file;
constexpr int kLogInterval = 10;  // 每10ms记录一次

constexpr size_t kLoopFreq = 1000;
constexpr double kDt = 0.001;

// ===================== 导纳控制参数 =====================
constexpr double kDesiredForceZ = 10.0;

constexpr double kM = 25.0;
constexpr double kD = 2600.0;//200
// constexpr double kKp = 0.5;//0.5
// constexpr double kKi = 1.0;


// ===================== 力方向修正 =====================
// 如果发现力越大机器人越往里压，把 1.0 改成 -1.0
constexpr double kForceSignZ = 1.0;

// ===================== 碰撞检测阈值 =====================
constexpr double kExtForceThreshold = 30.0;
constexpr double kExtTorqueThreshold = 5.0;

// ===================== Stage 1 慢速接触参数 =====================
constexpr double kApproachSpeedZ = 0.003;       // 继续下降速度 1 mm/s
constexpr double kMaxApproachDistanceZ = 0.05;  // 最多继续下降 5 cm
constexpr double kForceReachBand = 0.5;         // Fz >= 目标力 - 0.5 N 就进入 stage 2

// ===================== 硬保护 =====================
constexpr double kHardStopForceZ = 25.0;
//自适应参数
static constexpr double kSigma = 0.00;             // 自适应更新率

static constexpr double kVelocityEps = 1e-5;       // 防止 velocity_ 为 0 时除零


// 停止实时任务标志
std::atomic<bool> g_stop_sched = {false};
}

enum class ControlStage
{
    MoveToPreContact = 0,
    SlowApproach = 1,
    ForceControl = 2
};

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
class AdmittanceZ
{
public:
        double Update(double desired_force, double measured_force)
        {
            // 论文定义：
            // ΔF = Fe - Fd
            const double delta_force = measured_force - desired_force;

            // 论文 Eq.(14):
            // Phi(t) = Phi(t - lambda) + sigma * [Fd(t-lambda) - Fe(t-lambda)] / b
            //
            // 注意：这里严格使用上一周期的 force error
            // Fd - Fe = -ΔF
            phi_ += kSigma * (last_desired_force_ - last_measured_force_) / kD;

            // 论文 Eq.(14):
            // Δb(t) = b / e_dot_hat(t) * Phi(t)
            //
            // 在你的代码中，velocity_ 就对应 e_dot_hat 或 Xc_dot - Xe_dot 的近似
            double delta_b = last_delta_b_;

            if (std::abs(velocity_) > kVelocityEps) {
                delta_b = kD * phi_ / velocity_;
            }

            // 工程保护：防止除以很小速度导致 Δb 爆炸
            // 这不是论文核心律，只是数值保护

            // 论文中的实际阻尼：
            // B(t) = b + Δb(t)
            double adaptive_damping = kD + delta_b;

            // 论文 Eq.(15) 简化形式：
            // Xc_ddot(t) = 1/M * [ΔF(t) - B(t) * Xc_dot(t-1)]
            //
            // 这里没有 Xe_dot，因此按 Xe_dot = 0
            double acc = (delta_force - adaptive_damping * velocity_) / kM;

            velocity_ += acc * kDt;

            offset_ += velocity_ * kDt;


            last_desired_force_ = desired_force;
            last_measured_force_ = measured_force;
            last_force_error_ = delta_force;
            last_acc_ = acc;
            last_delta_b_ = delta_b;
            last_adaptive_damping_ = adaptive_damping;

            return offset_;
        }

    void Reset()
    {
        phi_ = 0.0;
        last_delta_b_ = 0.0;
        last_adaptive_damping_ = kD;

        velocity_ = 0.0;
        offset_ = 0.0;

        last_desired_force_ = kDesiredForceZ;
        last_measured_force_ = kDesiredForceZ;

        last_force_error_ = 0.0;
        last_acc_ = 0.0;
    }

    double offset() const { return offset_; }
    double velocity() const { return velocity_; }
    double force_error() const { return last_force_error_; }
    double phi() const { return phi_; }
    double delta_b() const { return last_delta_b_; }
    double adaptive_damping() const { return last_adaptive_damping_; }
private:
    double phi_ = 0.0;                       // 对应论文 Φ(t)
    double last_delta_b_ = 0.0;              // 对应论文 Δb(t)
    double last_adaptive_damping_ = kD;      // 对应论文 B(t)=b+Δb(t)

    double velocity_ = 0.0;                  // 近似 e_dot_hat
    double offset_ = 0.0;                    // 位置补偿

    double last_desired_force_ = kDesiredForceZ;
    double last_measured_force_ = kDesiredForceZ;

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
 * Stage 0: 运动到预接触位置
 * Stage 1: 慢速下降，检测到目标力附近后进入力控
 * Stage 2: Z 向 PI 导纳恒力控制
 */
void PeriodicTask(
    rdk::Robot& robot,
    const std::array<double, rdk::kPoseSize>& init_pose,
    bool enable_hold,
    bool enable_collision)
{
    static uint64_t loop_counter = 0;
    static bool initialized = false;

    static ControlStage stage = ControlStage::MoveToPreContact;
    static AdmittanceZ admittance_z;

    static std::array<std::array<double, 6>, rdk::kPoseSize> coeffs;
    static std::array<double, rdk::kPoseSize> final_pose;

    static double approach_start_z = 0.0;
    static double stage1_start_time = 0.0;
    static double last_pose =0.0;

    constexpr double kMoveTime = 5.0;

    try {
        if (robot.fault()) {
            throw std::runtime_error("Robot fault occurred");
        }

        const double time = loop_counter * kDt;
        auto target_pose = init_pose;

        // ===================== 初始化轨迹，只执行一次 =====================
        if (!initialized) {
            final_pose = init_pose;

            // 当前稳定版本使用绝对 Z = 0.02 m
            // 如果以后想改成相对下降，可改为：
            // final_pose[2] = init_pose[2] - 0.005;
            final_pose[2] = 0.01;
            final_pose[1] = init_pose[1]-0.19;//0.235


            QuinticPolynomial(coeffs, init_pose, final_pose, kMoveTime);

            initialized = true;
            stage = ControlStage::MoveToPreContact;

            spdlog::info("Stage 0: move to pre-contact pose");
        }

        // ===================== --hold 模式 =====================
        if (enable_hold) {
            robot.StreamCartesianMotionForce(init_pose);
            loop_counter++;
            return;
        }

        // ===================== 统一读取 Z 向力 =====================
        const double raw_force_z = robot.states().ext_wrench_in_world[2];
        const double measured_force_z = kForceSignZ * raw_force_z;
        const double force_error = measured_force_z - kDesiredForceZ;
        if (log_file.is_open() && loop_counter % kLogInterval == 0) {
            log_file << time << "," << measured_force_z << "\n";
        }

        // ===================== 状态机 =====================
        switch (stage) {
        case ControlStage::MoveToPreContact:
        {
            const double t = std::min(time, kMoveTime);

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
                approach_start_z = final_pose[2];
                stage1_start_time = time;
                stage = ControlStage::SlowApproach;

                spdlog::info("Stage 0 finished, enter Stage 1: slow approach");
            }
        }
        break;

        case ControlStage::SlowApproach:
        {

            const double approach_time = time - stage1_start_time;
            const double approach_distance
                = std::min(kApproachSpeedZ * approach_time, kMaxApproachDistanceZ);

            target_pose = final_pose;
            last_pose = final_pose[1];

            // 默认认为：Z 减小 = 下降 / 下压，Z 增大 = 退让
            // 如果实际机器人 Z 增大才是下降，把这里的 - 改成 +
            target_pose[2] = approach_start_z - approach_distance;

            const bool target_force_reached
                = measured_force_z >= kDesiredForceZ - kForceReachBand;

            const bool max_approach_reached
                = approach_distance >= kMaxApproachDistanceZ;

            if (target_force_reached || max_approach_reached) {
                final_pose = target_pose;
                admittance_z.Reset();
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
                    "Stage 1 | Fz: {:.3f} N, Ferr: {:.3f} N, approach: {:.6f} m, target_z: {:.6f}",
                    measured_force_z,
                    force_error,
                    approach_distance,
                    target_pose[2]);
            }
        }
        break;

        case ControlStage::ForceControl:
        {
            target_pose = final_pose;
            //y轴移动
            const double y_speed =0.003;//0.005
            const double y_offset =y_speed * kDt;
            target_pose[1] = last_pose + y_offset;
            last_pose = target_pose[1];


            //z方向导纳控制
            const double offset_z = admittance_z.Update(kDesiredForceZ, measured_force_z);

            // 默认认为：
            // Fz 大于目标时 offset_z 为正，target_pose[2] 增大，机器人退让
            target_pose[2] = final_pose[2] + offset_z;
            //target_pose[1] = final_pose[1] + offset_z;

            //打印
            if (loop_counter % kLoopFreq == 0) {
                spdlog::info(
                    "Stage 2 | Fz: {:.3f} N, Ferr: {:.3f} N, "
                    " vel_z: {:.6f}, offset_z: {:.6f}, target_z: {:.6f}",
                    measured_force_z,
                    admittance_z.force_error(),

                    admittance_z.velocity(),
                    admittance_z.offset(),
                    target_pose[2]);
            }


        }
        break;
        }

        // Simple collision detection: stop robot if collision is detected from either end-effector
        // or robot body防碰撞
        if (enable_collision) {
            bool collision_detected = false;
            Eigen::Vector3d ext_force = {robot.states().ext_wrench_in_world[0],
                robot.states().ext_wrench_in_world[1], robot.states().ext_wrench_in_world[2]};
            if (ext_force.norm() > kExtForceThreshold) {
                collision_detected = true;
            }
            for (const auto& v : robot.states().tau_ext) {
                if (fabs(v) > kExtTorqueThreshold) {
                    collision_detected = true;
                }
            }
            if (collision_detected) {
                robot.Stop();
                spdlog::warn("Collision detected, stopping robot and exit program ...");
                g_stop_sched = true;
                return;
            }
        }

        loop_counter++;
        robot.StreamCartesianMotionForce(target_pose);

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

    std::string robot_sn = argv[1];

    spdlog::info(
        ">>> Tutorial description <<<\n"
        "Stage 0: move to pre-contact pose by quintic polynomial.\n"
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

        log_file.open("force_control_log3.csv");
        log_file << "time,Fz\n";
        // log_file << "time,Fz,Fd,Ferr,target_y,target_z,vel_z,offset_z,acc_z,dB,B\n";

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