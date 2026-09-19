/**
 * Flexiv RDK + CoppeliaSim ZeroMQ joint position control example.
 *
 * Based on Flexiv intermediate1_realtime_joint_position_control.cpp.
 *
 * Modes:
 *   1) Real robot + CoppeliaSim mirror:
 *      ./flexiv_coppeliasim_joint_position_control Rizon4s-123456 --hold
 *      ./flexiv_coppeliasim_joint_position_control Rizon4s-123456
 *
 *   2) CoppeliaSim only, no real robot connection:
 *      ./flexiv_coppeliasim_joint_position_control --sim-only --hold
 *      ./flexiv_coppeliasim_joint_position_control --sim-only
 *
 * Optional:
 *      --sim-joint-prefix=/Rizon
 *      If your CoppeliaSim joint paths are /Rizon/joint1 ... /Rizon/joint7.
 *      Default joint paths are /joint1 ... /joint7.
 *
 * CoppeliaSim requirements:
 *   - Start CoppeliaSim first.
 *   - The ZeroMQ remote API add-on should be running.
 *   - The scene must contain revolute joints named /joint1 ... /joint7,
 *     or use --sim-joint-prefix to match your model path.
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/utility.hpp>
#include <spdlog/spdlog.h>

// Copy these headers from:
// CoppeliaSim/programming/zmqRemoteApi/clients/cpp
#include "RemoteAPIClient.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace flexiv;

namespace {
/** RT loop period [sec] */
constexpr double kLoopPeriod = 0.001;

/** Default robot DoF */
constexpr size_t kDefaultDoF = 7;

/** Sine-sweep trajectory amplitude and frequency */
constexpr double kSineAmp = 0.035;
constexpr double kSineFreq = 0.3;

/** Atomic signal to stop all loops */
std::atomic<bool> g_stop_sched = {false};

/** Latest target position shared from control loop to CoppeliaSim loop */
std::mutex g_target_mutex;
std::vector<double> g_latest_target_pos;
std::atomic<bool> g_target_ready = {false};
} // namespace

/** @brief Ctrl-C handler */
void SignalHandler(int)
{
    g_stop_sched = true;
}

/** @brief Print program usage help */
void PrintHelp()
{
    // clang-format off
    std::cout << "Usage:" << std::endl;
    std::cout << "  Real robot + CoppeliaSim:" << std::endl;
    std::cout << "    ./flexiv_coppeliasim_joint_position_control [robot_sn] [--hold] [--sim-joint-prefix=/Rizon]" << std::endl;
    std::cout << std::endl;
    std::cout << "  CoppeliaSim only, no real robot:" << std::endl;
    std::cout << "    ./flexiv_coppeliasim_joint_position_control --sim-only [--hold] [--sim-joint-prefix=/Rizon]" << std::endl;
    std::cout << std::endl;
    std::cout << "Arguments:" << std::endl;
    std::cout << "  robot_sn: Serial number of the robot to connect. Example: Rizon4s-123456" << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --hold: Hold initial joint positions. Otherwise run sine-sweep." << std::endl;
    std::cout << "  --sim-only: Do not connect to real robot. Only control CoppeliaSim." << std::endl;
    std::cout << "  --sim-joint-prefix=/xxx: CoppeliaSim joint path prefix. Default empty." << std::endl;
    std::cout << std::endl;
    // clang-format on
}

bool HasArg(int argc, char* argv[], const std::string& arg)
{
    for (int i = 1; i < argc; ++i) {
        if (argv[i] == arg) {
            return true;
        }
    }
    return false;
}

std::string GetOptionValue(int argc, char* argv[], const std::string& prefix,
    const std::string& default_value = "")
{
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg.rfind(prefix, 0) == 0) {
            return arg.substr(prefix.size());
        }
    }
    return default_value;
}

std::string GetRobotSN(int argc, char* argv[])
{
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (!arg.empty() && arg[0] != '-') {
            return arg;
        }
    }
    return "";
}

std::vector<double> ComputeTargetPosition(
    const std::string& motion_type, const std::vector<double>& init_pos, unsigned int loop_counter)
{
    std::vector<double> target_pos(init_pos.size(), 0.0);

    if (motion_type == "hold") {
        target_pos = init_pos;
    } else if (motion_type == "sine-sweep") {
        for (size_t i = 0; i < target_pos.size(); ++i) {
            target_pos[i] = init_pos[i]
                            + kSineAmp * std::sin(
                                2.0 * M_PI * kSineFreq * loop_counter * kLoopPeriod);
        }
    } else {
        throw std::invalid_argument(
            "Unknown motion type. Accepted motion types: hold, sine-sweep");
    }

    return target_pos;
}

void PublishTargetPosition(const std::vector<double>& target_pos)
{
    std::lock_guard<std::mutex> lock(g_target_mutex);
    g_latest_target_pos = target_pos;
    g_target_ready = true;
}

/** @brief Realtime periodic task for real Flexiv robot.
 *
 * Important:
 *   Do NOT call CoppeliaSim ZeroMQ API here. Network calls can block and add jitter.
 *   This RT task only streams to the real robot and publishes the latest target to a shared buffer.
 */
void PeriodicTask(rdk::Robot& robot, const std::string& motion_type,
    const std::vector<double>& init_pos)
{
    static unsigned int loop_counter = 0;

    try {
        if (robot.fault()) {
            throw std::runtime_error(
                "PeriodicTask: Fault occurred on the connected robot, exiting ...");
        }

        std::vector<double> target_pos =
            ComputeTargetPosition(motion_type, init_pos, loop_counter);
        std::vector<double> target_vel(robot.info().DoF, 0.0);
        std::vector<double> target_acc(robot.info().DoF, 0.0);

        // Send target joint position to real robot
        robot.StreamJointPosition(target_pos, target_vel, target_acc);

        // Publish the same target to CoppeliaSim communication thread
        PublishTargetPosition(target_pos);

        loop_counter++;

    } catch (const std::exception& e) {
        spdlog::error(e.what());
        g_stop_sched = true;
    }
}

/** @brief Target generator used only in --sim-only mode. */
void SimOnlyTargetGenerator(const std::string& motion_type, const std::vector<double>& init_pos)
{
    unsigned int loop_counter = 0;

    while (!g_stop_sched) {
        try {
            auto target_pos = ComputeTargetPosition(motion_type, init_pos, loop_counter);
            PublishTargetPosition(target_pos);
            loop_counter++;
        } catch (const std::exception& e) {
            spdlog::error(e.what());
            g_stop_sched = true;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(
            static_cast<int>(kLoopPeriod * 1e6)));
    }
}

/** @brief CoppeliaSim communication loop.
 *
 * Runs outside the real-time Flexiv scheduler to avoid blocking the RT robot control thread.
 */
template <typename SimT>
void CoppeliaSimThread(SimT& sim, const std::vector<int>& joint_handles)
{
    spdlog::info("CoppeliaSim communication thread started");

    while (!g_stop_sched) {
        std::vector<double> target_pos;
        {
            std::lock_guard<std::mutex> lock(g_target_mutex);
            if (g_target_ready) {
                target_pos = g_latest_target_pos;
            }
        }

        if (!target_pos.empty()) {
            try {
                const size_t n = std::min(target_pos.size(), joint_handles.size());
                for (size_t i = 0; i < n; ++i) {
                    sim.setJointTargetPosition(joint_handles[i], target_pos[i]);
                }

                // CoppeliaSim must be in stepping mode for deterministic sync.
                sim.step();
            } catch (const std::exception& e) {
                spdlog::error("CoppeliaSim communication error: {}", e.what());
                g_stop_sched = true;
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    spdlog::info("CoppeliaSim communication thread stopped");
}

int main(int argc, char* argv[])
{
    std::signal(SIGINT, SignalHandler);

    if (HasArg(argc, argv, "-h") || HasArg(argc, argv, "--help")) {
        PrintHelp();
        return 0;
    }

    const bool sim_only = HasArg(argc, argv, "--sim-only");
    const std::string robot_sn = GetRobotSN(argc, argv);
    const std::string sim_joint_prefix =
        GetOptionValue(argc, argv, "--sim-joint-prefix=", "");

    if (!sim_only && robot_sn.empty()) {
        spdlog::error("Missing robot_sn. Use --sim-only if you do not want to connect real robot.");
        PrintHelp();
        return 1;
    }

    std::string motion_type;
    if (HasArg(argc, argv, "--hold")) {
        spdlog::info("Motion type: hold");
        motion_type = "hold";
    } else {
        spdlog::info("Motion type: sine-sweep");
        motion_type = "sine-sweep";
    }

    try {
        // =========================================================================================
        // CoppeliaSim Initialization
        // =========================================================================================
        zmqRemoteApi::RemoteAPIClient client;
        auto sim = client.getObject().sim();

        spdlog::info("Connected to CoppeliaSim ZeroMQ Remote API");

        // For joint control, your CoppeliaSim scene should have joint paths:
        //   /joint1 ... /joint7
        // or with prefix:
        //   /Rizon/joint1 ... /Rizon/joint7
        const size_t dof = sim_only ? kDefaultDoF : kDefaultDoF;
        std::vector<int> sim_joint_handles(dof);
        for (size_t i = 0; i < dof; ++i) {
            std::string joint_path;
            if (sim_joint_prefix.empty()) {
                joint_path = "/joint" + std::to_string(i + 1);
            } else {
                joint_path = sim_joint_prefix + "/joint" + std::to_string(i + 1);
            }

            sim_joint_handles[i] = sim.getObject(joint_path);
            spdlog::info("CoppeliaSim joint {} -> handle {}", joint_path, sim_joint_handles[i]);
        }

        sim.setStepping(true);
        sim.startSimulation();
        spdlog::info("CoppeliaSim simulation started in stepping mode");

        std::thread sim_thread(CoppeliaSimThread<decltype(sim)>,
            std::ref(sim), std::cref(sim_joint_handles));

        // =========================================================================================
        // Mode 1: CoppeliaSim only
        // =========================================================================================
        if (sim_only) {
            std::vector<double> init_pos(kDefaultDoF, 0.0);
            spdlog::info("Running in --sim-only mode. Initial joints are all zeros.");
            spdlog::info("Press Ctrl-C to stop.");

            std::thread target_thread(SimOnlyTargetGenerator,
                std::cref(motion_type), std::cref(init_pos));

            while (!g_stop_sched) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            if (target_thread.joinable()) {
                target_thread.join();
            }
        }
        // =========================================================================================
        // Mode 2: Real Flexiv robot + CoppeliaSim mirror
        // =========================================================================================
        else {
            spdlog::info("Running in real robot + CoppeliaSim mode");

            // Instantiate robot interface
            rdk::Robot robot(robot_sn);

            // Clear fault on the connected robot if any
            if (robot.fault()) {
                spdlog::warn("Fault occurred on the connected robot, trying to clear ...");
                if (!robot.ClearFault()) {
                    spdlog::error("Fault cannot be cleared, exiting ...");
                    g_stop_sched = true;
                }
                spdlog::info("Fault on the connected robot is cleared");
            }

            if (!g_stop_sched) {
                // Enable the robot, make sure the E-stop is released before enabling
                spdlog::info("Enabling robot ...");
                robot.Enable();

                // Wait for the robot to become operational
                while (!robot.operational() && !g_stop_sched) {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
                spdlog::info("Robot is now operational");

                // Move robot to home pose
                spdlog::info("Moving to home pose");
                robot.SwitchMode(rdk::Mode::NRT_PLAN_EXECUTION);
                robot.ExecutePlan("PLAN-Home");
                while (robot.busy() && !g_stop_sched) {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }

                // Switch to real-time joint position control mode
                robot.SwitchMode(rdk::Mode::RT_JOINT_POSITION);

                // Set initial joint positions from real robot
                auto init_pos = robot.states().q;
                spdlog::info("Initial joint positions set to: {}",
                    rdk::utility::Vec2Str(init_pos));

                // Publish once so CoppeliaSim moves to the same initial q
                PublishTargetPosition(init_pos);

                // Create real-time scheduler to run periodic tasks
                rdk::Scheduler scheduler;
                scheduler.AddTask(
                    std::bind(PeriodicTask, std::ref(robot), std::ref(motion_type),
                        std::ref(init_pos)),
                    "HP periodic", 1, scheduler.max_priority());
                scheduler.Start();

                spdlog::info("Control started. Press Ctrl-C to stop.");
                while (!g_stop_sched) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }

                scheduler.Stop();
            }
        }

        g_stop_sched = true;
        if (sim_thread.joinable()) {
            sim_thread.join();
        }

        try {
            sim.stopSimulation();
            spdlog::info("CoppeliaSim simulation stopped");
        } catch (const std::exception& e) {
            spdlog::warn("Failed to stop CoppeliaSim simulation cleanly: {}", e.what());
        }

    } catch (const std::exception& e) {
        spdlog::error(e.what());
        g_stop_sched = true;
        return 1;
    }

    return 0;
}
