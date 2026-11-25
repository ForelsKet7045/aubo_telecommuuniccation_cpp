#include <aubo_sdk/rpc.h>
#include <aubo_sdk/rtde.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace arcs::aubo_sdk;
using namespace arcs::common_interface;
using Clock = std::chrono::steady_clock;
using Values = std::vector<double>;

namespace {
volatile std::sig_atomic_t interrupted = 0;
void onSignal(int) { interrupted = 1; }

void check(int result, const char* action)
{
    if (result != 0)
        throw std::runtime_error(std::string(action) + ": SDK return code " + std::to_string(result));
}

void checkCancelled()
{
    if (interrupted) throw std::runtime_error("Interrupted");
}

double number(const std::string& text)
{
    std::size_t used = 0;
    const double value = std::stod(text, &used);
    if (used != text.size() || !std::isfinite(value))
        throw std::invalid_argument("Expected a finite number: " + text);
    return value;
}

struct Options {
    std::string command = "read", ip = "127.0.0.1", user = "aubo", password = "123456";
    double seconds = 10, timeout = 60;
    double speedMmS = 50, accelMmS2 = 100;
    int hz = 10;
    Values pose, delta;
    bool help = false, verbose = false;
};

Options parse(int argc, char** argv)
{
    Options o;
    bool commandSet = false;
    bool watchOption = false, moveOption = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if (++i >= argc) throw std::invalid_argument("Missing value after " + arg);
            return argv[i];
        };
        if (arg == "--help" || arg == "-h") { o.help = true; return o; }
        if (arg == "read" || arg == "watch" || arg == "move") {
            if (commandSet) throw std::invalid_argument("Specify one command");
            o.command = arg;
            commandSet = true;
        } else if (arg == "--ip") o.ip = next();
        else if (arg == "--user") o.user = next();
        else if (arg == "--password") o.password = next();
        else if (arg == "--seconds") { o.seconds = number(next()); watchOption = true; }
        else if (arg == "--hz") {
            const double hz = number(next());
            if (hz < 1 || hz > 50 || std::floor(hz) != hz)
                throw std::invalid_argument("--hz must be an integer from 1 to 50");
            o.hz = static_cast<int>(hz);
            watchOption = true;
        } else if (arg == "--timeout") { o.timeout = number(next()); moveOption = true; }
        else if (arg == "--speed-mm-s") { o.speedMmS = number(next()); moveOption = true; }
        else if (arg == "--accel-mm-s2") { o.accelMmS2 = number(next()); moveOption = true; }
        else if (arg == "--verbose") { o.verbose = true; moveOption = true; }
        else if (arg == "--pose" || arg == "--delta-mm") {
            if (!o.pose.empty() || !o.delta.empty())
                throw std::invalid_argument("Specify exactly one --pose or --delta-mm");
            auto& target = arg == "--pose" ? o.pose : o.delta;
            const int count = arg == "--pose" ? 6 : 3;
            for (int axis = 0; axis < count; ++axis) target.push_back(number(next()));
            moveOption = true;
        } else throw std::invalid_argument("Unknown argument: " + arg);
    }
    if (o.seconds <= 0 || o.timeout <= 0)
        throw std::invalid_argument("--seconds and --timeout must be positive");
    if (o.speedMmS <= 0 || o.accelMmS2 <= 0)
        throw std::invalid_argument("--speed-mm-s and --accel-mm-s2 must be positive");
    if (watchOption && o.command != "watch")
        throw std::invalid_argument("--seconds and --hz require watch");
    if (moveOption && o.command != "move")
        throw std::invalid_argument("--pose, --delta-mm, --timeout, --speed-mm-s, --accel-mm-s2 and --verbose require move");
    if (o.command == "move" && o.pose.empty() && o.delta.empty())
        throw std::invalid_argument("move requires --pose or --delta-mm");
    return o;
}

// RAII: login failures and exceptions also release the connection.
template<class Client> struct Session {
    Client client;
    void open(const Options& o, int port)
    {
        client.setLogHandler([](int level, const char*, int, const std::string& message) {
            if (level <= 2) std::cerr << "SDK: " << message << '\n';
        });
        check(client.connect(o.ip, port), "Connect");
        if (!client.hasConnected()) throw std::runtime_error("Controller is not connected");
        check(client.login(o.user, o.password), "Login");
        if (!client.hasLogined()) throw std::runtime_error("Controller login failed");
    }
    ~Session()
    {
        try { if (client.hasLogined()) client.logout(); } catch (...) {}
        try { if (client.hasConnected()) client.disconnect(); } catch (...) {}
    }
};

void validate(const Values& values)
{
    if (values.size() != 6 || !std::all_of(values.begin(), values.end(),
            [](double v) { return std::isfinite(v); }))
        throw std::runtime_error("Expected six finite pose/joint values");
}

double timestamp()
{
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void printArray(const char* key, const Values& values, double scale = 1, std::size_t count = 6)
{
    std::cout << ",\"" << key << "\":[";
    for (std::size_t i = 0; i < count; ++i) {
        if (i) std::cout << ',';
        std::cout << values.at(i) * scale;
    }
    std::cout << ']';
}

// TCP: base frame, [x,y,z,rx,ry,rz], metres and RPY radians.
void showPose(const Values& pose, const Values& joints = {}, double time = timestamp())
{
    validate(pose);
    if (!joints.empty()) validate(joints);
    std::cout << std::setprecision(17) << "{\"time\":" << time;
    printArray("pose_m_rad", pose);
    printArray("xyz_mm", pose, 1000, 3);
    if (!joints.empty()) {
        printArray("joints_rad", joints);
        printArray("joints_deg", joints, 180 / std::acos(-1.0));
    }
    std::cout << '}' << std::endl;
}

void watch(const Options& o)
{
    struct Feedback {
        std::mutex mutex;
        std::condition_variable ready;
        Values pose, joints;
        double time = 0;
        bool pending = false;
        std::exception_ptr error;
    };
    // Keep the callback's state alive through SDK teardown; retain only the latest sample.
    auto feedback = std::make_shared<Feedback>();
    Session<RtdeClient> session;
    session.open(o, 30010);
    auto& rtde = session.client;
    const int topic = rtde.setTopic(false, {"R1_actual_TCP_pose", "R1_actual_q"}, o.hz, 0);
    if (topic < 0) throw std::runtime_error("RTDE setTopic failed");
    check(rtde.subscribe(topic, [feedback](InputParser& parser) {
        std::lock_guard<std::mutex> lock(feedback->mutex);
        try {
            feedback->pose = parser.popVectorDouble();
            feedback->joints = parser.popVectorDouble();
            validate(feedback->pose);
            validate(feedback->joints);
            feedback->time = timestamp();
            feedback->pending = true;
        } catch (...) { feedback->error = std::current_exception(); }
        feedback->ready.notify_one();
    }), "RTDE subscribe");
    const auto start = Clock::now();
    auto lastSample = start;
    std::size_t count = 0;
    while (!interrupted && std::chrono::duration<double>(Clock::now() - start).count() < o.seconds) {
        if (!rtde.hasConnected()) throw std::runtime_error("RTDE connection lost");
        std::unique_lock<std::mutex> lock(feedback->mutex);
        feedback->ready.wait_for(lock, std::chrono::milliseconds(100), [&] {
            return feedback->pending || feedback->error;
        });
        if (feedback->error) std::rethrow_exception(feedback->error);
        if (feedback->pending) {
            auto pose = feedback->pose, joints = feedback->joints;
            const double time = feedback->time;
            feedback->pending = false;
            lock.unlock();
            lastSample = Clock::now();
            showPose(pose, joints, time);
            ++count;
        } else if (Clock::now() - lastSample > std::chrono::seconds(2)) {
            throw std::runtime_error("No RTDE feedback for 2 seconds");
        }
    }
    check(rtde.removeTopic(false, topic), "RTDE removeTopic");
    std::cerr << "Received " << count << " feedback samples\n";
    if (!interrupted && count == 0) throw std::runtime_error("No RTDE feedback received");
}

void move(RpcClient& rpc, RobotInterfacePtr robot, const Options& o)
{
    auto state = robot->getRobotState();
    auto motion = robot->getMotionControl();
    auto maths = rpc.getMath();
    Values target = o.pose;
    if (!o.delta.empty()) {
        target = state->getTcpPose();
        validate(target);
        for (std::size_t axis = 0; axis < 3; ++axis) target[axis] += o.delta[axis] / 1000;
    }
    validate(target);
    auto checkState = [&] {
        if (state->getRobotModeType() != RobotModeType::Running)
            throw std::runtime_error("Robot must be Running; power on and start first");
        if (state->getSafetyModeType() != SafetyModeType::Normal)
            throw std::runtime_error("Robot safety mode must be Normal");
    };
    auto arrived = [&](const Values& actual) {
        validate(actual);
        return maths->poseDistance(actual, target) < 1e-5 &&
               maths->poseAngleDistance(actual, target) < 1e-4;
    };
    checkState();
    if (rpc.getRuntimeMachine()->getStatus() != RuntimeState::Stopped)
        throw std::runtime_error("Stop the existing robot program before sending a target");
    if (motion->getExecId() != -1 || !state->isSteady())
        throw std::runtime_error("Robot is moving; wait until it is steady");
    const auto initial = state->getTcpPose();
    validate(initial);
    if (arrived(initial)) {
        std::cerr << "Already at target; no motion requested\n";
        return;
    }
    std::cerr << "MoveL: speed=" << o.speedMmS << " mm/s, acceleration=" << o.accelMmS2
              << " mm/s^2, timeout=" << o.timeout << " s\n";
    // Report the operator's speed setting without changing it.
    try {
        const double fraction = motion->getSpeedFraction();
        if (!std::isfinite(fraction) || fraction < 0 || fraction > 1)
            throw std::runtime_error("Invalid speed fraction returned by SDK");
        std::cerr << "Controller speed fraction=" << fraction * 100 << "% (unchanged)\n";
    } catch (const std::exception& e) {
        std::cerr << "Could not read controller speed fraction: " << e.what() << '\n';
    }
    std::cerr << "Start TCP:\n";
    showPose(initial);
    std::cerr << "Target TCP:\n";
    showPose(target);
    checkCancelled();
    try {
        // CLI units are mm; the SDK expects metres. Send one target, no blending.
        check(motion->moveLine(target, o.accelMmS2 / 1000, o.speedMmS / 1000, 0, 0), "moveLine");
        const auto start = Clock::now();
        while (std::chrono::duration<double>(Clock::now() - start).count() < o.timeout) {
            checkCancelled();
            checkState();
            const auto actual = state->getTcpPose();
            if (o.verbose) showPose(actual);
            if (motion->getExecId() == -1 && state->isSteady() && arrived(actual)) {
                if (!o.verbose) showPose(actual);
                std::cerr << "Arrived: position error < 0.01 mm, orientation error < 0.0001 rad\n";
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        const auto actual = state->getTcpPose();
        validate(actual);
        showPose(actual);
        const double dx = target[0] - actual[0], dy = target[1] - actual[1], dz = target[2] - actual[2];
        throw std::runtime_error("Timed out waiting for the target after " + std::to_string(o.timeout) +
            " s; remaining XYZ distance=" + std::to_string(std::sqrt(dx * dx + dy * dy + dz * dz) * 1000) +
            " mm; check controller speed fraction, target feedback and --timeout");
    } catch (...) {
        // Disconnecting does not cancel a motion already accepted by the controller.
        try {
            check(motion->stopLine(0.1, 0.1), "stopLine");
            std::cerr << "Requested stopLine after interruption/error\n";
        } catch (const std::exception& e) { std::cerr << "Could not request stop: " << e.what() << '\n'; }
        catch (...) { std::cerr << "Could not request stop\n"; }
        throw;
    }
}
} // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    try {
        const auto o = parse(argc, argv);
        if (o.help) {
            std::cout << "robot_communication [read|watch|move] [--ip IP] [--user USER] [--password PASSWORD]\n"
                         "  read                         Read TCP pose and joint positions (default)\n"
                         "  watch [--seconds 10] [--hz 10]  RTDE feedback, 1..50 Hz; Ctrl+C to stop\n"
                         "  move --pose X Y Z RX RY RZ [motion options]\n"
                         "  move --delta-mm DX DY DZ [motion options]\n"
                         "Motion options: --speed-mm-s 50 --accel-mm-s2 100 --timeout 60 [--verbose]\n"
                         "Speed/acceleration/timeout must be positive; --verbose prints intermediate poses.\n"
                         "Pose: base frame, metres / RPY radians. Delta: base frame, millimetres.\n"
                         "Default: 127.0.0.1, aubo / 123456. RPC: 30004, RTDE: 30010.\n"
                         "move requests motion; it does not power on or change TCP configuration or speed fraction.\n";
            return 0;
        }
        checkCancelled();
        if (o.command == "watch") watch(o);
        else {
            Session<RpcClient> session;
            check(session.client.setRequestTimeout(5000), "Set RPC timeout");
            session.open(o, 30004);
            auto& rpc = session.client;
            const auto names = rpc.getRobotNames();
            if (names.empty()) throw std::runtime_error("No robot configured on the controller");
            auto robot = rpc.getRobotInterface(names.front());
            auto state = robot->getRobotState();
            std::cerr << "Robot=" << names.front() << " mode=" << state->getRobotModeType()
                      << " safety=" << state->getSafetyModeType() << '\n';
            if (o.command == "read") showPose(state->getTcpPose(), state->getJointPositions());
            else move(rpc, robot, o);
        }
        return interrupted ? 130 : 0;
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << '\n';
        return interrupted ? 130 : 1;
    } catch (...) {
        std::cerr << "ERROR: Unknown SDK exception\n";
        return interrupted ? 130 : 1;
    }
}
