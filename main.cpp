#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rsdef.h>

using namespace aubo_robot_namespace;
using Clock = std::chrono::steady_clock;
using Values = std::vector<double>;

// Fixed-size POD values cross the ABI boundary to the MinGW-built vendor DLL.
static_assert(sizeof(RSHD) == 2, "Unexpected SDK handle ABI");
static_assert(sizeof(wayPoint_S) == 104, "Unexpected SDK waypoint ABI");
static_assert(sizeof(Ori) == 32 && sizeof(Rpy) == 24, "Unexpected SDK pose ABI");
static_assert(sizeof(RobotDiagnosis) == 52, "Unexpected SDK diagnosis ABI");

namespace {
volatile std::sig_atomic_t interrupted = 0;
void onSignal(int) { interrupted = 1; }

void check(int result, const char* action)
{
    if (result != RS_SUCC)
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

int integer(const std::string& text, int low, int high, const char* option)
{
    const double value = number(text);
    if (value < low || value > high || std::floor(value) != value)
        throw std::invalid_argument(std::string(option) + " must be an integer from " +
                                    std::to_string(low) + " to " + std::to_string(high));
    return static_cast<int>(value);
}

struct Options {
    std::string command = "read", ip = "127.0.0.1";
    int port = 8899, hz = 10;
    double seconds = 10, timeout = 30;
    Values pose, delta;
    bool help = false, dryRun = false;
};

Options parse(int argc, char** argv)
{
    Options o;
    bool commandSet = false, watchOption = false, moveOption = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if (++i >= argc) throw std::invalid_argument("Missing value after " + arg);
            return argv[i];
        };
        if (arg == "--help" || arg == "-h") { o.help = true; return o; }
        if (arg == "read" || arg == "watch" || arg == "move" || arg == "diagnose") {
            if (commandSet) throw std::invalid_argument("Specify one command");
            o.command = arg;
            commandSet = true;
        } else if (arg == "--ip") o.ip = next();
        else if (arg == "--port") o.port = integer(next(), 1, 65535, "--port");
        else if (arg == "--seconds") { o.seconds = number(next()); watchOption = true; }
        else if (arg == "--hz") { o.hz = integer(next(), 1, 50, "--hz"); watchOption = true; }
        else if (arg == "--timeout") { o.timeout = number(next()); moveOption = true; }
        else if (arg == "--dry-run") { o.dryRun = true; moveOption = true; }
        else if (arg == "--pose" || arg == "--delta-mm") {
            if (!o.pose.empty() || !o.delta.empty())
                throw std::invalid_argument("Specify exactly one --pose or --delta-mm");
            auto& target = arg == "--pose" ? o.pose : o.delta;
            const int count = arg == "--pose" ? 6 : 3;
            for (int axis = 0; axis < count; ++axis) target.push_back(number(next()));
            moveOption = true;
        } else if (arg == "--user" || arg == "--password") {
            throw std::invalid_argument("The legacy rs_login API does not accept --user/--password");
        } else throw std::invalid_argument("Unknown argument: " + arg);
    }
    if (o.ip.empty()) throw std::invalid_argument("--ip must not be empty");
    if (o.seconds <= 0 || o.timeout <= 0)
        throw std::invalid_argument("--seconds and --timeout must be positive");
    if (watchOption && o.command != "watch")
        throw std::invalid_argument("--seconds and --hz require watch");
    if (moveOption && o.command != "move")
        throw std::invalid_argument("--pose, --delta-mm, --timeout and --dry-run require move");
    if (o.command == "move" && o.pose.empty() && o.delta.empty())
        throw std::invalid_argument("move requires --pose or --delta-mm");
    return o;
}

class Session {
public:
    RSHD handle = 0;
    Session() = default;
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;

    void open(const Options& o)
    {
        check(rs_initialize(), "rs_initialize");
        initialized_ = true;
        check(rs_create_context(&handle), "rs_create_context");
        created_ = true;
        std::cerr << "Connecting to " << o.ip << ':' << o.port << " (legacy AUBO SDK)\n";
        check(rs_login(handle, o.ip.c_str(), o.port), "rs_login");
        loggedIn_ = true;
        ensureConnected();
        std::cerr << "Connected and logged in\n";
    }

    void ensureConnected() const
    {
        bool loggedIn = false, connected = false;
        check(rs_get_login_status(handle, &loggedIn), "rs_get_login_status");
        check(rs_get_socket_status(handle, &connected), "rs_get_socket_status");
        if (!loggedIn || !connected) throw std::runtime_error("Controller connection lost");
    }

    ~Session()
    {
        if (loggedIn_) rs_logout(handle);
        if (created_) rs_destory_context(handle); // Spelling is part of the vendor API.
        if (initialized_) rs_uninitialize();
    }

private:
    bool initialized_ = false, created_ = false, loggedIn_ = false;
};

double timestamp()
{
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void validate(const wayPoint_S& waypoint)
{
    const auto& p = waypoint.cartPos.position;
    const auto& q = waypoint.orientation;
    const double norm = std::hypot(std::hypot(q.w, q.x), std::hypot(q.y, q.z));
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3 ||
        !std::all_of(std::begin(waypoint.jointpos), std::end(waypoint.jointpos),
                     [](double value) { return std::isfinite(value); }))
        throw std::runtime_error("Invalid pose/joint feedback from controller");
}

wayPoint_S readWaypoint(const Session& session)
{
    session.ensureConnected();
    wayPoint_S waypoint{};
    check(rs_get_current_waypoint(session.handle, &waypoint), "rs_get_current_waypoint");
    validate(waypoint);
    return waypoint;
}

void printArray(const char* key, const Values& values)
{
    std::cout << ",\"" << key << "\":[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i) std::cout << ',';
        std::cout << values[i];
    }
    std::cout << ']';
}

// Legacy waypoints are FLANGE poses in the base frame. They do not apply the
// TCP/tool selected on the teach pendant.
void showPose(RSHD handle, const wayPoint_S& waypoint)
{
    Rpy rpy{};
    check(rs_quaternion_to_rpy(handle, &waypoint.orientation, &rpy), "rs_quaternion_to_rpy");
    if (!std::isfinite(rpy.rx) || !std::isfinite(rpy.ry) || !std::isfinite(rpy.rz))
        throw std::runtime_error("Invalid RPY orientation");
    const auto& p = waypoint.cartPos.position;
    const auto& q = waypoint.orientation;
    Values joints(std::begin(waypoint.jointpos), std::end(waypoint.jointpos));
    Values degrees = joints;
    for (auto& value : degrees) value *= 180 / std::acos(-1.0);
    std::cout << std::setprecision(17) << "{\"time\":" << timestamp()
              << ",\"frame\":\"base\",\"pose_reference\":\"flange\"";
    printArray("pose_m_rad", {p.x, p.y, p.z, rpy.rx, rpy.ry, rpy.rz});
    printArray("xyz_mm", {p.x * 1000, p.y * 1000, p.z * 1000});
    printArray("quaternion_wxyz", {q.w, q.x, q.y, q.z});
    printArray("joints_rad", joints);
    printArray("joints_deg", degrees);
    std::cout << '}' << std::endl;
}

RobotState robotState(RSHD handle)
{
    RobotState state = RobotStopped;
    check(rs_get_robot_state(handle, &state), "rs_get_robot_state");
    return state;
}

void watch(const Session& session, const Options& o)
{
    const auto start = Clock::now();
    const auto period = std::chrono::duration<double>(1.0 / o.hz);
    auto next = start;
    std::size_t count = 0;
    // --hz controls polling frequency, not RTDE streaming.
    while (!interrupted && std::chrono::duration<double>(Clock::now() - start).count() < o.seconds) {
        showPose(session.handle, readWaypoint(session));
        ++count;
        next += std::chrono::duration_cast<Clock::duration>(period);
        if (next < Clock::now()) next = Clock::now();
        while (!interrupted && Clock::now() < next)
            std::this_thread::sleep_until(std::min(next, Clock::now() + std::chrono::milliseconds(50)));
    }
    std::cerr << "Received " << count << " legacy feedback samples\n";
}

RobotDiagnosis readDiagnosis(const Session& session)
{
    session.ensureConnected();
    RobotDiagnosis diagnosis{};
    check(rs_get_diagnosis_info(session.handle, &diagnosis), "rs_get_diagnosis_info");
    return diagnosis;
}

using DiagnosisFlags = std::vector<std::pair<const char*, int>>;

DiagnosisFlags blockingFlags(const RobotDiagnosis& d)
{
    // remoteHalt is a raw remote-power-off input, NOT an emergency/fault enum.
    // Its active polarity is not specified by this legacy header. On the V4.5
    // controller it reads 1 while pendant motion is available. Report it as-is;
    // do not infer a fault or invert its polarity. Controller interlocks remain
    // authoritative; explicit emergency/collision/hardware faults still block.
    DiagnosisFlags flags = {
        {"armPowerStatus", d.armPowerStatus},
        {"softEmergency", d.softEmergency}, {"remoteEmergency", d.remoteEmergency},
        {"robotCollision", d.robotCollision}, {"forceControlMode", d.forceControlMode},
        {"armCanbusStatus", d.armCanbusStatus}, {"encoderErrorStatus", d.encoderErrorStatus},
        {"encoderLinesError", d.encoderLinesError}, {"jointErrorStatus", d.jointErrorStatus},
        {"jointCollisionDetect", d.jointCollisionDetect},
        {"singularityOverSpeedAlarm", d.singularityOverSpeedAlarm},
        {"robotCurrentAlarm", d.robotCurrentAlarm}
    };
    flags.erase(std::remove_if(flags.begin(), flags.end(), [](const auto& flag) {
        return std::string(flag.first) == "armPowerStatus" ? flag.second != 0 : flag.second == 0;
    }), flags.end());
    return flags;
}

void printBlockers(const DiagnosisFlags& flags)
{
    std::cout << ",\"motion_blockers\":[";
    for (std::size_t i = 0; i < flags.size(); ++i) {
        if (i) std::cout << ',';
        std::cout << '"' << flags[i].first << '=' << flags[i].second << '"';
    }
    std::cout << ']';
}

void diagnose(const Session& session)
{
    const auto d = readDiagnosis(session);
    const auto flags = blockingFlags(d);
    std::cout << std::setprecision(17) << "{\"time\":" << timestamp()
              << ",\"robot_motion_state\":" << static_cast<int>(robotState(session.handle))
              << ",\"diagnosis\":{";
    const DiagnosisFlags rawFlags = {
        {"armPowerStatus", d.armPowerStatus}, {"remoteHalt_raw", d.remoteHalt},
        {"softEmergency", d.softEmergency}, {"remoteEmergency", d.remoteEmergency},
        {"robotCollision", d.robotCollision}, {"forceControlMode", d.forceControlMode},
        {"armCanbusStatus", d.armCanbusStatus}, {"encoderErrorStatus", d.encoderErrorStatus},
        {"encoderLinesError", d.encoderLinesError}, {"jointErrorStatus", d.jointErrorStatus},
        {"jointCollisionDetect", d.jointCollisionDetect},
        {"singularityOverSpeedAlarm", d.singularityOverSpeedAlarm},
        {"robotCurrentAlarm", d.robotCurrentAlarm}, {"brakeStuats_raw", d.brakeStuats},
        {"toolIoError", d.toolIoError}
    };
    for (std::size_t i = 0; i < rawFlags.size(); ++i) {
        if (i) std::cout << ',';
        std::cout << '"' << rawFlags[i].first << "\":" << rawFlags[i].second;
    }
    std::cout << '}';
    printBlockers(flags);
    std::cout << '}' << std::endl;
    std::cerr << "remoteHalt_raw=" << d.remoteHalt
              << " is a raw power-off input; 1 alone is not classified as a fault.\n"
                 "Diagnosis does not grant motion permission; controller interlocks still apply.\n";
}

void checkMotionSafety(const Session& session)
{
    const auto d = readDiagnosis(session);
    const auto flags = blockingFlags(d);
    if (!flags.empty()) {
        std::ostringstream message;
        message << "Motion blocked by diagnosis: ";
        for (std::size_t i = 0; i < flags.size(); ++i) {
            if (i) message << ", ";
            message << flags[i].first << '=' << flags[i].second;
        }
        message << "; remoteHalt_raw=" << d.remoteHalt;
        if (!d.armPowerStatus) message << "; power on/start from the teach pendant first";
        throw std::runtime_error(message.str());
    }
}

bool arrived(const wayPoint_S& actual, const Pos& target, const Ori& orientation)
{
    const auto& p = actual.cartPos.position;
    const auto& q = actual.orientation;
    const double positionError = std::hypot(std::hypot(p.x - target.x, p.y - target.y), p.z - target.z);
    const double dot = q.w * orientation.w + q.x * orientation.x + q.y * orientation.y + q.z * orientation.z;
    const double angleError = 2 * std::acos(std::clamp(std::abs(dot), 0.0, 1.0));
    return positionError < 1e-5 && angleError < 1e-4;
}

void move(const Session& session, const Options& o)
{
    checkMotionSafety(session);
    if (robotState(session.handle) != RobotStopped)
        throw std::runtime_error("Stop the existing robot motion/program before sending a target");
    const auto current = readWaypoint(session);
    Pos target = current.cartPos.position;
    Ori orientation = current.orientation;
    if (!o.pose.empty()) {
        target = {o.pose[0], o.pose[1], o.pose[2]};
        const Rpy rpy{o.pose[3], o.pose[4], o.pose[5]};
        check(rs_rpy_to_quaternion(session.handle, &rpy, &orientation), "rs_rpy_to_quaternion");
    } else {
        target.x += o.delta[0] / 1000;
        target.y += o.delta[1] / 1000;
        target.z += o.delta[2] / 1000;
    }
    if (!std::isfinite(target.x) || !std::isfinite(target.y) || !std::isfinite(target.z))
        throw std::invalid_argument("Target position overflow");
    if (arrived(current, target, orientation)) {
        std::cerr << "Already at target; no motion requested\n";
        return;
    }

    // Legacy MoveL takes JOINT targets; solve IK near the current configuration.
    wayPoint_S solution{};
    double seed[ARM_DOF];
    std::copy(std::begin(current.jointpos), std::end(current.jointpos), seed);
    check(rs_inverse_kin(session.handle, seed, &target, &orientation, &solution), "rs_inverse_kin");
    validate(solution);
    if (!arrived(solution, target, orientation))
        throw std::runtime_error("Inverse kinematics did not resolve the requested flange pose");

    if (o.dryRun) {
        std::cout << std::setprecision(17)
                  << "{\"dry_run\":true,\"frame\":\"base\",\"pose_reference\":\"flange\"";
        printArray("target_position_m", {target.x, target.y, target.z});
        printArray("target_quaternion_wxyz", {orientation.w, orientation.x, orientation.y, orientation.z});
        printArray("ik_joints_rad", Values(std::begin(solution.jointpos), std::end(solution.jointpos)));
        std::cout << '}' << std::endl;
        std::cerr << "Dry run: IK valid; no motion or motion-configuration command sent.\n";
        return;
    }

    check(rs_init_global_move_profile(session.handle), "rs_init_global_move_profile");
    check(rs_set_base_coord(session.handle), "rs_set_base_coord");
    check(rs_set_none_tool_kinematics_param(session.handle), "rs_set_none_tool_kinematics_param");
    MoveRelative offset{};
    offset.relativeOri.w = 1;
    check(rs_set_relative_offset_on_base(session.handle, &offset), "rs_set_relative_offset_on_base");
    check(rs_set_no_arrival_ahead(session.handle), "rs_set_no_arrival_ahead");
    JointVelcAccParam jointLimits{};
    std::fill(std::begin(jointLimits.jointPara), std::end(jointLimits.jointPara), 0.2);
    check(rs_set_global_joint_maxacc(session.handle, &jointLimits), "rs_set_global_joint_maxacc");
    check(rs_set_global_joint_maxvelc(session.handle, &jointLimits), "rs_set_global_joint_maxvelc");
    check(rs_set_global_end_max_line_acc(session.handle, 0.05), "rs_set_global_end_max_line_acc");
    check(rs_set_global_end_max_line_velc(session.handle, 0.01), "rs_set_global_end_max_line_velc");
    check(rs_set_global_end_max_angle_acc(session.handle, 0.1), "rs_set_global_end_max_angle_acc");
    check(rs_set_global_end_max_angle_velc(session.handle, 0.1), "rs_set_global_end_max_angle_velc");
    checkMotionSafety(session);
    if (robotState(session.handle) != RobotStopped)
        throw std::runtime_error("Robot started moving before the target could be sent");
    checkCancelled();
    try {
        check(rs_move_line(session.handle, solution.jointpos, false), "rs_move_line");
        const auto start = Clock::now();
        while (std::chrono::duration<double>(Clock::now() - start).count() < o.timeout) {
            checkCancelled();
            checkMotionSafety(session);
            const auto actual = readWaypoint(session);
            showPose(session.handle, actual);
            const auto state = robotState(session.handle);
            if (state == RobotStopped && arrived(actual, target, orientation)) {
                std::cerr << "Arrived: flange position error < 0.01 mm, orientation error < 0.0001 rad\n";
                return;
            }
            if (state == RobotPaused) throw std::runtime_error("Robot motion was paused");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        throw std::runtime_error("Timed out waiting for the target");
    } catch (...) {
        const int result = rs_move_stop(session.handle);
        std::cerr << "Requested rs_move_stop after interruption/error; return code " << result << '\n';
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
            std::cout << "robot_communication [read|watch|diagnose|move] [--ip IP] [--port 8899]\n"
                         "  read                          Read flange pose and joint positions (default)\n"
                         "  watch [--seconds 10] [--hz 10]  Poll legacy feedback at 1..50 Hz; Ctrl+C to stop\n"
                         "  diagnose                      Read raw diagnosis and application motion blockers\n"
                         "  move --pose X Y Z RX RY RZ [--timeout 30]\n"
                         "  move --delta-mm DX DY DZ [--timeout 30]\n"
                         "  move ... --dry-run             Check diagnosis and IK without sending motion/configuration\n"
                         "Pose: FLANGE in base frame, metres / RPY radians. Delta: base frame, millimetres.\n"
                         "The teach pendant's selected TCP is not applied.\n"
                         "Default: 127.0.0.1:8899. Legacy rs_login does not take user/password arguments.\n"
                         "read/watch/diagnose do not power on or move the robot. move requires power on/start first.\n";
            return 0;
        }
        checkCancelled();
        Session session;
        session.open(o);
        std::cerr << "Robot motion state=" << static_cast<int>(robotState(session.handle))
                  << " (0=Stopped, 1=Running, 2=Paused, 3=Resumed); pose reference=flange\n";
        checkCancelled();
        if (o.command == "read") showPose(session.handle, readWaypoint(session));
        else if (o.command == "watch") watch(session, o);
        else if (o.command == "diagnose") diagnose(session);
        else move(session, o);
        return interrupted ? 130 : 0;
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << '\n';
        return interrupted ? 130 : 1;
    } catch (...) {
        std::cerr << "ERROR: Unknown SDK exception\n";
        return interrupted ? 130 : 1;
    }
}
