// Read-only SDK probe: no power, startup, motion, stop, or configuration APIs.
#define NOMINMAX
#include <Windows.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <aubo_sdk/rpc.h>
#include <aubo_sdk/rtde.h>
#include <rsdef.h>
#include <serviceinterface.h>

using namespace arcs::aubo_sdk;

namespace {
struct Options {
    std::string api, ip = "169.254.138.6";
    int port = 8899, timeout = 3000;
};

int positiveInteger(const std::string& text, int max)
{
    std::size_t used = 0;
    const int value = std::stoi(text, &used);
    if (used != text.size() || value < 1 || value > max)
        throw std::invalid_argument("Invalid positive integer: " + text);
    return value;
}

Options parse(int argc, char** argv)
{
    if (argc < 2) throw std::invalid_argument("Specify rpc, rs or service");
    Options o;
    o.api = argv[1];
    if (o.api != "rpc" && o.api != "rs" && o.api != "service")
        throw std::invalid_argument("Specify rpc, rs or service");
    for (int i = 2; i < argc; ++i) {
        const std::string key = argv[i];
        if (++i >= argc) throw std::invalid_argument("Missing value after " + key);
        const std::string value = argv[i];
        if (key == "--ip") o.ip = value;
        else if (key == "--port") o.port = positiveInteger(value, 65535);
        else if (key == "--timeout-ms") o.timeout = positiveInteger(value, 60000);
        else throw std::invalid_argument("Unknown option: " + key);
    }
    return o;
}

void result(const char* stage, int code)
{
    std::cout << "stage=" << stage << " result=" << code << '\n';
    if (code != 0) throw std::runtime_error(std::string(stage) + " returned " + std::to_string(code));
}

void sdkLog(int level, const char*, int, const std::string& text)
{
    // Avoid trace/debug logging, which may include request credentials.
    if (level <= 3) std::cerr << "SDK[" << level << "]: " << text << '\n';
}

void showWaypoint(const aubo_robot_namespace::wayPoint_S& point)
{
    const auto& p = point.cartPos.position;
    const auto& q = point.orientation;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        throw std::runtime_error("Invalid position feedback");
    std::cout << std::setprecision(17) << "position_m=[" << p.x << ',' << p.y << ',' << p.z
              << "] quaternion_wxyz=[" << q.w << ',' << q.x << ',' << q.y << ',' << q.z
              << "] joints_rad=[";
    for (int i = 0; i < 6; ++i) {
        if (!std::isfinite(point.jointpos[i])) throw std::runtime_error("Invalid joint feedback");
        if (i) std::cout << ',';
        std::cout << point.jointpos[i];
    }
    std::cout << "]\nstage=read_completed result=0\n";
}

int probeRpc(const Options& o)
{
    struct Session {
        RpcClient client;
        ~Session()
        {
            try { if (client.hasLogined()) client.logout(); } catch (...) {}
            try { if (client.hasConnected()) client.disconnect(); } catch (...) {}
        }
    } session;
    auto& rpc = session.client;
    rpc.setLogHandler(sdkLog);
    result("setRequestTimeout", rpc.setRequestTimeout(o.timeout));
    std::cout << "stage=connect begin\n";
    result("connect", rpc.connect(o.ip, o.port));
    std::cout << "hasConnected=" << rpc.hasConnected() << '\n';
    std::cout << "stage=login begin\n";
    result("login", rpc.login("aubo", "123456"));
    std::cout << "hasLogined=" << rpc.hasLogined() << '\n';
    if (!rpc.hasLogined()) throw std::runtime_error("SDK did not establish a logged-in session");
    std::cout << "stage=getRobotNames begin\n";
    const auto names = rpc.getRobotNames();
    if (names.empty()) throw std::runtime_error("No robots returned");
    const auto state = rpc.getRobotInterface(names.front())->getRobotState();
    const auto pose = state->getTcpPose(), joints = state->getJointPositions();
    if (pose.size() != 6 || joints.size() != 6) throw std::runtime_error("Invalid feedback sizes");
    std::cout << "robot=" << names.front() << " pose_m_rad=[";
    for (std::size_t i = 0; i < pose.size(); ++i) {
        if (!std::isfinite(pose[i]) || !std::isfinite(joints[i]))
            throw std::runtime_error("Invalid pose/joint feedback");
        if (i) std::cout << ',';
        std::cout << std::setprecision(17) << pose[i];
    }
    std::cout << "]\nstage=read_completed result=0\n";
    return 0;
}

int probeRs(const Options& o)
{
    struct Session {
        RSHD handle{};
        bool initialized = false, created = false, loggedIn = false;
        ~Session()
        {
            if (loggedIn) rs_logout(handle);
            if (created) rs_destory_context(handle);
            if (initialized) rs_uninitialize();
        }
    } session;
    result("rs_initialize", rs_initialize());
    session.initialized = true;
    result("rs_create_context", rs_create_context(&session.handle));
    session.created = true;
    std::cout << "stage=rs_login begin\n";
    result("rs_login", rs_login(session.handle, o.ip.c_str(), o.port));
    session.loggedIn = true;
    bool loggedIn = false;
    result("rs_get_login_status", rs_get_login_status(session.handle, &loggedIn));
    if (!loggedIn) throw std::runtime_error("SDK did not establish a logged-in session");
    aubo_robot_namespace::wayPoint_S point{};
    result("rs_get_current_waypoint", rs_get_current_waypoint(session.handle, &point));
    showWaypoint(point);
    return 0;
}

int probeService(const Options& o)
{
    struct Session {
        ServiceInterface client;
        bool loggedIn = false;
        ~Session() { try { if (loggedIn) client.robotServiceLogout(); } catch (...) {} }
    } session;
    auto& service = session.client;
    std::cout << "sdk_version=" << service.getVersion() << '\n';
    if (const auto rpc = service.getRpcClient()) {
        rpc->setLogHandler(sdkLog);
        result("setRequestTimeout", rpc->setRequestTimeout(o.timeout));
    }
    if (const auto rtde = service.getRtdeClient()) rtde->setLogHandler(sdkLog);
    std::cout << "stage=robotServiceLogin begin\n";
    result("robotServiceLogin", service.robotServiceLogin(o.ip.c_str(), o.port, "aubo", "123456"));
    session.loggedIn = true;
    bool connected = false;
    service.robotServiceGetConnectStatus(connected);
    std::cout << "serviceConnected=" << connected << '\n';
    if (!connected) throw std::runtime_error("ServiceInterface reports disconnected");
    aubo_robot_namespace::wayPoint_S point{};
    result("robotServiceGetCurrentWaypointInfo", service.robotServiceGetCurrentWaypointInfo(point));
    showWaypoint(point);
    return 0;
}
} // namespace

int main(int argc, char** argv)
{
    std::cout << std::unitbuf;
    std::cerr << std::unitbuf;
    try {
        if (argc == 2 && std::string(argv[1]) == "--help") {
            std::cout << "official_sdk_probe rpc|rs|service [--ip IP] [--port 8899] [--timeout-ms 3000]\n"
                         "Read-only: connect, login, read pose/joints. No motion or configuration calls.\n"
                         "rs/service should also be run under an external process timeout.\n";
            return 0;
        }
        const auto o = parse(argc, argv);
        wchar_t modulePath[32768]{};
        if (GetModuleFileNameW(GetModuleHandleW(L"aubo_sdk.dll"), modulePath, 32768))
            std::wcout << L"loaded_dll=" << modulePath << L'\n';
        std::cout << "api=" << o.api << " endpoint=" << o.ip << ':' << o.port << '\n';
        if (o.api == "rpc") return probeRpc(o);
        if (o.api == "rs") return probeRs(o);
        return probeService(o);
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << '\n';
        return 1;
    } catch (...) {
        std::cerr << "ERROR: Unknown SDK exception\n";
        return 2;
    }
}
