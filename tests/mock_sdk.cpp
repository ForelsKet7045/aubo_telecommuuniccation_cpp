// Offline SDK double. This DLL has no sockets and cannot contact a robot.
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <rsdef.h>

namespace {
bool moving = false;
int reads = 0;
wayPoint_S target{};
bool scenario(const char* name)
{
    const auto* value = std::getenv("AUBO_TEST_SCENARIO");
    return value && std::string(value) == name;
}
wayPoint_S initial()
{
    wayPoint_S point{};
    point.cartPos.position = {0.1, 0.2, 0.3};
    point.orientation.w = 1;
    for (int i = 0; i < ARM_DOF; ++i) point.jointpos[i] = i * 0.1;
    return point;
}
void event(const char* name) { std::cerr << "MOCK {\"call\":\"" << name << "\"}\n"; }
}

#define API extern "C" int
API rs_initialize() { return 0; }
API rs_uninitialize() { event("uninitialize"); return 0; }
API rs_create_context(RSHD* handle) { *handle = 1; return 0; }
API rs_destory_context(RSHD) { event("destroy"); return 0; }
API rs_login(RSHD, const char*, int port)
{
    std::cerr << "MOCK {\"call\":\"login\",\"port\":" << port << "}\n";
    return scenario("login-failure") ? 11 : 0;
}
API rs_logout(RSHD) { event("logout"); return 0; }
API rs_get_login_status(RSHD, bool* value) { *value = true; return 0; }
API rs_get_socket_status(RSHD, bool* value) { *value = !scenario("disconnected"); return 0; }
API rs_get_robot_state(RSHD, RobotState* state)
{
    *state = scenario("preexisting") ? RobotRunning :
        moving && scenario("timeout") ? RobotRunning :
        moving && scenario("paused") ? RobotPaused : RobotStopped;
    return 0;
}
API rs_get_current_waypoint(RSHD, wayPoint_S* point)
{
    if (++reads > 1 && scenario("read-failure")) return 12;
    *point = moving && !scenario("timeout") ? target : initial();
    return 0;
}
API rs_quaternion_to_rpy(RSHD, const Ori*, Rpy* rpy) { *rpy = {}; return 0; }
API rs_rpy_to_quaternion(RSHD, const Rpy*, Ori* ori) { *ori = {1, 0, 0, 0}; return 0; }
API rs_get_diagnosis_info(RSHD, RobotDiagnosis* diagnosis)
{
    *diagnosis = {};
    // Reproduce the site's raw input: remoteHalt=1 while all real faults are 0.
    diagnosis->remoteHalt = true;
    diagnosis->armPowerStatus = !scenario("power-off");
    diagnosis->remoteEmergency = scenario("emergency");
    diagnosis->softEmergency = scenario("soft-emergency");
    diagnosis->robotCollision = scenario("collision");
    diagnosis->forceControlMode = scenario("force-mode");
    diagnosis->armCanbusStatus = scenario("can-error") ? 4 : 0;
    diagnosis->encoderErrorStatus = scenario("encoder-error");
    diagnosis->encoderLinesError = scenario("encoder-lines-error");
    diagnosis->jointErrorStatus = scenario("joint-error");
    diagnosis->jointCollisionDetect = scenario("joint-collision") ? 2 : 0;
    diagnosis->singularityOverSpeedAlarm = scenario("singularity");
    diagnosis->robotCurrentAlarm = scenario("current-alarm");
    return 0;
}
API rs_inverse_kin(RSHD, double[], const Pos* position, const Ori* ori, wayPoint_S* solution)
{
    if (scenario("ik-failure")) return 13;
    target = initial();
    target.cartPos.position = *position;
    target.orientation = *ori;
    for (int i = 0; i < ARM_DOF; ++i) target.jointpos[i] = i + 1;
    *solution = target;
    std::cerr << std::setprecision(17) << "MOCK {\"call\":\"ik\",\"position\":["
              << position->x << ',' << position->y << ',' << position->z << "]}\n";
    return 0;
}
API rs_move_line(RSHD, double joints[ARM_DOF], bool block)
{
    event("move_line");
    // The controller expects the IK JOINT result, never XYZ/RPY as joint angles.
    for (int i = 0; i < ARM_DOF; ++i) if (joints[i] != i + 1) return 14;
    if (block || scenario("line-failure")) return 15;
    moving = true;
    return 0;
}
API rs_move_stop(RSHD) { event("stop"); moving = false; return 0; }
API rs_init_global_move_profile(RSHD) { event("configure"); return 0; }
API rs_set_base_coord(RSHD) { return 0; }
API rs_set_none_tool_kinematics_param(RSHD) { return 0; }
API rs_set_relative_offset_on_base(RSHD, const MoveRelative* offset) { return offset->ena ? 16 : 0; }
API rs_set_no_arrival_ahead(RSHD) { return 0; }
API rs_set_global_joint_maxacc(RSHD, const JointVelcAccParam*) { return 0; }
API rs_set_global_joint_maxvelc(RSHD, const JointVelcAccParam*) { return 0; }
API rs_set_global_end_max_line_acc(RSHD, double value) { return value == 0.05 ? 0 : 17; }
API rs_set_global_end_max_line_velc(RSHD, double value) { return value == 0.01 ? 0 : 18; }
API rs_set_global_end_max_angle_acc(RSHD, double) { return 0; }
API rs_set_global_end_max_angle_velc(RSHD, double) { return 0; }
