#include "Kinematics.h"
#include <math.h>
#include "../Motion/MotionPlanner.h"

Joints Kinematics::inverse(Pose target) {
    Joints j = {0,0,0,0,0};
    j.j1 = atan2f(target.y, target.x) * RAD_TO_DEG;

    float pitch_rad = target.pitch * DEG_TO_RAD;
    float r_target = sqrtf(target.x * target.x + target.y * target.y);
    float z_target = target.z;

    float r_wc = r_target - L3 * cosf(pitch_rad);
    float z_wc = z_target + L3 * sinf(pitch_rad);

    float z_rel = z_wc - BASE_H;
    float L2_eff = sqrtf(L2_PAR*L2_PAR + L2_PERP*L2_PERP);
    float gamma = atan2f(L2_PERP, L2_PAR);

    float D2 = r_wc*r_wc + z_rel*z_rel;
    float C3 = (D2 - L1*L1 - L2_eff*L2_eff) / (2.0f * L1 * L2_eff);
    C3 = constrain(C3, -1.0f, 1.0f);

    float j3_eff = acosf(C3);
    float alpha = atan2f(L2_eff * sinf(j3_eff), L1 + L2_eff * C3);
    float beta = atan2f(r_wc, z_rel);

    j.j2 = (beta - alpha) * RAD_TO_DEG;
    j.j3 = (j3_eff - gamma) * RAD_TO_DEG;

    float ang34 = j.j2 + j.j3;
    j.j4 = target.pitch - ang34 + 90.0f;
    j.j5 = target.roll;

    return j;
}

Pose Kinematics::forward(Joints j) {
    Pose p;
    float j1r = j.j1 * DEG_TO_RAD;
    float j2r = j.j2 * DEG_TO_RAD;
    float j3r = j.j3 * DEG_TO_RAD;

    float j3_reach = L1 * sinf(j2r);
    float j3_z     = BASE_H + L1 * cosf(j2r);

    float ang34    = j2r + j3r;
    float j4_reach = j3_reach + L2_PAR * sinf(ang34) + L2_PERP * cosf(ang34);
    float j4_z     = j3_z + L2_PAR * cosf(ang34) - L2_PERP * sinf(ang34);

    float pitch_rad = (j.j4 + (j.j2 + j.j3) - 90.0f) * DEG_TO_RAD;
    float r_tip = j4_reach + L3 * cosf(pitch_rad);
    p.z = j4_z - L3 * sinf(pitch_rad);
    p.x = r_tip * cosf(j1r);
    p.y = r_tip * sinf(j1r);
    p.pitch = j.j4 + (j.j2 + j.j3) - 90.0f;
    p.roll = j.j5;
    return p;
}

bool Kinematics::checkWorkspace(Joints j) {
    if (j.j1 < LIMIT_J1_MIN || j.j1 > LIMIT_J1_MAX) return false;
    if (j.j2 < LIMIT_J2_MIN || j.j2 > LIMIT_J2_MAX) return false;
    if (j.j3 < LIMIT_J3_MIN || j.j3 > LIMIT_J3_MAX) return false;
    if (j.j4 < LIMIT_J4_MIN || j.j4 > LIMIT_J4_MAX) return false;
    if (j.j5 < LIMIT_J5_MIN || j.j5 > LIMIT_J5_MAX) return false;
    Pose p = forward(j);
    if ((p.x * p.x + p.y * p.y) < EXCL_R2 && p.z < BASE_H) return false;
    if (p.z < FLOOR_CLR) return false;
    return true;
}

void Kinematics::movL(Pose target, unsigned long duration, class MotionPlanner& planner) {
    Joints currentJ = planner.getCurrentJoints();
    Pose currentP = forward(currentJ);

    float dx = target.x - currentP.x;
    float dy = target.y - currentP.y;
    float dz = target.z - currentP.z;
    float dp = target.pitch - currentP.pitch;
    float dr = target.roll - currentP.roll;

    float dist = sqrtf(dx*dx + dy*dy + dz*dz);
    int segs = (int)(dist / 5.0f);
    segs = constrain(segs, 2, 20);
    unsigned long segDur = duration / segs;

    for (int i = 1; i <= segs; i++) {
        float t = (float)i / segs;
        Pose stepP = {
            currentP.x + dx * t,
            currentP.y + dy * t,
            currentP.z + dz * t,
            currentP.pitch + dp * t,
            currentP.roll + dr * t
        };
        planner.addWaypoint({inverse(stepP), segDur});
    }
}
