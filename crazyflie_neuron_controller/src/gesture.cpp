#include "gesture.h"
#include "bone.h"
#include "commands.h"


double Gesture::recognize(const bones_t &bones) const noexcept {
    double result = 0.0;

    for (size_t i = 0; i < m_standardBones.size(); ++i) {
        const StandardBone &sb = m_standardBones[i];
        tf::Quaternion q = bones[i]->getRotation();

        double delta_x = std::fabs(q.x() - sb.mean.x);
        double delta_y = std::fabs(q.y() - sb.mean.y);
        double delta_w = std::fabs(q.w() - sb.mean.w);

        if (delta_x < 0.03) delta_x = 0.0;
        if (delta_y < 0.03) delta_y = 0.0;
        if (delta_w < 0.03) delta_w = 0.0;

        result += std::max(1.0 - (delta_x) / (4.0 * sb.std.x), 0.0);
        result += std::max(1.0 - (delta_y) / (4.0 * sb.std.y), 0.0);
        result += std::max(1.0 - (delta_w) / (4.0 * sb.std.w), 0.0);
    }

    return result / (3.0 * static_cast<double>(bones.size()));
}


// ################################################
// #################### EMPTY #####################
// ################################################
Empty::Empty() {
    m_standardBones.push_back({.mean = { 0.0838,  0.2489, 0.9130}, .std = {0.0166, 0.0299, 0.0183}});
    m_standardBones.push_back({.mean = { 0.0812,  0.1010, 0.7672}, .std = {0.0141, 0.0102, 0.0772}});
    m_standardBones.push_back({.mean = { 0.0000,  0.0000, 0.7886}, .std = {0.0001, 0.0001, 0.0560}});
    m_standardBones.push_back({.mean = {-0.0824, -0.1009, 0.7715}, .std = {0.0081, 0.0065, 0.0491}});
    m_standardBones.push_back({.mean = {-0.1252, -0.1764, 0.7957}, .std = {0.0085, 0.0063, 0.0286}});
}

Command Empty::getCommand() const noexcept {
    return commands::empty;
}


// ################################################
// #################### TAKEOFF ###################
// ################################################
Takeoff::Takeoff() {
    m_standardBones.push_back({.mean = { 0.1070,  0.3799, 0.8844}, .std = {0.0135, 0.0279, 0.0122}});
    m_standardBones.push_back({.mean = { 0.0665,  0.1123, 0.8531}, .std = {0.0102, 0.0087, 0.0617}});
    m_standardBones.push_back({.mean = { 0.0000,  0.0000, 0.8143}, .std = {0.0001, 0.0001, 0.0097}});
    m_standardBones.push_back({.mean = {-0.0763, -0.1059, 0.7654}, .std = {0.0036, 0.0030, 0.0228}});
    m_standardBones.push_back({.mean = {-0.0167, -0.2158, 0.9734}, .std = {0.0095, 0.0011, 0.0048}});
}

Command Takeoff::getCommand() const noexcept {
    return commands::takeoff;
}


// ################################################
// #################### LANDING ###################
// ################################################
Landing::Landing() {
    m_standardBones.push_back({.mean = { 0.0318,  0.0739, 0.9154}, .std = {0.0342, 0.0332, 0.0157}});
    m_standardBones.push_back({.mean = { 0.0191,  0.1291, 0.9806}, .std = {0.0250, 0.0072, 0.0300}});
    m_standardBones.push_back({.mean = { 0.0000,  0.0000, 0.9989}, .std = {0.0001, 0.0001, 0.0128}});
    m_standardBones.push_back({.mean = {-0.0071, -0.1303, 0.9751}, .std = {0.0100, 0.0018, 0.0136}});
    m_standardBones.push_back({.mean = {-0.0226, -0.2153, 0.9709}, .std = {0.0274, 0.0052, 0.0235}});
}

Command Landing::getCommand() const noexcept {
    return commands::landing;
}


// ################################################
// #################### UPWARD ####################
// ################################################
Upward::Upward() {
}

Command Upward::getCommand() const noexcept {
    return commands::upward;
}


// ################################################
// ################### DOWNWARD ###################
// ################################################
Downward::Downward() {
}

Command Downward::getCommand() const noexcept {
    return commands::downward;
}


// ################################################
// ################### LEFTWARD ###################
// ################################################
Leftward::Leftward() {
    m_standardBones.push_back({.mean = { 0.1141,  0.3912, 0.8766}, .std = {0.0137, 0.0199, 0.0121}});
    m_standardBones.push_back({.mean = { 0.0130,  0.1298, 0.9862}, .std = {0.0071, 0.0006, 0.0044}});
    m_standardBones.push_back({.mean = { 0.0000,  0.0000, 0.9989}, .std = {0.0001, 0.0001, 0.0031}});
    m_standardBones.push_back({.mean = {-0.0083, -0.1303, 0.9873}, .std = {0.0080, 0.0025, 0.0192}});
    m_standardBones.push_back({.mean = {-0.1287, -0.1740, 0.7849}, .std = {0.0094, 0.0058, 0.0261}});
}

Command Leftward::getCommand() const noexcept {
    return commands::leftward;
}


// ################################################
// ################## RIGHTWARD ###################
// ################################################
Rightward::Rightward() {
    m_standardBones.push_back({.mean = { 0.1073,  0.3733, 0.8856}, .std = {0.0311, 0.0338, 0.0109}});
    m_standardBones.push_back({.mean = { 0.0139,  0.1298, 0.9858}, .std = {0.0284, 0.0035, 0.0161}});
    m_standardBones.push_back({.mean = { 0.0000,  0.0000, 0.9976}, .std = {0.0001, 0.0001, 0.0018}});
    m_standardBones.push_back({.mean = {-0.0062, -0.1304, 0.9903}, .std = {0.0087, 0.0006, 0.0045}});
    m_standardBones.push_back({.mean = {-0.0204, -0.2155, 0.9719}, .std = {0.0190, 0.0012, 0.0053}});

}

Command Rightward::getCommand() const noexcept {
    return commands::rightward;
}


// ################################################
// ################### FORWARD ####################
// ################################################
Forward::Forward() {
    m_standardBones.push_back({.mean = { 0.0913,  0.4433, 0.8733}, .std = {0.0155, 0.0123, 0.0093}});
    m_standardBones.push_back({.mean = { 0.0248,  0.1285, 0.9762}, .std = {0.0052, 0.0007, 0.0054}});
    m_standardBones.push_back({.mean = { 0.0000,  0.0000, 0.7597}, .std = {0.0001, 0.0001, 0.0920}});
    m_standardBones.push_back({.mean = {-0.0913, -0.0933, 0.7572}, .std = {0.0031, 0.0026, 0.0200}});
    m_standardBones.push_back({.mean = {-0.1222, -0.1786, 0.8056}, .std = {0.0128, 0.0087, 0.0392}});

}

Command Forward::getCommand() const noexcept {
    return commands::forward;
}


// ################################################
// ################## BACKWARD ####################
// ################################################
Backward::Backward() {
    m_standardBones.push_back({.mean = { 0.1442,  0.3022, 0.8505}, .std = {0.0127, 0.0151, 0.0111}});
    m_standardBones.push_back({.mean = { 0.0131,  0.1298, 0.9862}, .std = {0.0072, 0.0007, 0.0057}});
    m_standardBones.push_back({.mean = { 0.0000,  0.0000, 0.9994}, .std = {0.0001, 0.0001, 0.0053}});
    m_standardBones.push_back({.mean = {-0.0842, -0.0997, 0.8269}, .std = {0.0047, 0.0031, 0.0234}});
    m_standardBones.push_back({.mean = {-0.1461, -0.1596, 0.7201}, .std = {0.0155, 0.0118, 0.0532}});
}

Command Backward::getCommand() const noexcept {
    return commands::backward;
}


// ################################################
// ################## YAWRIGHT ####################
// ################################################
Yawright::Yawright() {
}

Command Yawright::getCommand() const noexcept {
    return commands::yawright;
}


// ################################################
// ################### YAWLEFT ####################
// ################################################
Yawleft::Yawleft() {
}

Command Yawleft::getCommand() const noexcept {
    return commands::yawleft;
}
