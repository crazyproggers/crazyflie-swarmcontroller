#include <tf/transform_listener.h>
#include <thread>
#include "hand.h"


Hand::Hand(const std::vector<BoneInfo> &bonesInfo, size_t minBonesAmountForGesture)
    : m_minBonesAmountForGesture (minBonesAmountForGesture)
    , m_movedBonesAmount         (0)
{
    for (auto bi: bonesInfo) {
        auto bone = std::make_shared<Bone>(bi);
        auto wptr = std::shared_ptr<Hand>(this, [](Hand*) {});
        bone->attach(shared_from_this());
        m_bones.push_back(bone);
        std::thread { &Hand::updateBoneRotation, this, m_bones.size()-1 }.detach();
    }
}


void Hand::updateBoneRotation(size_t boneIdx) {
    auto bone = m_bones[boneIdx];

    tf::TransformListener listener;
    listener.waitForTransform(bone->parentName(), bone->name(), ros::Time(0), ros::Duration(5.0));
    ros::Rate loop(10);

    while (ros::ok()) {
        tf::StampedTransform pose;

        try {
            listener.lookupTransform(bone->parentName(), bone->name(), ros::Time(0), pose);
            bone->updateRotation(pose.getRotation());
        }
        catch (tf::TransformException &exc) {
            ROS_ERROR("%s%s", bone->name().c_str(), ": could not get current pose!");
            ROS_ERROR("An exception was caught: %s", exc.what());
        }

        loop.sleep();
    }
}


void Hand::resetMovedBonesAmount() noexcept {
    m_movedBonesAmount = 0;
}


void Hand::update() {
    std::lock_guard<std::mutex> locker(m_mutex);

    m_movedBonesAmount++;
    if (m_movedBonesAmount >= m_minBonesAmountForGesture) {
        // Notify GestureRecognizer that hand has been changed
        notify();
    }
}


const bones_t& Hand::bones() const noexcept {
    return m_bones;
}