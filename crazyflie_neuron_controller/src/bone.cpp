#include "bone.h"

Bone::Bone(const BoneInfo &bi)
    : m_info             (bi)
    , m_quaternionIsNull (true)
{}

void Bone::updateRotation(const tf::Quaternion &quaternion) noexcept {
    if (m_quaternionIsNull) {
        m_quaternionIsNull = false;
        m_quaternion = quaternion;
        return;
    }

    constexpr double eps = 0.001;
    if (fabs(m_quaternion.x() - quaternion.x()) > eps ||
        fabs(m_quaternion.y() - quaternion.y()) > eps ||
        fabs(m_quaternion.z() - quaternion.z()) > eps)
    {
        m_quaternion = quaternion;

        // Notify hand that bone has been changed
        notify();
    }
}

tf::Quaternion Bone::getRotation() const noexcept { return m_quaternion;      }
std::string    Bone::name()        const noexcept { return m_info.name;       }
std::string    Bone::parentName()  const noexcept { return m_info.parentName; }