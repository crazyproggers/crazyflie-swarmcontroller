#ifndef BONE_H
#define BONE_H

#include <tf/tf.h>
#include "observer.h"


struct BoneInfo {
    std::string parentName;
    std::string name;
};


class Bone: public Subject {
public:
    Bone(const Bone&)            = delete;
    Bone& operator=(const Bone&) = delete;

    Bone(const BoneInfo &bi);

    void updateRotation(const tf::Quaternion &quaternion) noexcept;

    tf::Quaternion getRotation() const noexcept;
    std::string    name()        const noexcept;
    std::string    parentName()  const noexcept;

private:
    tf::Quaternion m_quaternion;
    BoneInfo       m_info;
    bool           m_quaternionIsNull;
};

#endif // BONE_H