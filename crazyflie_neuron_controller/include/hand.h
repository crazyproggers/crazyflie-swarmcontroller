#ifndef HAND_H
#define HAND_H

#include <mutex>
#include "bone.h"

using bones_t = std::vector<std::shared_ptr<Bone>>;


class Hand: public Observer, public Subject, public std::enable_shared_from_this<Hand> {
public:
    Hand(const Hand&)            = delete;
    Hand& operator=(const Hand&) = delete;
   ~Hand()                       = default;

    Hand(const std::vector<BoneInfo> &bonesInfo,
         size_t minBonesAmountForGesture = 2);

    const bones_t& bones() const noexcept;
    void resetMovedBonesAmount() noexcept;

private:
    bones_t             m_bones;
    size_t              m_movedBonesAmount;
    size_t              m_minBonesAmountForGesture;
    std::mutex          m_mutex;

    void update();
    void updateBoneRotation(size_t boneIdx);
};

#endif // HAND_H