#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>


class NeuronBroadcaster {
public:
    NeuronBroadcaster(const ros::NodeHandle &node)
        : m_node(node)
    {
         m_slink_children_names = {"HipsPosition"      , "Hips"             , "RightUpLeg"      , "RightLeg"        , "RightFoot",
                                   "LeftUpLeg"         , "LeftLeg"          , "LeftFoot"        , "Spine"           ,
                                   "Spine1"            , "Spine2"           , "Spine3"          ,
                                   "Neck"              , "Head"             ,
                                   "RightShoulder"     , "RightArm"         , "RightForeArm"    ,
                                   "RightHand"         , "RightHandThumb1"  , "RightHandThumb2" , "RightHandThumb3" ,
                                   "RightInHandIndex"  , "RightHandIndex1"  , "RightHandIndex2" , "RightHandIndex3" ,
                                   "RightInHandMiddle" , "RightHandMiddle1" , "RightHandMiddle2", "RightHandMiddle3",
                                   "RightInHandRing"   , "RightHandRing1"   , "RightHandRing2"  , "RightHandRing3"  ,
                                   "RightInHandPinky"  , "RightHandPinky1"  , "RightHandPinky2" , "RightHandPinky3" ,
                                   "LeftShoulder"      , "LeftArm"          , "LeftForeArm"     , "LeftHand"        ,
                                   "LeftHandThumb1"    , "LeftHandThumb2"   , "LeftHandThumb3"  ,
                                   "LeftInHandIndex"   , "LeftHandIndex1"   , "LeftHandIndex2"  , "LeftHandIndex3"  ,
                                   "LeftInHandMiddle"  , "LeftHandMiddle1"  , "LeftHandMiddle2" , "LeftHandMiddle3" ,
                                   "LeftInHandRing"    , "LeftHandRing1"    , "LeftHandRing2"   , "LeftHandRing3"   ,
                                   "LeftInHandPinky"   , "LeftHandPinky1"   , "LeftHandPinky2"  , "LeftHandPinky3"  };

        m_link_parents_names = {"WorldPerceptionNeuron", "HipsPosition"     , "Hips"            , "RightUpLeg"      , "RightLeg",
                                "Hips"                 , "LeftUpLeg"        , "LeftLeg"         , "Hips"            ,
                                "Spine"                , "Spine1"           , "Spine2"          ,
                                "Spine3"               , "Neck"             ,
                                "Spine3"               , "RightShoulder"    , "RightArm"        ,
        /* START FROM 17 */     "RightForeArm"         , "RightHand"        , "RightHandThumb1" , "RightHandThumb2" ,
                                "RightHand"            , "RightInHandIndex" , "RightHandIndex1" , "RightHandIndex2" ,
                                "RightHand"            , "RightInHandMiddle", "RightHandMiddle1", "RightHandMiddle2",
                                "RightHand"            , "RightInHandRing"  , "RightHandRing1"  , "RightHandRing2"  ,
                                "RightHand"            , "RightInHandPinky" , "RightHandPinky1" , "RightHandPinky2" ,  /* FINISH ON 36 */
                                "Spine3"               , "LeftShoulder"     , "LeftArm"         , "LeftForeArm"     ,
                                "LeftHand"             , "LeftHandThumb1"   , "LeftHandThumb2"  ,
                                "LeftHand"             , "LeftInHandIndex"  , "LeftHandIndex1"  , "LeftHandIndex2"  ,
                                "LeftHand"             , "LeftInHandMiddle" , "LeftHandMiddle1" , "LeftHandMiddle2" ,
                                "LeftHand"             , "LeftInHandRing"   , "LeftHandRing1"   , "LeftHandRing2"   ,
                                "LeftHand"             , "LeftInHandPinky"  , "LeftHandPinky1"  , "LeftHandPinky2"  };


        /*
         * Excluded exctented data
         * Only right hand data needed
         */

        std::vector<std::string> topics_names = {"/perception_neuron/data_1",
                                                 "/perception_neuron/data_2"
                                            /* , "/perception_neuron/data_3" */};

        m_subscribers.resize(topics_names.size());
        m_usefulLinkIdx = {17, 18, 22, 26, 30, 34};

        for (size_t i = 0; i < m_subscribers.size(); ++i)
            m_subscribers[i] = m_node.subscribe<std_msgs::Float64MultiArray>(topics_names[i], 5, boost::bind(&NeuronBroadcaster::callback_i, this, _1, i));
    }

    void sendStaticTransform() {
        // Sending Static transformation to ROS World
        tf::Transform world_frame;
        world_frame.setOrigin(tf::Vector3(0, 0, 0));
        world_frame.setRotation(tf::Quaternion(0.70711, 0, 0, 0.70711));
        m_tf_broadcaster.sendTransform(tf::StampedTransform(world_frame, ros::Time::now(), "world", "WorldPerceptionNeuron"));
    }

private:
    tf::Quaternion eulerToQuaternion(float eulerY, float eulerX, float eulerZ) const noexcept {
        Eigen::Matrix3f rx, ry, rz;

        rx = Eigen::AngleAxisf(eulerX * M_PI / 180.0, Eigen::Vector3f::UnitX());
        ry = Eigen::AngleAxisf(eulerY * M_PI / 180.0, Eigen::Vector3f::UnitY());
        rz = Eigen::AngleAxisf(eulerZ * M_PI / 180.0, Eigen::Vector3f::UnitZ());

        // Check Ordering in Axis Neuron! Here = YXZ
        Eigen::Matrix3f rxyz = ry * rx * rz;

        Eigen::Quaternionf qf(rxyz);
        tf::Quaternion q;

        q.setW(qf.w());
        q.setX(qf.x());
        q.setY(qf.y());
        q.setZ(qf.z());

        return q;
    }

    void callback_i(const std_msgs::Float64MultiArrayConstPtr &bone_data, size_t i) {
        size_t finish = bone_data->data.size() / 6;

        for (size_t j = 0; j < finish; j++) {
            size_t linkIdx = j + i * 20;

            if (!std::binary_search(m_usefulLinkIdx.begin(), m_usefulLinkIdx.end(), linkIdx))
                continue;

            size_t startIdx = j * 6;

            // conversion to meters
            tf::Vector3 position;
            position.setX(0.01 * bone_data->data[startIdx+0]);
            position.setY(0.01 * bone_data->data[startIdx+1]);
            position.setZ(0.01 * bone_data->data[startIdx+2]);

            float eulerY = bone_data->data[startIdx+3];
            float eulerX = bone_data->data[startIdx+4];
            float eulerZ = bone_data->data[startIdx+5];

            tf::Quaternion rotation = eulerToQuaternion(eulerY, eulerX, eulerZ);

            tf::Transform pose;
            pose.setOrigin(position);
            pose.setRotation(rotation);

            ros::Time time = ros::Time::now();

            m_tf_broadcaster.sendTransform(tf::StampedTransform(pose, time, m_link_parents_names[linkIdx], m_slink_children_names[linkIdx]));
        }
    }

private:
     ros::NodeHandle              m_node;
     std::vector<ros::Subscriber> m_subscribers;
     std::vector<std::string>     m_slink_children_names;
     std::vector<std::string>     m_link_parents_names;
     std::array<size_t, 6>        m_usefulLinkIdx;
     tf::TransformBroadcaster     m_tf_broadcaster;   
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "perception_neuron_tf_broadcaster_node");

    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::NodeHandle node;
    NeuronBroadcaster broadcaster(node);

    ros::Rate loop(10.0);
    while (node.ok()) {
        broadcaster.sendStaticTransform();
        loop.sleep();
    }

    return 0;
}