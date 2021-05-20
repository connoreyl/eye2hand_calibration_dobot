#include "ros/ros.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"

#include <thread>
#include <chrono>
#include <mutex>
#include <random>
#include <condition_variable>
#include <vector>

class ARConvert
{
public:
    ARConvert(ros::NodeHandle nh);
    ~ARConvert();

    void publishTag(void);
    ros::NodeHandle nh_;

private:
    void tagCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr &msg);
    
    ros::Publisher tag_pub_;
    ros::Subscriber tag_sub_;

    struct tagData
    {
        std::mutex mtx_;
        std::condition_variable cond_;
        geometry_msgs::PoseArray tags_;
        bool tag_;
    };
    tagData tag_data_;
        
};