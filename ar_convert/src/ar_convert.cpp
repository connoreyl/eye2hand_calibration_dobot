#include "ar_convert.h"

ARConvert::ARConvert(ros::NodeHandle nh)
    : nh_(nh)
{
    tag_sub_ = nh_.subscribe("/ar_pose_marker", 10, &ARConvert::tagCallback, this);
    tag_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/tags", 1);
}

ARConvert::~ARConvert()
{
}

void ARConvert::tagCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr &msg)
{
    std::unique_lock<std::mutex> lock(tag_data_.mtx_);
    tag_data_.tags_.poses.clear();
    tag_data_.tag_ = false;
    std::vector<std::vector<int>> id;
    std::vector<ar_track_alvar_msgs::AlvarMarker> markers;
    if (msg->markers.size() > 0)
    {
        // ar_track_alvar_msgs::AlvarMarker marker = msg->markers.at(0);
        tag_data_.tag_ = true;
        for (int i = 0; i < msg->markers.size(); i++)
        {
            markers.push_back(msg->markers.at(i));
        }

        //bubble sort tags in ascending order by id
        int n = markers.size();
        int newn =0;
        while (n > 1)
        {
            newn = 0;
            for(int i=1; i<=n-1; i++) 
            {
                if (markers.at(i-1).id > markers.at(i).id)
                {
                    // markers.at(i-1).swap(markers.at(i));
                    ar_track_alvar_msgs::AlvarMarker marker1 = markers.at(i-1);
                    ar_track_alvar_msgs::AlvarMarker marker2 = markers.at(i);
                    markers.at(i) = marker1;
                    markers.at(i-1) = marker2;

                    newn = i;
                }
            }
            n = newn;
        }
        std::cout << "Sorted Ids:";
        for (int i=0;i<markers.size();i++)
        {
            std::cout << " "<< markers.at(i).id;
            tag_data_.tags_.poses.push_back(msg->markers.at(i).pose.pose);
        }
        std::cout << std::endl;

    }
    lock.unlock();
    tag_data_.cond_.notify_all();
}

void ARConvert::publishTag(void)
{
    while (ros::ok())
    {
        std::unique_lock<std::mutex> lock(tag_data_.mtx_);
        tag_data_.cond_.wait(lock);
        tag_pub_.publish(tag_data_.tags_);
        lock.unlock();
    }
}