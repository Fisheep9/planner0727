#ifndef _OCC_MAP_FUSION_H
#define _OCC_MAP_FUSION_H

#include <ros/ros.h>

#include <iostream>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <functional>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

using namespace std;

namespace odom_converter{

#define USE_EXACT_TIME_SYNC false

class odom_converter{
private:
    int capture_num_;

public:

    void Pose_Twist_callback(const geometry_msgs::PoseStampedConstPtr& pose_ptr, const geometry_msgs::TwistStampedConstPtr& twist_ptr,int i);
    void Init(ros::NodeHandle & n);
    odom_converter(){
    }
    ~odom_converter(){
        delete[] pose_sub_ptr_;
        delete[] twist_sub_ptr_;
        delete[] sync_exact_;
        delete[] sync_approximate_;
    }
protected:

    typedef message_filters::sync_policies::ExactTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> SyncPolicyExact;
    typedef unique_ptr<message_filters::Synchronizer<SyncPolicyExact>> SynchronizerExact;

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> SyncPolicyApproximate;
    typedef unique_ptr<message_filters::Synchronizer<SyncPolicyApproximate>> SynchronizerApproximate;

    message_filters::Subscriber<geometry_msgs::PoseStamped>** pose_sub_ptr_;
    message_filters::Subscriber<geometry_msgs::TwistStamped>** twist_sub_ptr_;


    SynchronizerExact* sync_exact_;
    SynchronizerApproximate* sync_approximate_;

    void pose_twist_callback(const geometry_msgs::PoseStampedConstPtr& pose_ptr, const geometry_msgs::TwistStampedConstPtr& twist_ptr);

    
    std::vector<ros::Publisher> odom_pub_;
    void registerSub(ros::NodeHandle &n);
    void registerPub(ros::NodeHandle &n);

};

}

#endif