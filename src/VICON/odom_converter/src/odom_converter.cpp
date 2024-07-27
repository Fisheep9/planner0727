#include <odom_converter.h>

namespace odom_converter{

void odom_converter::Init(ros::NodeHandle& n){
    n.param("cap_num",capture_num_,1);
    sync_exact_ = new SynchronizerExact[capture_num_];
    sync_approximate_ = new SynchronizerApproximate[capture_num_];

    registerSub(n);
    registerPub(n);

}

void odom_converter::registerSub(ros::NodeHandle &n){
    pose_sub_ptr_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>*[capture_num_];;
    twist_sub_ptr_ = new message_filters::Subscriber<geometry_msgs::TwistStamped>*[capture_num_];
    for(int i =0; i <capture_num_; i++){
        pose_sub_ptr_[i] = new message_filters::Subscriber<geometry_msgs::PoseStamped> (n, "pose"+std::to_string(i), 100, ros::TransportHints().tcpNoDelay(true));
        twist_sub_ptr_[i] = new message_filters::Subscriber<geometry_msgs::TwistStamped> (n, "twist"+std::to_string(i), 100, ros::TransportHints().tcpNoDelay(true));
    }
    
    if(USE_EXACT_TIME_SYNC){
        for(int i =0; i< capture_num_;i++){
            sync_exact_[i].reset(new message_filters::Synchronizer<SyncPolicyExact>(SyncPolicyExact(100), (*pose_sub_ptr_)[i], (*twist_sub_ptr_)[i]));
            auto callback = boost::bind(&odom_converter::Pose_Twist_callback, this, _1, _2, i);
            sync_exact_[i]->registerCallback(callback);
            sync_approximate_[i].release();    
        }
        

    }else{
        for(int i=0; i< capture_num_;i++){
            sync_approximate_[i].reset(new message_filters::Synchronizer<SyncPolicyApproximate>(SyncPolicyApproximate(10), *(pose_sub_ptr_[i]), *(twist_sub_ptr_[i])));
            auto callback = boost::bind(&odom_converter::Pose_Twist_callback, this, _1, _2, i);
            sync_approximate_[i]->registerCallback(callback);
            sync_exact_[i].release();
        }
        
    }
}

 void odom_converter::Pose_Twist_callback(const geometry_msgs::PoseStampedConstPtr& pose_ptr, const geometry_msgs::TwistStampedConstPtr& twist_ptr,int i){
        nav_msgs::Odometry odom;
        odom.header = pose_ptr->header;
        odom.pose.pose = pose_ptr->pose;
        odom.twist.twist = twist_ptr->twist;
        odom_pub_[i].publish(odom);
    }


void odom_converter::registerPub(ros::NodeHandle &n){
    odom_pub_.resize(capture_num_);
    for(int i=0; i< capture_num_;i++){
        string name = "converted_odom"+std::to_string(i);
        odom_pub_[i] = n.advertise<nav_msgs::Odometry>(name, 10);
    }
}



void odom_converter::pose_twist_callback(const geometry_msgs::PoseStampedConstPtr& pose_ptr, const geometry_msgs::TwistStampedConstPtr& twist_ptr){
    nav_msgs::Odometry odom;
    odom.header = pose_ptr->header;
    odom.pose.pose = pose_ptr->pose;
    odom.twist.twist = twist_ptr->twist;
}

}