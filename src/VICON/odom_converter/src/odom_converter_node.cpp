#include <odom_converter.h>
// #include <backward.hpp>

// namespace backward {
// backward::SignalHandling sh;
// }

int main(int argc, char** argv){
    
    ros::init(argc, argv, "odom_converter_node");
    ros::NodeHandle n("~");

    odom_converter::odom_converter oc;
    oc.Init(n);

    while(ros::ok())
    {
        ros::spin();
    }
    return 1;
}