#include <ros/ros.h>
#include "bbox.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rviz_detections");
    ros::NodeHandle nh("~");

    bool debug;
    double run_rate_hz; 
    nh.param<bool>("debug", debug, false);
    nh.param<double>("run_rate_hz", run_rate_hz, 100);
    if (debug && ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    VisualizerNode bbox_visualizer(nh);

    ros::Rate rate(run_rate_hz);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}
