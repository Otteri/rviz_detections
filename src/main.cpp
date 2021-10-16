#include <ros/ros.h>
#include "bbox.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rviz_detections");
    ros::NodeHandle nh("~");

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    VisualizerNode bbox_visualizer(nh);

    ros::Rate r(100);
    ros::spin();
}
