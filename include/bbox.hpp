#pragma once

#include <ros/ros.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "rviz_detections/Detection3D.h"
#include "rviz_detections/Detection3DArray.h"

class VisualizerNode
{
public:

    VisualizerNode(const ros::NodeHandle& nh);

    void VizBboxCallback(const rviz_detections::Detection3DArrayConstPtr& msg);

    void createBboxWithCenterPoint(std::vector<geometry_msgs::Point>& points, const geometry_msgs::Point size);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub; // input data
    ros::Publisher arr_pub; // ouput
};
