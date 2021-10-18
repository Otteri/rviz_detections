#pragma once

#include <ros/ros.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "rviz_detections/Detection3D.h"
#include "rviz_detections/Detection3DArray.h"

class VisualizerNode
{
public:
    VisualizerNode(ros::NodeHandle& nh);

private:
    void VizBboxCallback(const rviz_detections::Detection3DArrayConstPtr& msg);
    void createBboxWithCenterPoint(std::vector<geometry_msgs::Point>& points, const geometry_msgs::Point size);

    ros::NodeHandle nh_;
    ros::Subscriber sub_;      // input data
    ros::Publisher pub_;       // ouput data
    double marker_lifetime_;   // how long marker stays visible (s)
    std::vector<double> rgba_; // color, rgba
    double scale_;             // line thickness
};
