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
    visualization_msgs::Marker createMarker(const size_t id, const std_msgs::Header header, const rviz_detections::Detection3D& detection);

    ros::NodeHandle nh_;
    ros::Subscriber sub_;           // input data

    ros::Publisher bbox_pub_;       // outputs bounding-boxes
    ros::Publisher class_pub_;      // outputs class labels
    ros::Publisher confidence_pub_; // outputs confidences

    double marker_lifetime_;   // how long marker stays visible (s)
    std::vector<double> rgba_; // color, rgba
    double scale_;             // line thickness
};
