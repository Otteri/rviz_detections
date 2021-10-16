#include "bbox.hpp"

VisualizerNode::VisualizerNode(const ros::NodeHandle& nh)
{
    sub = nh_.subscribe("/pointpillars/lidar_detections", 1, &VisualizerNode::VizBboxCallback, this); 
    arr_pub = nh_.advertise<visualization_msgs::MarkerArray>("/visualization/bbox", 1);
}

void VisualizerNode::createBboxWithCenterPoint(std::vector<geometry_msgs::Point>& points, const geometry_msgs::Point size)
{
    double dx = size.x / 2.0;
    double dy = size.y / 2.0;
    double dz = size.z / 2.0;

    geometry_msgs::Point p; // one point is enough, since push_back() copies

    // Bottom plane (-dz)
    p.x = - dx;
    p.y = - dy;
    p.z = - dz;
    points.push_back(p);
    p.x = + dx;
    p.y = - dy;
    p.z = - dz;
    points.push_back(p);

    p.x = - dx;
    p.y = - dy;
    p.z = - dz;
    points.push_back(p);
    p.x = - dx;
    p.y = + dy;
    p.z = - dz;
    points.push_back(p);

    p.x = + dx;
    p.y = + dy;
    p.z = - dz;
    points.push_back(p);
    p.x = - dx;
    p.y = + dy;
    p.z = - dz;
    points.push_back(p);

    p.x = + dx;
    p.y = + dy;
    p.z = - dz;
    points.push_back(p);
    p.x = + dx;
    p.y = - dy;
    p.z = - dz;
    points.push_back(p);


    // Vertical lines 
    p.x = - dx;
    p.y = - dy;
    p.z = - dz;
    points.push_back(p);
    p.x = - dx;
    p.y = - dy;
    p.z = + dz;
    points.push_back(p);

    p.x = + dx;
    p.y = + dy;
    p.z = - dz;
    points.push_back(p);
    p.x = + dx;
    p.y = + dy;
    p.z = + dz;
    points.push_back(p);

    p.x = - dx;
    p.y = + dy;
    p.z = - dz;
    points.push_back(p);
    p.x = - dx;
    p.y = + dy;
    p.z = + dz;
    points.push_back(p);

    p.x = + dx;
    p.y = - dy;
    p.z = - dz;
    points.push_back(p);
    p.x = + dx;
    p.y = - dy;
    p.z = + dz;
    points.push_back(p);


    // Top plane (+dz)
    p.x = - dx;
    p.y = - dy;
    p.z = + dz;
    points.push_back(p);
    p.x = + dx;
    p.y = - dy;
    p.z = + dz;
    points.push_back(p);

    p.x = - dx;
    p.y = - dy;
    p.z = + dz;
    points.push_back(p);
    p.x = - dx;
    p.y = + dy;
    p.z = + dz;
    points.push_back(p);

    p.x = + dx;
    p.y = + dy;
    p.z = + dz;
    points.push_back(p);
    p.x = - dx;
    p.y = + dy;
    p.z = + dz;
    points.push_back(p);

    p.x = + dx;
    p.y = + dy;
    p.z = + dz;
    points.push_back(p);
    p.x = + dx;
    p.y = - dy;
    p.z = + dz;
    points.push_back(p);
}

void VisualizerNode::VizBboxCallback(const rviz_detections::Detection3DArrayConstPtr& msg)
{
    static size_t id;

    visualization_msgs::MarkerArray marker_array;

    for (const auto& detection : msg->detections)
    {
        // General line strip stuff
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = msg->header.frame_id;

        line_strip.header.stamp = msg->header.stamp;
        line_strip.ns = "lines";
        line_strip.pose = detection.pose;

        line_strip.id = id++; // tag id
        line_strip.lifetime = ros::Duration(0.5);
        line_strip.type = visualization_msgs::Marker::LINE_LIST;  //tag type
        line_strip.scale.x = 0.05;
        line_strip.color.r = 0.1;
        line_strip.color.g = 1.0;
        line_strip.color.b = 0.1;
        line_strip.color.a = 1.0;

        createBboxWithCenterPoint(line_strip.points, detection.size);

        marker_array.markers.push_back(line_strip);
    }

    arr_pub.publish(marker_array);
}
