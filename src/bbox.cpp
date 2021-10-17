#include "bbox.hpp"

VisualizerNode::VisualizerNode(ros::NodeHandle& nh)
{
    nh.param<double>("marker_lifetime", marker_lifetime_, 0.5);
    
    std::string input_topic, output_topic;
    nh.param<std::string>("input_topic", input_topic, "/lidar/detections");
    nh.param<std::string>("output_topic", output_topic, "/rviz/detections/bbox");

    size_t queue_size = 1u;
    sub_ = nh_.subscribe(input_topic, queue_size, &VisualizerNode::VizBboxCallback, this); 
    pub_ = nh_.advertise<visualization_msgs::MarkerArray>(output_topic, queue_size);

    ROS_DEBUG_STREAM("VisualizerNode has been initialized and is listening '" << input_topic << "'");
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
    ROS_DEBUG_STREAM("Recieved a detection");

    static size_t id;

    visualization_msgs::MarkerArray marker_array;

    for (const auto& detection : msg->detections)
    {
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = msg->header.frame_id;
        line_strip.header.stamp = msg->header.stamp;
        line_strip.id = id++;
        line_strip.ns = "lines";
        line_strip.pose = detection.pose;
        line_strip.lifetime = ros::Duration(marker_lifetime_);
        line_strip.type = visualization_msgs::Marker::LINE_LIST;
        line_strip.scale.x = 0.05;
        line_strip.color.r = 0.1;
        line_strip.color.g = 1.0;
        line_strip.color.b = 0.1;
        line_strip.color.a = 1.0;

        createBboxWithCenterPoint(line_strip.points, detection.size);

        marker_array.markers.push_back(line_strip);
    }

    pub_.publish(marker_array);
    ROS_DEBUG_STREAM(">>> published a message");
}
