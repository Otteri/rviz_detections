#include "bbox.hpp"

VisualizerNode::VisualizerNode(ros::NodeHandle& nh)
{
    double hz;
    nh.getParam("color", rgba_); // array
    nh.getParam("scale", scale_);
    nh.getParam("run_rate_hz", hz);
    nh.param<double>("marker_lifetime", marker_lifetime_, 1.0/hz);

    std::string input_topic, output_topic;
    nh.param<std::string>("input_topic", input_topic, "/lidar/detections");
    nh.param<std::string>("output_topic", output_topic, "/rviz/detections/bbox");

    size_t queue_size = 1u;
    sub_ = nh_.subscribe(input_topic, queue_size, &VisualizerNode::VizBboxCallback, this); 

    bbox_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(output_topic, queue_size);
    class_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/rviz/detections/classes", queue_size);
    confidence_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/rviz/detections/confidences", queue_size);

    ROS_DEBUG_STREAM("VisualizerNode has been initialized and is listening '" << input_topic << "'");
}

// Covnerts float to string showing value with two decimal accuracy
std::string floatToString(float value)
{
    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << value;
    return stream.str();
}

// A helper, which allows to set x,y,z in one line
// (geometry_msgs::Point does not have proper assignment operator)
geometry_msgs::Point createPoint(double x, double y, double z)
{
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

// createPoint constructs a new point, so we can then just emplace_back in order to avoid making extra copies
void VisualizerNode::createBboxWithCenterPoint(std::vector<geometry_msgs::Point>& points, const geometry_msgs::Point size)
{
    double dx = size.x / 2.0;
    double dy = size.y / 2.0;
    double dz = size.z / 2.0;

    // Bottom plane (-dz)
    points.emplace_back(createPoint(-dx, -dy, -dz));
    points.emplace_back(createPoint(+dx, -dy, -dz));
    points.emplace_back(createPoint(-dx, -dy, -dz));
    points.emplace_back(createPoint(-dx, +dy, -dz));
    points.emplace_back(createPoint(+dx, +dy, -dz));
    points.emplace_back(createPoint(-dx, +dy, -dz));
    points.emplace_back(createPoint(+dx, +dy, -dz));
    points.emplace_back(createPoint(+dx, -dy, -dz));

    // Vertical lines 
    points.emplace_back(createPoint(-dx, -dy, -dz));
    points.emplace_back(createPoint(-dx, -dy, +dz));
    points.emplace_back(createPoint(+dx, +dy, -dz));
    points.emplace_back(createPoint(+dx, +dy, +dz));
    points.emplace_back(createPoint(-dx, +dy, -dz));
    points.emplace_back(createPoint(-dx, +dy, +dz));
    points.emplace_back(createPoint(+dx, -dy, -dz));
    points.emplace_back(createPoint(+dx, -dy, +dz));

    // Top plane (+dz)
    points.emplace_back(createPoint(-dx, -dy, +dz));
    points.emplace_back(createPoint(+dx, -dy, +dz));
    points.emplace_back(createPoint(-dx, -dy, +dz));
    points.emplace_back(createPoint(-dx, +dy, +dz));
    points.emplace_back(createPoint(+dx, +dy, +dz));
    points.emplace_back(createPoint(-dx, +dy, +dz));
    points.emplace_back(createPoint(+dx, +dy, +dz));
    points.emplace_back(createPoint(+dx, -dy, +dz));
}

// Creates a new marker and sets common fields, so that marker are uniform
visualization_msgs::Marker VisualizerNode::createMarker(const size_t id, const std_msgs::Header header, const rviz_detections::Detection3D& detection)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = header.frame_id;
    marker.header.stamp = header.stamp;
    marker.id = id;
    marker.pose = detection.pose;
    marker.color.r = rgba_[0];
    marker.color.g = rgba_[1];
    marker.color.b = rgba_[2];
    marker.color.a = rgba_[3];
    marker.lifetime = ros::Duration(marker_lifetime_);
    return marker;
}

void VisualizerNode::VizBboxCallback(const rviz_detections::Detection3DArrayConstPtr& msg)
{
    ROS_DEBUG_STREAM("Recieved a detection");

    static size_t id; // keep track of id, so we can assign same id for bbox, class and confidence

    visualization_msgs::MarkerArray bbox_array;
    visualization_msgs::MarkerArray class_array;
    visualization_msgs::MarkerArray confidence_array;

    for (const auto& detection : msg->detections)
    {
        // Bbox
        visualization_msgs::Marker bbox_marker = createMarker(id, msg->header, detection);
        bbox_marker.type = visualization_msgs::Marker::LINE_LIST;
        bbox_marker.ns = "bboxes";
        bbox_marker.scale.x = scale_; // line thickness
        createBboxWithCenterPoint(bbox_marker.points, detection.size);
        bbox_array.markers.push_back(bbox_marker);

        // Class marker
        visualization_msgs::Marker class_marker = createMarker(id, msg->header, detection);
        class_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        class_marker.ns = "classes";
        class_marker.scale.z = 0.5f; // text scale
        class_marker.text = detection.categories[0]; // assign the first class
        class_array.markers.push_back(class_marker);

        // Confidence marker
        visualization_msgs::Marker confidence_marker = createMarker(id, msg->header, detection);
        confidence_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        confidence_marker.ns = "confidences";
        confidence_marker.scale.z = 0.5f; // text scale
        confidence_marker.pose.position.z = detection.pose.position.z - confidence_marker.scale.z; // show confidence text under class text
        confidence_marker.text = floatToString(detection.category_confidences[0]); // assign the first confidence
        confidence_array.markers.push_back(confidence_marker);

        id++;
    }

    bbox_pub_.publish(bbox_array);
    class_pub_.publish(class_array);
    confidence_pub_.publish(confidence_array);

    ROS_DEBUG_STREAM(">>> published a message");
}
