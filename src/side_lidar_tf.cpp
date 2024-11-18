#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

void broadcastStaticTransform(
    double x, double y, double z, 
    double roll, double pitch, double yaw,
    const std::string& parent_frame, const std::string& child_frame)
{
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = parent_frame;
    transformStamped.child_frame_id = child_frame;
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = z;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    static_broadcaster.sendTransform(transformStamped);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "side_lidar_tf");
    ros::NodeHandle nh("~");

    // Load parameters from the YAML file
    double x, y, z, roll, pitch, yaw;
    std::string lidar_side, parent_frame, child_frame;

    if (!nh.getParam("lidar_side", lidar_side)) {
        ROS_ERROR("Failed to get 'lidar_side' parameter.");
        return -1;
    }

    ROS_INFO("Loaded lidar_side: %s", lidar_side.c_str());

    // Retrieve all transform parameters for the specified lidar_side
    if (!nh.getParam(lidar_side + "/x", x) ||
        !nh.getParam(lidar_side + "/y", y) ||
        !nh.getParam(lidar_side + "/z", z) ||
        !nh.getParam(lidar_side + "/roll", roll) ||
        !nh.getParam(lidar_side + "/pitch", pitch) ||
        !nh.getParam(lidar_side + "/yaw", yaw) ||
        !nh.getParam(lidar_side + "/parent_frame", parent_frame) ||
        !nh.getParam(lidar_side + "/child_frame", child_frame))
    {
        ROS_ERROR("Failed to retrieve all parameters for '%s'.", lidar_side.c_str());
        return -1;
    }

    ROS_INFO("Broadcasting Transform:");
    ROS_INFO("Parent Frame: %s", parent_frame.c_str());
    ROS_INFO("Child Frame: %s", child_frame.c_str());
    ROS_INFO("Translation: x=%f, y=%f, z=%f", x, y, z);
    ROS_INFO("Rotation: roll=%f, pitch=%f, yaw=%f", roll, pitch, yaw);

    broadcastStaticTransform(x, y, z, roll, pitch, yaw, parent_frame, child_frame);

    ros::spin();
    return 0;
}
