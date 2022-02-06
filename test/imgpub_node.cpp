#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_imgpub");

    ros::NodeHandle nh("~");

    std::string rgb_path, sparse_depth_path;
    nh.getParam("camera_path", rgb_path);
    nh.getParam("sparse_depth_path", sparse_depth_path);

    sensor_msgs::CameraInfo camerainfo_msg;
    nh.getParam("Fx", camerainfo_msg.K[0]);
    nh.getParam("Cx", camerainfo_msg.K[2]);
    nh.getParam("Fy", camerainfo_msg.K[4]);
    nh.getParam("Cy", camerainfo_msg.K[5]);
    nh.getParam("frame_id", camerainfo_msg.header.frame_id);

    cv_bridge::CvImage rgb_cv, sparse_depth_cv;
    rgb_cv.image = cv::imread(rgb_path, cv::IMREAD_ANYCOLOR);
    if (rgb_cv.image.empty() == true) {
        ROS_ERROR_STREAM("\"" << rgb_path << "\" is empty.");
    }
    rgb_cv.encoding = sensor_msgs::image_encodings::BGR8;
    camerainfo_msg.height = rgb_cv.image.rows;
    camerainfo_msg.width = rgb_cv.image.cols;

    sparse_depth_cv.image = cv::imread(sparse_depth_path, cv::IMREAD_UNCHANGED);
    if (sparse_depth_cv.image.empty() == true) {
        ROS_ERROR_STREAM("\"" << sparse_depth_path << "\" is empty.");
    }
    sparse_depth_cv.encoding = sensor_msgs::image_encodings::TYPE_32FC1;

    ros::Publisher camerainfo_pub = nh.advertise<sensor_msgs::CameraInfo>("/pmod/camera/camera_info", 1);
    ros::Publisher rgb_pub = nh.advertise<sensor_msgs::Image>("/pmod/camera", 1);
    ros::Publisher sparse_depth_pub = nh.advertise<sensor_msgs::Image>("/pmod/sparse_depth", 1);

    sensor_msgs::ImagePtr rgb_msg = rgb_cv.toImageMsg();
    sensor_msgs::ImagePtr sparse_depth_msg = sparse_depth_cv.toImageMsg();

    ros::Duration rate(1.0);

    while(true) {
        camerainfo_msg.header.stamp = ros::Time::now();
        rgb_cv.header = camerainfo_msg.header;
        sparse_depth_cv.header = camerainfo_msg.header;

        camerainfo_pub.publish(camerainfo_msg);
        rgb_pub.publish(rgb_msg);
        sparse_depth_pub.publish(sparse_depth_msg);

        ROS_INFO_ONCE("publishing and latching message. Press ctrl-C to terminate.");

        rate.sleep();
        ros::spinOnce();
    }

    return EXIT_SUCCESS;
}
