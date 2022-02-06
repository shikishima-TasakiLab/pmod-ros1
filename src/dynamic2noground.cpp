#include "dynamic2noground.hpp"

Dynamic2NoGround::Dynamic2NoGround()
{
    // Initialize NodeHandle
    this->_nh.reset(new ros::NodeHandle("~"));

    this->_nh->getParam("hz", this->_hz);

    this->_tf_buffer.reset(new tf2_ros::Buffer());
    this->_tf_listener.reset(new tf2_ros::TransformListener(*this->_tf_buffer));

    this->_noground_pub.reset(new
        ros::Publisher(this->_nh->advertise<sensor_msgs::PointCloud2>("points_no_ground", 1))
    );

    this->_static_noground_sub.reset(new
        StaticNoGroundMapSubscriber(*this->_nh, "points_map/noground", 1)
    );

    this->_dynamic_sub.reset(new
        Subscriber<sensor_msgs::PointCloud2>(*this->_nh, "points_dynamic", 1)
    );

    this->_timer.reset(new
        ros::Timer(this->_nh->createTimer(ros::Duration(1.0 / this->_hz), &Dynamic2NoGround::_timer_callback, this))
    );
}

void Dynamic2NoGround::_transform_pointcloud(
    const pcl::PointCloud<pcl::PointXYZ> &src,
    pcl::PointCloud<pcl::PointXYZ> &dst,
    const geometry_msgs::TransformStamped &tf_src2dst
) {
    size_t len = src.points.size();

    if (len == 0ul) {
        dst = src;
        return;
    }

    Eigen::Isometry3d tmp_tf = tf2::transformToEigen(tf_src2dst);
    Eigen::Quaternionf tmp_q = Eigen::Quaternionf(tmp_tf.rotation().cast<float>()).conjugate();
    Eigen::Vector3f tmp_tr(-(tmp_q * tmp_tf.translation().cast<float>()));

    if (
        tmp_tr == Eigen::Vector3f::Zero() &&
        tmp_q.x() == 0.0F &&
        tmp_q.y() == 0.0F &&
        tmp_q.z() == 0.0F &&
        tmp_q.w() == 1.0F
    ) {
        dst = src;
        return;
    }

    if (&src != &dst) {
        dst.points.resize(len);
        dst.width = src.width;
        dst.height = src.height;
        dst.is_dense = src.is_dense;
        dst.header = src.header;
    }

    #ifdef _OPENMP
        #pragma omp parallel for
    #endif
    for (size_t i = 0UL; i < len; i++) {
        pcl::PointXYZ src_point_pcl = src.points[i];
        Eigen::Vector3f src_point_eigen(src_point_pcl.x, src_point_pcl.y, src_point_pcl.z);
        Eigen::Vector3f dst_point_eigen = tmp_q * src_point_eigen + tmp_tr;

        dst.points[i].x = dst_point_eigen[0];
        dst.points[i].y = dst_point_eigen[1];
        dst.points[i].z = dst_point_eigen[2];
    }
}

void Dynamic2NoGround::_timer_callback(const ros::TimerEvent &e)
{
    std::string map_frame_id = this->_static_noground_sub->get_frame_id();
    sensor_msgs::PointCloud2ConstPtr dynamic_msg = this->_dynamic_sub->get_msg();

    if (dynamic_msg == nullptr) {
        ROS_WARN_STREAM_THROTTLE(THROTTLE_PERIOD, "No \"PointCloud2\" msgs subscribed.");
        return;
    }

    geometry_msgs::TransformStamped tf_map2camera;
    try {
        // tf_map2camera = this->_tf_buffer->lookupTransform(map_frame_id, dynamic_msg->header.frame_id, ros::Time(0));
        tf_map2camera = this->_tf_buffer->lookupTransform(map_frame_id, dynamic_msg->header.frame_id, dynamic_msg->header.stamp);
    }
    catch(const std::exception& e)
    {
        ROS_WARN_STREAM_THROTTLE(THROTTLE_PERIOD, "[map(" << map_frame_id << ") -> camera (" << dynamic_msg->header.frame_id << ")] Transform Error: " << e.what());
        return;
    }

    ROS_INFO_STREAM_THROTTLE(THROTTLE_PERIOD, "Processing...");

    pcl::PointCloud<pcl::PointXYZ> noground_map;
    pcl::fromROSMsg(*dynamic_msg, noground_map);

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr static_noground_map = this->_static_noground_sub->get_map(tf_map2camera);
    pcl::PointCloud<pcl::PointXYZ> static_noground_camera;
    this->_transform_pointcloud(*static_noground_map, static_noground_camera, tf_map2camera);

    noground_map += static_noground_camera;

    sensor_msgs::PointCloud2 noground_msg;
    pcl::toROSMsg(noground_map, noground_msg);
    noground_msg.header.stamp = dynamic_msg->header.stamp;
    noground_msg.header.frame_id = dynamic_msg->header.frame_id;

    this->_noground_pub->publish(noground_msg);
}
