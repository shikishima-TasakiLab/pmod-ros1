#include <torch/script.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "subscriber.hpp"

using PublisherPtr = boost::shared_ptr<ros::Publisher>;
template<typename Msg_T>
using MfSubscriberPtr = boost::shared_ptr<message_filters::Subscriber<Msg_T> >;
template<typename M0, typename M1>
using TimeSyncSubscriberPtr = boost::shared_ptr<TimeSyncSubscriber<M0, M1> >;

#define POINTS_DYNAMIC std::string("points_dynamic")

struct seg_label {
    int id;
    torch::Tensor color;
    bool is_ground;
    bool is_dynamic;
};

class CameraInfoPubSub : public Subscriber<sensor_msgs::CameraInfo>
{
public:
    CameraInfoPubSub(
        ros::NodeHandle &node_handle,
        const std::string &sub_topic,
        const std::string &pub_topic,
        const uint32_t height,
        const uint32_t width,
        const uint32_t queue_size
    ) : Subscriber(node_handle, sub_topic, queue_size)
    {
        this->_height = height;
        this->_width = width;
        this->_pub_camerainfo.reset(
            new ros::Publisher(node_handle.advertise<sensor_msgs::CameraInfo>(pub_topic, queue_size))
        );
    }

protected:
    PublisherPtr _pub_camerainfo;
    uint32_t _height;
    uint32_t _width;

    void _callback(const sensor_msgs::CameraInfoConstPtr &msg) override
    {
        double rate_x = static_cast<double>(this->_width) / static_cast<double>(msg->width);
        double rate_y = static_cast<double>(this->_height) / static_cast<double>(msg->height);

        sensor_msgs::CameraInfoPtr camerainfo(new sensor_msgs::CameraInfo(*msg));
        camerainfo->K[0] *= rate_x;
        camerainfo->K[2] *= rate_x;
        camerainfo->K[4] *= rate_y;
        camerainfo->K[5] *= rate_y;
        camerainfo->height = this->_height;
        camerainfo->width = this->_width;

        this->_pub_camerainfo->publish(camerainfo);

        std::lock_guard<std::mutex> lock(*this->_mtx);
        this->_msg = camerainfo;
    };
};

using CameraInfoPubSubPtr = boost::shared_ptr<CameraInfoPubSub>;
using ImageSub = Subscriber<sensor_msgs::Image>;
using ImageSubPtr = boost::shared_ptr<ImageSub>;

class PMOD
{
public:
    PMOD();

protected:
    bool _use_sync = false;

    std::unordered_map<int, seg_label> _seg_labels;
    bool _use_optical_frame = false;
    std::string _camera_frame_id = "";
    cv::Size _shape = cv::Size(512, 256);
    float _range_norm = 80.0F;
    torch::DeviceType _device = torch::kCPU;
    torch::TensorOptions _default_options;

    ros::NodeHandlePtr _nh;
    boost::shared_ptr<ros::Timer> _timer;

    ImageSubPtr _sub_camera_rgb;
    ImageSubPtr _sub_camera_depth;
    TimeSyncSubscriberPtr<sensor_msgs::Image, sensor_msgs::Image> _sub_camera;
    CameraInfoPubSubPtr _sub_camera_info;

    PublisherPtr _pub_seg_id;
    PublisherPtr _pub_seg_color;
    PublisherPtr _pub_depth;
    PublisherPtr _pub_points_ground;
    PublisherPtr _pub_points_no_ground;
    PublisherPtr _pub_points_static;
    PublisherPtr _pub_points_dynamic;

    bool _can_transform = false;
    boost::shared_ptr<tf2_ros::StaticTransformBroadcaster> _static_br;

    boost::shared_ptr<torch::jit::script::Module> _module;

    void _timer_callback(const ros::TimerEvent &e);

    template<class Msg_T>
    void _create_publisher(PublisherPtr &pub_ptr, const std::string &topic, int queue_size, bool pub_on = false)
    {
        this->_nh->getParam("pub_" + topic, pub_on);
        if (pub_on == true) {
            pub_ptr.reset(new ros::Publisher(
                this->_nh->advertise<Msg_T>(topic, queue_size)
            ));
        }
    }

    void _broadcast_static_tf(const std::string &parent, std::string &child);
};
