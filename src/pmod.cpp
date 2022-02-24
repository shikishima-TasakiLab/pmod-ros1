#include "pmod.hpp"

PMOD::PMOD()
{
    this->_default_options = torch::TensorOptions().dtype(torch::kFloat32);
    if (torch::hasCUDA() == true) {
        ROS_INFO("Use CUDA.");
        this->_device = torch::kCUDA;
    }
    this->_default_options = this->_default_options.device(this->_device);

    // Initialize NodeHandle
    this->_nh.reset(new ros::NodeHandle("~"));

    // hz
    double hz = 5.0;
    this->_nh->getParam("hz", hz);

    // height
    this->_nh->getParam("height", this->_shape.height);
    // width
    this->_nh->getParam("width", this->_shape.width);

    // sub_queue_size
    int sub_queue_size = 10;
    this->_nh->getParam("sub_queue_size", sub_queue_size);

    // pub_queue_size
    int pub_queue_size = 2;
    this->_nh->getParam("pub_queue_size", pub_queue_size);

    // use_optical_frame
    this->_nh->getParam("use_optical_frame", this->_use_optical_frame);

    // camera_frame_id
    if (this->_use_optical_frame == false) {
        this->_nh->getParam("camera_frame_id", this->_camera_frame_id);
    }

    // pub_seg_color (1)
    bool pub_seg_color = true;
    this->_nh->getParam("pub_seg_color", pub_seg_color);

    // seg_labels
    XmlRpc::XmlRpcValue seg_labels;
    this->_nh->getParam("seg_labels", seg_labels);
    if (seg_labels.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (size_t i = 0UL; i < seg_labels.size(); i++) {
            seg_label seglabel;

            // id
            if (seg_labels[i]["id"].valid() == false) {ROS_WARN("No \"id\"."); continue;}
            if (seg_labels[i]["id"].getType() != XmlRpc::XmlRpcValue::TypeInt) {ROS_WARN("\"id\" is not int."); continue;}
            seglabel.id = static_cast<int>(seg_labels[i]["id"]);

            // is_ground
            if (seg_labels[i]["is_ground"].valid() == false) {ROS_WARN("No \"is_ground\"."); continue;}
            if (seg_labels[i]["is_ground"].getType() != XmlRpc::XmlRpcValue::TypeBoolean) {ROS_WARN("\"is_ground\" is not bool."); continue;}
            seglabel.is_ground = static_cast<bool>(seg_labels[i]["is_ground"]);

            // is_dynamic
            if (seg_labels[i]["is_dynamic"].valid() == false) {ROS_WARN("No \"is_dynamic\"."); continue;}
            if (seg_labels[i]["is_dynamic"].getType() != XmlRpc::XmlRpcValue::TypeBoolean) {ROS_WARN("\"is_dynamic\" is not bool."); continue;}
            seglabel.is_dynamic = static_cast<bool>(seg_labels[i]["is_dynamic"]);

            // color
            if (seg_labels[i]["color"].valid() == true && pub_seg_color == true) {
                if (seg_labels[i]["color"].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
                    ROS_WARN("\"color\" is not struct.");
                    pub_seg_color = false;
                }
                else {
                    std::vector<std::string> color_tag = {"r", "g", "b"};
                    int color_value[] = {0, 0, 0};

                    for (size_t c = 0UL; c < 3UL && pub_seg_color == true; c++) {
                        if (seg_labels[i]["color"][color_tag[c]].valid() == false) {
                            ROS_WARN_STREAM("No \"color." << color_tag[c] << "\".");
                            pub_seg_color = false;
                            break;
                        }
                        if (seg_labels[i]["color"][color_tag[c]].getType() != XmlRpc::XmlRpcValue::TypeInt) {
                            ROS_WARN_STREAM("\"color." << color_tag[c] << "\" is not int.");
                            pub_seg_color = false;
                            break;
                        }
                        color_value[c] = static_cast<int>(seg_labels[i]["color"][color_tag[c]]);
                    }

                    if (pub_seg_color == true) {
                        seglabel.color = torch::tensor({
                            static_cast<u_int8_t>(color_value[2]),
                            static_cast<u_int8_t>(color_value[1]),
                            static_cast<u_int8_t>(color_value[0])
                        }, torch::TensorOptions().dtype(torch::kUInt8).device(this->_device));
                    }
                }
            }
            else {
                pub_seg_color = false;
            }

            this->_seg_labels[seglabel.id] = seglabel;
        }
    }

    // pub_seg_color (2)
    if (pub_seg_color == true) {
        this->_pub_seg_color.reset(new ros::Publisher(
            this->_nh->advertise<sensor_msgs::Image>("seg_color", pub_queue_size)
        ));
    }

    // pub_seg_id
    this->_create_publisher<sensor_msgs::Image>(this->_pub_seg_id, "seg_id", pub_queue_size);
    // pub_depth
    this->_create_publisher<sensor_msgs::Image>(this->_pub_depth, "depth", pub_queue_size);
    // pub_points_ground
    this->_create_publisher<sensor_msgs::PointCloud2>(this->_pub_points_ground, "points_ground", pub_queue_size);
    // pub_points_no_ground
    this->_create_publisher<sensor_msgs::PointCloud2>(this->_pub_points_no_ground, "points_no_ground", pub_queue_size);
    // pub_points_static
    this->_create_publisher<sensor_msgs::PointCloud2>(this->_pub_points_static, "points_static", pub_queue_size);
    // pub_points_dynamic
    this->_create_publisher<sensor_msgs::PointCloud2>(this->_pub_points_dynamic, "points_dynamic", pub_queue_size);

    // load checkpoint
    std::string checkpoint_path;
    this->_nh->getParam("checkpoint", checkpoint_path);
    this->_module.reset(new torch::jit::script::Module(
        torch::jit::load(checkpoint_path, this->_device)
    ));
    this->_module->eval();

    this->_sub_camera_info.reset(new
        CameraInfoPubSub(
            *this->_nh,
            "camera/camera_info",
            "sparse_depth/camera_info",
            this->_shape.height,
            this->_shape.width,
            sub_queue_size
        )
    );

    this->_nh->getParam("use_sync", this->_use_sync);

    if (this->_use_sync == true) {
        this->_sub_camera.reset(new
            TimeSyncSubscriber<sensor_msgs::Image, sensor_msgs::Image>(*this->_nh, "camera", "sparse_depth", sub_queue_size)
        );
    }
    else {
        this->_sub_camera_rgb.reset(new ImageSub(*this->_nh, "camera", sub_queue_size));
        this->_sub_camera_depth.reset(new ImageSub(*this->_nh, "sparse_depth", sub_queue_size));
    }

    this->_timer.reset(new
        ros::Timer(this->_nh->createTimer(ros::Duration(1.0 / hz), &PMOD::_timer_callback, this))
    );
}

void PMOD::_timer_callback(const ros::TimerEvent &e)
{
    torch::NoGradGuard no_grad;

    sensor_msgs::CameraInfoConstPtr camerainfo = this->_sub_camera_info->get_msg();
    sensor_msgs::ImageConstPtr camera_msg, sparse_depth_msg;

    if (this->_use_sync == true) {
        this->_sub_camera->get_msgs(camera_msg, sparse_depth_msg);
    }
    else {
        camera_msg = this->_sub_camera_rgb->get_msg();
        sparse_depth_msg = this->_sub_camera_depth->get_msg();
    }
    if (camera_msg == nullptr || sparse_depth_msg == nullptr) {
        ROS_WARN_THROTTLE(30, "No \"Image\" msgs subscribed.");
        return;
    }
    ROS_INFO_THROTTLE(30, "Processing...");

    std::vector<torch::jit::IValue> inputs;

    cv_bridge::CvImagePtr rgb_cv = cv_bridge::toCvCopy(camera_msg, camera_msg->encoding);
    if (rgb_cv->encoding == sensor_msgs::image_encodings::BGRA8) {
        cv::cvtColor(rgb_cv->image, rgb_cv->image, CV_BGRA2BGR);
    }
    rgb_cv->image.convertTo(rgb_cv->image, CV_32FC3);
    inputs.push_back(torch::from_blob(rgb_cv->image.data, {1, rgb_cv->image.rows, rgb_cv->image.cols, 3}, torch::kFloat32).to(this->_default_options).permute({0,3,1,2}));

    cv_bridge::CvImageConstPtr sparse_depth_cv = cv_bridge::toCvShare(sparse_depth_msg, sparse_depth_msg->encoding);
    inputs.push_back(torch::from_blob(sparse_depth_cv->image.data, {1, this->_shape.height, this->_shape.width, 1}, torch::kFloat32).to(this->_default_options).permute({0,3,1,2}));

    if (rgb_cv->image.rows != this->_shape.height || rgb_cv->image.cols != this->_shape.width) {
        inputs[0] = at::upsample_bilinear2d(inputs[0].toTensor(), {this->_shape.height, this->_shape.width}, false);
    }
    inputs[0] = inputs[0].toTensor() / 255.0F;
    inputs[1] = torch::where((0.0F < inputs[1].toTensor() & inputs[1].toTensor() < this->_range_norm), inputs[1].toTensor() / this->_range_norm, torch::full_like(inputs[1].toTensor(), 2.0F, this->_default_options));

    std::vector<c10::IValue> outputs = this->_module->forward(inputs).toTuple()->elements();

    torch::Tensor out_seg = outputs[0].toTensor().argmax(1).to(torch::kUInt8).squeeze_();
    torch::Tensor out_depth = torch::where(
        (outputs[1].toTensor() < 0.0F | 1.0F < outputs[1].toTensor()),
        torch::full_like(outputs[1].toTensor(), INFINITY, this->_default_options),
        outputs[1].toTensor() * this->_range_norm
    ).squeeze_();

    if (this->_pub_seg_id != nullptr) {
        torch::Tensor label_tensor_cpu = out_seg.to(torch::kCPU);
        cv_bridge::CvImage label_cv(camera_msg->header, sensor_msgs::image_encodings::TYPE_8UC1, cv::Mat(this->_shape, CV_8UC1, label_tensor_cpu.data_ptr()));
        sensor_msgs::ImagePtr label_msg = label_cv.toImageMsg();
        label_msg->header.stamp = camera_msg->header.stamp;
        this->_pub_seg_id->publish(label_msg);
    }

    if (this->_pub_seg_color != nullptr) {
        torch::Tensor label_tensor = torch::zeros({this->_shape.height, this->_shape.width, 3}, torch::TensorOptions().dtype(torch::kUInt8).device(this->_device));

        for (auto seg_labels_itr = this->_seg_labels.begin(); seg_labels_itr != this->_seg_labels.end(); seg_labels_itr++) {
            std::vector<at::Tensor> label_idx = torch::where(out_seg == seg_labels_itr->second.id);
            label_tensor = label_tensor.index_put_(label_idx, seg_labels_itr->second.color);
        }

        torch::Tensor label_tensor_cpu = label_tensor.to(torch::kCPU);
        cv_bridge::CvImage label_cv(camera_msg->header, sensor_msgs::image_encodings::BGR8, cv::Mat(this->_shape, CV_8UC3, label_tensor_cpu.data_ptr()));
        sensor_msgs::ImagePtr label_msg = label_cv.toImageMsg();
        label_msg->header.stamp = camera_msg->header.stamp;
        this->_pub_seg_color->publish(label_msg);
    }

    if (this->_pub_depth != nullptr) {
        torch::Tensor depth_tensor_cpu = out_depth.to(torch::kCPU);
        cv_bridge::CvImage depth_cv(camera_msg->header, sensor_msgs::image_encodings::TYPE_32FC1, cv::Mat(this->_shape, CV_32FC1, depth_tensor_cpu.data_ptr()));
        sensor_msgs::ImagePtr depth_msg = depth_cv.toImageMsg();
        depth_msg->header.stamp = camera_msg->header.stamp;
        this->_pub_depth->publish(depth_msg);
    }

    if (this->_pub_points_dynamic == nullptr && this->_pub_points_static == nullptr && this->_pub_points_ground == nullptr && this->_pub_points_no_ground == nullptr) {
        return;
    }
    if (camerainfo == nullptr) {
        ROS_WARN_THROTTLE(30, "No \"CameraInfo\" msgs subscribed.");
        return;
    }

    torch::Tensor image_u = torch::arange(this->_shape.width, this->_default_options).expand({this->_shape.height, -1});
    torch::Tensor image_v = torch::arange(this->_shape.height, this->_default_options).expand({this->_shape.width, -1}).permute({1, 0});
    torch::Tensor points_x = (out_depth * (image_u - static_cast<float>(camerainfo->K[2])) / static_cast<float>(camerainfo->K[0])).to(torch::kCPU);
    torch::Tensor points_y = (out_depth * (image_v - static_cast<float>(camerainfo->K[5])) / static_cast<float>(camerainfo->K[4])).to(torch::kCPU);
    torch::Tensor points_z = out_depth.to(torch::kCPU);
    torch::Tensor label_tensor_cpu = out_seg.to(torch::kCPU);

    cv::Mat points_x_cv(this->_shape, CV_32FC1, points_x.data_ptr());
    cv::Mat points_y_cv(this->_shape, CV_32FC1, points_y.data_ptr());
    cv::Mat points_z_cv(this->_shape, CV_32FC1, points_z.data_ptr());
    cv::Mat label_cv(this->_shape, CV_8UC1, label_tensor_cpu.data_ptr());

    cv::Mat label_edge;
    cv::Laplacian(label_cv, label_edge, CV_8U, 5);

    pcl::PointCloud<pcl::PointXYZ>::Ptr points_ground;
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_no_ground;
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_dynamic;
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_static;

    if (this->_use_optical_frame == true) {
        this->_camera_frame_id = camerainfo->header.frame_id;
    }
    else if (this->_can_transform == false) {
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener(tf_buffer);
        if (this->_camera_frame_id == "") {
            this->_broadcast_static_tf(camerainfo->header.frame_id, this->_camera_frame_id);
        }
        else if (tf_buffer.canTransform(this->_camera_frame_id, camerainfo->header.frame_id, ros::Time(0)) == false) {
            this->_broadcast_static_tf(camerainfo->header.frame_id, this->_camera_frame_id);
        }
        this->_can_transform = true;
    }

    if (this->_pub_points_ground != nullptr) {
        points_ground.reset(new pcl::PointCloud<pcl::PointXYZ>);
        points_ground->header.frame_id = this->_camera_frame_id;
    }
    if (this->_pub_points_no_ground != nullptr) {
        points_no_ground.reset(new pcl::PointCloud<pcl::PointXYZ>);
        points_no_ground->header.frame_id = this->_camera_frame_id;
    }
    if (this->_pub_points_dynamic != nullptr) {
        points_dynamic.reset(new pcl::PointCloud<pcl::PointXYZ>);
        points_dynamic->header.frame_id = this->_camera_frame_id;
    }
    if (this->_pub_points_static != nullptr) {
        points_static.reset(new pcl::PointCloud<pcl::PointXYZ>);
        points_static->header.frame_id = this->_camera_frame_id;
    }

    for (int y = 0; y < this->_shape.height; y++) {
        float *points_x_cv_ptr = points_x_cv.ptr<float>(y);
        float *points_y_cv_ptr = points_y_cv.ptr<float>(y);
        float *points_z_cv_ptr = points_z_cv.ptr<float>(y);
        uint8_t *label_cv_ptr = label_cv.ptr<uint8_t>(y);
        uint8_t *label_edge_ptr = label_edge.ptr<uint8_t>(y);

        for (int x = 0; x < this->_shape.width; x++) {
            if (std::isinf(points_z_cv_ptr[x]) == true) continue;
            if (label_edge_ptr[x] != 0) continue;
            auto seg_label_itr = this->_seg_labels.find(static_cast<int>(label_cv_ptr[x]));
            if (seg_label_itr == this->_seg_labels.end()) continue;

            pcl::PointXYZ point;
            if (this->_use_optical_frame == true) {
                point.x = points_x_cv_ptr[x];
                point.y = points_y_cv_ptr[x];
                point.z = points_z_cv_ptr[x];
            }
            else {
                point.x = points_z_cv_ptr[x];
                point.y = -points_x_cv_ptr[x];
                point.z = -points_y_cv_ptr[x];
            }
            if (seg_label_itr->second.is_ground == true && this->_pub_points_ground != nullptr) points_ground->points.push_back(point);
            else if (seg_label_itr->second.is_ground == false && this->_pub_points_no_ground != nullptr) points_no_ground->points.push_back(point);
            if (seg_label_itr->second.is_dynamic == true && this->_pub_points_dynamic != nullptr) points_dynamic->points.push_back(point);
            else if (seg_label_itr->second.is_dynamic == false && this->_pub_points_static != nullptr) points_static->points.push_back(point);
        }
    }

    if (this->_pub_points_ground != nullptr) {
        sensor_msgs::PointCloud2 points_ground_msg;
        pcl::toROSMsg(*points_ground, points_ground_msg);
        points_ground_msg.header.stamp = camera_msg->header.stamp;
        this->_pub_points_ground->publish(points_ground_msg);
    }

    if (this->_pub_points_no_ground != nullptr) {
        sensor_msgs::PointCloud2 points_no_ground_msg;
        pcl::toROSMsg(*points_no_ground, points_no_ground_msg);
        points_no_ground_msg.header.stamp = camera_msg->header.stamp;
        this->_pub_points_no_ground->publish(points_no_ground_msg);
    }

    if (this->_pub_points_dynamic != nullptr) {
        sensor_msgs::PointCloud2 points_dynamic_msg;
        pcl::toROSMsg(*points_dynamic, points_dynamic_msg);
        points_dynamic_msg.header.stamp = camera_msg->header.stamp;
        this->_pub_points_dynamic->publish(points_dynamic_msg);
    }

    if (this->_pub_points_static != nullptr) {
        sensor_msgs::PointCloud2 points_static_msg;
        pcl::toROSMsg(*points_static, points_static_msg);
        points_static_msg.header.stamp = camera_msg->header.stamp;
        this->_pub_points_static->publish(points_static_msg);
    }
}

void PMOD::_broadcast_static_tf(const std::string &parent, std::string &child)
{
    child = parent + "/base";
    this->_static_br.reset(new tf2_ros::StaticTransformBroadcaster);
    geometry_msgs::TransformStamped tfstamp_msg;
    tfstamp_msg.header.frame_id = parent;
    tfstamp_msg.child_frame_id = child;
    tfstamp_msg.transform.rotation.w = 0.5;
    tfstamp_msg.transform.rotation.x = 0.5;
    tfstamp_msg.transform.rotation.y = -0.5;
    tfstamp_msg.transform.rotation.z = 0.5;
    this->_static_br->sendTransform(tfstamp_msg);
}
