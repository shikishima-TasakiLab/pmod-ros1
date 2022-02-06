#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/extract_indices.h>

#include "subscriber.hpp"

#define THROTTLE_PERIOD 30
#define DEFAULT_OCTREE_RESOLUTION 0.3F
#define DEFAULT_OCTREE_RADIUS 10.0F
#define DEFAULT_FRAME_ID "map"

class StaticNoGroundMapSubscriber: public Subscriber<sensor_msgs::PointCloud2>
{
public:
    StaticNoGroundMapSubscriber(
        ros::NodeHandle &node_handle,
        const std::string &topic,
        const uint32_t queue_size
    ):  Subscriber(node_handle, topic, queue_size)
    {
        XmlRpc::XmlRpcValue init_maps;
        node_handle.getParam("init_maps", init_maps);
        node_handle.getParam("init_frame_id", this->_frame_id);
        node_handle.getParam("radius", this->_radius);
        node_handle.getParam("resolution", this->_octree_resolution);

        this->_octree.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(this->_octree_resolution));

        if (init_maps.getType() != XmlRpc::XmlRpcValue::TypeArray) return;

        pcl::PointCloud<pcl::PointXYZ> init_map;

        for (size_t i = 0UL; i < init_maps.size(); i++) {
            if (init_maps[i].getType() != XmlRpc::XmlRpcValue::TypeString) continue;

            pcl::PointCloud<pcl::PointXYZ> tmp_map;
            std::string pcd_path = static_cast<std::string>(init_maps[i]);
            if (pcl::io::loadPCDFile(pcd_path, tmp_map) == -1) {
                ROS_ERROR_STREAM("Couldn't read file \"" << pcd_path <<  "\".");
                continue;
            }
            ROS_INFO_STREAM("Load \"" << pcd_path << "\".");

            init_map += tmp_map;
        }

        this->_map.reset(new pcl::PointCloud<pcl::PointXYZ>(init_map));

        this->_octree->setInputCloud(this->_map);
        this->_octree->addPointsFromInputCloud();
    }

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr get_map(
        const geometry_msgs::TransformStamped &tf_map2camera
    ) {
        Eigen::Vector3f tr(tf2::transformToEigen(tf_map2camera).translation().cast<float>());
        pcl::PointXYZ search_point(tr.x(), tr.y(), tr.z());

        if (
            this->_search_point.x == search_point.x &&
            this->_search_point.y == search_point.y &&
            this->_search_point.z == search_point.z
        ) {
            std::lock_guard<std::mutex> lock(*this->_mtx);
            return this->_filterd_map;
        }
        this->_search_point = search_point;

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

        std::vector<float> point_squared_distance;

        {
            std::lock_guard<std::mutex> lock(*this->_mtx);

            this->_octree->radiusSearch(search_point, this->_radius, inliers->indices, point_squared_distance);
            extract.setInputCloud(this->_map);
            extract.setIndices(inliers);
            this->_filterd_map.reset(new pcl::PointCloud<pcl::PointXYZ>);
            extract.filter(*this->_filterd_map);

            return this->_filterd_map;
        }
    }

    std::string get_frame_id() {return this->_frame_id;}

protected:
    boost::shared_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> > _octree;
    float _octree_resolution = DEFAULT_OCTREE_RESOLUTION;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _map;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _filterd_map;
    pcl::PointXYZ _search_point;
    float _radius = DEFAULT_OCTREE_RADIUS;
    std::string _frame_id = DEFAULT_FRAME_ID;

    void _callback(const sensor_msgs::PointCloud2::ConstPtr &msg) override {
        std::lock_guard<std::mutex> lock(*this->_mtx);
        this->_frame_id = msg->header.frame_id;

        this->_map.reset(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *this->_map);

        this->_octree.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(this->_octree_resolution));
        this->_octree->setInputCloud(this->_map);
        this->_octree->addPointsFromInputCloud();
    }
};

class Dynamic2NoGround
{
public:
    Dynamic2NoGround();
protected:
    ros::NodeHandlePtr _nh;
    boost::shared_ptr<StaticNoGroundMapSubscriber> _static_noground_sub;
    boost::shared_ptr<Subscriber<sensor_msgs::PointCloud2> > _dynamic_sub;

    boost::shared_ptr<tf2_ros::Buffer> _tf_buffer;
    boost::shared_ptr<tf2_ros::TransformListener> _tf_listener;
    boost::shared_ptr<ros::Publisher> _noground_pub;
    boost::shared_ptr<ros::Timer> _timer;

    double _hz = 10.0;
    void _transform_pointcloud(
        const pcl::PointCloud<pcl::PointXYZ> &src,
        pcl::PointCloud<pcl::PointXYZ> &dst,
        const geometry_msgs::TransformStamped &tf
    );
    void _timer_callback(const ros::TimerEvent &e);
};
