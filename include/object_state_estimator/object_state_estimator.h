#ifndef OBJECT_STATE_ESTIMATOR
#define OBJECT_STATE_ESTIMATOR

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <camera_apps_msgs/BoundingBox.h>

namespace camera_apps
{
    class ObjectStateEstimator
    {
        public:
            ObjectStateEstimator(ros::NodeHandle &nh, ros::NodeHandle &pnh);
        private:
            void bbox_callback(const camera_apps_msgs::BoundingBoxConstPtr &msg);
            void pcl_callback(const sensor_msgs::PointCloud2ConstPtr &msg);
            void create_object_pcl();

            bool pcl_get_flag_ = false;

            camera_apps_msgs::BoundingBox bbox_;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pcl_ {new pcl::PointCloud<pcl::PointXYZRGB>};
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pcl_ {new pcl::PointCloud<pcl::PointXYZRGB>};

            ros::Subscriber bbox_sub_;
            ros::Subscriber pcl_sub_;
            ros::Publisher pcl_pub_;
    };
}

#endif
