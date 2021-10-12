#ifndef OBJECT_STATE_ESTIMATOR
#define OBJECT_STATE_ESTIMATOR

#include <typeinfo>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>


#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <camera_apps_msgs/BoundingBox.h>
#include <camera_apps_msgs/ObjectState.h>

namespace camera_apps
{
    class ObjectStateEstimator
    {
        public:
            ObjectStateEstimator(ros::NodeHandle &nh, ros::NodeHandle &pnh);
            ~ObjectStateEstimator();
        private:
            // void bbox_callback(const camera_apps_msgs::BoundingBoxConstPtr &msg);
            // void pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg);
            void sync_callback(const camera_apps_msgs::BoundingBoxConstPtr &bbox_msg,
                    const sensor_msgs::PointCloud2ConstPtr &pc_msg);

            void create_object_pc();
            void downsampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pc);
            void downsampling_pcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pc,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_pc);
            void remove_outlier();           
            void coloring_outlier_pc();
            void clustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_in);

            // bool pc_get_flag_ = false;

            int img_width_;
            int img_height_;
            int mean_k_;
            double std_dev_th_;
            double leafsize_;
            int points_limit_;
            double cluster_tolerance_;
            int min_cluster_size_;
            int max_cluster_size_;

            camera_apps_msgs::BoundingBox bbox_;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pc_ {new pcl::PointCloud<pcl::PointXYZRGB>};
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pc_ {new pcl::PointCloud<pcl::PointXYZRGB>};
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr outlier_pc_ {new pcl::PointCloud<pcl::PointXYZRGB>};
            pcl::PointXYZ centroid_pcl_;
            geometry_msgs::PointStamped centroid_msg_;
            // sensor_msgs::PointCloud2 object_pc_msg_;
            // camera_apps_msgs::ObjectState object_state_;

            typedef message_filters::sync_policies::ApproximateTime<camera_apps_msgs::BoundingBox,
                    sensor_msgs::PointCloud2> MySyncPolicy;
            
            message_filters::Subscriber<camera_apps_msgs::BoundingBox> *bbox_sub_;
            message_filters::Subscriber<sensor_msgs::PointCloud2> *pc_sub_;
            message_filters::Synchronizer<MySyncPolicy> *sync_;

            // ros::Subscriber bbox_sub_;
            // ros::Subscriber pc_sub_;
            // ros::Publisher object_state_pub_;
            ros::Publisher object_pc_pub_;
            ros::Publisher outlier_pc_pub_;
            ros::Publisher centroid_pub_;
    };
}

#endif
