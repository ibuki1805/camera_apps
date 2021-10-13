#ifndef OBJECT_STATE_ESTIMATOR
#define OBJECT_STATE_ESTIMATOR

#include <typeinfo>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <camera_apps_msgs/BoundingBox.h>
#include <camera_apps_msgs/BoundingBoxes.h>
#include <camera_apps_msgs/ObjectState.h>
#include <camera_apps_msgs/ObjectStates.h>

namespace camera_apps
{
    class ObjectStateEstimator
    {
        public:
            ObjectStateEstimator(ros::NodeHandle &nh, ros::NodeHandle &pnh);
            ~ObjectStateEstimator();
        private:
            void sync_callback(const camera_apps_msgs::BoundingBoxesConstPtr &bboxes_msg,
                    const sensor_msgs::PointCloud2ConstPtr &pc_msg);
            // void sync_callback(const camera_apps_msgs::BoundingBoxConstPtr &bbox_msg,
            //         const sensor_msgs::PointCloud2ConstPtr &pc_msg);

            void adjust_bbox(camera_apps_msgs::BoundingBox& bbox);
            void create_object_pc(camera_apps_msgs::BoundingBox bbox);
            // void create_object_pc();
            void downsampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_in);
            void downsampling_pcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_in);
            void remove_outlier(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_in);           
            void coloring_pc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_in, int red, int green, int blue);
            void clustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_in);
            geometry_msgs::PointStamped caluculate_centroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_in);


            int img_width_;
            int img_height_;
            int mean_k_;
            double std_dev_th_;
            double leafsize_;
            int points_limit_;
            double cluster_tolerance_;
            int min_cluster_size_;
            int max_cluster_size_;

            camera_apps_msgs::BoundingBoxes bboxes_;
            // camera_apps_msgs::BoundingBox bbox_;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pc_ {new pcl::PointCloud<pcl::PointXYZRGB>};
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pc_ {new pcl::PointCloud<pcl::PointXYZRGB>};
            // sensor_msgs::PointCloud2 object_pc_msg_;
            camera_apps_msgs::ObjectStates object_states_;

            typedef message_filters::sync_policies::ApproximateTime<camera_apps_msgs::BoundingBoxes,
                    sensor_msgs::PointCloud2> MySyncPolicy;
            // typedef message_filters::sync_policies::ApproximateTime<camera_apps_msgs::BoundingBox,
            //         sensor_msgs::PointCloud2> MySyncPolicy;
            
            message_filters::Subscriber<camera_apps_msgs::BoundingBoxes> *bboxes_sub_;
            // message_filters::Subscriber<camera_apps_msgs::BoundingBox> *bbox_sub_;
            message_filters::Subscriber<sensor_msgs::PointCloud2> *pc_sub_;
            message_filters::Synchronizer<MySyncPolicy> *sync_;

            ros::Publisher object_states_pub_;
            ros::Publisher object_pc_pub_;
            ros::Publisher centroid_pub_;
            
            tf2_ros::Buffer tf_buffer_;
            tf2_ros::TransformListener* tf2_listener_;
    };
}

#endif
