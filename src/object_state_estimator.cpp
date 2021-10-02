#include <object_state_estimator/object_state_estimator.h>

namespace camera_apps
{
    ObjectStateEstimator::ObjectStateEstimator(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    {
        bbox_sub_ = nh.subscribe("/bounding_box", 5, &ObjectStateEstimator::bbox_callback, this);
        pcl_sub_ = nh.subscribe("/camera/depth_registered/points", 5, &ObjectStateEstimator::pcl_callback, this);
        pcl_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/object_pcl", 1);
    }

    void ObjectStateEstimator::bbox_callback(const camera_apps_msgs::BoundingBoxConstPtr &msg)
    {
        // std::cout << "bbox get" << std::endl;
        bbox_ = *msg;
        if(pcl_get_flag_){
            create_object_pcl();
            pcl_pub_.publish(object_pcl_);
        }
    }
    void ObjectStateEstimator::pcl_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        input_pcl_->points.clear();
        pcl::fromROSMsg(*msg, *input_pcl_);
        pcl_get_flag_ = true;
    }

    void ObjectStateEstimator::create_object_pcl()
    {
        object_pcl_->header.stamp = input_pcl_->header.stamp;
        object_pcl_->header.frame_id = input_pcl_->header.frame_id;

        object_pcl_->width = bbox_.xmax - bbox_.xmin + 1;
        object_pcl_->height = bbox_.ymax - bbox_.ymin + 1;
        object_pcl_->points.resize(object_pcl_->width * object_pcl_->height);

        int index = 0;
        for(int y=bbox_.ymin; y<bbox_.ymax; y++){
            for(int x=bbox_.xmin; y<bbox_.xmax; x++)
            {
                object_pcl_->points[index] = input_pcl_->points[y * input_pcl_->width + x];
                std::cout << "hi" << std::endl:
                index++;
            }
        }
    }
}
