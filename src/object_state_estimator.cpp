#include <object_state_estimator/object_state_estimator.h>

namespace camera_apps
{
    ObjectStateEstimator::ObjectStateEstimator(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    {
        pnh.param("img_width", img_width_, 640);
        pnh.param("img_height", img_height_, 480);
        pnh.param("mean_k", mean_k_, 50);
        pnh.param("std_dev_th", std_dev_th_, 0.1);
        pnh.param("leafsize", leafsize_, 0.01);
        pnh.param("points_limit", points_limit_, 10000);
        pnh.param("cluster_tolerance", cluster_tolerance_, 0.02);
        pnh.param("min_cluster_size", min_cluster_size_, 1000);
        pnh.param("max_cluster_size", max_cluster_size_, 20000);

        object_pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/object_state/object_pc", 1);

        bboxes_sub_  = new message_filters::Subscriber<camera_apps_msgs::BoundingBoxes> (nh, "/bounding_boxes", 5);
        // bbox_sub_  = new message_filters::Subscriber<camera_apps_msgs::BoundingBox> (nh, "/bounding_box", 5);
        pc_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/camera/depth_registered/points", 5);
        sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *bboxes_sub_, *pc_sub_);
        sync_->registerCallback(&ObjectStateEstimator::sync_callback, this);

        object_states_pub_ = nh.advertise<camera_apps_msgs::ObjectStates>("/object_states", 1);
        centroid_pub_ = nh.advertise<geometry_msgs::PointStamped>("/object_state/centroid", 1);

        tf2_listener_ = new tf2_ros::TransformListener(tf_buffer_);
    }

    ObjectStateEstimator::~ObjectStateEstimator()
    {
        delete this->bboxes_sub_;
        delete this->pc_sub_;
        delete this->sync_;
        delete this->tf2_listener_;
    }

    void ObjectStateEstimator::sync_callback(const camera_apps_msgs::BoundingBoxesConstPtr &bboxes_msg,
            const sensor_msgs::PointCloud2ConstPtr &pc_msg)
    {
        try{
            geometry_msgs::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_.lookupTransform("camera_fixed_frame", "camera_color_optical_frame", ros::Time(0));
            Eigen::Matrix4f mat = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();

            sensor_msgs::PointCloud2 pc_msg_temp;
            pcl_ros::transformPointCloud(mat, *pc_msg, pc_msg_temp);
            pcl::fromROSMsg(pc_msg_temp, *input_pc_);
            input_pc_->header.frame_id = "camera_fixed_frame";
        }
        catch(tf2::TransformException &ex){
            ROS_WARN("%s", ex.what());
            return;
        }

        bboxes_ = *bboxes_msg;
        pcl_conversions::fromPCL(input_pc_->header, object_states_.header);
        object_states_.object_states.clear();
        for(auto bbox: bboxes_msg->bounding_boxes){
            camera_apps_msgs::ObjectState object_state;

            object_state.label = bbox.label;
            object_state.confidence = bbox.confidence;

            adjust_bbox(bbox);
            create_object_pc(bbox);

            geometry_msgs::PointStamped centroid = caluculate_centroid(object_pc_);
            object_state.centroid = centroid;

            pcl::toROSMsg(*object_pc_, object_state.object_pc);

            object_states_.object_states.push_back(object_state);
            object_pc_pub_.publish(object_pc_);
            centroid_pub_.publish(centroid);
        }
        object_states_pub_.publish(object_states_);
    }
    // void ObjectStateEstimator::sync_callback(const camera_apps_msgs::BoundingBoxConstPtr &bbox_msg,
    //         const sensor_msgs::PointCloud2ConstPtr &pc_msg)
    //
    // {
    //     try{
    //         geometry_msgs::TransformStamped transform_stamped;
    //         transform_stamped = tf_buffer_.lookupTransform("camera_fixed_frame", "camera_color_optical_frame", ros::Time(0));
    //         Eigen::Matrix4f mat = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
    //
    //         sensor_msgs::PointCloud2 pc_msg_temp;
    //         pcl_ros::transformPointCloud(mat, *pc_msg, pc_msg_temp);
    //         pcl::fromROSMsg(pc_msg_temp, *input_pc_);
    //         input_pc_->header.frame_id = "camera_fixed_frame";
    //     }
    //     catch(tf2::TransformException &ex){
    //         ROS_WARN("%s", ex.what());
    //         return;
    //     }
    //
    //     bbox_ = *bbox_msg;
    //     if(bbox_.xmin < 0) bbox_.xmin = 0;
    //     if(bbox_.ymin < 0) bbox_.ymin = 0;
    //     if(bbox_.xmax >= img_width_) bbox_.xmax = img_width_ - 1;
    //     if(bbox_.ymax >= img_height_) bbox_.ymax = img_height_ - 1;
    //
    //
    //     create_object_pc();
    //     // object_state_.label = bbox_.label;
    //     // object_state_.confidence = bbox_.confidence;
    //
    //     object_pc_pub_.publish(object_pc_);
    //     centroid_pub_.publish(centroid_msg_);
    //     // object_state_pub_.publish(object_state_);
    //
    // }

    void ObjectStateEstimator::adjust_bbox(camera_apps_msgs::BoundingBox& bbox)
    {
        if(bbox.xmin < 0) bbox.xmin = 0;
        if(bbox.ymin < 0) bbox.ymin = 0;
        if(bbox.xmax >= img_width_) bbox.xmax = img_width_ - 1;
        if(bbox.ymax >= img_height_) bbox.ymax = img_height_ - 1;
    }

    void ObjectStateEstimator::create_object_pc(camera_apps_msgs::BoundingBox bbox)
    {

        object_pc_->header = input_pc_->header;
        object_pc_->is_dense = 0;

        object_pc_->points.clear();
        for(int y=bbox.ymin; y<bbox.ymax; y++){
            for(int x=bbox.xmin; x < bbox.xmax; x++)
            {
                object_pc_->points.push_back(input_pc_->points[y * input_pc_->width + x]);
            }
        }

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*object_pc_, *object_pc_, indices);
        
        downsampling(object_pc_);
        clustering(object_pc_);

        // pcl::toROSMsg(*object_pc_, object_pc_msg_); 
        // object_state_.object_pc = object_pc_msg_;
        // object_state_.header = object_pc_msg_.header;
        // object_state_.centroid.header = object_pc_msg_.header;
    }
    // void ObjectStateEstimator::create_object_pc()
    // {
    //     object_pc_->header = input_pc_->header;
    //     object_pc_->is_dense = 0;
    //
    //     object_pc_->width = bbox_.xmax - bbox_.xmin + 1;
    //     object_pc_->height = bbox_.ymax - bbox_.ymin + 1;
    //     object_pc_->points.clear();
    //     for(int y=bbox_.ymin; y<bbox_.ymax; y++){
    //         for(int x=bbox_.xmin; x < bbox_.xmax; x++)
    //         {
    //             object_pc_->points.push_back(input_pc_->points[y * input_pc_->width + x]);
    //         }
    //     }
    //
    //     std::vector<int> indices;
    //     pcl::removeNaNFromPointCloud(*object_pc_, *object_pc_, indices);
    //     
    //     downsampling(object_pc_);
    //     // std::cout << "before: " << object_pc_->points.size();
    //     // remove_outlier();
    //     clustering(object_pc_);
    //     // std::cout << " after: " << object_pc_->points.size() << std::endl;
    //     pcl::computeCentroid(*object_pc_, centroid_pcl_);
    //     // std::cout << "x: " << centroid_pcl_.x << " y: " << centroid_pcl_.y << " z: " << centroid_pcl_.z << std::endl;
    //     
    //     pcl_conversions::fromPCL(object_pc_->header, centroid_msg_.header);
    //     centroid_msg_.point.x = centroid_pcl_.x;
    //     centroid_msg_.point.y = centroid_pcl_.y;
    //     centroid_msg_.point.z = centroid_pcl_.z;
    //     // object_state_.centroid.point.x = x_sum / object_pc_->points.size();
    //     // object_state_.centroid.point.y = y_sum / object_pc_->points.size();
    //     // object_state_.centroid.point.z = z_sum / object_pc_->points.size();
    //
    //     // pcl::toROSMsg(*object_pc_, object_pc_msg_); 
    //     // object_state_.object_pc = object_pc_msg_;
    //     // object_state_.header = object_pc_msg_.header;
    //     // object_state_.centroid.header = object_pc_msg_.header;
    // }

    void ObjectStateEstimator::downsampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_in)
    {
        int step = pc_in->points.size() / points_limit_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_pc(new pcl::PointCloud<pcl::PointXYZRGB>);

        tmp_pc->header = pc_in->header;
        tmp_pc->is_dense = true;

        for(int i=0; i<pc_in->points.size(); i+=step){
            tmp_pc->points.push_back(pc_in->points[i]);
        }
        tmp_pc->width = tmp_pc->points.size();
        tmp_pc->height = 1;

        *pc_in = *tmp_pc;
    }
    void ObjectStateEstimator::downsampling_pcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_in)
    {
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud(pc_in);
        vg.setLeafSize(leafsize_, leafsize_, leafsize_);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
        vg.filter(*tmp);
        *pc_in = *tmp;
    }

    void ObjectStateEstimator::remove_outlier(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_in)
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(pc_in);
        sor.setMeanK (mean_k_);
        sor.setStddevMulThresh (std_dev_th_);
        sor.setNegative (false);
        sor.filter (*pc_in);
    }

    void ObjectStateEstimator::coloring_pc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_in, int red, int green, int blue)
    {
        for(auto &p: pc_in->points){
            p.r = red;
            p.g = green;
            p.b = blue;
        }
    }

    void ObjectStateEstimator::clustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_in)
    {
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree->setInputCloud (pc_in);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance (cluster_tolerance_);
        ec.setMinClusterSize (min_cluster_size_);//最小のクラスターの値を設定
        ec.setMaxClusterSize (max_cluster_size_);//最大のクラスターの値を設定
        ec.setSearchMethod (tree);//検索に使用する手法を指定
        ec.setInputCloud (pc_in);//点群を入力
        ec.extract (cluster_indices);//クラスター情報を出力


        // std::cout << "cluster num: " << cluster_indices.size() << std::endl;
        std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
            cloud_cluster->points.push_back (pc_in->points[*pit]); 
        }

        cloud_cluster->header = pc_in->header;
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        *pc_in = *cloud_cluster;
    }

    geometry_msgs::PointStamped ObjectStateEstimator::caluculate_centroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_in)
    {
        geometry_msgs::PointStamped centroid_msg;
        pcl::PointXYZ centroid_pcl;

        pcl::computeCentroid(*pc_in, centroid_pcl);
        
        pcl_conversions::fromPCL(pc_in->header, centroid_msg.header);
        centroid_msg.point.x = centroid_pcl.x;
        centroid_msg.point.y = centroid_pcl.y;
        centroid_msg.point.z = centroid_pcl.z;

        return centroid_msg;
    }

}
