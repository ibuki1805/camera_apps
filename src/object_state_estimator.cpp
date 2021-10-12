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

        // bbox_sub_ = nh.subscribe("/bounding_box", 10, &ObjectStateEstimator::bbox_callback, this);
        // pc_sub_ = nh.subscribe("/camera/depth_registered/points", 10, &ObjectStateEstimator::pc_callback, this);
        object_pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/object_state/object_pc", 1);

        bbox_sub_  = new message_filters::Subscriber<camera_apps_msgs::BoundingBox> (nh, "/bounding_box", 5);
        pc_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/camera/depth_registered/points", 5);
        sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *bbox_sub_, *pc_sub_);
        // sync_->registerCallback(boost::bind(&ObjectStateEstimator::sync_callback, this, _1, _2));
        sync_->registerCallback(&ObjectStateEstimator::sync_callback, this);

        // object_state_pub_ = nh.advertise<camera_apps_msgs::ObjectState>("/object_state", 5);
        centroid_pub_ = nh.advertise<geometry_msgs::PointStamped>("/object_state/centroid", 1);
        // outlier_pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/outlier_pc", 1);
    }

    ObjectStateEstimator::~ObjectStateEstimator()
    {
        delete this->bbox_sub_;
        delete this->pc_sub_;
        delete this->sync_;
    }

    void ObjectStateEstimator::sync_callback(const camera_apps_msgs::BoundingBoxConstPtr &bbox_msg,
            const sensor_msgs::PointCloud2ConstPtr &pc_msg)

    {
        // std::cout << "sync_callback" << std::endl;
        pcl::fromROSMsg(*pc_msg, *input_pc_);

        bbox_ = *bbox_msg;
        if(bbox_.xmin < 0) bbox_.xmin = 0;
        if(bbox_.ymin < 0) bbox_.ymin = 0;
        if(bbox_.xmax >= img_width_) bbox_.xmax = img_width_ - 1;
        if(bbox_.ymax >= img_height_) bbox_.ymax = img_height_ - 1;


        create_object_pc();
        // object_state_.label = bbox_.label;
        // object_state_.confidence = bbox_.confidence;

        object_pc_pub_.publish(object_pc_);
        centroid_pub_.publish(centroid_msg_);
        // coloring_outlier_pc();
        // outlier_pc_pub_.publish(outlier_pc_);
        // object_state_pub_.publish(object_state_);

    }
    void ObjectStateEstimator::create_object_pc()
    {
        object_pc_->header.stamp = input_pc_->header.stamp;
        object_pc_->header.frame_id = input_pc_->header.frame_id;
        object_pc_->is_dense = 0;

        object_pc_->width = bbox_.xmax - bbox_.xmin + 1;
        object_pc_->height = bbox_.ymax - bbox_.ymin + 1;
        // object_pc_->points.resize(object_pc_->width * object_pc_->height);
        int width = bbox_.xmax - bbox_.xmin + 1;
        int height = bbox_.ymax - bbox_.ymin + 1;
        int size = width * height;
        object_pc_->points.clear();
        int step = size / points_limit_;
        for(int y=bbox_.ymin; y<bbox_.ymax; y++){
            // for(int x=bbox_.xmin; x < bbox_.xmax; x+=step)
            for(int x=bbox_.xmin; x < bbox_.xmax; x++)
            {
                object_pc_->points.push_back(input_pc_->points[y * input_pc_->width + x]);
            }
        }

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*object_pc_, *object_pc_, indices);
        
        downsampling(object_pc_);
        std::cout << "before: " << object_pc_->points.size();
        // remove_outlier();
        clustering(object_pc_);
        std::cout << " after: " << object_pc_->points.size() << std::endl;
        pcl::computeCentroid(*object_pc_, centroid_pcl_);
        
        centroid_msg_.header.frame_id = object_pc_->header.frame_id;
        centroid_msg_.header.stamp = ros::Time::now();
        centroid_msg_.point.x = centroid_pcl_.x;
        centroid_msg_.point.y = centroid_pcl_.y;
        centroid_msg_.point.z = centroid_pcl_.z;
        // object_state_.centroid.point.x = x_sum / object_pc_->points.size();
        // object_state_.centroid.point.y = y_sum / object_pc_->points.size();
        // object_state_.centroid.point.z = z_sum / object_pc_->points.size();

        // pcl::toROSMsg(*object_pc_, object_pc_msg_); 
        // object_state_.object_pc = object_pc_msg_;
        // object_state_.header = object_pc_msg_.header;
        // object_state_.centroid.header = object_pc_msg_.header;
    }

    void ObjectStateEstimator::downsampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pc)
    {
        int step = input_pc->points.size() / points_limit_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_pc(new pcl::PointCloud<pcl::PointXYZRGB>);

        tmp_pc->header = input_pc->header;
        tmp_pc->is_dense = true;

        for(int i=0; i<input_pc->points.size(); i+=step){
            tmp_pc->points.push_back(input_pc->points[i]);
        }
        tmp_pc->width = tmp_pc->points.size();
        tmp_pc->height = 1;

        *input_pc = *tmp_pc;

    }
    void ObjectStateEstimator::downsampling_pcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pc,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_pc)
    {
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud(input_pc);
        vg.setLeafSize(leafsize_, leafsize_, leafsize_);

        if(input_pc == output_pc){
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
            vg.filter(*tmp);
                *output_pc = *tmp;
        }
        else{
            vg.filter(*output_pc);
        }
    }

    void ObjectStateEstimator::remove_outlier()
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(object_pc_);
        sor.setMeanK (mean_k_);
        sor.setStddevMulThresh (std_dev_th_);
        sor.setNegative (true);
        sor.filter (*outlier_pc_);
        sor.setNegative (false);
        sor.filter (*object_pc_);
    }

    void ObjectStateEstimator::coloring_outlier_pc()
    {
        for(auto &p: outlier_pc_->points){
            p.r = 255;
            p.g = 0;
            p.b = 0;
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


        std::cout << "cluster num: " << cluster_indices.size() << std::endl;
        std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
            cloud_cluster->points.push_back (pc_in->points[*pit]); //*
        }

        cloud_cluster->header = pc_in->header;
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        *pc_in = *cloud_cluster;
    }

}
