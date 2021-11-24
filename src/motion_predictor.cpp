#include <motion_predictor/motion_predictor.h>

namespace camera_apps
{
    MotionPredictor::MotionPredictor(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    {
        pnh.param("error_threshold", error_threshold_, 0.5);
        pnh.param("time_threshold", time_threshold_, 2.0);
        pnh.param("past_path_threshold", past_path_threshold_, 50);
        pnh.param("person_num_limit", person_num_limit_, 10);
        pnh.param("colorful_trajectory_flag", colorful_trajectory_flag_, false);
        pnh.param("observation_noise_ratio", observation_noise_ratio_, 0.05);
        pnh.param("sigma_initial_P_theta", sigma_initial_P_theta_, 2 * M_PI);
        pnh.param("sigma_initial_P_velocity", sigma_initial_P_velocity_, 3.0);
        pnh.param("sigma_initial_P_omega", sigma_initial_P_omega_, M_PI);
        pnh.param("sigma_Q_x", sigma_Q_x_, 1.0);
        pnh.param("sigma_Q_y", sigma_Q_y_, 1.0);
        pnh.param("sigma_Q_theta", sigma_Q_theta_, M_PI);
        pnh.param("sigma_Q_velocity", sigma_Q_velocity_, 3.0);
        pnh.param("sigma_Q_omega", sigma_Q_omega_, M_PI);
        pnh.param("trajectory_z", trajectory_z_, 0.0);

        object_states_sub_ = nh.subscribe("/object_states", 5, &MotionPredictor::object_states_callback, this);
        past_trajectory_pub_ = nh.advertise<nav_msgs::Path>("/past_trajectory", 20);
        filtered_past_trajectory_pub_ = nh.advertise<nav_msgs::Path>("/filtered_past_trajectory", 20);
        filtered_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/filtered_pose", 1);
        filtered_pose_array_pub_ = nh.advertise<geometry_msgs::PoseArray>("/filtered_pose_array", 1);

        past_trajectory_pub1_ = nh.advertise<nav_msgs::Path>("/past_trajectory1", 10);
        past_trajectory_pub2_ = nh.advertise<nav_msgs::Path>("/past_trajectory2", 10);
        past_trajectory_pub3_ = nh.advertise<nav_msgs::Path>("/past_trajectory3", 10);
        past_trajectory_pub4_ = nh.advertise<nav_msgs::Path>("/past_trajectory4", 10);
        past_trajectory_pub5_ = nh.advertise<nav_msgs::Path>("/past_trajectory5", 10);
        
        tf2_listener_ = new tf2_ros::TransformListener(tf_buffer_);

        for(int i=0; i<person_num_limit_; i++) free_id_list_.push_back(i);

        set_invariable_matrix();
    }

    MotionPredictor::~MotionPredictor()
    {
        delete this->tf2_listener_;
    }

    void MotionPredictor::object_states_callback(const camera_apps_msgs::ObjectStatesConstPtr &msg)
    {
        object_states_ = *msg;

        for(auto& object_state: object_states_.object_states){
            int best_id = -1;
            double best_error = 1000000000;
            for(const auto& id: valid_id_list_){
                int index = id_to_index(id);
                double registered_x = person_list_[index].centroid.point.x;
                double registered_y = person_list_[index].centroid.point.y;
                double input_x = object_state.centroid.point.x;
                double input_y = object_state.centroid.point.y;

                double error = std::sqrt(std::pow(input_x - registered_x, 2) + std::pow(input_y - registered_y, 2));
                if(error < best_error){
                    best_id = id;
                    best_error = error;
                }

            }
            if(best_error > error_threshold_){
                if(free_id_list_.size() != 0){
                    register_person(object_state);
                }
                else std::cout << "cannot add more person" << std::endl;
            }
            else{
                update_person(best_id, object_state);
            }
        }
        lost_judge();
        if(!colorful_trajectory_flag_){
            visualize_trajectory();
        }
        else{
            visualize_trajectory2();
        }
        visualize_filtered_trajectory();
        visualize_filtered_pose();
        // std::cout << "person num: " << valid_id_list_.size() << std::endl;
    }

    void MotionPredictor::register_person(camera_apps_msgs::ObjectState& object_state)
    {
        PersonInfo new_person;
        int new_id = free_id_list_[0];
        free_id_list_.erase(free_id_list_.begin());

        new_person.id = new_id;
        new_person.centroid = object_state.centroid;
        new_person.filtered_pose.pose.position = object_state.centroid.point;
        new_person.latest_time = object_state.centroid.header.stamp;

        nav_msgs::Path trajectory;
        trajectory.header = object_state.centroid.header;
        update_trajectory(trajectory, object_state.centroid);
        new_person.trajectory = trajectory;
        new_person.filtered_trajectory = trajectory;

        Eigen::VectorXd X(5);
        X(0) = object_state.centroid.point.x;
        X(1) = object_state.centroid.point.y;
        X(2) = 0;
        X(3) = 0;
        X(4) = 0;
        // std::cout << "X: " << std::endl << X << std::endl;
        // std::cout << "X_hat: " << std::endl << calculate_X_hat(X, 0.1) << std::endl;
        // std::cout << "F: " << std::endl << calculate_F(X, 0.1) << std::endl;
        new_person.X = X;

        Eigen::MatrixXd P(5,5);
        P.setZero();
        P(0,0) = X(0) * observation_noise_ratio_;
        P(1,1) = X(1) * observation_noise_ratio_;
        P(2,2) = sigma_initial_P_theta_;
        P(3,3) = sigma_initial_P_velocity_;
        P(4,4) = sigma_initial_P_omega_;
        P *= P;
        new_person.P = P;

        person_list_.push_back(new_person);
        valid_id_list_.push_back(new_id);
    }

    void MotionPredictor::update_person(int id, camera_apps_msgs::ObjectState& object_state)
    {
        int index = id_to_index(id);
        person_list_[index].centroid = object_state.centroid;
        double dt = (object_state.centroid.header.stamp - person_list_[index].latest_time).nsec
            / std::pow(10, 9);
        // std::cout << "dt: " << dt << std::endl;
        person_list_[index].latest_time = object_state.centroid.header.stamp;

        update_trajectory(person_list_[index].trajectory, object_state.centroid);

        Eigen::MatrixXd X_hat = adjust_X(calculate_X_hat(person_list_[index].X, dt));
        // Eigen::MatrixXd X_hat = calculate_X_hat(person_list_[index].X, dt);
        Eigen::MatrixXd P_hat = calculate_P_hat(person_list_[index].X, person_list_[index].P, dt);
        person_list_[index].X = X_hat;
        person_list_[index].P = P_hat;

        double Z_x = object_state.centroid.point.x;
        double Z_y = object_state.centroid.point.y;
        Eigen::MatrixXd K = update_K(P_hat, Z_x, Z_y);
        person_list_[index].K = K;
        person_list_[index].X = adjust_X(update_X(X_hat, K, Z_x, Z_y));
        // person_list_[index].X = update_X(X_hat, K, Z_x, Z_y);
        person_list_[index].P = update_P(K, P_hat);

        geometry_msgs::PoseStamped filtered_pose = create_pose_from_X(person_list_[index].X);
        filtered_pose.header = object_state.centroid.header;
        person_list_[index].filtered_pose = filtered_pose;
        person_list_[index].filtered_trajectory.poses.push_back(filtered_pose);

        if(person_list_[index].trajectory.poses.size() > past_path_threshold_){
            person_list_[index].trajectory.poses.erase(person_list_[index].trajectory.poses.begin());
            person_list_[index].filtered_trajectory.poses.erase(person_list_[index].filtered_trajectory.poses.begin());
        }
    }

    void MotionPredictor::update_trajectory(nav_msgs::Path& trajectory, geometry_msgs::PointStamped centroid)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = centroid.header;
        pose.pose.position.x = centroid.point.x;
        pose.pose.position.y = centroid.point.y;
        pose.pose.position.z = trajectory_z_;
        trajectory.poses.push_back(pose);
    }

    void MotionPredictor::update_trajectory(nav_msgs::Path& trajectory, geometry_msgs::PoseStamped filtered_pose)
    {
        // geometry_msgs::PoseStamped pose;
        // pose.header = filtered_pose.header;
        // pose.pose.position.x = filtered_pose.pose.position.x;
        // pose.pose.position.y = filtered_pose.pose.position.y;
        // pose.pose.position.z = trajectory_z_;
        // pose.pose.orientation = filtered_pose.pose.orientation;
        trajectory.poses.push_back(filtered_pose);
    }
    
    void MotionPredictor::delete_person(int id)
    {
        int index = id_to_index(id);
        for(int i=0; i<valid_id_list_.size(); i++){
            if(valid_id_list_[i] == id){
                // person_list_[index].trajectory.poses.clear();
                person_list_.erase(person_list_.begin() + index);
                valid_id_list_.erase(valid_id_list_.begin() + i);
                free_id_list_.push_back(id);

                return;
            }
        }
    }

    void MotionPredictor::lost_judge()
    {
        ros::Time current_time = object_states_.header.stamp; 
        for(const auto& id: valid_id_list_){
            int index = id_to_index(id);
            ros::Duration time_blank = current_time - person_list_[index].latest_time;
            if(std::abs(time_blank.sec) > time_threshold_) delete_person(id);
        }
    }
    
    int MotionPredictor::id_to_index(int id)
    {
        for(int index=0; index<person_list_.size(); index++){
            if(person_list_[index].id == id) return index;
        }
    }

    void MotionPredictor::visualize_trajectory()
    {
        // std::cout << "person_list_size: " << person_list_.size() << std::endl;
        for(const auto& person_info: person_list_){
            past_trajectory_pub_.publish(person_info.trajectory);
        }
    }
    void MotionPredictor::visualize_trajectory2()
    {
        // std::cout << "person_list_size: " << person_list_.size() << std::endl;
        // for(const auto& person_info: person_list_){
        //     past_trajectory_pub_.publish(person_info.trajectory);
        // }
        int limit = 5;
        if(limit > person_list_.size()) limit = person_list_.size();
        // for(int i=0; i<limit; i++){
        //     
        // }
        int i=0;
        if(i>limit-1) return;
        past_trajectory_pub1_.publish(person_list_[0].trajectory);
        i++;
        if(i>limit-1) return;
        past_trajectory_pub2_.publish(person_list_[1].trajectory);
        i++;
        if(i>limit-1) return;
        past_trajectory_pub3_.publish(person_list_[2].trajectory);
        i++;
        if(i>limit-1) return;
        past_trajectory_pub4_.publish(person_list_[3].trajectory);
        i++;
        if(i>limit-1) return;
        past_trajectory_pub5_.publish(person_list_[4].trajectory);
    }

    void MotionPredictor::visualize_filtered_trajectory()
    {
        for(const auto& person_info: person_list_){
            filtered_past_trajectory_pub_.publish(person_info.filtered_trajectory);
        }
    }

    void MotionPredictor::visualize_filtered_pose()
    {
        geometry_msgs::PoseArray filtered_pose_array;
        filtered_pose_array.header.frame_id = person_list_[0].trajectory.header.frame_id;
        for(const auto& person_info: person_list_){
            filtered_pose_array.poses.push_back(person_info.filtered_pose.pose);
            std::cout << "velocity: " << person_info.X(3) << std::endl;
        }
        filtered_pose_array_pub_.publish(filtered_pose_array);
    }

    double MotionPredictor::adjust_yaw(double yaw)
    {
        if(yaw > M_PI){yaw -= 2*M_PI;}
        if(yaw < -M_PI){yaw += 2*M_PI;}

        return yaw;
    }

    Eigen::VectorXd MotionPredictor::adjust_X(Eigen::VectorXd X)
    {
        Eigen::VectorXd X_out = X;
        if(X_out(3) < 0){
            X_out(3) *= -1;
            X_out(2) = adjust_yaw(X_out(2) + M_PI);
        }
        return X_out;
    }

    geometry_msgs::PoseStamped MotionPredictor::create_pose_from_X(Eigen::MatrixXd X)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = X(0);
        pose.pose.position.y = X(1);
        pose.pose.position.z = trajectory_z_;
        quaternionTFToMsg(tf::createQuaternionFromYaw(X(2)), pose.pose.orientation);

        return pose;
    }

    void MotionPredictor::set_invariable_matrix()
    {
        Eigen::MatrixXd Q(5,5);
        Q.setZero();
        Q(0,0) = sigma_Q_x_;
        Q(1,1) = sigma_Q_y_;
        Q(2,2) = sigma_Q_theta_;
        Q(3,3) = sigma_Q_velocity_;
        Q(4,4) = sigma_Q_omega_;
        Q_ = Q;

        Eigen::MatrixXd H(2,5);
        H.setIdentity();
        H_ = H;
        // std::cout << "Q: " << std::endl << Q << std::endl;
        // std::cout << "H: " << std::endl << H << std::endl;
    }

    Eigen::VectorXd MotionPredictor::calculate_X_hat(Eigen::VectorXd X, double dt)
    {
        Eigen::VectorXd X_hat(5);
        X_hat(0) = X(0) + X(3) * dt * std::cos(X(2) + X(4) * dt/2);
        X_hat(1) = X(1) + X(3) * dt * std::sin(X(2) + X(4) * dt/2);
        X_hat(2) = adjust_yaw(X(2) + X(4) * dt);
        X_hat(3) = X(3);
        X_hat(4) = X(4);

        return X_hat;
    }

    Eigen::MatrixXd MotionPredictor::calculate_F(Eigen::VectorXd X, double dt)
    {
        Eigen::MatrixXd F(5,5);
        F.setIdentity();
        F(0,2) = -X(3) * dt * std::sin(X(2) + X(4) * dt/2);
        F(0,3) = dt * std::cos(X(2) + X(4) * dt/2);
        F(0,4) = -X(3) * dt*dt/2 * std::sin(X(2) + X(4) * dt/2);
        F(1,2) = X(3) * dt * std::cos(X(2) + X(4) * dt/2);
        F(1,3) = dt * std::sin(X(2) + X(4) * dt/2);
        F(1,4) = X(3) * dt*dt/2 * std::cos(X(2) + X(4) * dt/2);
        F(2,4) = dt;

        return F;
    }

    Eigen::MatrixXd MotionPredictor::calculate_P_hat(Eigen::VectorXd X, Eigen::MatrixXd P, double dt)
    {
        Eigen::MatrixXd P_hat;
        Eigen::MatrixXd F = calculate_F(X, dt);
        P_hat = F * P * F.transpose() + Q_ * dt;

        return P_hat;
    }
    Eigen::VectorXd MotionPredictor::update_X(Eigen::VectorXd X_hat, Eigen::MatrixXd K, double Z_x, double Z_y)
    {
        Eigen::Vector2d Z(Z_x,Z_y);
        Eigen::VectorXd X;
        X = X_hat + K * (Z - H_ * X_hat);
        X(2) = adjust_yaw(X(2));

        return X;
    }

    Eigen::MatrixXd MotionPredictor::update_P(Eigen::MatrixXd K, Eigen::MatrixXd P_hat)
    {
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(5,5);
        Eigen::MatrixXd P;
        P = (I - K * H_) * P_hat;

        return P;
    }

    Eigen::MatrixXd MotionPredictor::update_K(Eigen::MatrixXd P_hat, double Z_x, double Z_y)
    {
        Eigen::MatrixXd R(2,2);
        R.setZero();
        R(0,0) = Z_x * observation_noise_ratio_;
        R(1,1) = Z_y * observation_noise_ratio_;
        R *= R;

        Eigen::MatrixXd K = P_hat * H_.transpose() * (H_ * P_hat * H_.transpose() + R).inverse();

        return K;
    }

}
