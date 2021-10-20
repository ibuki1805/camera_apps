#include <motion_predictor/motion_predictor.h>

namespace camera_apps
{
    MotionPredictor::MotionPredictor(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    {
        pnh.param("error_threshold", error_threshold_, 0.5);
        pnh.param("time_threshold", time_threshold_, 2.0);
        pnh.param("past_path_threshold", past_path_threshold_, 50);
        pnh.param("person_num_limit", person_num_limit_, 10);
        object_states_sub_ = nh.subscribe("/object_states", 5, &MotionPredictor::object_states_callback, this);
        past_trajectory_pub_ = nh.advertise<nav_msgs::Path>("/past_trajectory", 20);

        
        tf2_listener_ = new tf2_ros::TransformListener(tf_buffer_);

        for(int i=0; i<person_num_limit_; i++) free_id_list_.push_back(i);
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
        visualize_trajectory();
        // std::cout << "person num: " << valid_id_list_.size() << std::endl;
    }

    void MotionPredictor::register_person(camera_apps_msgs::ObjectState& object_state)
    {
        PersonInfo new_person;
        int new_id = free_id_list_[0];
        free_id_list_.erase(free_id_list_.begin());

        new_person.id = new_id;
        new_person.centroid = object_state.centroid;
        new_person.latest_time = object_state.centroid.header.stamp;

        nav_msgs::Path trajectory;
        trajectory.header = object_state.centroid.header;
        geometry_msgs::PoseStamped pose;
        pose.header = object_state.centroid.header;
        pose.pose.position.x = object_state.centroid.point.x;
        pose.pose.position.y = object_state.centroid.point.y;
        pose.pose.position.z = object_state.centroid.point.z;
        trajectory.poses.push_back(pose);
        new_person.trajectory = trajectory;

        person_list_.push_back(new_person);
        valid_id_list_.push_back(new_id);
    }

    void MotionPredictor::update_person(int id, camera_apps_msgs::ObjectState& object_state)
    {
        int index = id_to_index(id);
        person_list_[index].centroid = object_state.centroid;
        person_list_[index].latest_time = object_state.centroid.header.stamp;

        
        geometry_msgs::PoseStamped pose;
        pose.header = object_state.centroid.header;
        pose.pose.position.x = object_state.centroid.point.x;
        pose.pose.position.y = object_state.centroid.point.y;
        pose.pose.position.z = object_state.centroid.point.z;
        person_list_[index].trajectory.poses.push_back(pose);

        if(person_list_[index].trajectory.poses.size() > past_path_threshold_){
            person_list_[index].trajectory.poses.erase(person_list_[index].trajectory.poses.begin());
        }
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
}
