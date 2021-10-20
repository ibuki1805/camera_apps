#ifndef MOTION_PREDICTOR
#define MOTION_PREDICTOR

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>

#include <camera_apps_msgs/ObjectState.h>
#include <camera_apps_msgs/ObjectStates.h>

namespace camera_apps
{
    struct PersonInfo
    {
        int id;
        ros::Time latest_time;
        geometry_msgs::PointStamped centroid;
        nav_msgs::Path trajectory;
    };

    class MotionPredictor
    {
        public:
            MotionPredictor(ros::NodeHandle &nh, ros::NodeHandle &pnh);
            ~MotionPredictor();
        private:
            void object_states_callback(const camera_apps_msgs::ObjectStatesConstPtr &msg);
            void register_person(camera_apps_msgs::ObjectState& object_state);
            void update_person(int id, camera_apps_msgs::ObjectState& object_state);
            void visualize_trajectory();
            void delete_person(int id);
            void lost_judge();
            int id_to_index(int id);

            double error_threshold_;
            double time_threshold_;
            int past_path_threshold_;
            int person_num_limit_;
            int id_now_ = 0;

            std::vector<int> free_id_list_;
            std::vector<int> valid_id_list_;
            std::vector<PersonInfo> person_list_;
            camera_apps_msgs::ObjectStates object_states_;

            ros::Subscriber object_states_sub_;
            ros::Publisher past_trajectory_pub_;

            tf2_ros::Buffer tf_buffer_;
            tf2_ros::TransformListener* tf2_listener_;
    };
}


#endif
