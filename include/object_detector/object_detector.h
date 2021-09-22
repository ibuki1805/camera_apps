#ifndef OBJECT_DETECTOR
#define OBJECT_DETECTOR

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn/dnn.hpp>

#include <fstream>
#include <sstream>

namespace camera_apps
{
    class ObjectDetector
    {
        public:
            ObjectDetector(ros::NodeHandle &nh, ros::NodeHandle &pnh);
            void process();
        private:
            void image_callback(const sensor_msgs::ImageConstPtr &msg);
            std::vector<std::string> read_file(std::string filename, char delimiter='\n');
            void set_network();
            void object_detect(cv::Mat &image);

            std::string camera_topic_name_;
            std::string model_path_;
            double conf_threshold_;

            std::vector<std::string> class_names_;

            cv::Mat input_image_;
            cv::Mat detection_image_;
            cv::dnn::Net net_;

            image_transport::Subscriber image_sub_;
            image_transport::Publisher image_pub_;
    };
}
#endif
