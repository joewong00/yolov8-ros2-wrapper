#ifndef DETECTOR_HPP_
#define DETECTOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <torch/script.h>
#include <torch/torch.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn/dnn.hpp>


class Detector : public rclcpp::Node
{
    public:
        Detector();
        void run();

    private:
        std::string file_path;
        std::string data_path;
        double conf_thres;
        double iou_thres;
        bool gpu;
        bool pub_image;
        int crowd_threshold;

        torch::jit::script::Module module_;
        torch::DeviceType device_;
        bool half_;
        std::vector<std::string> class_names;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

        void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

        float generate_scale(cv::Mat& image, const std::vector<int>& target_size);
        float letterbox(cv::Mat &input_image, cv::Mat &output_image, const std::vector<int> &target_size);
        std::vector<std::string> LoadNames(const std::string& path);
        torch::Tensor xyxy2xywh(const torch::Tensor& x);
        torch::Tensor xywh2xyxy(const torch::Tensor& x);
        torch::Tensor nms(const torch::Tensor& bboxes, const torch::Tensor& scores, float iou_threshold);
        torch::Tensor non_max_suppression(torch::Tensor& prediction, float conf_thres, float iou_thres, int max_det);
        torch::Tensor clip_boxes(torch::Tensor& boxes, const std::vector<int>& shape);
        torch::Tensor scale_boxes(const std::vector<int>& img1_shape, torch::Tensor& boxes, const std::vector<int>& img0_shape);
    
};

#endif  