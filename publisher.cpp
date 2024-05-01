#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <termio.h>
#include <unistd.h>
#include <opencv2/core/types.hpp>
#include <string>
#include <memory>
#include <yaml-cpp/yaml.h>
#include "yolov8_lib.h"
#include "BYTETracker.h"
#include "rclcpp/rclcpp.hpp"
#include "bupt_rc_cv_interfaces/msg/cv_camera_array.hpp"
#include "bupt_rc_cv_interfaces/msg/cv_inference.hpp"
#include "bupt_rc_cv_interfaces/msg/cv_inference_result.hpp"
#include "bupt_rc_cv_interfaces/msg/cv_inference_array.hpp"
#include "bupt_rc_cv_interfaces/srv/cv_inference_switch.hpp"
#include <iostream>

//------------------------------default configuration------------------------------
std::string cam_ball_engine_path = "/home/bupt-rc/Code/yolo/module/engine_module/s_e_200_031201.engine";
std::string cam_seedlings_engine_path = "/home/bupt-rc/Code/yolo/module/engine_module/s_e_200_031201.engine";

std::string map_yaml_path = "/home/bupt-rc/ros2_ws/bupt_rc_cv_ws/src/bupt_rc_cv_inference_Yolov8TensorrtBytetrack/name-map/rc.yaml";

std::map<std::string, std::vector<int>> cam_track_classes {
    {"camera_ball", {0, 1, 2}},                         // ball_blue, ball_red, ball_purple
    {"camera_seedlings", {3, 4}},                       // seedlings_blue, seedlings_red
};
std::shared_ptr<YAML::Node> name_map_ptr = std::make_shared<YAML::Node>(YAML::LoadFile(map_yaml_path));
std::shared_ptr<YoloDetecter> cam_ball_detecter_ptr = std::make_shared<YoloDetecter>(cam_ball_engine_path);
std::shared_ptr<YoloDetecter> cam_seedlings_detecter_ptr = std::make_shared<YoloDetecter>(cam_seedlings_engine_path);
std::shared_ptr<BYTETracker> cam_ball_tracker_ptr = std::make_shared<BYTETracker>(50, 30);
std::shared_ptr<BYTETracker> cam_seedlings_tracker_ptr = std::make_shared<BYTETracker>(50, 30);

bool is_tracking_class(std::string cam_name, int class_id) {
    for (auto& c : cam_track_classes[cam_name]) {
        if (class_id == c)
            return true;
    }
    return false;
}

class InferenceAndTrack : public rclcpp::Node {
public:
    InferenceAndTrack() : Node("bupt_rc_cv_inference_and_track") {
        this->is_inference_ball_ = true;
        this->is_inference_seedlings_ = false;
        this->subscription_ = this->create_subscription<bupt_rc_cv_interfaces::msg::CVCameraArray>("bupt_rc_cv/cameras", 1, std::bind(&InferenceAndTrack::topic_callback, this, std::placeholders::_1));
        this->service_ = this->create_service<bupt_rc_cv_interfaces::srv::CVInferenceSwitch>("bupt_rc_cv/inference/switch", std::bind(&InferenceAndTrack::service_callback, this, std::placeholders::_1, std::placeholders::_2));
        this->publisher_ = this->create_publisher<bupt_rc_cv_interfaces::msg::CVInferenceArray>("bupt_rc_cv/inference/result", 1);
    }

    ~InferenceAndTrack() {
        std::cout << "The inference process is over" << std::endl;
    }

    void spin(){
        int key;
        struct termios old_settings, new_settings;
        tcgetattr(STDIN_FILENO, &old_settings);
        new_settings = old_settings;
        new_settings.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
        while (rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());
            if (kbhit()) {
                key = getchar();
                if (key == 'q') {
                    break;  // 按下 'q' 键退出循环
                }
            }
        }
        tcsetattr(STDIN_FILENO, TCSANOW, &old_settings);
    }
private:
    int kbhit(){
        struct timeval tv;
        fd_set read_fd;
        tv.tv_sec = 0;
        tv.tv_usec = 0;
        FD_ZERO(&read_fd);
        FD_SET(STDIN_FILENO, &read_fd);
        if (select(STDIN_FILENO + 1, &read_fd, nullptr, nullptr, &tv) == -1) {
          return 0;
        }
        return FD_ISSET(STDIN_FILENO, &read_fd);
    }

    void topic_callback(const bupt_rc_cv_interfaces::msg::CVCameraArray::SharedPtr msg) {
        auto all_result = bupt_rc_cv_interfaces::msg::CVInferenceArray();
        auto message_ball = bupt_rc_cv_interfaces::msg::CVInferenceResult();
        auto message_seedlings = bupt_rc_cv_interfaces::msg::CVInferenceResult();
        // 进行球的推理
        for (auto& cam : msg->cameras) {
            if (cam.cam_name == "camera_ball") {
                this->img_ball_ = cv::Mat(cam.img.frame_height, cam.img.frame_width, CV_8UC3, cam.img.frame_data.data());
                if (this->is_inference_ball_) {

                    // yolo inference
                    std::vector<DetectResult> res = cam_ball_detecter_ptr->inference(this->img_ball_);

                    // yolo output format to bytetrack input format, and filter bbox by class id
                    std::vector<Object> objects;

                    for (long unsigned int i = 0; i < res.size(); ++i) {
                        cv::Rect r = res[i] .tlwh;
                        float conf = (float)res[i].conf;
                        int class_id = (int)res[i].class_id;
                        if (is_tracking_class(cam.cam_name, class_id)) {
                            cv::Rect_<float> rect((float)r.x, (float)r.y, (float)r.width, (float)r.height);
                            Object obj {rect, class_id, conf};
                            objects.push_back(obj);
                        }
                    }
                    
                    // track
                    auto output_stracks = cam_ball_tracker_ptr->update(objects);

                    for (size_t i = 0; i < output_stracks.size(); ++i) {
                        bupt_rc_cv_interfaces::msg::CVInference msg;
                        msg.tlwh = output_stracks[i].tlwh;
                        msg.track_id = output_stracks[i].track_id;
                        msg.score = output_stracks[i].score;
                        msg.label_id = output_stracks[i].label_id;
                        message_ball.inference_result.push_back(msg);
                        std::vector<float> tlwh = output_stracks[i].tlwh;

                        if (tlwh[2] * tlwh[3] > 20)
                        {
                                cv::Scalar s = cam_ball_tracker_ptr->get_color(output_stracks[i].track_id);
                                cv::putText(this->img_ball_, cv::format("%d", output_stracks[i].track_id), cv::Point(tlwh[0], tlwh[1] - 5), 0, 0.6, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
                                cv::putText(this->img_ball_, cv::format("%.2f", output_stracks[i].score), cv::Point(tlwh[0] + tlwh[2] / 2, tlwh[1] - 5), 0, 0.6, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
                                cv::putText(this->img_ball_, (*name_map_ptr)["names"][output_stracks[i].label_id].as<std::string>(), cv::Point(tlwh[0] + tlwh[2] * 5 / 6, tlwh[1] - 5), 0, 0.6, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
                                cv::rectangle(this->img_ball_, cv::Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), s, 2);
                        }
                    }
                }

                message_ball.img.frame_width = this->img_ball_.cols;
                message_ball.img.frame_height = this->img_ball_.rows;
                message_ball.img.frame_data.assign(this->img_ball_.data, this->img_ball_.data + this->img_ball_.total() * this->img_ball_.elemSize());
                message_ball.cam_name = cam.cam_name;
            }
            else if (cam.cam_name == "camera_seedlings") {
                this->img_seedlings_ = cv::Mat(cam.img.frame_height, cam.img.frame_width, CV_8UC3, cam.img.frame_data.data());
                if (this->is_inference_seedlings_) {
                    // yolo inference
                    std::vector<DetectResult> res = cam_ball_detecter_ptr->inference(this->img_seedlings_);

                    // yolo output format to bytetrack input format, and filter bbox by class id
                    std::vector<Object> objects;

                    for (long unsigned int i = 0; i < res.size(); ++i) {
                        cv::Rect r = res[i] .tlwh;
                        float conf = (float)res[i].conf;
                        int class_id = (int)res[i].class_id;
                        if (is_tracking_class(cam.cam_name, class_id)) {
                            cv::Rect_<float> rect((float)r.x, (float)r.y, (float)r.width, (float)r.height);
                            Object obj {rect, class_id, conf};
                            objects.push_back(obj);
                        }
                    }
                    
                    // track
                    auto output_stracks = cam_ball_tracker_ptr->update(objects);

                    for (size_t i = 0; i < output_stracks.size(); ++i) {
                        bupt_rc_cv_interfaces::msg::CVInference msg;
                        msg.tlwh = output_stracks[i].tlwh;
                        msg.track_id = output_stracks[i].track_id;
                        msg.score = output_stracks[i].score;
                        msg.label_id = output_stracks[i].label_id;
                        message_seedlings.inference_result.push_back(msg);
                        std::vector<float> tlwh = output_stracks[i].tlwh;

                        if (tlwh[2] * tlwh[3] > 20)
                        {
                                cv::Scalar s = cam_ball_tracker_ptr->get_color(output_stracks[i].track_id);
                                cv::putText(this->img_seedlings_, cv::format("%d", output_stracks[i].track_id), cv::Point(tlwh[0], tlwh[1] - 5), 0, 0.6, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
                                cv::putText(this->img_seedlings_, cv::format("%.2f", output_stracks[i].score), cv::Point(tlwh[0] + tlwh[2] / 2, tlwh[1] - 5), 0, 0.6, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
                                cv::putText(this->img_seedlings_, (*name_map_ptr)["names"][output_stracks[i].label_id].as<std::string>(), cv::Point(tlwh[0] + tlwh[2] * 5 / 6, tlwh[1] - 5), 0, 0.6, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
                                cv::rectangle(this->img_seedlings_, cv::Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), s, 2);
                        }
                    }
                }

                message_seedlings.img.frame_width = this->img_seedlings_.cols;
                message_seedlings.img.frame_height = this->img_seedlings_.rows;
                message_seedlings.img.frame_data.assign(this->img_seedlings_.data, this->img_seedlings_.data + this->img_seedlings_.total() * this->img_seedlings_.elemSize());
                message_seedlings.cam_name = cam.cam_name;
            }
        }

        all_result.result.push_back(message_ball);
        all_result.result.push_back(message_seedlings);
        this->publisher_->publish(all_result);
    }

    void service_callback(const std::shared_ptr<bupt_rc_cv_interfaces::srv::CVInferenceSwitch::Request> request, const::std::shared_ptr<bupt_rc_cv_interfaces::srv::CVInferenceSwitch::Response> response) {
        this->is_inference_ball_ = request->cmaera_ball_inference_enable;
        this->is_inference_seedlings_ = request->camera_seedlings_inference_enable;
        std::cout << "raw data: " << std::endl;
        std::cout << "status of ball        : " << request->cmaera_ball_inference_enable << std::endl;
        std::cout << "status of seedlings   : " << request->camera_seedlings_inference_enable << std::endl;

        std::cout << "[get service]The status of two cameras has been changed" << std::endl;
        std::cout << "inference ball        : " << (is_inference_ball_ ? "on" : "off") << std::endl;
        std::cout << "inference seedlings   : " << (is_inference_seedlings_ ? "on" : "off") << std::endl;
        
        response->camera_ball_inference_status = this->is_inference_ball_;
        response->camera_seedlings_inference_status = this->is_inference_seedlings_;
    }

private:
    rclcpp::Subscription<bupt_rc_cv_interfaces::msg::CVCameraArray>::SharedPtr subscription_;
    rclcpp::Publisher<bupt_rc_cv_interfaces::msg::CVInferenceArray>::SharedPtr publisher_;
    rclcpp::Service<bupt_rc_cv_interfaces::srv::CVInferenceSwitch>::SharedPtr service_;
    bool is_inference_ball_;
    bool is_inference_seedlings_;
    cv::Mat img_ball_;
    cv::Mat img_seedlings_;
};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "|                    press [q] to exit                    |" << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;

    InferenceAndTrack inference_and_track_node;  
    inference_and_track_node.spin();

    rclcpp::shutdown();
    return 0;
}
