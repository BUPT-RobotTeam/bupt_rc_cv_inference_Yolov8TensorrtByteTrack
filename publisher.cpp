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
#include "bupt_rc_cv_interfaces/msg/cv_cameras.hpp"
#include "bupt_rc_cv_interfaces/msg/cv_inference.hpp"
#include "bupt_rc_cv_interfaces/msg/cv_inference_array.hpp"
#include <iostream>
using namespace std;
//------------------------------default configuration------------------------------
std::string default_engine = "/home/bupt-rc/Code/yolo/module/engine_module/s_e_200_031201.engine";                          // 根据自己的路径进行更改
std::string default_cam_type = "auto";                                                                                      // auto 的意思是自动检测
std::string default_map_yaml = "/home/bupt-rc/Code_Teaching/tensorrt/TensorRT-YOLOv8-ByteTrack/name-map/rc.yaml";           // 名称映射yaml文件
// 需要跟踪的类别，可以根据自己需求调整，筛选自己想要跟踪的对象的种类
std::vector<int>  trackClasses {0, 1, 2, 3, 4, 5};  // ball_blue, ball_red, ball_purple, seedling_blue, seedling_red

std::shared_ptr<YAML::Node> name_map_ptr = std::make_shared<YAML::Node>(YAML::LoadFile(default_map_yaml));
std::shared_ptr<YoloDetecter> detecter_ptr = std::make_shared<YoloDetecter>(default_engine);
std::shared_ptr<BYTETracker> tracker_ptr = std::make_shared<BYTETracker>(50, 30);

bool isTrackingClass(int class_id){
    for (auto& c : trackClasses){
        if (class_id == c) return true;
    }
    return false;
}

class InferenceAndTrack : public rclcpp::Node {
public:
    InferenceAndTrack() : Node("inference") {
        std::cout << "Init Finish" << std::endl;
        cv::namedWindow("img", WINDOW_NORMAL);
        // 订阅图片数据并推理
        subscription_ = this->create_subscription<bupt_rc_cv_interfaces::msg::CVCameras>("bupt_rc_cv/cameras", 1, std::bind(&InferenceAndTrack::topic_callback, this, std::placeholders::_1));

        // 发布推理结果
        publisher_ = this->create_publisher<bupt_rc_cv_interfaces::msg::CVInferenceArray>("bupt_rc_cv/inference/result", 1);

    }

    ~InferenceAndTrack() {
        std::cout << "The program has safely exited" << std::endl;
        cv::destroyAllWindows();
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

    void topic_callback(const bupt_rc_cv_interfaces::msg::CVCameras::SharedPtr msg) {

        // 获取数据
        this->img = cv::Mat(msg->img.frame_height, msg->img.frame_width, CV_8UC3, msg->img.frame_data.data());

        // yolo inference
        std::vector<DetectResult> res = detecter_ptr->inference(this->img);

        // yolo output format to bytetrack input format, and filter bbox by class id
        std::vector<Object> objects;

        for (long unsigned int j = 0; j < res.size(); j++)
        {
            cv::Rect r = res[j].tlwh;
                    float conf = (float)res[j].conf;
                    int class_id = (int)res[j].class_id;
            if (isTrackingClass(class_id)){
                cv::Rect_<float> rect((float)r.x, (float)r.y, (float)r.width, (float)r.height);
                Object obj {rect, class_id, conf};
                objects.push_back(obj);
            }
        }

        // track
        auto output_stracks = tracker_ptr->update(objects);
        auto message = bupt_rc_cv_interfaces::msg::CVInferenceArray();
        
        for (int i = 0; i < output_stracks.size(); i++)
        {

            bupt_rc_cv_interfaces::msg::CVInference msg;
            msg.tlwh = output_stracks[i].tlwh;
            msg.track_id = output_stracks[i].track_id;
            msg.score = output_stracks[i].score;
            message.inference_result.push_back(msg);

            std::vector<float> tlwh = output_stracks[i].tlwh;

            if (tlwh[2] * tlwh[3] > 20)
            {
                    cv::Scalar s = tracker_ptr->get_color(output_stracks[i].track_id);
                    cv::putText(img, cv::format("%d", output_stracks[i].track_id), cv::Point(tlwh[0], tlwh[1] - 5), 0, 0.6, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
                    cv::putText(img, cv::format("%.2f", output_stracks[i].score), cv::Point(tlwh[0] + tlwh[2] / 2, tlwh[1] - 5), 0, 0.6, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
                    cv::putText(img, (*name_map_ptr)["names"][output_stracks[i].label_id].as<std::string>(), cv::Point(tlwh[0] + tlwh[2] * 5 / 6, tlwh[1] - 5), 0, 0.6, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
                    cv::rectangle(img, cv::Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), s, 2);
            }
        }

        message.img.frame_width = img.cols;
        message.img.frame_height = img.rows;
        message.img.frame_data.assign(img.data, img.data + img.total() * img.elemSize());

        publisher_->publish(message);
    }



private:
    rclcpp::Subscription<bupt_rc_cv_interfaces::msg::CVCameras>::SharedPtr subscription_;

    rclcpp::Publisher<bupt_rc_cv_interfaces::msg::CVInferenceArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    cv::Mat img;

};
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "|                    press [q] to exit                    |" << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;

    InferenceAndTrack inference_node;
    inference_node.spin();
    rclcpp::shutdown();
    return 0;
}
