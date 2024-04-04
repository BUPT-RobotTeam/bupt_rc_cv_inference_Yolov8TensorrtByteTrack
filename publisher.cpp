#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <termio.h>
#include <unistd.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "bupt_rc_cv_interfaces/msg/cv_cameras.hpp"

class InferenceAndTrack : public rclcpp::Node {
public:
    InferenceAndTrack() : Node("inference") {
        cv::namedWindow("Cam", cv::WINDOW_NORMAL);
        auto topic_callbakc = [] (const bupt_rc_cv_interfaces::msg::CVCameras::SharedPtr msg) {
            cv::Mat frame(msg->frame_height, msg->frame_width, CV_8UC3, msg->frame_data.data());
            cv::imshow("Cam", frame);
            cv::waitKey(1);
        };
        subscription_ = this->create_subscription<bupt_rc_cv_interfaces::msg::CVCameras>("bupt_rc_cv/cameras", 10, topic_callbakc);
    }

    ~InferenceAndTrack() {
        std::cout << "All windows has been destroyed" << std::endl;
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
private:
    rclcpp::Subscription<bupt_rc_cv_interfaces::msg::CVCameras>::SharedPtr subscription_;

};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    InferenceAndTrack inference_node;
    inference_node.spin();
    rclcpp::shutdown();
    return 0;
}
