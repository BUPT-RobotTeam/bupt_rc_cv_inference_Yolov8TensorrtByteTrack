#include "rclcpp/rclcpp.hpp"
#include "bupt_rc_cv_interfaces/msg/cv_inference_array.hpp"
#include <termio.h>
#include <unistd.h>

class InferenceSubscriber : public rclcpp::Node {
public:
    InferenceSubscriber() : Node("inference_subscriber") {
        subscription_ = this->create_subscription<bupt_rc_cv_interfaces::msg::CVInferenceArray>("bupt_rc_cv/inference/result", 10, std::bind(&InferenceSubscriber::inferenceArrayCallback, this, std::placeholders::_1));
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

    void inferenceArrayCallback(const bupt_rc_cv_interfaces::msg::CVInferenceArray::SharedPtr msg) {
        RCLCPP_INFO(rclcpp::get_logger("inference_subscriber"), "============================================================\n");
        for (const auto& inference : msg->inference_result) {
            RCLCPP_INFO(rclcpp::get_logger("inference_subscriber"), "track id: %d\n", inference.track_id);
            RCLCPP_INFO(rclcpp::get_logger("inference_subscriber"), "score: %f\n", inference.score);
            RCLCPP_INFO(rclcpp::get_logger("inference_subscriber"), "tlwh: (%f, %f, %f, %f)\n", inference.tlwh[0], inference.tlwh[1], inference.tlwh[2], inference.tlwh[3]);
        RCLCPP_INFO(rclcpp::get_logger("inference_subscriber"), "------------------------------------------------------------\n");
        }
        RCLCPP_INFO(rclcpp::get_logger("inference_subscriber"), "============================================================\n");
    }

private:
    rclcpp::Subscription<bupt_rc_cv_interfaces::msg::CVInferenceArray>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    InferenceSubscriber node_inference_subscriber;
    node_inference_subscriber.spin();
    rclcpp::shutdown();
    return 0;
}
