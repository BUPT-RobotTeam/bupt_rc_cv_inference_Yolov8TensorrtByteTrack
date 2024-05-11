#include "rclcpp/rclcpp.hpp"
#include "bupt_rc_cv_interfaces/msg/cv_inference_array.hpp"
#include "bupt_rc_cv_interfaces/msg/cv_frame.hpp"
#include "bupt_rc_cv_interfaces/srv/cv_sel_track.hpp"
#include <termio.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <algorithm>

class InferenceTracker : public rclcpp::Node {
public:
    InferenceTracker() : Node("inference_subscriber") {
        this->track_id_ = -1;
        this->track_lable_id_ = 0;                  // 追蓝的
        
        cv::namedWindow("camera_ball", cv::WINDOW_NORMAL);
        cv::namedWindow("camera_seedlings", cv::WINDOW_NORMAL);

        this->subscription_ = this->create_subscription<bupt_rc_cv_interfaces::msg::CVInferenceArray>("bupt_rc_cv/inference/result", 1, std::bind(&InferenceTracker::inferenceArrayCallback, this, std::placeholders::_1));
        this->service_ = this->create_service<bupt_rc_cv_interfaces::srv::CVSelTrack>("bupt_rc_cv/inference/track_id", std::bind(&InferenceTracker::service_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

    ~InferenceTracker() {
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

    void inferenceArrayCallback(const bupt_rc_cv_interfaces::msg::CVInferenceArray::SharedPtr result_msg) {
        for (auto msg : result_msg->result) {
            cv::Mat frame(msg.img.frame_height, msg.img.frame_width, CV_8UC3, msg.img.frame_data.data());
            cv::Mat frame_binary;
            cv::Mat frame_gray;

            if (!msg.inference_result.empty()) {
                bupt_rc_cv_interfaces::msg::CVInference track_target;
                int tlwh[4];
                int track_id = this->track_id_;
                auto it = std::find_if(msg.inference_result.begin(), msg.inference_result.end(), [track_id](const bupt_rc_cv_interfaces::msg::CVInference& t){
                    return t.track_id == track_id;
                });

                // 说明没找到了
                if (it == msg.inference_result.end()) {
                    double max_area = 0.0;
                    for (const auto& inference : msg.inference_result) {
                        if ((inference.tlwh[2] * inference.tlwh[3] > max_area) && inference.label_id == this->track_lable_id_) {

                            max_area = inference.tlwh[2] * inference.tlwh[3];

                            track_target.track_id = inference.track_id;
                            track_target.score = inference.score;
                            track_target.label_id = inference.label_id;

                            for (int i = 0; i < 4; ++i) {
                                tlwh[i] = inference.tlwh[i];
                            }
                        }
                    }
                    this->track_id_ = track_target.track_id;
                }
                else {
                    track_target.track_id = it->track_id;
                    track_target.score = it->score;
                    track_target.label_id = it->label_id;
                    for (int i = 0; i < 4; ++i) {
                        tlwh[i] = it->tlwh[i];
                    }
                }

                // 画十字标识
                cv::Point center(std::clamp(tlwh[0] + tlwh[2] / 2, 0, frame.cols - 1), std::clamp(tlwh[1] + tlwh[3] / 2, 0, frame.rows - 1));
                cv::line(frame, cv::Point(center.x, tlwh[1]), cv::Point(center.x, tlwh[1] + tlwh[3]), cv::Scalar(0, 0, 255), 2);
                cv::line(frame, cv::Point(tlwh[0], center.y), cv::Point(tlwh[0] + tlwh[2], center.y), cv::Scalar(0, 0, 255), 2);

                // 话白框标识
                cv::rectangle(frame, cv::Rect(std::clamp(tlwh[0], 0, frame.cols - 1), std::clamp(tlwh[1], 0, frame.rows - 1), std::clamp(tlwh[2], 0, frame.cols - tlwh[0] - 1), std::clamp(tlwh[3], 0, frame.rows - tlwh[1] - 1)), cv::Scalar(255, 255, 255), 3);
                
            }
            cv::imshow(msg.cam_name, frame);
            cv::waitKey(1);
        }
    }

    void service_callback(const std::shared_ptr<bupt_rc_cv_interfaces::srv::CVSelTrack::Request> request, const std::shared_ptr<bupt_rc_cv_interfaces::srv::CVSelTrack::Response> response) {
        this->track_lable_id_ = request->track_label_id;

        std::cout << "[track_id]Now the track id is " << this->track_lable_id_ << std::endl;

        response->track_id = this->track_lable_id_;
    }
private:
    rclcpp::Subscription<bupt_rc_cv_interfaces::msg::CVInferenceArray>::SharedPtr subscription_;
    rclcpp::Service<bupt_rc_cv_interfaces::srv::CVSelTrack>::SharedPtr service_;
    int track_id_;
    int track_lable_id_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    InferenceTracker node_inference_subscriber;
    node_inference_subscriber.spin();
    rclcpp::shutdown();
    return 0;
}
