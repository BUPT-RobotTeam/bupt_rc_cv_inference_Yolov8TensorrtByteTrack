#include "rclcpp/rclcpp.hpp"
#include "bupt_rc_cv_interfaces/srv/cv_inference_switch.hpp"
#include <chrono>
using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    if (argc != 3) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: publish_client [0/1](camera_ball) [0/1](camera_seedlings)");
    }
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("inference_client");
    rclcpp::Client<bupt_rc_cv_interfaces::srv::CVInferenceSwitch>::SharedPtr client = node->create_client<bupt_rc_cv_interfaces::srv::CVInferenceSwitch>("bupt_rc_cv/inference/switch");
    auto request = std::make_shared<bupt_rc_cv_interfaces::srv::CVInferenceSwitch::Request>();

    if (std::atoll(argv[1]) == 0)
        request->cmaera_ball_inference_enable = false;
    else
        request->cmaera_ball_inference_enable = true;

    if (std::atoll(argv[2]) == 0)
        request->camera_seedlings_inference_enable = false;
    else 
        request->camera_seedlings_inference_enable = true;

    if (std::atoll(argv[3]) == 0)
        request->camera_ball_draw_img_enable = false;
    else 
        request->camera_ball_draw_img_enable = true;

    if (std::atoll(argv[4]) == 0)
        request->camera_seedlings_draw_img_enable = false;
    else 
        request->camera_seedlings_draw_img_enable = true;

    
    std::cout << "status of ball        : " << request->cmaera_ball_inference_enable << std::endl;
    std::cout << "status of seedlings   : " << request->camera_seedlings_inference_enable << std::endl;

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        std::cout << "ball          : " << result.get()->camera_ball_inference_status << std::endl;
        std::cout << "seedlings     : " << result.get()->camera_seedlings_inference_status << std::endl;
    }
    else {
        std::cerr << "Failed to call service" << std::endl;
    }
    rclcpp::shutdown();
}
