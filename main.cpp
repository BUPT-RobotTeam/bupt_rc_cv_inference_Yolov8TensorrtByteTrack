#include <iostream>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include "yolov8_lib.h"
#include "BYTETracker.h"
#include <yaml-cpp/yaml.h>

//------------------------------default configuration------------------------------
std::string default_engine = "/home/bupt-rc/Code/yolo/module/engine_module/s_e_200_031201.engine";                          // 根据自己的路径进行更改
std::string default_cam_type = "auto";                                                                                      // auto 的意思是自动检测
std::string default_map_yaml = "/home/bupt-rc/Code_Teaching/tensorrt/TensorRT-YOLOv8-ByteTrack/name-map/rc.yaml";           // 名称映射yaml文件

// 需要跟踪的类别，可以根据自己需求调整，筛选自己想要跟踪的对象的种类（以下对应COCO数据集类别索引）
std::vector<int>  trackClasses {0, 1, 2, 3, 4, 5};  // ball_blue, ball_red, ball_purple, seedling_blue, seedling_red

bool isTrackingClass(int class_id){
	for (auto& c : trackClasses){
		if (class_id == c) return true;
	}
	return false;
}

bool parseArgs(int argc, char** argv, std::string& engine, std::string& cam_type, std::string& map_yaml) {
    if (argc == 1) {
        engine = default_engine;
        cam_type = default_cam_type;
        map_yaml = default_map_yaml;
    }
    else if (argc == 2) {
        engine = std::string(argv[1]);
        cam_type = default_cam_type;
        map_yaml = default_map_yaml;
    }
    else if (argc == 3) {
        engine = std::string(argv[1]);
        cam_type = std::string(argv[2]);
        map_yaml = default_map_yaml;
    }
    else if (argc == 4) {
        engine = std::string(argv[1]);
        cam_type = std::string(argv[2]);
        map_yaml = std::string(argv[3]);
    }
    else
        return false;
    return true;
}

int run(std::string& engine, std::string& cam_type, std::string& map_yaml)
{

    // camera init

    // Load the YAML configuration file
    YAML::Node name_map = YAML::LoadFile(map_yaml);

    // YOLOv8 predictor
    YoloDetecter detecter(engine);
    
    // ByteTrack tracker
    BYTETracker tracker(50, 30);

    cv::Mat img;
    while (true)
    {
        
        if (img.empty())
            continue;

        // yolo inference
        std::vector<DetectResult> res = detecter.inference(img);

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
        std::vector<STrack> output_stracks = tracker.update(objects);


        for (int i = 0; i < output_stracks.size(); i++)
		{
			std::vector<float> tlwh = output_stracks[i].tlwh;

			if (tlwh[2] * tlwh[3] > 20)
			{
				cv::Scalar s = tracker.get_color(output_stracks[i].track_id);
				cv::putText(img, cv::format("%d", output_stracks[i].track_id), cv::Point(tlwh[0], tlwh[1] - 5), 0, 0.6, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
				cv::putText(img, cv::format("%.2f", output_stracks[i].score), cv::Point(tlwh[0] + tlwh[2] / 2, tlwh[1] - 5), 0, 0.6, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
				cv::putText(img, name_map["names"][output_stracks[i].label_id].as<std::string>(), cv::Point(tlwh[0] + tlwh[2] * 5 / 6, tlwh[1] - 5), 0, 0.6, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
                cv::rectangle(img, cv::Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), s, 2);
			}
		}
        cv::imshow("img", img);
        int c = cv::waitKey(1);
        if (c == 27) break;  // ESC to exit
    }
    return 0;
}


int main(int argc, char *argv[])
{
    std::string engine;
    std::string cam_type;
    std::string map_yaml;
    if (!parseArgs(argc, argv, engine, cam_type, map_yaml)) {
        std::cerr << "arguments not right!" << std::endl;
        std::cerr << "Usage: ./main [engine path] [cmaera type] [map yaml file]" << std::endl;
        std::cerr << "Example: ./main /home/bupt-rc/Code/yolo/module/engine_module/yolov8s.engine" << std::endl;
        std::cerr << "Example: ./main /home/bupt-rc/Code/yolo/module/engine_module/yolov8s.engine h" << std::endl;
        std::cerr << "Example: ./main /home/bupt-rc/Code/yolo/module/engine_module/yolov8s.engine h ../name-map/coco.yaml" << std::endl;
        return -1;
    }

    return run(engine, cam_type, map_yaml);
}
