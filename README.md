# YOLOv8 + TensorRT + ByteTrack ROS 包

> 你可以随意更改代码然后push到仓库中
> 
> 文章写于: 2024年4月5日

## 效果演示
https://github.com/BUPT-RobotTeam/bupt_rc_cv_inference_Yolov8TensorrtByteTrack/assets/129849375/14a2f5fc-c0fa-4aef-a8a6-69966cfe2c74

## 环境要求(仅支持Linux环境)

> 1. OpenCV
> 2. eigen3
> 3. TensorRT
> 4. CUDA
> 5. yaml-cpp
> 6. bupt_rc_cv_interfaces
> 7. ros2 (作者使用的是 humble)

## 效果说明
> 目的：为了与项目中的ROS相结合，同时也为了与将图像获取和推理功能解耦‘’
>
> 效果：publisher通过接受bupt_rc_cv/cameras(由bupt_rc_cv_cameras ROS包的camera_publisher发布)的图片数据，然后进行推理，推理完成后发布bupt_rc_cv/inference/result话题出去
> 
>       话题bupt_rc_cv/inference/result包括画过框的图像数据、同一张图片中的所有追踪目标的 track_id、score、tlwh

## 注意事项
> 1. 如果你觉得结果话题的发布由获取图片的时间具有一个明显的时间延时
> 
>    举个例子：就是你在当前时间做的是，它0.3s甚至0.5s后才处理，这时候的原因可能是你消息队列设置的太长了
>    
>    我实际测试所有消息队列设置为1延迟没有那么高，处理不来的图片数据丢了也就丢了，影响不大
