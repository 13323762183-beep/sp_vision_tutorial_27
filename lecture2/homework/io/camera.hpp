
#pragma once

#include <opencv2/opencv.hpp>
#include "hikrobot/include/MvCameraControl.h"

class Camera {
public:
    Camera();
    ~Camera();

    cv::Mat read();

private:
    void* handle_ = nullptr;
    bool opened_ = false;
    bool grabbing_ = false;

    // 将海康相机原始图像转换为 OpenCV 图像
    cv::Mat transfer(const MV_FRAME_OUT& raw);

    // 清理相机资源
    void cleanup() noexcept;

    // 禁止复制，避免重复释放相机句柄
    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;
};
