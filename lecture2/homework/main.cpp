
#include "io/camera.hpp"
#include "tasks/yolo.hpp"
#include "tools/img_tools.hpp"

#include <opencv2/opencv.hpp>
#include <exception>
#include <iostream>

int main()
{
    try {
        // 1. 初始化相机和 YOLO 检测器
        Camera camera;
        auto_aim::YOLO yolo("configs/yolo.yaml", false);

        while (true) {
            // 2. 读取一帧图像
            cv::Mat img = camera.read();

            if (img.empty()) {
                continue;
            }

            // 3. YOLO 检测装甲板
            auto armors = yolo.detect(img);

            // 4. 绘制装甲板四个关键点形成的绿色矩形
            for (const auto& armor : armors) {
                tools::draw_points(
                    img,
                    armor.points,
                    cv::Scalar(0, 255, 0),
                    2
                );
            }

            // 5. 显示图像
            cv::imshow("Armor Detection", img);

            // 按 q 退出
            if (cv::waitKey(1) == 'q') {
                break;
            }
        }

        cv::destroyAllWindows();

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
