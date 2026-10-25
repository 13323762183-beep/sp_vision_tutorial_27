
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

            
            // 4. 绘制绿色矩形框，并在上方显示颜色和图案
        for (const auto& armor : armors) {

            // 绘制绿色矩形框
            tools::draw_points(
                img,
                armor.points,
                cv::Scalar(0, 255, 0),
                2
            );

            // 获取颜色和图案
            std::string info =
                auto_aim::COLORS.at(armor.color) + " " +
                auto_aim::ARMOR_NAMES.at(armor.name);

            // 获取矩形框左上角
            int x = armor.box.x;
            int y = armor.box.y;

            // 在矩形框上方显示文字
            cv::putText(
                img,
                info,
                cv::Point(x, std::max(20, y - 10)),
                cv::FONT_HERSHEY_SIMPLEX,
                0.6,
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
