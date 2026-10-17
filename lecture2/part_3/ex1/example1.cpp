#include <iostream>
#include <opencv2/opencv.hpp>

int main()
{
    // 1. 读取图片，路径相对于"运行目录"（build 文件夹）
    cv::Mat img = cv::imread("assets/demo.jpeg");

    // 2. 检查是否读取成功
    if (img.empty())
    {
        std::cout << "读取图片失败！请检查路径" << std::endl;
        return -1;
    }

    // 3. 显示图片：第一个参数是窗口标题
    cv::imshow("Hello OpenCV", img);

    std::cout << "按任意键关闭窗口..." << std::endl;

    // 4. 等待按键（毫秒），0 表示一直等待
    cv::waitKey(0);

    return 0;
}
