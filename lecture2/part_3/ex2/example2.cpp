#include <iostream>
#include <opencv2/opencv.hpp>

int main()
{
    // 图像在 OpenCV 中就是 Mat 类的对象：
    // 一行代码完成构造，Mat 帮你管理图像内存（这就是封装！）
    cv::Mat img = cv::imread("assets/demo.jpeg");

    if (img.empty())
    {
        std::cout << "读取图片失败！请检查路径" << std::endl;
        return -1;
    }

    // 通过成员变量 / 成员函数查看图像信息
    std::cout << "图像的宽   cols : " << img.cols << std::endl;
    std::cout << "图像的高   rows : " << img.rows << std::endl;
    std::cout << "通道数 channels : " << img.channels() << std::endl;  // 彩色图为 3

    // clone() 拷贝出一张新图像，两个对象互不影响
    cv::Mat img_copy = img.clone();

    // 在副本上画一个圆，原图不受影响
    cv::circle(img_copy, cv::Point(100, 100), 80, cv::Scalar(0, 0, 255), 3);

    cv::imshow("original", img);
    cv::imshow("copy", img_copy);
    cv::waitKey(0);

    return 0;
}
