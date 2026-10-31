//循环中使用camera.read(img, t)读取图像和使用camera.try_read_for(img, t, timeout)尝试读取图像的区别
//(?)看不太懂两个函数的定义内容

//循环里面auto armors = yolo.detect(img),在哪里初始化armor对象？神经网络里面吗？用的是armor.hpp中的哪一个初始化函数？

//config_path是怎么完成初始化的？

#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  // 初始化工具类
  tools::Exiter exiter;
  tools::Plotter plotter;   // 注意plotter工具的使用

  // 初始化io类
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  // 初始化auto_aim类
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    // Your code start
    // 读取图像和时间戳
    camera.read(img, t);

    //yolo识别图像中的装甲板
    auto armors = yolo.detect(img);

    //从识别出的装甲板中选择一个目标装甲板,这里直接取第一个，不做目标选择算法
    auto target = armors.begin();

    //获取云台姿态
    auto q = gimbal.q(t);

    //将云台姿态转换为世界坐标系下的旋转矩阵
    solver.set_R_gimbal2world(q);

    //求解装甲板坐标系到相机坐标系的变换
    solver.solve(target);//*target？

    //向云台发送指令
    gimbal.send(true, false, target->ypr_in_world[0], target->ypr_in_world[1]);



    // Your code end
  }

  return 0;
}