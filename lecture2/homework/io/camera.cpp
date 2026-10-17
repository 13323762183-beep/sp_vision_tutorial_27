
#include "camera.hpp"

#include <stdexcept>
#include <string>

namespace {

// 检查海康 SDK 的返回值
void check(int ret, const std::string& operation)
{
    if (ret != MV_OK) {
        throw std::runtime_error(
            operation + " failed, error code: " +
            std::to_string(ret)
        );
    }
}

} // namespace

// 构造函数：初始化、配置并启动相机
Camera::Camera()
{
    try {
        // 1. 枚举 USB 相机
        MV_CC_DEVICE_INFO_LIST device_list{};

        check(
            MV_CC_EnumDevices(MV_USB_DEVICE, &device_list),
            "EnumDevices"
        );

        if (device_list.nDeviceNum == 0) {
            throw std::runtime_error("No USB camera found");
        }

        // 2. 创建相机句柄
        check(
            MV_CC_CreateHandle(
                &handle_,
                device_list.pDeviceInfo[0]
            ),
            "CreateHandle"
        );

        // 3. 打开设备
        check(
            MV_CC_OpenDevice(handle_),
            "OpenDevice"
        );
        opened_ = true;

        // 4. 设置相机参数
        check(
            MV_CC_SetEnumValue(
                handle_,
                "BalanceWhiteAuto",
                MV_BALANCEWHITE_AUTO_CONTINUOUS
            ),
            "Set BalanceWhiteAuto"
        );

        check(
            MV_CC_SetEnumValue(
                handle_,
                "ExposureAuto",
                MV_EXPOSURE_AUTO_MODE_OFF
            ),
            "Set ExposureAuto"
        );

        check(
            MV_CC_SetEnumValue(
                handle_,
                "GainAuto",
                MV_GAIN_MODE_OFF
            ),
            "Set GainAuto"
        );

        check(
            MV_CC_SetFloatValue(handle_, "ExposureTime", 10000),
            "Set ExposureTime"
        );

        check(
            MV_CC_SetFloatValue(handle_, "Gain", 20),
            "Set Gain"
        );

        check(
            MV_CC_SetFrameRate(handle_, 60),
            "Set FrameRate"
        );

        // 5. 开始采集
        check(
            MV_CC_StartGrabbing(handle_),
            "StartGrabbing"
        );
        grabbing_ = true;

    } catch (...) {
        // 构造失败时，析构函数不会被调用
        // 因此需要在这里主动释放已获取的资源
        cleanup();
        throw;
    }
}


// 读取一帧图像
cv::Mat Camera::read()
{
    if (!grabbing_ || handle_ == nullptr) {
        return {};
    }

    MV_FRAME_OUT raw{};

    // 最长等待 100 ms
    int ret = MV_CC_GetImageBuffer(handle_, &raw, 100);

    if (ret != MV_OK) {
        return {};
    }

    cv::Mat img;

    try {
        // 转换图像
        img = transfer(raw);
    } catch (...) {
        MV_CC_FreeImageBuffer(handle_, &raw);
        throw;
    }

    // 释放 SDK 原始图像缓冲区
    check(
        MV_CC_FreeImageBuffer(handle_, &raw),
        "FreeImageBuffer"
    );

    return img;
}


// 将相机原始图像转换为 BGR 格式
cv::Mat Camera::transfer(const MV_FRAME_OUT& raw)
{
    const int width = static_cast<int>(raw.stFrameInfo.nWidth);
    const int height = static_cast<int>(raw.stFrameInfo.nHeight);

    if (width <= 0 || height <= 0 || raw.pBufAddr == nullptr) {
        throw std::runtime_error("Invalid camera frame");
    }

    // 创建独立的三通道图像缓冲区
    cv::Mat img(height, width, CV_8UC3);

    MV_CC_PIXEL_CONVERT_PARAM param{};

    param.nWidth = raw.stFrameInfo.nWidth;
    param.nHeight = raw.stFrameInfo.nHeight;

    param.pSrcData = raw.pBufAddr;
    param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
    param.enSrcPixelType = raw.stFrameInfo.enPixelType;

    param.pDstBuffer = img.data;
    param.nDstBufferSize =
        static_cast<unsigned int>(
            img.total() * img.elemSize()
        );

    param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

    check(
        MV_CC_ConvertPixelType(handle_, &param),
        "ConvertPixelType"
    );

    return img;
}


// 释放相机资源
void Camera::cleanup() noexcept
{
    if (handle_ == nullptr) {
        return;
    }

    if (grabbing_) {
        MV_CC_StopGrabbing(handle_);
        grabbing_ = false;
    }

    if (opened_) {
        MV_CC_CloseDevice(handle_);
        opened_ = false;
    }

    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
}


// 析构函数
Camera::~Camera()
{
    cleanup();
}
