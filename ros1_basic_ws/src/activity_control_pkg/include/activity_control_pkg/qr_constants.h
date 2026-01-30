#pragma once

/**
 * @brief 二维码目标像素坐标常量
 * 
 * 统一定义二维码目标坐标，所有使用二维码对准的模块都应引用此文件
 * 修改这里的值会同时影响 qr_alignment_task 和 pid_controller
 */
namespace qr_constants {

// 二维码目标像素坐标（相机画面中心偏移后的目标位置）
constexpr double QR_TARGET_X = 380.0;  // 像素X坐标
constexpr double QR_TARGET_Y = 170.0;  // 像素Y坐标

// 二维码对准容差（像素）
// 调大此值可以实现“快一点”：只要飞机在移动过程中靠近了中心（80像素内）就立刻打激光
constexpr double QR_ALIGN_ERROR = 100.0;

// 激光射击等待时间（秒）
constexpr double QR_LASER_SHOT_WAIT = 0.5;

// 二维码数据超时时间（秒）
constexpr double QR_DATA_TIMEOUT = 0.5;

// 视觉伺服超时时间（秒）
constexpr double QR_VISUAL_SERVO_TIMEOUT = 2.0;

// 平滑与趋势判定默认参数
constexpr int QR_SMOOTH_WINDOW_SIZE = 5;
constexpr int QR_TREND_WINDOW_SIZE = 5;
constexpr double QR_TREND_PERCENT_THRESHOLD = 0.05;

} // namespace qr_constants
