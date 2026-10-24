/**
 * @file    DigitalRecognition.h
 * @brief   装甲板数字识别类 (ONNX Runtime CPU 推理)
 *
 * 封装 ONNX Runtime 推理流程, 输入一张已经 warp 矫正后的装甲板小图,
 * 输出识别出的数字 (0~5); 识别失败(哨兵/无有效数字或异常)返回 -1。
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>
#include <opencv2/core.hpp>

class DigitalRecognition {
public:
    /**
     * 构造函数, 加载 ONNX 模型并创建推理会话
     * @param model_path ONNX 模型文件路径
     */
    explicit DigitalRecognition(const std::string &model_path = "../model/model.onnx");

    /**
     * 单张图片识别
     * @param img 已经 warp 矫正后的装甲板小图 (支持单通道灰度图或 3/4 通道彩色图)
     * @param confidence 可选输出: 预测类别的 softmax 概率 (0~1); 失败时置 0
     * @return 识别出的数字 (0~5); 识别失败(哨兵或无有效数字)返回 -1
     */
    int recognize(const cv::Mat &img, double *confidence = nullptr);

private:
    static constexpr int kImageSize = 28;      // 网络输入尺寸 28*28
    static constexpr int kNumClasses = 7;      // 输出类别数 (数字 0~5 + 哨兵)
    static constexpr int kSentinelClass = 6;   // 哨兵(无有效数字)类别索引

    Ort::Env env_;                            // ONNX Runtime 环境, 必须先于 session 构造
    std::unique_ptr<Ort::Session> session_;   // 推理会话
    std::string input_name_;                  // 模型输入节点名
    std::string output_name_;                 // 模型输出节点名
    std::vector<float> input_buffer_;         // 归一化后的输入数据 (28*28, NCHW)
};
