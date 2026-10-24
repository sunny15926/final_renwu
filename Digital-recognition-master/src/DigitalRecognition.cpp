/**
 * @file    DigitalRecognition.cpp
 * @brief   装甲板数字识别类实现 (ONNX Runtime CPU 推理)
 */

#include "DigitalRecognition.h"

#include <array>
#include <cmath>
#include <cstring>
#include <iostream>

#include <opencv2/imgproc.hpp>

DigitalRecognition::DigitalRecognition(const std::string &model_path)
    : env_(ORT_LOGGING_LEVEL_WARNING, "DigitalRecognition") {
    // 创建会话选项: 单线程, 开启全部图优化
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), options);

    // 缓存输入/输出节点名
    Ort::AllocatorWithDefaultOptions allocator;
    input_name_ = session_->GetInputNameAllocated(0, allocator).get();
    output_name_ = session_->GetOutputNameAllocated(0, allocator).get();

    input_buffer_.resize(kImageSize * kImageSize);
}

int DigitalRecognition::recognize(const cv::Mat &img, double *confidence) {
    if (confidence) *confidence = 0.0;
    try {
        if (img.empty()) {
            return -1;
        }

        // 1. 统一转成单通道灰度图 (与原仓库 cv::imread(..., CV_8UC1) 行为一致)
        cv::Mat gray;
        if (img.channels() == 1) {
            gray = img;
        } else if (img.channels() == 3) {
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        } else if (img.channels() == 4) {
            cv::cvtColor(img, gray, cv::COLOR_BGRA2GRAY);
        } else {
            return -1;
        }

        // 2. 缩放到 28*28 (与原仓库预处理尺寸对齐)
        cv::Mat resized;
        cv::resize(gray, resized, cv::Size(kImageSize, kImageSize), 0, 0, cv::INTER_LINEAR);

        // 3. 归一化到 [0, 1] (除以 255), 与原仓库 convertTo(img, CV_32FC1, 1.0/255) 对齐
        cv::Mat normalized;
        resized.convertTo(normalized, CV_32FC1, 1.0f / 255.0f);

        // 4. 拷贝到输入 buffer: 单通道 NCHW, 即 1*1*28*28, 内存与 28*28 连续一致
        std::memcpy(input_buffer_.data(), normalized.data,
                    kImageSize * kImageSize * sizeof(float));

        // 5. 构造输入 tensor 并推理
        std::array<int64_t, 4> input_shape{1, 1, kImageSize, kImageSize};
        Ort::MemoryInfo memory_info =
            Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info, input_buffer_.data(), input_buffer_.size(),
            input_shape.data(), input_shape.size());

        const char *input_names[] = {input_name_.c_str()};
        const char *output_names[] = {output_name_.c_str()};
        auto outputs =
            session_->Run(Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);

        // 6. 取 argmax 作为分类结果
        const float *logits = outputs[0].GetTensorMutableData<float>();
        int ans = 0;
        for (int i = 1; i < kNumClasses; ++i) {
            if (logits[i] > logits[ans]) {
                ans = i;
            }
        }

        // 哨兵(无有效数字)类别 -> 识别失败
        if (ans == kSentinelClass) {
            return -1;
        }

        // 计算预测类别的 softmax 概率作为置信度 (数值稳定: 减去最大 logit)
        if (confidence) {
            float max_logit = logits[0];
            for (int i = 1; i < kNumClasses; ++i) {
                if (logits[i] > max_logit) max_logit = logits[i];
            }
            double sum_exp = 0.0;
            for (int i = 0; i < kNumClasses; ++i) {
                sum_exp += std::exp(logits[i] - max_logit);
            }
            *confidence = std::exp(logits[ans] - max_logit) / sum_exp;
        }
        return ans;
    } catch (const std::exception &e) {
        std::cerr << "[DigitalRecognition] 推理失败: " << e.what() << std::endl;
        return -1;
    }
}
