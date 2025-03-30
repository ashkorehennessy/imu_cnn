#include <esp_log.h>
#include "imu_cnn.id.h"
#include "imu_cnn.mem.h"
#include "sdkconfig.h"
#include "ncnn/net.h"

#define SAMPLE_SIZE 200
extern float sample_data[SAMPLE_SIZE][6];
ncnn::Net net;

void load_ncnn_model() {

    net.opt.use_sgemm_convolution = false;
    net.opt.use_local_pool_allocator = false;
    // 加载模型结构
    net.load_param(imu_cnn_ncnn_param_bin);

    // 加载模型权重
    net.load_model(imu_cnn_ncnn_bin);

    ESP_LOGI("imu_cnn", "ncnn model loaded");
}

int predict_imu_data() {
    // 创建输入Mat (shape: 6通道 × 200时间步)
    ncnn::Mat in(200, 6); // 通道数=6，时间步=200
    int index = 0;
    for (int c = 0; c < 6; c++) {
        for (int t = 0; t < 200; t++) {
            in[c * 200 + t] = sample_data[index%200][index/200];
            index++;
        }
    }
    ESP_LOGI("imu_cnn", "input data prepared");

    // 执行推理
    ncnn::Extractor ex = net.create_extractor();
    ex.input(0, in);
    ESP_LOGI("imu_cnn", "input data set");
    ncnn::Mat out;
    ex.extract(7, out);
    ESP_LOGI("imu_cnn", "output data extracted");
    // 解析输出
    int max_index = 0;
    float max_prob = out[0];
    for (int i = 1; i < out.w; i++) {
        if (out[i] > max_prob) {
            max_prob = out[i];
            max_index = i;
        }
    }
    ESP_LOGI("imu_cnn", "predict result: %d", max_index);
    return max_index;
}
