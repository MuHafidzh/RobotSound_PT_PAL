// Tencent is pleased to support the open source community by making ncnn available.
//
// Copyright (C) 2020 THL A29 Limited, a Tencent company. All rights reserved.
//
// Licensed under the BSD 3-Clause License (the "License"); you may not use this file except
// in compliance with the License. You may obtain a copy of the License at
//
// https://opensource.org/licenses/BSD-3-Clause
//
// Unless required by applicable law or agreed to in writing, software distributed
// under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// modified 12-31-2021 Q-engineering

#include "ncnn/layer.h"
#include "ncnn/net.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <vector>
#include <chrono>
#include <iostream>
#include <thread>

ncnn::Net yolov5;

const int target_size = 288;
const float prob_threshold = 0.6f;
const float nms_threshold = 0.3f;
const float norm_vals[3] = {1 / 255.f, 1 / 255.f, 1 / 255.f};

const char* class_names[] = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
    "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
    "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush"
};


class YoloV5Focus : public ncnn::Layer
{
public:
    YoloV5Focus()
    {
        one_blob_only = true;
    }

    virtual int forward(const ncnn::Mat& bottom_blob, ncnn::Mat& top_blob, const ncnn::Option& opt) const
    {
        int w = bottom_blob.w;
        int h = bottom_blob.h;
        int channels = bottom_blob.c;

        int outw = w / 2;
        int outh = h / 2;
        int outc = channels * 4;

        top_blob.create(outw, outh, outc, 4u, 1, opt.blob_allocator);
        if (top_blob.empty())
            return -100;

        #pragma omp parallel for num_threads(opt.num_threads)
        for (int p = 0; p < outc; p++)
        {
            const float* ptr = bottom_blob.channel(p % channels).row((p / channels) % 2) + ((p / channels) / 2);
            float* outptr = top_blob.channel(p);

            for (int i = 0; i < outh; i++)
            {
                for (int j = 0; j < outw; j++)
                {
                    *outptr = *ptr;

                    outptr += 1;
                    ptr += 2;
                }

                ptr += w;
            }
        }

        return 0;
    }
};

DEFINE_LAYER_CREATOR(YoloV5Focus)

struct Object
{
    cv::Rect_<float> rect;
    int label;
    float prob;
};

static inline float intersection_area(const Object& a, const Object& b)
{
    cv::Rect_<float> inter = a.rect & b.rect;
    return inter.area();
}

static void qsort_descent_inplace(std::vector<Object>& faceobjects, int left, int right)
{
    int i = left;
    int j = right;
    float p = faceobjects[(left + right) / 2].prob;

    while (i <= j)
    {
        while (faceobjects[i].prob > p)
            i++;

        while (faceobjects[j].prob < p)
            j--;

        if (i <= j)
        {
            // swap
            std::swap(faceobjects[i], faceobjects[j]);

            i++;
            j--;
        }
    }

    #pragma omp parallel sections
    {
        #pragma omp section
        {
            if (left < j) qsort_descent_inplace(faceobjects, left, j);
        }
        #pragma omp section
        {
            if (i < right) qsort_descent_inplace(faceobjects, i, right);
        }
    }
}

static void qsort_descent_inplace(std::vector<Object>& faceobjects)
{
    if (faceobjects.empty())
        return;

    qsort_descent_inplace(faceobjects, 0, faceobjects.size() - 1);
}

static void nms_sorted_bboxes(const std::vector<Object>& faceobjects, std::vector<int>& picked, float nms_threshold)
{
    picked.clear();

    const int n = faceobjects.size();

    std::vector<float> areas(n);
    for (int i = 0; i < n; i++)
    {
        areas[i] = faceobjects[i].rect.area();
    }

    for (int i = 0; i < n; i++)
    {
        const Object& a = faceobjects[i];

        int keep = 1;
        for (int j = 0; j < (int)picked.size(); j++)
        {
            const Object& b = faceobjects[picked[j]];

            // intersection over union
            float inter_area = intersection_area(a, b);
            float union_area = areas[i] + areas[picked[j]] - inter_area;
            // float IoU = inter_area / union_area
            if (inter_area / union_area > nms_threshold)
                keep = 0;
        }

        if (keep)
            picked.push_back(i);
    }
}

static inline float sigmoid(float x)
{
    return static_cast<float>(1.f / (1.f + exp(-x)));
}

static void generate_proposals(const ncnn::Mat& anchors, int stride, const ncnn::Mat& in_pad, const ncnn::Mat& feat_blob, float prob_threshold, std::vector<Object>& objects)
{
    const int num_grid = feat_blob.h;

    int num_grid_x;
    int num_grid_y;
    if (in_pad.w > in_pad.h)
    {
        num_grid_x = in_pad.w / stride;
        num_grid_y = num_grid / num_grid_x;
    }
    else
    {
        num_grid_y = in_pad.h / stride;
        num_grid_x = num_grid / num_grid_y;
    }

    const int num_class = feat_blob.w - 5;

    const int num_anchors = anchors.w / 2;

    for (int q = 0; q < num_anchors; q++)
    {
        const float anchor_w = anchors[q * 2];
        const float anchor_h = anchors[q * 2 + 1];

        const ncnn::Mat feat = feat_blob.channel(q);

        for (int i = 0; i < num_grid_y; i++)
        {
            for (int j = 0; j < num_grid_x; j++)
            {
                const float* featptr = feat.row(i * num_grid_x + j);

                // find class index with max class score
                int class_index = 0;
                float class_score = -FLT_MAX;
                for (int k = 0; k < num_class; k++)
                {
                    float score = featptr[5 + k];
                    if (score > class_score)
                    {
                        class_index = k;
                        class_score = score;
                    }
                }

                float box_score = featptr[4];

                float confidence = sigmoid(box_score) * sigmoid(class_score);

                if (confidence >= prob_threshold)
                {
                    // yolov5/models/yolo.py Detect forward
                    // y = x[i].sigmoid()
                    // y[..., 0:2] = (y[..., 0:2] * 2. - 0.5 + self.grid[i].to(x[i].device)) * self.stride[i]  # xy
                    // y[..., 2:4] = (y[..., 2:4] * 2) ** 2 * self.anchor_grid[i]  # wh

                    float dx = sigmoid(featptr[0]);
                    float dy = sigmoid(featptr[1]);
                    float dw = sigmoid(featptr[2]);
                    float dh = sigmoid(featptr[3]);

                    float pb_cx = (dx * 2.f - 0.5f + j) * stride;
                    float pb_cy = (dy * 2.f - 0.5f + i) * stride;

                    float pb_w = pow(dw * 2.f, 2) * anchor_w;
                    float pb_h = pow(dh * 2.f, 2) * anchor_h;

                    float x0 = pb_cx - pb_w * 0.5f;
                    float y0 = pb_cy - pb_h * 0.5f;
                    float x1 = pb_cx + pb_w * 0.5f;
                    float y1 = pb_cy + pb_h * 0.5f;

                    Object obj;
                    obj.rect.x = x0;
                    obj.rect.y = y0;
                    obj.rect.width = x1 - x0;
                    obj.rect.height = y1 - y0;
                    obj.label = class_index;
                    obj.prob = confidence;

                    objects.push_back(obj);
                }
            }
        }
    }
}

static int detect_yolov5(const cv::Mat& bgr, std::vector<Object>& objects)
{
    int img_w = bgr.cols;
    int img_h = bgr.rows;

    // letterbox pad to multiple of 32
    int w = img_w;
    int h = img_h;
    float scale = 1.f;
    if (w > h)
    {
        scale = (float)target_size / w;
        w = target_size;
        h = h * scale;
    }
    else
    {
        scale = (float)target_size / h;
        h = target_size;
        w = w * scale;
    }

    ncnn::Mat in = ncnn::Mat::from_pixels_resize(bgr.data, ncnn::Mat::PIXEL_BGR2RGB, img_w, img_h, w, h);

    // pad to target_size rectangle
    // yolov5/utils/datasets.py letterbox
    int wpad = (w + 31) / 32 * 32 - w;
    int hpad = (h + 31) / 32 * 32 - h;
    ncnn::Mat in_pad;
    ncnn::copy_make_border(in, in_pad, hpad / 2, hpad - hpad / 2, wpad / 2, wpad - wpad / 2, ncnn::BORDER_CONSTANT, 114.f);

    const float norm_vals[3] = {1 / 255.f, 1 / 255.f, 1 / 255.f};
    in_pad.substract_mean_normalize(0, norm_vals);

    ncnn::Extractor ex = yolov5.create_extractor();

    ex.input("images", in_pad);

    std::vector<Object> proposals;

    // anchor setting from yolov5/models/yolov5s.yaml

    // stride 8
    {
        ncnn::Mat out;
        ex.extract("output", out);

        ncnn::Mat anchors(6);
        anchors[0] = 10.f;
        anchors[1] = 13.f;
        anchors[2] = 16.f;
        anchors[3] = 30.f;
        anchors[4] = 33.f;
        anchors[5] = 23.f;

        std::vector<Object> objects8;
        generate_proposals(anchors, 8, in_pad, out, prob_threshold, objects8);

        proposals.insert(proposals.end(), objects8.begin(), objects8.end());
    }

    // stride 16
    {
        ncnn::Mat out;
        ex.extract("781", out);

        ncnn::Mat anchors(6);
        anchors[0] = 30.f;
        anchors[1] = 61.f;
        anchors[2] = 62.f;
        anchors[3] = 45.f;
        anchors[4] = 59.f;
        anchors[5] = 119.f;

        std::vector<Object> objects16;
        generate_proposals(anchors, 16, in_pad, out, prob_threshold, objects16);

        proposals.insert(proposals.end(), objects16.begin(), objects16.end());
    }

    // stride 32
    {
        ncnn::Mat out;
        ex.extract("801", out);

        ncnn::Mat anchors(6);
        anchors[0] = 116.f;
        anchors[1] = 90.f;
        anchors[2] = 156.f;
        anchors[3] = 198.f;
        anchors[4] = 373.f;
        anchors[5] = 326.f;

        std::vector<Object> objects32;
        generate_proposals(anchors, 32, in_pad, out, prob_threshold, objects32);

        proposals.insert(proposals.end(), objects32.begin(), objects32.end());
    }

    // sort all proposals by score from highest to lowest
    qsort_descent_inplace(proposals);

    // apply nms with nms_threshold
    std::vector<int> picked;
    nms_sorted_bboxes(proposals, picked, nms_threshold);

    int count = picked.size();

    objects.resize(count);
    for (int i = 0; i < count; i++)
    {
        objects[i] = proposals[picked[i]];

        // adjust offset to original unpadded
        float x0 = (objects[i].rect.x - (wpad / 2)) / scale;
        float y0 = (objects[i].rect.y - (hpad / 2)) / scale;
        float x1 = (objects[i].rect.x + objects[i].rect.width - (wpad / 2)) / scale;
        float y1 = (objects[i].rect.y + objects[i].rect.height - (hpad / 2)) / scale;

        // clip
        x0 = std::max(std::min(x0, (float)(img_w - 1)), 0.f);
        y0 = std::max(std::min(y0, (float)(img_h - 1)), 0.f);
        x1 = std::max(std::min(x1, (float)(img_w - 1)), 0.f);
        y1 = std::max(std::min(y1, (float)(img_h - 1)), 0.f);

        objects[i].rect.x = x0;
        objects[i].rect.y = y0;
        objects[i].rect.width = x1 - x0;
        objects[i].rect.height = y1 - y0;
    }

    return 0;
}

static void draw_objects(cv::Mat& bgr, const std::vector<Object>& objects)
{
    // Pre-calculate colors to avoid repeated calculations
    static const cv::Scalar box_color(255, 0, 0);
    static const cv::Scalar text_bg_color(255, 255, 255);
    static const cv::Scalar text_color(0, 0, 0);
    
    for (size_t i = 0; i < objects.size(); i++)
    {
        const Object& obj = objects[i];

        // Draw rectangle with thinner line for speed
        cv::rectangle(bgr, obj.rect, box_color, 1);

        // Simplified text rendering
        char text[64]; // Reduced buffer size
        sprintf(text, "%s %.0f%%", class_names[obj.label], obj.prob * 100);

        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine);

        int x = obj.rect.x;
        int y = obj.rect.y - label_size.height - baseLine;
        if (y < 0) y = 0;
        if (x + label_size.width > bgr.cols) x = bgr.cols - label_size.width;

        // Draw text background
        cv::rectangle(bgr, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
                      text_bg_color, -1);

        // Draw text with smaller font
        cv::putText(bgr, text, cv::Point(x, y + label_size.height),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, text_color, 1);
    }
}

static void detect_webcam()
{
    cv::VideoCapture cap;
    bool camera_found = false;
    
    // Try different camera indices and backends
    std::vector<std::pair<int, int>> camera_options = {
        {0, cv::CAP_V4L2},    // V4L2 backend (Linux native)
        {1, cv::CAP_V4L2},
        {0, cv::CAP_ANY},     // Any backend
        {1, cv::CAP_ANY},
        {2, cv::CAP_V4L2},
        {2, cv::CAP_ANY}
    };
    
    for (auto& option : camera_options) {
        std::cout << "Trying camera index " << option.first << " with backend " << option.second << std::endl;
        
        cap.open(option.first, option.second);
        if (cap.isOpened()) {
            // Test if we can actually read frames
            cv::Mat test_frame;
            bool can_read = cap.read(test_frame);
            if (can_read && !test_frame.empty()) {
                std::cout << "Success! Camera " << option.first << " is working" << std::endl;
                camera_found = true;
                break;
            } else {
                std::cout << "Camera " << option.first << " opened but cannot read frames" << std::endl;
                cap.release();
            }
        } else {
            std::cout << "Cannot open camera " << option.first << std::endl;
        }
    }
    
    if (!camera_found) {
        fprintf(stderr, "Error: Cannot open any working webcam\n");
        fprintf(stderr, "Try running: \n");
        fprintf(stderr, "  ls /dev/video*\n");
        fprintf(stderr, "  v4l2-ctl --list-devices\n");
        return;
    }

    // Set camera properties with error checking
    std::cout << "Setting camera properties..." << std::endl;
    
    if (!cap.set(cv::CAP_PROP_FRAME_WIDTH, 480)) {
        std::cout << "Warning: Cannot set frame width" << std::endl;
    }
    if (!cap.set(cv::CAP_PROP_FRAME_HEIGHT, 360)) {
        std::cout << "Warning: Cannot set frame height" << std::endl;
    }
    if (!cap.set(cv::CAP_PROP_BUFFERSIZE, 1)) {
        std::cout << "Warning: Cannot set buffer size" << std::endl;
    }
    
    // Get actual camera properties
    int actual_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int actual_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    std::cout << "Actual camera resolution: " << actual_width << "x" << actual_height << std::endl;
    
    cv::Mat frame;
    std::vector<Object> objects;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    int frame_count = 0;
    double fps = 0.0;
    
    int base_skip_frames = 3;
    int adaptive_skip = base_skip_frames;
    int frame_counter = 0;
    
    std::cout << "Press 'q' to quit webcam detection\n";
    
    while (true) {
        bool ret = cap.read(frame);
        if (!ret || frame.empty()) {
            fprintf(stderr, "Warning: Cannot read frame, retrying...\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        // Adaptive frame skipping berdasarkan jumlah deteksi
        if (frame_counter % (adaptive_skip + 1) == 0) {
            objects.clear();
            detect_yolov5(frame, objects);
            
            // Adjust skip rate based on detection count
            if (objects.size() > 2) {
                adaptive_skip = 5; // Skip lebih banyak jika banyak deteksi
            } else if (objects.size() > 0) {
                adaptive_skip = 4; // Skip sedang jika ada deteksi
            } else {
                adaptive_skip = base_skip_frames; // Skip normal jika tidak ada deteksi
            }
        }
        frame_counter++;
        
        // Draw detections (simplified)
        draw_objects(frame, objects);
        
        // Calculate FPS
        frame_count++;
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
        
        if (elapsed.count() >= 1000) {
            fps = frame_count * 1000.0 / elapsed.count();
            frame_count = 0;
            start_time = current_time;
        }
        
        // Simplified FPS display
        char fps_text[32];
        sprintf(fps_text, "FPS:%.0f D:%d", fps, (int)objects.size());
        cv::putText(frame, fps_text, cv::Point(10, 25), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1);
        
        cv::imshow("YOLOv5 Detection", frame);
        
        char key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) {
            break;
        }
    }
    
    cap.release();
    cv::destroyAllWindows();
}

int main(int argc, char** argv)
{
    yolov5.register_custom_layer("YoloV5Focus", YoloV5Focus_layer_creator);
    
    // Load YOLOv5 model
    yolov5.load_param("yolov5s.param");
    yolov5.load_model("yolov5s.bin");
    
    if (argc == 1) {
        // No arguments - run webcam detection
        std::cout << "Running webcam detection mode...\n";
        detect_webcam();
    }
    else if (argc == 2) {
        // Image file provided - run original image detection
        const char* imagepath = argv[1];
        
        cv::Mat m = cv::imread(imagepath, 1);
        if (m.empty()) {
            fprintf(stderr, "cv::imread %s failed\n", imagepath);
            return -1;
        }
        
        std::vector<Object> objects;
        detect_yolov5(m, objects);
        draw_objects(m, objects);
        
        cv::imshow("YOLOv5 Image Detection", m);
        cv::waitKey(0);
    }
    else {
        fprintf(stderr, "Usage: %s [imagepath] (leave empty for webcam)\n", argv[0]);
        return -1;
    }
    
    return 0;
}
