#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>

#include "ncnn/layer.h"
#include "ncnn/net.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <cfloat>
#include <cstdio>
#include <chrono>
#include <iostream>
#include <thread>


// Tambahkan include di bagian atas
#include <yaml-cpp/yaml.h>
#include <fstream>

// Tambahkan struct untuk config
struct ColorConfig {
    int lower_hue = 100;
    int upper_hue = 130;
    int lower_saturation = 50;
    int upper_saturation = 255;
    int lower_value = 50;
    int upper_value = 255;
    float min_percentage = 15.0f;
};

struct DetectionConfig {
    ColorConfig blue;
    int target_size = 288;
    float prob_threshold = 0.6f;
    float nms_threshold = 0.3f;
    int camera_width = 480;
    int camera_height = 360;
    int base_skip_frames = 3;
    int timer_ms = 50;
    int num_threads = 2;
};

// Global config variable
DetectionConfig g_config;

// Function untuk load config
bool load_config(const std::string& config_path) {
    try {
        YAML::Node config = YAML::LoadFile(config_path);
        
        // Load blue color config
        if (config["detection"]["blue"]) {
            auto blue = config["detection"]["blue"];
            g_config.blue.lower_hue = blue["lower_hue"].as<int>();
            g_config.blue.upper_hue = blue["upper_hue"].as<int>();
            g_config.blue.lower_saturation = blue["lower_saturation"].as<int>();
            g_config.blue.upper_saturation = blue["upper_saturation"].as<int>();
            g_config.blue.lower_value = blue["lower_value"].as<int>();
            g_config.blue.upper_value = blue["upper_value"].as<int>();
            g_config.blue.min_percentage = blue["min_percentage"].as<float>();
        }
        
        // Load YOLO config
        if (config["yolo"]) {
            auto yolo = config["yolo"];
            g_config.target_size = yolo["target_size"].as<int>();
            g_config.prob_threshold = yolo["prob_threshold"].as<float>();
            g_config.nms_threshold = yolo["nms_threshold"].as<float>();
        }
        
        // Load camera config
        if (config["camera"]) {
            auto camera = config["camera"];
            g_config.camera_width = camera["width"].as<int>();
            g_config.camera_height = camera["height"].as<int>();
        }
        
        // Load performance config
        if (config["performance"]) {
            auto perf = config["performance"];
            g_config.base_skip_frames = perf["base_skip_frames"].as<int>();
            g_config.timer_ms = perf["timer_ms"].as<int>();
            g_config.num_threads = perf["num_threads"].as<int>();
        }
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error loading config: " << e.what() << std::endl;
        return false;
    }
}

// Optimized parameters from standalone version
const int target_size = 288;  // Reduced from 640 for better performance
const float prob_threshold = 0.6f;  // Increased from 0.25f for fewer false positives
const float nms_threshold = 0.3f;   // Reduced from 0.45f for more aggressive NMS
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

// Optimized YoloV5Focus class from standalone version
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

// Optimized helper functions from standalone version
static inline float intersection_area(const Object& a, const Object& b)
{
    cv::Rect_<float> inter = a.rect & b.rect;
    return inter.area();
}

static inline float sigmoid(float x)
{
    return static_cast<float>(1.f / (1.f + exp(-x)));
}

// Optimized sorting and NMS functions from standalone version
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
            float inter_area = intersection_area(a, b);
            float union_area = areas[i] + areas[picked[j]] - inter_area;
            if (inter_area / union_area > nms_threshold)
                keep = 0;
        }
        if (keep)
            picked.push_back(i);
    }
}

// Optimized generate_proposals function from standalone version
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

// Modifikasi draw_objects function untuk highlight first person
static void draw_objects(cv::Mat& bgr, const std::vector<Object>& objects)
{
    for (size_t i = 0; i < objects.size(); i++)
    {
        const Object& obj = objects[i];

        // Beda warna untuk person pertama vs yang lain
        cv::Scalar box_color = (i == 0) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 255, 255);  // Green untuk first, yellow untuk others
        int thickness = (i == 0) ? 3 : 2;  // Lebih tebal untuk first person
        
        // Draw rectangle
        cv::rectangle(bgr, obj.rect, box_color, thickness);

        // Custom text
        char text[64];
        if (i == 0) {
            sprintf(text, "FIRST-Person-Blue %.0f%%", obj.prob * 100);
        } else {
            sprintf(text, "Person-Blue %.0f%%", obj.prob * 100);
        }

        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

        int x = obj.rect.x;
        int y = obj.rect.y - label_size.height - baseLine;
        if (y < 0) y = 0;
        if (x + label_size.width > bgr.cols) x = bgr.cols - label_size.width;

        // Draw text background
        cv::Scalar text_bg_color = (i == 0) ? cv::Scalar(255, 255, 255) : cv::Scalar(200, 200, 200);
        cv::rectangle(bgr, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
                      text_bg_color, -1);

        // Draw text
        cv::putText(bgr, text, cv::Point(x, y + label_size.height),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    }
}

// Update fungsi is_wearing_blue untuk menggunakan config
static bool is_wearing_blue(const cv::Mat& image, const Object& person)
{
    if (person.label != 0) return false;
    
    cv::Rect roi;
    roi.x = std::max(0, (int)person.rect.x);
    roi.y = std::max(0, (int)person.rect.y);
    roi.width = std::min(image.cols - roi.x, (int)person.rect.width);
    roi.height = std::min(image.rows - roi.y, (int)person.rect.height);
    
    if (roi.width <= 0 || roi.height <= 0) return false;
    
    cv::Mat person_roi = image(roi);
    cv::Mat hsv;
    cv::cvtColor(person_roi, hsv, cv::COLOR_BGR2HSV);
    
    // Gunakan config untuk range warna
    cv::Scalar lower_blue(g_config.blue.lower_hue, 
                         g_config.blue.lower_saturation, 
                         g_config.blue.lower_value);
    cv::Scalar upper_blue(g_config.blue.upper_hue, 
                         g_config.blue.upper_saturation, 
                         g_config.blue.upper_value);
    
    cv::Mat blue_mask;
    cv::inRange(hsv, lower_blue, upper_blue, blue_mask);
    
    int total_pixels = person_roi.rows * person_roi.cols;
    int blue_pixels = cv::countNonZero(blue_mask);
    float blue_percentage = (float)blue_pixels / total_pixels * 100.0f;
    
    // Gunakan threshold dari config
    return blue_percentage > g_config.blue.min_percentage;
}

// Ganti bagian akhir detect_yolov5() yang error (mulai dari setelah NMS)
static int detect_yolov5(const cv::Mat& bgr, std::vector<Object>& objects, ncnn::Net& yolov5_net)
{
    int img_w = bgr.cols;
    int img_h = bgr.rows;

    // letterbox pad to multiple of 32 - using optimized target_size
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
    int wpad = (w + 31) / 32 * 32 - w;
    int hpad = (h + 31) / 32 * 32 - h;
    ncnn::Mat in_pad;
    ncnn::copy_make_border(in, in_pad, hpad / 2, hpad - hpad / 2, wpad / 2, wpad - wpad / 2, ncnn::BORDER_CONSTANT, 114.f);

    in_pad.substract_mean_normalize(0, norm_vals);

    ncnn::Extractor ex = yolov5_net.create_extractor();
    ex.input("images", in_pad);

    std::vector<Object> proposals;

    // stride 8 - exact same anchor values from standalone
    {
        ncnn::Mat out;
        ex.extract("output", out);
        ncnn::Mat anchors(6);
        anchors[0] = 10.f; anchors[1] = 13.f; anchors[2] = 16.f;
        anchors[3] = 30.f; anchors[4] = 33.f; anchors[5] = 23.f;
        std::vector<Object> objects8;
        generate_proposals(anchors, 8, in_pad, out, prob_threshold, objects8);
        proposals.insert(proposals.end(), objects8.begin(), objects8.end());
    }

    // stride 16
    {
        ncnn::Mat out;
        ex.extract("781", out);
        ncnn::Mat anchors(6);
        anchors[0] = 30.f; anchors[1] = 61.f; anchors[2] = 62.f;
        anchors[3] = 45.f; anchors[4] = 59.f; anchors[5] = 119.f;
        std::vector<Object> objects16;
        generate_proposals(anchors, 16, in_pad, out, prob_threshold, objects16);
        proposals.insert(proposals.end(), objects16.begin(), objects16.end());
    }

    // stride 32
    {
        ncnn::Mat out;
        ex.extract("801", out);
        ncnn::Mat anchors(6);
        anchors[0] = 116.f; anchors[1] = 90.f; anchors[2] = 156.f;
        anchors[3] = 198.f; anchors[4] = 373.f; anchors[5] = 326.f;
        std::vector<Object> objects32;
        generate_proposals(anchors, 32, in_pad, out, prob_threshold, objects32);
        proposals.insert(proposals.end(), objects32.begin(), objects32.end());
    }

    // Use optimized sorting and NMS
    qsort_descent_inplace(proposals);
    std::vector<int> picked;
    nms_sorted_bboxes(proposals, picked, nms_threshold);

    // PERBAIKAN: Filter hanya person dengan baju biru
    std::vector<Object> filtered_objects;
    for (int i = 0; i < (int)picked.size(); i++)
    {
        Object obj = proposals[picked[i]];
        
        // adjust offset to original unpadded
        float x0 = (obj.rect.x - (wpad / 2)) / scale;
        float y0 = (obj.rect.y - (hpad / 2)) / scale;
        float x1 = (obj.rect.x + obj.rect.width - (wpad / 2)) / scale;
        float y1 = (obj.rect.y + obj.rect.height - (hpad / 2)) / scale;

        // clip
        x0 = std::max(std::min(x0, (float)(img_w - 1)), 0.f);
        y0 = std::max(std::min(y0, (float)(img_h - 1)), 0.f);
        x1 = std::max(std::min(x1, (float)(img_w - 1)), 0.f);
        y1 = std::max(std::min(y1, (float)(img_h - 1)), 0.f);

        obj.rect.x = x0;
        obj.rect.y = y0;
        obj.rect.width = x1 - x0;
        obj.rect.height = y1 - y0;
        
        // Filter: hanya person (label 0) dan yang memakai baju biru
        if (obj.label == 0 && is_wearing_blue(bgr, obj)) {
            filtered_objects.push_back(obj);
        }
    }
    
    // Return hanya person dengan baju biru
    objects = filtered_objects;
    return 0;
}

class YOLONode : public rclcpp::Node
{
public:
    YOLONode() : Node("yolo_node")
    {
        // Parameters with optimized defaults
        input_mode_ = this->declare_parameter("input_mode", std::string("webcam"));
        model_path_ = this->declare_parameter("model_path", std::string("./"));
        camera_device_ = this->declare_parameter("camera_device", 0);
        image_path_ = this->declare_parameter("image_path", std::string(""));
        enable_display_ = this->declare_parameter("enable_display", false);
        
        // Load config file DULU sebelum init mode
        std::string config_path = this->declare_parameter("config_path", std::string("./color_config.yaml"));
        
        if (!load_config(config_path)) {
            RCLCPP_WARN(this->get_logger(), "Failed to load config, using defaults");
        } else {
            RCLCPP_INFO(this->get_logger(), "Config loaded from: %s", config_path.c_str());
        }
        
        // Print current config
        RCLCPP_INFO(this->get_logger(), "Blue HSV range: H(%d-%d) S(%d-%d) V(%d-%d), Min%%:%.1f", 
                    g_config.blue.lower_hue, g_config.blue.upper_hue,
                    g_config.blue.lower_saturation, g_config.blue.upper_saturation,
                    g_config.blue.lower_value, g_config.blue.upper_value,
                    g_config.blue.min_percentage);
        
        // Initialize YOLO with optimized settings from standalone
        if (!init_yolo()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize YOLO");
            return;
        }
        
        // Publishers
        detection_pub_ = this->create_publisher<std_msgs::msg::String>("/camera/detections", 1);
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_annotated", 1);
        
        // Initialize FPS tracking
        frame_count_ = 0;
        start_time_ = std::chrono::high_resolution_clock::now();
        
        // Adaptive frame processing from standalone (with optimizations)
        base_skip_frames_ = 3;
        adaptive_skip_ = base_skip_frames_;
        frame_counter_ = 0;
        
        // Initialize based on input mode
        if (input_mode_ == "webcam") {
            init_webcam_mode();
        } else if (input_mode_ == "topic") {
            init_topic_mode();
        } else if (input_mode_ == "image") {
            init_image_mode();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid input_mode: %s", input_mode_.c_str());
        }
        
        RCLCPP_INFO(this->get_logger(), "YOLO Node initialized in %s mode", input_mode_.c_str());
    }
    
    ~YOLONode()
    {
        if (cap_.isOpened()) {
            cap_.release();
        }
        cv::destroyAllWindows();
    }

private:
    bool init_yolo()
    {
        std::string param_file = model_path_ + "/yolov5s.param";
        std::string bin_file = model_path_ + "/yolov5s.bin";
        
        // Apply optimized settings BEFORE loading model (from standalone)
        yolov5_.opt.num_threads = 2;  // Optimized for Pi 4
        yolov5_.opt.use_vulkan_compute = false;
        yolov5_.opt.use_winograd_convolution = true;
        yolov5_.opt.use_sgemm_convolution = true;
        yolov5_.opt.use_int8_inference = false;
        
        yolov5_.register_custom_layer("YoloV5Focus", YoloV5Focus_layer_creator);
        
        if (yolov5_.load_param(param_file.c_str()) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load %s", param_file.c_str());
            return false;
        }
        
        if (yolov5_.load_model(bin_file.c_str()) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load %s", bin_file.c_str());
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "YOLO model loaded successfully with optimizations");
        return true;
    }
    
    void init_webcam_mode()
    {
        // Try multiple camera options like standalone version (robust camera detection)
        bool camera_found = false;
        std::vector<std::pair<int, int>> camera_options = {
            {0, cv::CAP_V4L2},    // V4L2 backend (Linux native)
            {1, cv::CAP_V4L2},
            {0, cv::CAP_ANY},     // Any backend
            {1, cv::CAP_ANY},
            {2, cv::CAP_V4L2},
            {2, cv::CAP_ANY}
        };
        
        for (auto& option : camera_options) {
            RCLCPP_INFO(this->get_logger(), "Trying camera %d with backend %d", option.first, option.second);
            
            cap_.open(option.first, option.second);
            if (cap_.isOpened()) {
                cv::Mat test_frame;
                if (cap_.read(test_frame) && !test_frame.empty()) {
                    RCLCPP_INFO(this->get_logger(), "Camera %d working!", option.first);
                    camera_found = true;
                    break;
                } else {
                    cap_.release();
                }
            }
        }
        
        if (!camera_found) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open any camera");
            return;
        }
        
        // Optimized camera settings from standalone
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 480);   // Lower resolution for performance
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 360);
        cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);      // Reduce latency
        cap_.set(cv::CAP_PROP_FPS, 30);
        
        // Get actual camera properties
        int actual_width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
        int actual_height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
        RCLCPP_INFO(this->get_logger(), "Camera resolution: %dx%d", actual_width, actual_height);
        
        // Timer for webcam capture (optimized frequency)
            // HANYA buat timer jika di webcam mode
        if (input_mode_ == "webcam") {
            // Timer for webcam capture (optimized frequency)
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(50), // 20 FPS max, allows adaptive processing
                std::bind(&YOLONode::webcam_callback, this));
        }
            
        RCLCPP_INFO(this->get_logger(), "Webcam initialized with optimizations");
    }
    
    void init_topic_mode()
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 1,
            std::bind(&YOLONode::image_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Subscribed to /camera/image_raw");
    }

    void init_image_mode()
    {
        if (image_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "image_path parameter required for image mode");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Image mode initialized: %s", image_path_.c_str());
        
        // Langsung proses image sekali saja
        process_single_image();
    }

    // Tambahkan function baru untuk proses image sekali
    void process_single_image()
    {
        cv::Mat image = cv::imread(image_path_);
        if (image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load image: %s", image_path_.c_str());
            rclcpp::shutdown();
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Processing image: %s (%dx%d)", 
                image_path_.c_str(), image.cols, image.rows);
        
        // Run YOLO detection
        std::vector<Object> objects;
        detect_yolov5(image, objects, yolov5_);
        
        // Create annotated image
        cv::Mat annotated = image.clone();
        draw_objects(annotated, objects);
        
        // Publish results
        publish_detections(objects);
        publish_annotated_image(annotated);
        
        // // Show result window
        // cv::imshow("YOLOv5 ROS2 Detection", annotated);
        // RCLCPP_INFO(this->get_logger(), "Detection complete! Found %zu person(s) with blue clothes. Press any key or wait 10 seconds...", objects.size());
        
        // cv::waitKey(10000);  // 10 second timeout
        // cv::destroyAllWindows();
        if (enable_display_) {
            // Show result window
            cv::imshow("YOLOv5 ROS2 Detection", annotated);
            RCLCPP_INFO(this->get_logger(), "Detection complete! Found %zu person(s) with blue clothes. Press any key or wait 10 seconds...", objects.size());
            
            cv::waitKey(10000);  // 10 second timeout
            cv::destroyAllWindows();
        } else {
            RCLCPP_INFO(this->get_logger(), "Detection complete! Found %zu person(s) with blue clothes.", objects.size());
        }
        
        RCLCPP_INFO(this->get_logger(), "Image processing finished, shutting down...");
        rclcpp::shutdown();
    }
    
    // Optimized webcam callback with adaptive frame skipping from standalone
    void webcam_callback()
    {
        cv::Mat frame;
        if (!cap_.read(frame) || frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Cannot read frame, retrying...");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            return;
        }
        
        // Adaptive frame skipping logic from standalone (EXACT COPY)
        bool should_process = (frame_counter_ % (adaptive_skip_ + 1) == 0);
        
        if (should_process) {
            objects_.clear();
            detect_yolov5(frame, objects_, yolov5_);
            
            // Adjust skip rate based on detection count (from standalone)
            if (objects_.size() > 2) {
                adaptive_skip_ = 5; // Skip lebih banyak jika banyak deteksi
            } else if (objects_.size() > 0) {
                adaptive_skip_ = 4; // Skip sedang jika ada deteksi
            } else {
                adaptive_skip_ = base_skip_frames_; // Skip normal jika tidak ada deteksi
            }
        }
        frame_counter_++;
        
        // Create annotated frame for display (using last detection results)
        cv::Mat display_frame = frame.clone();
        draw_objects(display_frame, objects_);
        
        // Calculate and display FPS (from standalone)
        frame_count_++;
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time_);
        
        if (elapsed.count() >= 1000) {
            fps_ = frame_count_ * 1000.0 / elapsed.count();
            frame_count_ = 0;
            start_time_ = current_time;
        }
        
        // Add FPS text like standalone
        char fps_text[32];
        sprintf(fps_text, "FPS:%.0f D:%d", fps_, (int)objects_.size());
        cv::putText(display_frame, fps_text, cv::Point(10, 25), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1);
        
        // // Show webcam feed with detections
        // cv::imshow("YOLOv5 ROS2 Detection", display_frame);
        
        // // Non-blocking waitKey untuk bisa di-close dengan ESC
        // int key = cv::waitKey(1) & 0xFF;
        // if (key == 27) { // ESC key
        //     RCLCPP_INFO(this->get_logger(), "ESC pressed, shutting down...");
        //     cv::destroyAllWindows();
        //     rclcpp::shutdown();
        //     return;
        // }
        // Show webcam feed with detections (hanya jika display enabled)
        if (enable_display_) {
            cv::imshow("YOLOv5 ROS2 Detection", display_frame);
            
            // Non-blocking waitKey untuk bisa di-close dengan ESC
            int key = cv::waitKey(1) & 0xFF;
            if (key == 27) { // ESC key
                RCLCPP_INFO(this->get_logger(), "ESC pressed, shutting down...");
                cv::destroyAllWindows();
                rclcpp::shutdown();
                return;
            }
        }
        
        // Publish ke ROS topics (hanya jika ada processing)
        // Di webcam_callback(), update logging:
        if (should_process) {
            publish_detections(objects_);  // SELALU publish, baik ada detection atau tidak
            publish_annotated_image(display_frame);
            
            if (!objects_.empty()) {
                RCLCPP_INFO(this->get_logger(), "Found %zu person(s) with blue clothes, published FIRST person (FPS: %.1f)", 
                            objects_.size(), fps_);
            } else {
                RCLCPP_DEBUG(this->get_logger(), "No person detected, published empty (FPS: %.1f)", fps_);
            }
        }
    }

    // Perbaiki image_process_callback() di YOLONode class
    void image_process_callback()
    {
        timer_->cancel(); // Stop timer after first run
        
        cv::Mat image = cv::imread(image_path_);
        if (image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load image: %s", image_path_.c_str());
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Processing image: %s (%dx%d)", 
                image_path_.c_str(), image.cols, image.rows);
        
        // Run YOLO detection
        std::vector<Object> objects;
        detect_yolov5(image, objects, yolov5_);
        
        // Create annotated image
        cv::Mat annotated = image.clone();
        draw_objects(annotated, objects);
        
        // Publish results
        publish_detections(objects);
        publish_annotated_image(annotated);
        
        // Show result window
        cv::imshow("YOLOv5 ROS2 Detection", annotated);
        RCLCPP_INFO(this->get_logger(), "Detection complete! Found %zu person(s) with blue clothes. Press any key or wait 10 seconds...", objects.size());
        
        cv::waitKey(10000);  // 10 second timeout
        cv::destroyAllWindows();
        
        RCLCPP_INFO(this->get_logger(), "Image processing finished, shutting down...");
        rclcpp::shutdown();
    }
        
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            process_image(cv_ptr->image);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
        }
    }
    
    void process_image(const cv::Mat& image)
    {
        // Run YOLO detection
        std::vector<Object> objects;
        detect_yolov5(image, objects, yolov5_);        
        // Create annotated image
        cv::Mat annotated = image.clone();
        draw_objects(annotated, objects);
        
        // Publish results
        publish_detections(objects);
        publish_annotated_image(annotated);
        
        // Log results
        if (!objects.empty()) {
            RCLCPP_INFO(this->get_logger(), "Detected %zu objects", objects.size());
        }
    }

    void publish_detections(const std::vector<Object>& objects)
    {
        std_msgs::msg::String msg;
        
        if (objects.empty()) {
            // Publish empty string jika tidak ada deteksi
            msg.data = "";
        } else {
            // HANYA ambil person pertama (index 0)
            const Object& first_person = objects[0];
            
            // Format: person_blue:confidence:x:y:width:height (hanya satu person)
            msg.data = "person_blue:" + std::to_string(first_person.prob) + ":" +
                    std::to_string((int)first_person.rect.x) + ":" +
                    std::to_string((int)first_person.rect.y) + ":" +
                    std::to_string((int)first_person.rect.width) + ":" +
                    std::to_string((int)first_person.rect.height);
        }
        
        detection_pub_->publish(msg);
        
        if (!objects.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "Published first person: %s", msg.data.c_str());
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Published empty detection");
        }
    }
    
    void publish_annotated_image(const cv::Mat& image)
    {
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        image_pub_->publish(*msg);
    }
    
    // Member variables
    std::string input_mode_;
    std::string model_path_;
    std::string image_path_;
    int camera_device_;
    bool enable_display_;
    
    // YOLO model
    ncnn::Net yolov5_;
    
    // Optimized parameters from standalone (EXACT COPY)
    // const int target_size = 288;  // Reduced from 640
    // const float prob_threshold = 0.6f;  // Increased from 0.25f
    // const float nms_threshold = 0.3f;   // Reduced from 0.45f
    
    // FPS tracking variables from standalone
    int frame_count_ = 0;
    double fps_ = 0.0;
    std::chrono::high_resolution_clock::time_point start_time_;
    
    // Adaptive frame processing variables from standalone
    int base_skip_frames_ = 3;
    int adaptive_skip_ = 3;
    int frame_counter_ = 0;
    std::vector<Object> objects_;  // Store last detection results
    
    // ROS2 communication
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detection_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Camera
    cv::VideoCapture cap_;
};

// Main function
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<YOLONode>();
        
        // Check if node was properly initialized
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("main"), "Node initialization failed");
            return -1;
        }
        
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting YOLOv5 ROS2 Node with optimizations...");
        
        // Spin the node
        rclcpp::spin(node);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception in main: %s", e.what());
        return -1;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "YOLOv5 Node shutting down...");
    rclcpp::shutdown();
    return 0;
}