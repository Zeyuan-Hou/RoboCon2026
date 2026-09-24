// 二维码识别节点


#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/aruco.hpp>
#include <rclcpp/rclcpp.hpp>

// 引入自定义消息类型 (请替换为你的实际包名)
#include "quad/msg/qr_result.hpp" 

static std::mutex mutex;
static std::atomic_bool isOpen(true);
cv::Mat frame;
cv::Mat process;

// 相机线程函数保持不变
void cameraThreadFunc(int camId, int height, int width, int fps, const std::string& fourcc_str, cv::Mat *pFrame)
{
    cv::VideoCapture capture(camId, cv::CAP_V4L2);
    if (fourcc_str.length() == 4) {
        capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc(fourcc_str[0], fourcc_str[1], fourcc_str[2], fourcc_str[3]));
    }
    capture.set(cv::CAP_PROP_FRAME_WIDTH, width);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    capture.set(cv::CAP_PROP_FPS, fps);

    if (!capture.isOpened()) {
        isOpen.store(false);
        return;
    }

    cv::Mat localFrame;
    while (rclcpp::ok() && isOpen.load()) {
        capture >> localFrame;
        if (localFrame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        if (mutex.try_lock()) {
            localFrame.copyTo(*pFrame);
            mutex.unlock();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    capture.release();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("aruco_processor");

    // 【修改点1】整合 Publisher，只发布一个综合话题
    auto qr_pub = node->create_publisher<quad::msg::QrResult>("qr_detection_result", 10);
    
    node->declare_parameter<bool>("show_image", false);
    node->declare_parameter<int>("camera_id", 4);
    node->declare_parameter<int>("image_width", 640);
    node->declare_parameter<int>("image_height", 480);
    node->declare_parameter<int>("camera_fps", 60);
    node->declare_parameter<std::string>("camera_fourcc", "YUYV");
    node->declare_parameter<double>("marker_size_mm", 168.0);
    node->declare_parameter<double>("camera_fx", 618.11);
    node->declare_parameter<double>("camera_fy", 617.80);
    node->declare_parameter<double>("camera_cx", 326.69);
    node->declare_parameter<double>("camera_cy", 244.64);
    node->declare_parameter<std::vector<double>>("dist_coeffs", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    bool show_image = node->get_parameter("show_image").as_bool();
    int camera_id = node->get_parameter("camera_id").as_int();
    int image_width = node->get_parameter("image_width").as_int();
    int image_height = node->get_parameter("image_height").as_int();
    int camera_fps = node->get_parameter("camera_fps").as_int();
    std::string camera_fourcc = node->get_parameter("camera_fourcc").as_string();
    double marker_size_mm = node->get_parameter("marker_size_mm").as_double();
    double camera_fx = node->get_parameter("camera_fx").as_double();
    double camera_fy = node->get_parameter("camera_fy").as_double();
    double camera_cx = node->get_parameter("camera_cx").as_double();
    double camera_cy = node->get_parameter("camera_cy").as_double();
    std::vector<double> dist_coeffs_vec = node->get_parameter("dist_coeffs").as_double_array();

    if (show_image) {
        cv::namedWindow("image", cv::WINDOW_NORMAL);
        cv::resizeWindow("image", 960, 540);
    }

    std::thread camThread(cameraThreadFunc, camera_id, image_height, image_width, camera_fps, camera_fourcc, &frame);

    int loop_mark = 0;
    clock_t start = clock();

    while (rclcpp::ok() && isOpen.load())
    {
        mutex.lock();
        frame.copyTo(process);
        mutex.unlock();

        if (process.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_250);
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

        cv::aruco::detectMarkers(process, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
            camera_fx, 0, camera_cx,
              0, camera_fy, camera_cy,
              0, 0, 1.0000);
        cv::Mat distCoeffs = cv::Mat(dist_coeffs_vec, true).reshape(1, 1);

        // 打印调试信息：检查相机内参和畸变系数是否正确加载
        static bool printed_params = false;
        if (!printed_params) {
            RCLCPP_INFO_STREAM(node->get_logger(), "Loaded Camera Matrix:\n" << cameraMatrix);
            RCLCPP_INFO_STREAM(node->get_logger(), "Loaded Dist Coeffs:\n" << distCoeffs);
            printed_params = true;
        }

        if (!markerIds.empty())
        {
            cv::aruco::drawDetectedMarkers(process, markerCorners, markerIds);
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(markerCorners, marker_size_mm, cameraMatrix, distCoeffs, rvecs, tvecs);

            for (size_t i = 0; i < markerIds.size(); i++)
            {
                // 计算欧拉角
                cv::Mat rotMat;
                cv::Rodrigues(rvecs[i], rotMat);
                double sy = sqrt(rotMat.at<double>(0, 0) * rotMat.at<double>(0, 0) + rotMat.at<double>(1, 0) * rotMat.at<double>(1, 0));
                double pitch = atan2(-rotMat.at<double>(2, 0), sy) * 180.0 / CV_PI;
                double distance = sqrt(tvecs[i][0]*tvecs[i][0] + tvecs[i][1]*tvecs[i][1] + tvecs[i][2]*tvecs[i][2]);

                // 【修改点2】填充并发布整合后的消息
                quad::msg::QrResult res;
                res.marker_id = markerIds[i];
                res.task_type = markerIds[i] - 22;
                res.x = tvecs[i][0];   // 水平
                res.y = tvecs[i][2];   // 垂直
                // res.z = tvecs[i][2];   // 深度
                res.yaw = pitch;       // 偏航
                res.distance = distance;

                qr_pub->publish(res);

                // 绘制调试信息
                cv::aruco::drawAxis(process, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], marker_size_mm);
                std::string label = "ID:" + std::to_string(res.marker_id) + " Dist:" + std::to_string(int(distance)) + "mm";
                cv::putText(process, label, markerCorners[i][0], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,255), 2);
            }
        }

        // 帧率计算与显示
        loop_mark++;
        clock_t end = clock();
        double fps = double(loop_mark) / ((double)(end - start) / CLOCKS_PER_SEC);
        
        if (show_image) {
            cv::putText(process, "FPS: " + std::to_string(fps), cv::Point(30,30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,255,0), 2);
            cv::imshow("image", process);
            if (cv::waitKey(1) == 'q') {
                isOpen.store(false);
                break;
            }
        }
        rclcpp::spin_some(node);
    }

    isOpen.store(false);
    if (camThread.joinable()) camThread.join();
    if (show_image) {
        cv::destroyAllWindows();
    }
    rclcpp::shutdown();
    return 0;
}
