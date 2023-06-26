#include <cstdio>
#include <chrono>
#include <string>
#include <vector>
#include <unistd.h>

// Unitree SDK
#include <UnitreeCameraSDK.hpp>

// ROS stuff
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;

std::string type2str(int type) {
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
    }

    r += "C";
    r += (chans+'0');

    return r;
}

int main(int argc, char **argv)
{
    int deviceNode = 0; ///< default 0 -> /dev/video0
    cv::Size frameSize(1856, 800); ///< default frame size 1856x800
    int fps = 30; ///< default camera fps: 30
    
    if(argc >= 2)
    {
        deviceNode = std::atoi(argv[1]);
        if(argc >= 4){
            frameSize = cv::Size(std::atoi(argv[2]), std::atoi(argv[3]));
        }
        if(argc >=5)
            fps = std::atoi(argv[4]);
    }

    std::string topic_name;
    if(deviceNode == 0)
    {
        topic_name = "left_stereo";
    }
    else if(deviceNode == 1)
    {
        topic_name = "right_stereo";
    }
    
    // Load camera stuf
    UnitreeCamera cam(deviceNode); //"stereo_camera_config.yaml"); ///< init camera by device node number
    if(!cam.isOpened())   ///< get camera open state
        exit(EXIT_FAILURE);
    
    cam.setRawFrameSize(frameSize); ///< set camera frame size
    cam.setRawFrameRate(fps);       ///< set camera camera fps
    cam.setRectFrameSize(cv::Size(frameSize.width >> 2, frameSize.height >> 1)); ///< set camera rectify frame size
    
    cam.startCapture(); ///< disable image h264 encoding and share memory sharing
    // cam.startStereoCompute(); ///< start disparity computing
    
    usleep(500000);

    // Start ROS stuff
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_publisher", options);
    image_transport::ImageTransport it(node);
    // image_transport::Publisher color_pub = it.advertise("camera/image", 1);
    image_transport::CameraPublisher left_cam_pub = it.advertiseCamera(topic_name + "/left_camera/rect_image", 1);
    image_transport::CameraPublisher right_cam_pub = it.advertiseCamera(topic_name + "/right_camera/rect_image", 1);
    // image_transport::Publisher depth_pub = it.advertise("camera/depth", 1);

    std_msgs::msg::Header image_header;
    sensor_msgs::msg::Image::SharedPtr left_msg, right_msg; //, depth_msg;
    sensor_msgs::msg::CameraInfo::SharedPtr left_cam_info(new sensor_msgs::msg::CameraInfo());
    sensor_msgs::msg::CameraInfo::SharedPtr right_cam_info(new sensor_msgs::msg::CameraInfo());

    // Get calibration parameters for the right camera
    std::vector<cv::Mat> right_params;
    if(cam.getCalibParams(right_params, true))
    {
        // intrinsic, distortion, xi, rotation, translation, kfe
        for(std::size_t i=0; i<right_params.size(); i++)
        {
            std::cout << "\ndata:\n" << right_params[i] << std::endl;
        }

        // Populate CameraInfo message
        right_cam_info->distortion_model = "rational_polynomial";
        for(int i=0; i<right_params[0].rows*right_params[0].cols; i++)
        {
            right_cam_info->k[i] = right_params[0].reshape(1).at<double>(i);
        }
        right_cam_info->d.resize(4, 0.0);
        for(int i=0; i<right_params[1].rows*right_params[1].cols; i++)
        {
            right_cam_info->d[i] = right_params[1].reshape(1).at<double>(i);
        }
        for(int i=0; i<right_params[3].rows*right_params[3].cols; i++)
        {
            right_cam_info->r[i] = right_params[3].reshape(1).at<double>(i);
        }
        for(int i=0; i<right_params[5].rows*right_params[5].cols; i++)
        {
            right_cam_info->p[i] = right_params[5].reshape(1).at<double>(i);
        }
    }

    // Get calibration parameters for the left camera
    std::vector<cv::Mat> left_params;
    if(cam.getCalibParams(left_params, false))
    {
        // intrinsic, distortion, xi, rotation, translation, kfe
        for(std::size_t i=0; i<left_params.size(); i++)
        {
            std::cout << "\ndata:\n" << left_params[i] << std::endl;
        }

        // Populate CameraInfo message
        left_cam_info->distortion_model = "rational_polynomial";
        for(int i=0; i<left_params[0].rows*left_params[0].cols; i++)
        {
            left_cam_info->k[i] = left_params[0].reshape(1).at<double>(i);
        }
        left_cam_info->d.resize(4, 0.0);
        for(int i=0; i<left_params[1].rows*left_params[1].cols; i++)
        {
            left_cam_info->d[i] = left_params[1].reshape(1).at<double>(i);
        }
        for(int i=0; i<left_params[3].rows*left_params[3].cols; i++)
        {
            left_cam_info->r[i] = left_params[3].reshape(1).at<double>(i);
        }
        for(int i=0; i<left_params[5].rows*left_params[5].cols; i++)
        {
            left_cam_info->p[i] = left_params[5].reshape(1).at<double>(i);
        }
    }
    
    rclcpp::WallRate loop_rate(30);
    while(rclcpp::ok() && cam.isOpened())
    {
        cv::Mat left, right; //, depth; //, dispf;
        std::chrono::microseconds t;

        // Get rectified left andright frames  
        if(!cam.getRectStereoFrame(left, right))
        {
            usleep(50);
            continue;
        }

        // Get stereo camera depth image
        // cam.getDepthFrame(dispf, depth);

        // if(!cam.getDepthFrame(depth, false, t))
        // {
        //     usleep(50);
        //     continue;
        // }

        image_header.stamp = node->get_clock()->now();;

        if (!left.empty())
        {
            image_header.frame_id = "left_camera_frame";
            // Debug color image type
            // std::string ty =  type2str( left.type() );
            // printf("Color: %s %dx%d \n", ty.c_str(), left.cols, left.rows);

            cv::flip(left, left, -1);
            left_msg = cv_bridge::CvImage(image_header, "bgr8", left).toImageMsg();

            left_cam_info->height = left.rows;
            left_cam_info->width = left.cols;
            left_cam_info->header = image_header;

            // Publish via image_transport
            left_cam_pub.publish(left_msg, left_cam_info);
            // cv::waitKey(1);
        }

        if (!right.empty())
        {
            image_header.frame_id = "right_camera_frame";
            // Debug color image type
            // std::string ty =  type2str( left.type() );
            // printf("Color: %s %dx%d \n", ty.c_str(), left.cols, left.rows);
            cv::flip(right, right, -1);
            right_msg = cv_bridge::CvImage(image_header, "bgr8", right).toImageMsg();

            right_cam_info->height = right.rows;
            right_cam_info->width = right.cols;
            right_cam_info->header = image_header;

            // Publish via image_transport
            right_cam_pub.publish(right_msg, right_cam_info);
            // cv::waitKey(1);
        }

        // if (!depth.empty())
        // {
        //     // Debug depth image type
        //     // std::string ty =  type2str( depth.type() );
        //     // printf("Depth: %s %dx%d \n", ty.c_str(), depth.cols, depth.rows);

        //     // std::string ty2 =  type2str( dispf.type() );
        //     // printf("Disparity: %s %dx%d \n", ty2.c_str(), dispf.cols, dispf.rows);

        //     depth_msg = cv_bridge::CvImage(image_header, "bgr8", depth).toImageMsg();
        //     depth_pub.publish(depth_msg);
        //     // cv::waitKey(1);
        // }

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    
    // cam.stopStereoCompute();
    cam.stopCapture(); ///< stop camera capturing

    return 0;
}
