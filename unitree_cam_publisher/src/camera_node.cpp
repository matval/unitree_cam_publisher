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
    
    // Load camera stuf
    UnitreeCamera cam(deviceNode); //"stereo_camera_config.yaml"); ///< init camera by device node number
    if(!cam.isOpened())   ///< get camera open state
        exit(EXIT_FAILURE);
    
    cam.setRawFrameSize(frameSize); ///< set camera frame size
    cam.setRawFrameRate(fps);       ///< set camera camera fps
    cam.setRectFrameSize(cv::Size(frameSize.width >> 2, frameSize.height >> 1)); ///< set camera rectify frame size
    
    cam.startCapture(); ///< disable image h264 encoding and share memory sharing
    cam.startStereoCompute(); ///< start disparity computing
    
    usleep(500000);

    // Start ROS stuff
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_publisher", options);
    image_transport::ImageTransport it(node);
    // image_transport::Publisher color_pub = it.advertise("camera/image", 1);
    image_transport::CameraPublisher cam_pub = it.advertiseCamera("camera/rect_image", 1);
    image_transport::Publisher depth_pub = it.advertise("camera/depth", 1);

    std_msgs::msg::Header image_header;
    sensor_msgs::msg::Image::SharedPtr color_msg, depth_msg;
    sensor_msgs::msg::CameraInfo::SharedPtr cam_info(new sensor_msgs::msg::CameraInfo());

    // Get calibration parameters
    std::vector<cv::Mat> paramsArray;
    if(cam.getCalibParams(paramsArray))
    {
        // intrinsic, distortion, xi, rotation, translation, kfe
        for(std::size_t i=0; i<paramsArray.size(); i++)
        {
            std::cout << "\ndata:\n" << paramsArray[i] << std::endl;
        }

        // Populate CameraInfo message
        cam_info->distortion_model = "rational_polynomial";
        for(int i=0; i<paramsArray[0].rows*paramsArray[0].cols; i++)
        {
            cam_info->k[i] = paramsArray[0].reshape(1).at<double>(i);
        }
        cam_info->d.resize(4, 0.0);
        for(int i=0; i<paramsArray[1].rows*paramsArray[1].cols; i++)
        {
            cam_info->d[i] = paramsArray[1].reshape(1).at<double>(i);
        }
        for(int i=0; i<paramsArray[3].rows*paramsArray[3].cols; i++)
        {
            cam_info->r[i] = paramsArray[3].reshape(1).at<double>(i);
        }
        for(int i=0; i<paramsArray[5].rows*paramsArray[5].cols; i++)
        {
            cam_info->p[i] = paramsArray[5].reshape(1).at<double>(i);
        }
    }
    
    rclcpp::WallRate loop_rate(30);
    while(rclcpp::ok() && cam.isOpened())
    {
        cv::Mat left, right, depth;
        std::chrono::microseconds t;

        if(!cam.getRectStereoFrame(left, right)){ ///< get rectify left,right frame  
            usleep(1000);
            continue;
        }

        // Try with disparity
        getDepthFrame(cv::Mat& dispf, cv::Mat& depth);
        if(!cam.getDepthFrame(depth, false, t)){  ///< get stereo camera depth image 
            usleep(1000);
            continue;
        }

        image_header.stamp = node->get_clock()->now();;
        image_header.frame_id = "camera_frame";

        if (!left.empty())
        {
            // Debug color image type
            // std::string ty =  type2str( left.type() );
            // printf("Color: %s %dx%d \n", ty.c_str(), left.cols, left.rows);

            color_msg = cv_bridge::CvImage(image_header, "bgr8", left).toImageMsg();

            cam_info->height = left.rows;
            cam_info->width = left.cols;
            cam_info->header = image_header;

            // Publish via image_transport
            cam_pub.publish(color_msg, cam_info);
            cv::waitKey(1);
        }

        if (!depth.empty())
        {
            // Debug depth image type
            // std::string ty =  type2str( depth.type() );
            // printf("Depth: %s %dx%d \n", ty.c_str(), depth.cols, depth.rows);

            depth_msg = cv_bridge::CvImage(image_header, "bgr8", depth).toImageMsg();
            depth_pub.publish(depth_msg);
            cv::waitKey(1);
        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    
    am.stopStereoCompute();
    cam.stopCapture(); ///< stop camera capturing

    return 0;
}
