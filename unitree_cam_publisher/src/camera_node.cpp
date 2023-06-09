#include <cstdio>
#include <chrono>
#include <string>
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

class ImagePublisher : public rclcpp::Node
{
    public:
        ImagePublisher(int argc, char **argv) : Node("minimal_publisher")
        {
            
        }
};

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
    
    UnitreeCamera cam("stereo_camera_config.yaml"); ///< init camera by device node number
    if(!cam.isOpened())   ///< get camera open state
        exit(EXIT_FAILURE);
    
    cam.setRawFrameSize(frameSize); ///< set camera frame size
    cam.setRawFrameRate(fps);       ///< set camera camera fps
    cam.setRectFrameSize(cv::Size(frameSize.width >> 2, frameSize.height >> 1)); ///< set camera rectify frame size
    cam.startCapture(); ///< disable image h264 encoding and share memory sharing
    
    usleep(500000);

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_publisher", options);
    image_transport::ImageTransport it(node);
    image_transport::Publisher color_pub = it.advertise("camera/image", 1);
    image_transport::Publisher depth_pub = it.advertise("camera/depth", 1);

    std_msgs::msg::Header hdr;
    sensor_msgs::msg::Image::SharedPtr color_msg, depth_msg;
    
    rclcpp::WallRate loop_rate(5);
    while(cam.isOpened() && rclcpp::ok())
    {
        cv::Mat left, right, depth;
        std::chrono::microseconds t;

        if(!cam.getRectStereoFrame(left, right)){ ///< get rectify left,right frame  
            usleep(1000);
            continue;
        }

        if(!cam.getDepthFrame(depth, true, t)){  ///< get stereo camera depth image 
            usleep(1000);
            continue;
        }

        if (!left.empty())
        {
            color_msg = cv_bridge::CvImage(hdr, "bgr8", left).toImageMsg();
            color_pub.publish(color_msg);
            cv::waitKey(1);
        }

        if (!depth.empty())
        {
            // Depbug depth image type
            std::string ty =  type2str( depth.type() );
            printf("Depth: %s %dx%d \n", ty.c_str(), depth.cols, depth.rows );

            depth_msg = cv_bridge::CvImage(hdr, "mono16", depth).toImageMsg();
            depth_pub.publish(depth_msg);
            cv::waitKey(1);
        }
        
        // cv::Mat stereo;
        // cv::flip(left,left, -1);
        // cv::flip(right,right, -1);
        // cv::hconcat(left, right, stereo); 
        // cv::flip(stereo,stereo, -1);
        // cv::imshow("Longlat_Rect", stereo);
        // char key = cv::waitKey(10);
        // if(key == 27) // press ESC key
        //    break;

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    
    cam.stopCapture(); ///< stop camera capturing

    return 0;
}
