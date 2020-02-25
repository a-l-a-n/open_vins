#include <ros/ros.h>
// #include <rosbag/bag.h>
// #include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <cv_bridge/cv_bridge.h>

#include "core/VioManager.h"
#include "core/RosVisualizer.h"
#include "utils/dataset_reader.h"

#include "data_player.h"

using namespace ov_msckf;

VioManager* sys;
RosVisualizer* viz;

// Main function
int main(int argc, char** argv)
{

    // Launch our ros node
    ros::init(argc, argv, "run_serial_msckf");
    ros::NodeHandle nh("~");

    // Create our VIO system
    sys = new VioManager(nh);
    viz = new RosVisualizer(nh, sys);

    // Location of the ROS bag we want to read in
    std::string path_to_bag;
    //nhPrivate.param<std::string>("path_bag", path_to_bag, "/home/keck/catkin_ws/V1_01_easy.bag");
    nh.param<std::string>("path_bag", path_to_bag, "/home/patrick/datasets/eth/V1_01_easy.bag");
    ROS_INFO("ros bag path is: %s", path_to_bag.c_str());
    VioDataPlayer player;
    player.open(path_to_bag.c_str());
    if(!player.isOpen()) {return 0;}

    std::map<double, Eigen::Matrix<double, 17, 1>> gt_states;

    // Get our start location and how much of the bag we want to play
    // Make the bag duration < 0 to just process to the end of the bag
    double bag_start;
    nh.param<double>("bag_start", bag_start, 0);
    ROS_INFO("bag start: %.1f",bag_start);

    // Read in what mode we should be processing in (1=mono, 2=stereo)
    int max_cameras;
    nh.param<int>("max_cameras", max_cameras, 1);

    if(!player.readImu())
    {
        printf("[ERROR]:imu data empty.\n");
        return 0;
    }
    if(!player.readImage())
    {
        printf("[ERROR]:image data empty.\n");
        return 0;
    }
    if(bag_start > 0)
    {
        while(player.img_ts < bag_start)
            player.readImage();
        while(player.imu_ts < bag_start)
            player.readImu();
    }

    // Buffer variables for our system (so we always have imu to use)
    bool has_left = false;
    bool has_right = false;
    cv::Mat img0, img1;
    cv::Mat img0_buffer, img1_buffer;
    double time = player.img_ts;
    double time_buffer = time;

    while(ros::ok())
    {
        if(player.imuFirst()) // process imu
        {
            Eigen::Matrix<double, 3, 1> wm, am;
            wm << player.imu_gyro[0], player.imu_gyro[1], player.imu_gyro[2];
            am << player.imu_acc[0], player.imu_acc[1], player.imu_acc[2];
            // send it to our VIO system
            sys->feed_measurement_imu(player.imu_ts, wm, am);
            if(!player.readImu()){break;}
        }
        else // process image
        {
            time = player.img_ts;
            has_left = true;
            has_right = true;
            img0 = player.imgl.clone();
            img1 = player.imgr.clone();
            if(!player.readImage()){break;}
        }

        // Fill our buffer if we have not
        if(has_left && img0_buffer.rows == 0) {
            has_left = false;
            time_buffer = time;
            img0_buffer = img0.clone();
        }

        // Fill our buffer if we have not
        if(has_right && img1_buffer.rows == 0) {
            has_right = false;
            img1_buffer = img1.clone();
        }


        // If we are in monocular mode, then we should process the left if we have it
        if(max_cameras==1 && has_left) {
            if(gt_states.empty() || sys->intialized()) {
                sys->feed_measurement_monocular(time_buffer, img0_buffer, 0);
            }
            // visualize
            viz->visualize();
            // reset bools
            has_left = false;
            // move buffer forward
            time_buffer = time;
            img0_buffer = img0.clone();
        }


        // If we are in stereo mode and have both left and right, then process
        if(max_cameras==2 && has_left && has_right) {
            if(gt_states.empty() || sys->intialized()) {
                sys->feed_measurement_stereo(time_buffer, img0_buffer, img1_buffer, 0, 1);
            }
            // visualize
            viz->visualize();
            // reset bools
            has_left = false;
            has_right = false;
            // move buffer forward
            time_buffer = time;
            img0_buffer = img0.clone();
            img1_buffer = img1.clone();
        }
    }

    // Final visualization
    viz->visualize_final();

    // Finally delete our system
    delete sys;
    delete viz;


    // Done!
    return EXIT_SUCCESS;

}


















