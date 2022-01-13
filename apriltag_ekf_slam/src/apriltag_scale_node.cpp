#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include <string>
#include <stdint.h>

namespace state_estimation{
  void Scale(cv::Mat& input_img, cv::Mat& output_img, int16_t scale){
    int16_t size_x_px, size_y_px;
    size_x_px = (int16_t)input_img.rows;
    size_y_px = (int16_t)input_img.cols;
    output_img = cv::Mat::zeros( scale*size_y_px, scale*size_x_px, CV_8UC1);
    for(int y=0; y<size_y_px; y++){
      for(int s_y=0; s_y<scale; s_y++){
        for(int x=0; x<size_x_px; x++){
          for(int s_x=0; s_x<scale; s_x++){
            output_img.at<uchar>((y*scale) + s_y, (x*scale) + s_x) = input_img.at<uchar>(y,x);
          }
        }
      }
    }
  }
  void Error(){
    ROS_INFO("Usage: 1. Goto apriltag data directory");
    ROS_INFO("Usage: 2. apriltag_scale <input_image> <scale>");
    exit(1);
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "apriltag_scale");
  ros::NodeHandle nh;
  if(argc != 3){
    state_estimation::Error();
  }
  std::string path_input_img = argv[1];
  int16_t scale = std::stoi(argv[2]);
  cv::Mat input_img = cv::imread(path_input_img, cv::IMREAD_GRAYSCALE);
  if(input_img.empty()){
    state_estimation::Error();
  }
  cv::Mat output_img;
  state_estimation::Scale(input_img, output_img, scale);
  cv::imwrite("../img/"+ path_input_img, output_img);
  return 0;
}
