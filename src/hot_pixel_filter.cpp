#include "dvs_hot_pixel_filter/utils.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>

constexpr double NUM_STD_DEVS = 5; // could make input parameter

int main(int argc, char* argv[])
{ 
  // parse input arguments and setup input and output rosbags
  std::string path_to_input_rosbag;
  int num_hot_pixels;

  if(!dvs_hot_pixel_filter::utils::parse_arguments(
      argc, argv, &path_to_input_rosbag, &num_hot_pixels) )
  {
    return -1;
  }

  std::string bag_name = dvs_hot_pixel_filter::utils::extract_bag_name(
      path_to_input_rosbag);

  std::string path_to_output_rosbag = path_to_input_rosbag + ".filtered";

  rosbag::Bag input_bag;
  try
  {
    input_bag.open(path_to_input_rosbag, rosbag::bagmode::Read);
  }
  catch(rosbag::BagIOException e)
  {
    std::cerr << "Error: could not open rosbag: " << path_to_input_rosbag << std::endl;
    return -1;
  }

  rosbag::Bag output_bag;
  output_bag.open(path_to_output_rosbag, rosbag::bagmode::Write);

  rosbag::View view(input_bag);

  // initialise variables and start computation
  dvs_hot_pixel_filter::utils::topic_mats histograms_by_topic;
  dvs_hot_pixel_filter::utils::topic_points hot_pixels_by_topic;

  dvs_hot_pixel_filter::utils::build_histograms(
      view, histograms_by_topic);

  dvs_hot_pixel_filter::utils::detect_hot_pixels(
      histograms_by_topic, NUM_STD_DEVS, num_hot_pixels, hot_pixels_by_topic);

  dvs_hot_pixel_filter::utils::write_all_msgs(
      view, hot_pixels_by_topic, output_bag);

  std::cout << "Computing stats..." << std::endl;

  output_bag.close();
  input_bag.close();

  // write statistics
  const bool one_topic = histograms_by_topic.size() == 1;
  std::cout << "Topic\t\t# Events\t#Hot pixels\t% Events discarded" << std::endl;
  for(auto topic : histograms_by_topic)
  {
    const std::string topic_name = topic.first;
    cv::Mat& histogram = topic.second;
    std::vector<cv::Point>& hot_pixels = hot_pixels_by_topic[topic_name];
    dvs_hot_pixel_filter::utils::save_stats(
        bag_name, topic_name, histogram, hot_pixels, one_topic);
  }

  return 0;
}
