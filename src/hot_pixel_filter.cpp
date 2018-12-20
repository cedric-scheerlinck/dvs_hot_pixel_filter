#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <unordered_map>
#include <sstream>
// opencv
// #include <cv_bridge/cv_bridge.h>
// #include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
// #include <opencv2/imgproc/imgproc.hpp>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

constexpr double NUM_STD_DEVS = 4;

typedef std::unordered_map<std::string, std::vector<dvs_msgs::Event>> topic_events;
typedef std::unordered_map<std::string, cv::Mat> topic_mats;
typedef std::unordered_map<std::string, std::vector<cv::Point>> topic_points;

bool parse_arguments(int argc, char* argv[],
                     std::string* path_to_input_rosbag,
                     int* num_hot_pixels)
{
  if(argc < 2)
  {
    std::cerr << "Not enough arguments" << std::endl;
    std::cerr << "Usage: rosrun dvs_hot_pixel_filter dvs_hot_pixel_filter path_to_bag.bag num_hot_pixels (optional)";
    return false;
  }

  *path_to_input_rosbag = std::string(argv[1]);
  std::cout << "Input bag: " << *path_to_input_rosbag << std::endl;

  if(argc == 3)
  {
    try
    {
      *num_hot_pixels = std::stoi(argv[2]);
      std::cout << "Number of hot pixels: " << *num_hot_pixels << std::endl;
    }
    catch(std::invalid_argument e)
    {
      std::cerr << "Invalid number of hot pixels. Will use default (50 pixels)." << std::endl;
      *num_hot_pixels = 50;
    }
  } else
  {
    *num_hot_pixels = -1;
    std::cout << "Number of hot pixels will be determined automatically" << std::endl;
  }

  return true;
}

template <typename T>
bool contains(T element, std::vector<T> my_vector)
{
  // returns true if my_vector contains element, otherwise false.
  for (auto topic_name : my_vector)
  {
    if (element == topic_name)
    {
      return true;
    }
  }
  return false;
}

void write_histogram_image(const std::string filename_in, const cv::Mat& histogram,
                           const std::vector<cv::Point>& hot_pixels = std::vector<cv::Point>())
{
  std::string filename = filename_in;
  std::replace( filename.begin(), filename.end(), '/', '_'); // replace all '/' to '_'
  std::replace( filename.begin(), filename.end(), '\\', '_');
  if (filename[-4] != '.')
  {
    filename += ".png";
  }
  cv::Mat display_image;
  cv::normalize(histogram, display_image, 0, 255, cv::NORM_MINMAX, CV_8UC1);
  cv::applyColorMap(display_image, display_image, cv::COLORMAP_HOT);
  if (!hot_pixels.empty())
  {
    cv::Vec3b green = cv::Vec3b(0, 255, 0);
    for (auto point : hot_pixels)
    {
      display_image.at<cv::Vec3b>(point) = green;
    }
  }
  cv::imwrite(filename, display_image);
}

std::string extract_bag_name(const std::string fullname)
{
  int pos = 0;
  int len = fullname.length();
  // go from the back to the first forward- or back-slash.
  for (int i = len; i > 0; i--)
  {
    if (fullname[i] == '/' || fullname[i] == '\\')
    {
      pos = i + 1;
      break;
    }
  }
  int count = 4;
  // now go from there to the first '.'
  for (int i = 0; i < len; i++)
  {
    if (fullname[pos + i] == '.')
    {
      count = i;
      break;
    }
  }
  std::string bag_name = fullname.substr(pos, count);
  return bag_name;
}

void build_histograms(rosbag::View& view, std::unordered_map<std::string, cv::Mat>& histograms)
{
  std::cout << "Building event count histogram(s)..." << std::endl;

  std::vector<std::string> seen_topics;
  foreach(rosbag::MessageInstance const m, view)
  {
    if(m.getDataType() == "dvs_msgs/EventArray")
    {
      const std::string topic_name = m.getTopic();
      // pointer to the message
      dvs_msgs::EventArrayConstPtr s = m.instantiate<dvs_msgs::EventArray>();
      const cv::Size msg_size = cv::Size(s->width, s->height);

      cv::Mat& histogram = histograms[topic_name];

      // initialise event_count_histogram if we haven't seen the topic yet
      if ( !contains(topic_name, seen_topics) )
      {
        histogram = cv::Mat::zeros(msg_size, CV_64FC1);
        seen_topics.push_back(topic_name);
        std::cout << "added " << topic_name << " to seen_topics" << std::endl;
      }

      if (msg_size != histogram.size())
      {
        std::cerr << "Error: a new event message in " << topic_name <<
        " does not match the existing topic histogram size.\n message: " <<
        msg_size << "\t histogram: " << histogram.size() << std::endl;
        return;
      }

      for(auto e : s->events)
      {
        // accumulate events without discrimination
        histogram.at<double>(e.y, e.x)++;
      }
    }
  }

  std::cout << "...done!" << std::endl;
}

void hot_pixels_by_threshold(const cv::Mat& histogram, const double& threshold,
                             std::vector<cv::Point>& hot_pixels)
{
  for (int y = 0; y < histogram.rows; y++)
  {
    for (int x = 0; x < histogram.cols; x++)
    {
      if (histogram.at<double>(y, x) > threshold)
      {
        hot_pixels.push_back(cv::Point(x, y));
      }
    }
  }
}
void hot_pixels_by_ranking(const cv::Mat& histogram, const double& num_hot_pixels,
                           std::vector<cv::Point>& hot_pixels)
{
  cv::Mat local_hist;
  histogram.copyTo(local_hist);

  for (int i = 0; i < num_hot_pixels; i++)
  {
    double max;
    cv::Point maxLoc;
    cv::minMaxLoc(local_hist, nullptr, &max, nullptr, &maxLoc);

    hot_pixels.push_back(maxLoc);
    local_hist.at<double>(maxLoc) = 0;
  }
}

void find_threshold(const cv::Mat& histogram, const double num_std_devs, double& threshold)
{
  double mean;
  double stdDev;
  cv::Scalar mean_Scalar, stdDev_Scalar;
  cv::meanStdDev(histogram, mean_Scalar, stdDev_Scalar);
  mean = mean_Scalar[0];
  stdDev = stdDev_Scalar[0];
  threshold = mean + num_std_devs*stdDev;
}

void write_event_msg(std::string topic_name, const dvs_msgs::EventArrayConstPtr event_array_ptr,
                     std::vector<cv::Point>& hot_pixels, rosbag::Bag& output_bag)
{
  std::vector<dvs_msgs::Event> events;
  for(auto e : event_array_ptr->events)
  {
    if (!contains(cv::Point(e.x, e.y), hot_pixels))
    {
      events.push_back(e);
    }
  }
  // Write new event array message to output rosbag
  dvs_msgs::EventArray event_array_msg;
  event_array_msg.events = events;
  event_array_msg.width = event_array_ptr->width;
  event_array_msg.height = event_array_ptr->height;
  event_array_msg.header.stamp = events.back().ts;

  output_bag.write(topic_name, event_array_msg.header.stamp, event_array_msg);
}

void write_msg(const rosbag::MessageInstance m, topic_points& hot_pixels_topic,
               rosbag::Bag& output_bag)
{
  if(m.getDataType() == "dvs_msgs/EventArray")
  {
    std::string topic_name = m.getTopic();
    std::vector<cv::Point>& hot_pixels = hot_pixels_topic[topic_name];
    dvs_msgs::EventArrayConstPtr event_array_ptr = m.instantiate<dvs_msgs::EventArray>();
    write_event_msg(topic_name, event_array_ptr, hot_pixels, output_bag);
  }
  else if(m.getDataType() == "sensor_msgs/Image")
  {
    sensor_msgs::ImageConstPtr img_msg = m.instantiate<sensor_msgs::Image>();
    output_bag.write(m.getTopic(), img_msg->header.stamp, m);
  }
  else if(m.getDataType() == "sensor_msgs/Imu")
  {
    sensor_msgs::ImuConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
    output_bag.write(m.getTopic(), imu_msg->header.stamp, m);
  }
  else
  {
    output_bag.write(m.getTopic(), m.getTime(), m);
  }
}

void print_stats(const std::string bag_name, const std::string topic_name,
                 const cv::Mat& histogram, const std::vector<cv::Point>& hot_pixels)
{
  cv::Mat histogram_after;
  histogram.copyTo(histogram_after);
  for (auto point : hot_pixels)
  {
    histogram_after.at<double>(point) = 0;
  }

  const double num_events = cv::sum(histogram)[0];
  const double num_events_after = cv::sum(histogram_after)[0];
  const double percent_events_discarded = (1 - num_events_after/num_events)*100;

  std::cout << std::setprecision(4) << topic_name << "\t" << num_events <<
      "\t" << hot_pixels.size() << "\t\t0\t(before)" << std::endl;

  std::cout << std::setprecision(4) << topic_name << "\t" << num_events_after <<
      "\t0\t\t" << percent_events_discarded << "\t(after)" << std::endl;

  write_histogram_image("hist_" + bag_name + "_" + topic_name + "_before", histogram);
  write_histogram_image("hist_" + bag_name + "_" + topic_name + "_after", histogram_after, hot_pixels);
}
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{ 
  std::string path_to_input_rosbag;
  int num_hot_pixels;

  if( !parse_arguments(argc, argv, &path_to_input_rosbag, &num_hot_pixels) )
  {
    return -1;
  }

  std::string bag_name = extract_bag_name(path_to_input_rosbag);

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

  topic_mats event_count_histogram_for_each_event_topic;
  topic_points hot_pixels_for_each_event_topic;

  // first place all event messages into their own topic
  // iterate over all messages

  build_histograms(view, event_count_histogram_for_each_event_topic);

  // iterate over all event topics
  for(auto topic : event_count_histogram_for_each_event_topic)
  {
    const std::string topic_name = topic.first;
    cv::Mat& histogram = topic.second;
    std::vector<cv::Point>& hot_pixels = hot_pixels_for_each_event_topic[topic_name];
    if (num_hot_pixels == -1)
    {
      // auto-detect hot pixels
      const double num_std_devs = NUM_STD_DEVS;
      double threshold;
      find_threshold(histogram, num_std_devs, threshold);
      hot_pixels_by_threshold(histogram, threshold, hot_pixels);
    }
    else
    {
      hot_pixels_by_ranking(histogram, num_hot_pixels, hot_pixels);
    }
  }

  ////////////////////////////////

  constexpr int log_every_n_messages = 10000;
  const uint32_t num_messages = view.size();
  uint32_t message_index = 0;
  // write the new rosbag without hot pixels by iterating over all messages
  foreach(rosbag::MessageInstance const m, view)
  {
    write_msg(m, hot_pixels_for_each_event_topic, output_bag);
    if(message_index++ % log_every_n_messages == 0)
    {
      std::cout << "Message: " << message_index << " / " << num_messages << std::endl;
    }
  }

  output_bag.close();
  input_bag.close();

  // write statistics
  std::cout << "Topic\t\t# Events\t#Hot pixels\t% Red." << std::endl;
  for(auto topic : event_count_histogram_for_each_event_topic)
  {
    const std::string topic_name = topic.first;
    cv::Mat& histogram = topic.second;
    std::vector<cv::Point>& hot_pixels = hot_pixels_for_each_event_topic[topic_name];
    print_stats(bag_name, topic_name, histogram, hot_pixels);
  }

  return 0;
}
