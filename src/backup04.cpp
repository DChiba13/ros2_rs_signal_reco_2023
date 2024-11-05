#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <ros2_rs_interfaces/msg/traffic_signal.hpp>

using sensor_msgs::msg::Image;
using ros2_rs_interfaces::msg::TrafficSignal;

// 赤青黃のHSV表色系での閾値を設定
#define MIN_H_RED 170
#define MAX_H_RED 255
#define MIN_S_RED 60
#define MAX_S_RED 255
#define MIN_V_RED 100
#define MAX_V_RED 255

#define MIN_H_GREEN 60
#define MAX_H_GREEN 100
#define MIN_S_GREEN 60
#define MAX_S_GREEN 255
#define MIN_V_GREEN 30
#define MAX_V_GREEN 255

#define MIN_H_YELLOW 0
#define MAX_H_YELLOW 25
#define MIN_S_YELLOW 60
#define MAX_S_YELLOW 255
#define MIN_V_YELLOW 30
#define MAX_V_YELLOW 255

// 信号が青なのか赤なのか判断するフラグ
bool green_light_flag = false;
bool red_light_flag = false;

// 画像の上から何％の高さを表示するのか設定　
constexpr float IMAGE_ABOVE_RASIO = 0.4;

// IMAGE_THRESHフレーム連続で赤,青が認識されると信号とみなす
constexpr int IMAGE_THRESH = 20;

//  青,赤信号が何フレーム連続で検出されたか数えるcount
int green_cnt = 0;
int red_cnt = 0;

rclcpp::Node::SharedPtr g_signal_node = nullptr;
rclcpp::Publisher<TrafficSignal>::SharedPtr light_pub;
rclcpp::Publisher<Image>::SharedPtr image_pub;

// カメラ画像から赤色の画素を抽出する関数
void extractRedSignal(cv::Mat &rgb, cv::Mat &hsv, cv::Mat &extract_red)
{
    for(int y = 0; y < rgb.rows; y++){
        for(int x = 0; x < rgb.cols; x++){
            cv::Vec3b val = hsv.at<cv::Vec3b>(y,x);
            if(    MIN_H_RED <= val[0] && val[0] <= MAX_H_RED
                && MIN_S_RED <= val[1] && val[1] <= MAX_S_RED
                && MIN_V_RED <= val[2] && val[2] <= MAX_V_RED)
            {
                extract_red.at<cv::Vec3b>(y,x) = rgb.at<cv::Vec3b>(y,x);
            }
        }
    }
}
// カメラ画像から青色の画素を抽出する関数
void extractGreenSignal(cv::Mat &rgb, cv::Mat &hsv, cv::Mat &extract_green)
{
    for(int y = 0; y < rgb.rows; y++){
        for(int x = 0; x < rgb.cols; x++){
            cv::Vec3b val = hsv.at<cv::Vec3b>(y,x);
            if(    MIN_H_GREEN <= val[0] && val[0] <= MAX_H_GREEN
                && MIN_S_GREEN <= val[1] && val[1] <= MAX_S_GREEN
                && MIN_V_GREEN <= val[2] && val[2] <= MAX_V_GREEN){
                extract_green.at<cv::Vec3b>(y,x) = rgb.at<cv::Vec3b>(y,x);
            }
        }
    }
}
// 抽出した色を白くし、二値化する関数
void binalizeImage(cv::Mat &src, cv::Mat &gray_img)
{
    for(int y = 0; y<src.rows; y++)
    {
        for(int x = 0; x<src.cols; x++)
        {
            if(src.at<cv::Vec3b>(y,x)!=cv::Vec3b(0, 0, 0))
            {
                gray_img.at<uchar>(y,x) = 255;
            }
        }
    }
}
// ピンク色または水色の中に黄色が見えたら赤or青の矩形でラベリングする関数
// isRedLightがtrueなら赤信号用の処理、falseなら青信号用の処理になる
void extractYellowInBlob(cv::Mat &rgb, cv::Mat &bin_img, int num_labels, const std::vector<int> &widths, const std::vector<int> &heights, const std::vector<int> &lefts, const std::vector<int> &tops, bool isRedLight)
{
    cv::Mat hsv;
    cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);

    for (int label = 1; label < num_labels; ++label)
    {
        int width = widths[label-1];
        int height = heights[label-1];
        int left = lefts[label-1];
        int top = tops[label-1];

        cv::Mat blob_hsv(hsv, cv::Rect(left, top, width, height));

        cv::Mat extract_yellow = cv::Mat::zeros(blob_hsv.size(), blob_hsv.type());
        cv::medianBlur(blob_hsv, blob_hsv, 3);
        for (int y = 0; y < blob_hsv.rows; ++y)
        {
            for (int x = 0; x < blob_hsv.cols; ++x)
            {
                cv::Vec3b val = blob_hsv.at<cv::Vec3b>(y, x);
                if (   MIN_H_YELLOW <= val[0] && val[0] <= MAX_H_YELLOW
                    && MIN_S_YELLOW <= val[1] && val[1] <= MAX_S_YELLOW
                    && MIN_V_YELLOW <= val[2] && val[2] <= MAX_V_YELLOW)
                {
                    extract_yellow.at<cv::Vec3b>(y, x) = blob_hsv.at<cv::Vec3b>(y, x);
                }
            }
        }

        cv::Mat bin_img_yellow = cv::Mat::zeros(blob_hsv.size(), CV_8UC1);
        binalizeImage(extract_yellow, bin_img_yellow);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

        cv::Mat labeled_yellow;
        cv::Mat stats_yellow, centroids_yellow;
        int num_labels_yellow = cv::connectedComponentsWithStats(bin_img_yellow, labeled_yellow, stats_yellow, centroids_yellow);
        for (int label = 1; label < num_labels_yellow; ++label)
        {
            int yellow_width = stats_yellow.at<int>(label, cv::CC_STAT_WIDTH);
            int yellow_height = stats_yellow.at<int>(label, cv::CC_STAT_HEIGHT);
            int yellow_left = stats_yellow.at<int>(label, cv::CC_STAT_LEFT);
            int yellow_top = stats_yellow.at<int>(label, cv::CC_STAT_TOP);

            cv::rectangle(bin_img_yellow, cv::Rect(yellow_left, yellow_top, yellow_width, yellow_height), cv::Scalar(256/2), 2);
            if (isRedLight)
            {
                cv::rectangle(rgb, cv::Rect(left, top, width, height), cv::Scalar(0, 0, 255), 2); // 赤信号は赤い矩形
                red_light_flag = true;
            }
            else
            {
                cv::rectangle(rgb, cv::Rect(left, top, width, height), cv::Scalar(255, 0, 0), 2); // 青信号は青い矩形
                green_light_flag = true;
            }
        }
    }
}

void cvImageToROSImage(const cv::Mat &src, Image &dst)
{
    dst.height = src.rows;
    dst.width = src.cols;
    if(src.type() == CV_8UC1) dst.encoding = "mono8";
    else if(src.type() == CV_8UC3) dst.encoding = "bgr8";
    dst.step = (uint32_t)(src.step);
    size_t size = src.step * src.rows;
    dst.data.resize(size);
    memcpy(&dst.data[0], src.data, size);
    dst.header.frame_id="img";
    dst.header.stamp = g_signal_node->now();
}

// パブリッシャから画像msgを受け取ったら実行されるコールバック関数
void onImageSubscribed(const Image::SharedPtr msg)
{
  /*ROS2のImage型 -> OpenCVのMat型への変換*/
  auto cv_img = cv_bridge::toCvShare(msg);
  cv::Mat frame = cv_img->image;

  int height = frame.rows;
  int width = frame.cols;

  int top_region_height = height * IMAGE_ABOVE_RASIO;

  cv::Mat top_region = frame(cv::Rect(0,0,width,top_region_height));

  cv::Mat hsv;
  cv::cvtColor(top_region, hsv, cv::COLOR_BGR2HSV);

  // カメラから赤青を抽出
  cv::Mat extract_red(top_region.size(), top_region.type(), cv::Scalar(0, 0, 0));
  cv::Mat extract_green(top_region.size(), top_region.type(), cv::Scalar(0, 0, 0));
  extractRedSignal(top_region, hsv, extract_red);
  extractGreenSignal(top_region, hsv, extract_green);

  // メディアンフィルターにかける
  cv::Mat red_median(top_region.size(), top_region.type(), cv::Scalar(0, 0, 0));
  cv::Mat green_median(top_region.size(), top_region.type(), cv::Scalar(0, 0, 0));
  cv::medianBlur(extract_red, red_median, 3);
  cv::medianBlur(extract_green, green_median, 3);

  // 二値化
  cv::Mat bin_img_red = cv::Mat::zeros(top_region.size(), CV_8UC1);
  cv::Mat bin_img_green = cv::Mat::zeros(top_region.size(), CV_8UC1);
  binalizeImage(red_median, bin_img_red);
  binalizeImage(green_median, bin_img_green);

  // ラベリング
  cv::Mat labeled_red, labeled_green;
  cv::Mat stats_red, stats_green, centroids_red, centroids_green;
  int num_labels_red, num_labels_green;

  num_labels_red = cv::connectedComponentsWithStats(bin_img_red, labeled_red, stats_red, centroids_red);
  std::vector<int> red_width, red_height, red_left, red_top;
  for (int label = 1; label < num_labels_red; ++label)
  {
      int width = stats_red.at<int>(label, cv::CC_STAT_WIDTH);
      int height = stats_red.at<int>(label, cv::CC_STAT_HEIGHT);
      int left = stats_red.at<int>(label, cv::CC_STAT_LEFT);
      int top = stats_red.at<int>(label, cv::CC_STAT_TOP);

      red_width.push_back(width);
      red_height.push_back(height);
      red_left.push_back(left);
      red_top.push_back(top);

      // ピンク色の矩形を描く
      cv::rectangle(frame, cv::Rect(red_left[label-1], red_top[label-1], red_width[label-1], red_height[label-1]), cv::Scalar(255,0,255), 2);
  }

  num_labels_green = cv::connectedComponentsWithStats(bin_img_green, labeled_green, stats_green, centroids_green);
  std::vector<int> green_width, green_height, green_left, green_top;
  for (int label = 1; label < num_labels_green; ++label)
  {
      int width = stats_green.at<int>(label, cv::CC_STAT_WIDTH);
      int height = stats_green.at<int>(label, cv::CC_STAT_HEIGHT);
      int left = stats_green.at<int>(label, cv::CC_STAT_LEFT);
      int top = stats_green.at<int>(label, cv::CC_STAT_TOP);

      green_width.push_back(width);
      green_height.push_back(height);
      green_left.push_back(left);
      green_top.push_back(top);

      //　水色の矩形を描く
      cv::rectangle(frame, cv::Rect(green_left[label-1], green_top[label-1], green_width[label-1], green_height[label-1]), cv::Scalar(255,255,0), 2);
  }

  extractYellowInBlob(frame, bin_img_red, num_labels_red, red_width, red_height, red_left, red_top, true);
  extractYellowInBlob(frame, bin_img_green, num_labels_green, green_width, green_height, green_left, green_top, false);

  //赤,青信号が連続で検出されるほどcountが加算されていく
  // red_flag, green_flag はextractYellowBlob関数から出力されている
  if(red_light_flag)
  {
    ++red_cnt;
  }
  else
  {
    red_cnt = 0;
  }

  if(green_light_flag)
  {
    ++green_cnt;
  }
  else
  {
    green_cnt = 0;
  }

  // IMAGE_THRESHフレーム連続で赤,青信号が検出されたら,"RedLight" or "GreenLight"をパブリッシュ
  if(red_cnt>IMAGE_THRESH)
  {
    TrafficSignal light_msg;
    light_msg.state = "RedLight";
    RCLCPP_INFO(g_signal_node->get_logger(),"%s",light_msg.state.c_str());
    light_pub->publish(light_msg);
    red_cnt = 0;
  }
  if(green_cnt>IMAGE_THRESH)
  {
    TrafficSignal light_msg;
    light_msg.state = "GreenLight";
    RCLCPP_INFO(g_signal_node->get_logger(),"%s",light_msg.state.c_str());
    light_pub->publish(light_msg);
    green_cnt = 0;
  }

  red_light_flag = false;
  green_light_flag = false;

  // カメラ画像を表示
//   cv::imshow("Image", frame);
//   cv::waitKey(1);

  // Mat型からImage型に変換
  Image ros_img;
  cvImageToROSImage(frame,ros_img);
  //  結果画像をパブリッシュ
  image_pub->publish(ros_img);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc,argv);

  g_signal_node = rclcpp::Node::make_shared("traffic_signal_reco");
  auto sub = g_signal_node->create_subscription<Image>("/camera1/image",10,onImageSubscribed);

  light_pub = g_signal_node->create_publisher<TrafficSignal>("light_msg",10);
  image_pub = g_signal_node->create_publisher<Image>("signal_image",10);

  rclcpp::spin(g_signal_node);
//   cv::destroyAllWindows();
  g_signal_node = nullptr;
  rclcpp::shutdown();

  return 0;
}
