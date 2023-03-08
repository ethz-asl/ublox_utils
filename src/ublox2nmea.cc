/*
MIT License
Copyright (c) 2022 Rik Baehnemann, ASL, ETH Zurich, Switzerland
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <rclcpp/rclcpp.hpp>

#include <limits>

#include <nmea_msgs/msg/sentence.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>
#include <stdio.h>
#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>


class Transformer : public rclcpp::Node {
 public:
  Transformer();

 private:
  rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr nmea_pub_;
  rclcpp::Subscription<ublox_msgs::msg::NavPVT>::SharedPtr navpvt_sub_;

  void receiveNavPVT(const ublox_msgs::msg::NavPVT::SharedPtr navsat_msg);
};

Transformer::Transformer() : Node("ublox2nmea") {
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  navpvt_sub_ = this->create_subscription<ublox_msgs::msg::NavPVT>("ublox_gps_node/navpvt",1,std::bind(&Transformer::receiveNavPVT,this,std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribing to NavPVT topic %s", navpvt_sub_->get_topic_name());
  nmea_pub_ = this->create_publisher<nmea_msgs::msg::Sentence>("nmea", 1);
  RCLCPP_INFO(this->get_logger(), "Advertising NMEA topic %s", nmea_pub_->get_topic_name());
}

void Transformer::receiveNavPVT(const ublox_msgs::msg::NavPVT::SharedPtr navpvt_msg) {
  RCLCPP_INFO_ONCE(this->get_logger(), "Received first NavPVT message.");

  char buf[255];

  // Time conversion
  auto now = this->get_clock()->now();
  boost::posix_time::time_duration time;

  auto now_secs = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::nanoseconds(now.nanoseconds())).count();
  // Get current time of day in hours, minutes and seconds
  auto hour = now_secs / 3600 % 24;
  auto min = now_secs / 60 % 60;
  auto sec = now_secs % 60;

  // Lat conversion
  char lat_dir = navpvt_msg->lat < 0 ? 'S' : 'N';
  int8_t lat_degs = navpvt_msg->lat / 1e7;
  double lat_mins = (navpvt_msg->lat - lat_degs * 1e7) / 1e7 * 60.0;

  // Lon conversion
  char lon_dir = navpvt_msg->lon < 0 ? 'W' : 'E';
  int8_t lon_degs = navpvt_msg->lon / 1e7;
  double lon_mins = (navpvt_msg->lon - lon_degs * 1e7) / 1e7 * 60.0;

  // Status conversion
  int8_t status = (navpvt_msg->fix_type == ublox_msgs::msg::NavPVT::FIX_TYPE_3D) ? 1 : 0;
  
  // FIXME (calculate dec_secs correctly)
  long int dec_secs = 24;
  uint8_t
      len = sprintf(buf,
                    "$GPGGA,%02ld%02ld%02ld.%ld,%02d%08.5f,%c,%03d%08.5f,%c,%d,%d,%.1f,%d.%d,M,%d.%d,M,,",
                    (int64_t)hour,
                    (int64_t)min,
                    (int64_t)sec,
                    dec_secs,
                    lat_degs,
                    lat_mins,
                    lat_dir,
                    lon_degs,
                    lon_mins,
                    lon_dir,
                    status,
                    navpvt_msg->num_sv,
                    navpvt_msg->p_dop / 100.0,
                    navpvt_msg->h_msl / 1000,
                    navpvt_msg->h_msl % 1000,
                    (navpvt_msg->height - navpvt_msg->h_msl) / 1000,
                    std::abs((navpvt_msg->height - navpvt_msg->h_msl)) % 1000
  );

// RCLCPP_INFO_STREAM(get_logger(),"hours: "<< hour);
// RCLCPP_INFO_STREAM(get_logger(),"minutes: "<< min);
// RCLCPP_INFO_STREAM(get_logger(),"seconds: "<< sec);

  // Calculate checksum of sentence and add it to the end of the sentence
  uint8_t checksum = 0;
  for (int i = 1; i < len; i++) {
    checksum ^= buf[i];
  }
  sprintf(&buf[len], "*%02X\r\n", checksum);

  nmea_msgs::msg::Sentence nmea_msg;
  nmea_msg.header.stamp = now;
  nmea_msg.header.frame_id = "gps";
  nmea_msg.sentence = buf;
  //RCLCPP_DEBUG(this->get_logger(), nmea_msg);
  nmea_pub_->publish(nmea_msg);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Transformer>());
  rclcpp::shutdown();

  return 0;
}
