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

#include <limits>

#include <nmea_msgs/Sentence.h>
#include <ros/ros.h>
#include <ublox_msgs/NavPVT.h>
#include <boost/date_time/posix_time/posix_time_types.hpp>

class Transformer {
 public:
  Transformer();

 private:
  ros::Publisher nmea_pub_;
  ros::Subscriber navpvt_sub_;

  void receiveNavPVT(const ublox_msgs::NavPVT::ConstPtr &navsat_msg);
};

Transformer::Transformer() {
  ros::NodeHandle nh;
  navpvt_sub_ = nh.subscribe("navpvt", 1, &Transformer::receiveNavPVT, this);
  ROS_INFO("Subscribing to NavPVT topic %s", navpvt_sub_.getTopic().c_str());
  nmea_pub_ = nh.advertise<nmea_msgs::Sentence>("nmea", 1);
  ROS_INFO("Advertising NMEA topic %s", nmea_pub_.getTopic().c_str());
}

void Transformer::receiveNavPVT(const ublox_msgs::NavPVT::ConstPtr &navpvt_msg) {
  ROS_INFO_ONCE("Received first NavPVT message.");

  char buf[255];

  // Time conversion
  auto now = ros::Time::now();
  auto time = now.toBoost().time_of_day();
  long int deci_seconds = now.nsec / 1e7;

  // Lat conversion
  char lat_dir = navpvt_msg->lat < 0 ? 'S' : 'N';
  int8_t lat_degs = navpvt_msg->lat / 1e7;
  double lat_mins = (navpvt_msg->lat - lat_degs * 1e7) / 1e7 * 60.0;

  // Lon conversion
  char lon_dir = navpvt_msg->lon < 0 ? 'W' : 'E';
  int8_t lon_degs = navpvt_msg->lon / 1e7;
  double lon_mins = (navpvt_msg->lon - lon_degs * 1e7) / 1e7 * 60.0;

  // Status conversion
  int8_t status = (navpvt_msg->fixType == ublox_msgs::NavPVT::FIX_TYPE_3D) ? 1 : 0;

  uint8_t
      len = sprintf(buf,
                    "$GPGGA,%02ld%02ld%02ld.%ld,%02d%08.5f,%c,%03d%08.5f,%c,%d,%d,%.1f,%d.%d,M,%d.%d,M,,",
                    time.hours(),
                    time.minutes(),
                    time.seconds(),
                    deci_seconds,
                    lat_degs,
                    lat_mins,
                    lat_dir,
                    lon_degs,
                    lon_mins,
                    lon_dir,
                    status,
                    navpvt_msg->numSV,
                    navpvt_msg->pDOP / 100.0,
                    navpvt_msg->hMSL / 1000,
                    navpvt_msg->hMSL % 1000,
                    (navpvt_msg->height - navpvt_msg->hMSL) / 1000,
                    std::abs((navpvt_msg->height - navpvt_msg->hMSL)) % 1000
  );
  // Calculate checksum of sentence and add it to the end of the sentence
  uint8_t checksum = 0;
  for (int i = 1; i < len; i++) {
    checksum ^= buf[i];
  }
  sprintf(&buf[len], "*%02X\r\n", checksum);

  nmea_msgs::Sentence nmea_msg;
  nmea_msg.header.stamp = now;
  nmea_msg.header.frame_id = "gps";
  nmea_msg.sentence = buf;
  nmea_pub_.publish(nmea_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ublox2nmea");

  Transformer transformer;

  ros::spin();
  return 0;
}
