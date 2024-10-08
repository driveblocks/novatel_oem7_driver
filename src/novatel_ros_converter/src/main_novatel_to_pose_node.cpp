// Copyright 2023 driveblocks GmbH
// driveblocks proprietary license
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "novatel_oem7_msgs/msg/bestpos.hpp"
#include "novatel_oem7_msgs/msg/bestvel.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;

/**
 * Simple node that converts Novatel BESTPOS and BESTVEL messages on the topic
 * "input/novatel_bestpos" and "input/novatel_bestvel" to geometry_msgs
 * StampedPose messages on the topic "output/pose".
 */
class NovatelToPoseNode : public rclcpp::Node {
public:
  NovatelToPoseNode() : Node("novatel_to_pose") {
    // initialize subscriber and publisher

    // create the subscriber for the BESTPOS message
    sub_bestpos_ = this->create_subscription<novatel_oem7_msgs::msg::BESTPOS>(
        "input/pos", 1,
        std::bind(&NovatelToPoseNode::BestposCallback_, this, _1));

    // create the subscriber for the BESTVEL message
    sub_bestvel_ = this->create_subscription<novatel_oem7_msgs::msg::BESTVEL>(
        "input/vel", 1,
        std::bind(&NovatelToPoseNode::BestvelCallback_, this, _1));

    // create the publisher for the PoseStamped message
    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "output/pose", 1);
  }

private:
  void BestposCallback_(const novatel_oem7_msgs::msg::BESTPOS &msg) {
    latest_bestpos_ = msg;

    if (!origin_conv[0] && !origin_conv[1]) {
      // store the latitude and longitude of the first received pose
      origin_conv[0] = msg.lat / (180.0 / M_PI);
      origin_conv[1] = msg.lon / (180.0 / M_PI);
      origin_conv[2] = 0.0;
    }

    // check if we have received both BESTPOS and BESTVEL messages
    if (received_bestvel_once_) {
      // create a PoseStamped message
      geometry_msgs::msg::PoseStamped pose_stamped_msg;

      // convert the latitude, longitude, and altitude to the position fields of
      // the PoseStamped message
      // precise computation of x, y, z from nav_msg
      // lateral, longitudinal, height
      std::vector<double> llh = {msg.lat, msg.lon, msg.hgt};
      // telescope control system (tcs) measures in x, y, z topocentric
      std::vector<double> tcs = Llh2Tcs_(llh);
      pose_stamped_msg.pose.position.x = tcs[0];
      pose_stamped_msg.pose.position.y = tcs[1];
      pose_stamped_msg.pose.position.z =
          0.0; // forced to 0.0, but could be retrieved from tcs[2];

      // convert the yaw orientation (trk_gnd is direction of motion over ground
      // with respect to True North, in degrees) to a Quaternion representation
      tf2::Quaternion orientation_quat;
      orientation_quat.setRPY(0.0, 0.0,
                              (-latest_bestvel_.trk_gnd + 90.0) * M_PI / 180.0);
      pose_stamped_msg.pose.orientation = tf2::toMsg(orientation_quat);

      // set the timestamp and frame ID of the PoseStamped message
      pose_stamped_msg.header.stamp = latest_bestpos_.header.stamp;
      pose_stamped_msg.header.frame_id = "map";

      // publish the PoseStamped message
      pub_pose_->publish(pose_stamped_msg);
    }
  }

  void BestvelCallback_(const novatel_oem7_msgs::msg::BESTVEL &msg) {
    // only read bestvel message, if moving (otherwise the estimate is
    // significantly off/jumps)
    if (msg.hor_speed > 1.0) {
      latest_bestvel_ = msg;
      received_bestvel_once_ = true;
    }
  }

  // declare ROS2 publisher and subscriber
  rclcpp::Subscription<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr sub_bestpos_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::BESTVEL>::SharedPtr sub_bestvel_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;

  // variables to store the latest BESTPOS and BESTVEL messages
  novatel_oem7_msgs::msg::BESTPOS latest_bestpos_;
  novatel_oem7_msgs::msg::BESTVEL latest_bestvel_;
  bool received_bestvel_once_ = false;

  // store reference (first init)
  std::vector<double> origin_conv = {0.0, 0.0, 0.0};

  // llh to tcs conversion helper functions, mainly re-used from the following
  // M-Code, converted to C++
  // https://de.mathworks.com/matlabcentral/fileexchange/28813-gps-coordinate-transformations
  // Copyright (c) 2010, Charles Rino
  // START OF LLH TO TCS HELPERS -----------------------------------------------
  std::vector<double> Llh2Tcs_(std::vector<double> llh) {
    // This function returns the x, y, and z topocentric (TCS) coordinates of
    // the point specified by llh [lat lon hgt], relative to the input origin
    // [lat lon alt] convert lat, lon and origin lat, lon from deg to rad

    // return value
    std::vector<double> tcs = {0, 0, 0};

    std::vector<double> llh_conv = {0, 0, 0};
    llh_conv[0] = llh[0] / (180.0 / M_PI);
    llh_conv[1] = llh[1] / (180.0 / M_PI);
    llh_conv[2] = llh[2];

    // transform from llh to ecf (earth-centered frame)
    std::vector<double> ecf = Llh2Ecf_(llh_conv);

    // transform from ecf (earth-centered frame) to uvw; the ecf coordinates are
    // rotated such that the u axis is colinear with the longitude of the origin
    std::vector<double> uvw = Ecf2Uvw(ecf);

    // transform uvw to tcs (topocentric)
    Uvw2Tcs_(uvw, tcs);

    return tcs;
  }

  std::vector<double> Llh2Ecf_(std::vector<double> llh) {
    // This function returns the x, y, and z earth centered fixed (ECF)
    // coordinates of the point specified by llh [lat lon hgt]. Note that
    // longitude is positive east of the Greenwich meridian.

    // Setup WGS-84 constants
    const double kEarthEquRadius =
        6378137.0; // earth equatorial radius in meters
    const double kFlatteningFactor =
        1.0 / 298.257223563; // flattening factor fo the earth ellipsoid

    const double lat = llh[0];
    const double lon = llh[1];
    const double hgt = llh[2];

    const double s_lat = sin(lat);
    const double N =
        kEarthEquRadius /
        sqrt(1 - kFlatteningFactor * (2 - kFlatteningFactor) * pow(s_lat, 2.0));
    const double Nplushgtclat = (N + hgt) * cos(lat);

    const double x = Nplushgtclat * cos(lon);
    const double y = Nplushgtclat * sin(lon);
    const double z = (pow(1 - kFlatteningFactor, 2.0) * N + hgt) * s_lat;
    std::vector<double> ecf = {x, y, z};

    return ecf;
  }

  std::vector<double> Ecf2Uvw(std::vector<double> ecf) {
    // This function will rotate ECF coordinates into UVW coordinates: X axis (U
    // axis) is colinear with longitude of origin
    const int kYawType = 1;

    std::vector<std::vector<double>> DC = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    GetDircosForward_(origin_conv[1], kYawType, DC);

    std::vector<double> uvw = {0, 0, 0};

    for (size_t i = 0; i < 3; i++) {
      for (size_t j = 0; j < 3; j++) {
        uvw[i] += DC[i][j] * ecf[j];
      }
    }

    return uvw;
  }

  void Uvw2Tcs_(std::vector<double> uvw, std::vector<double> &tcs) {
    // This function will convert a position vector from UVW to TCS coordinates
    // relative to origin

    // Transform Origin to ECF and UVW
    std::vector<double> origin_ECF = {0, 0, 0};
    std::vector<double> origin_UVW = {0, 0, 0};
    origin_ECF = Llh2Ecf_(origin_conv);
    origin_UVW = Ecf2Uvw(origin_ECF);

    // Define Rotation Types
    const int kPitchType = 2;
    const int kRollType = 3;
    std::vector<std::vector<double>> DC1 = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    std::vector<std::vector<double>> DC2 = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    std::vector<std::vector<double>> DC3 = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    GetDircosForward_(M_PI / 2, kRollType, DC1);
    GetDircosForward_(M_PI / 2, kPitchType, DC2);
    GetDircosForward_(-origin_conv[0], kRollType, DC3);

    std::vector<std::vector<double>> DC21 = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    std::vector<std::vector<double>> DC31 = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    for (size_t i = 0; i < 3; i++) {
      for (size_t j = 0; j < 3; j++)

      {
        double product = 0;
        for (size_t k = 0; k < 3; k++) {
          product += DC2[i][k] * DC1[k][j];
        }
        DC21[i][j] = product;
      }
    }

    for (size_t i = 0; i < 3; i++) {
      for (size_t j = 0; j < 3; j++) {
        double product = 0;
        for (size_t k = 0; k < 3; k++) {
          product += DC3[i][k] * DC21[k][j];
        }
        DC31[i][j] = product;
      }
    }
    std::vector<double> tcs_offset = {0, 0, 0};
    tcs_offset[0] = uvw[0] - origin_UVW[0];
    tcs_offset[1] = uvw[1] - origin_UVW[1];
    tcs_offset[2] = uvw[2] - origin_UVW[2];

    for (size_t i = 0; i < 3; i++) {
      for (size_t j = 0; j < 3; j++) {
        tcs[i] += DC31[i][j] * tcs_offset[j];
      }
    }

    return;
  }

  void GetDircosForward_(double ang, int matrix_flavor,
                         std::vector<std::vector<double>> &dc) {
    // Fills a direction cosine matrix defined by positive right-hand rule Euler
    // angles that transforms from an INS type basis to a body type basis.
    const int kYawType = 1;
    const int kPitchType = 2;
    const int kRollType = 3;

    const double cos_a = cos(ang);
    const double sin_a = sin(ang);

    switch (matrix_flavor) {
    case kYawType:
      dc[0][0] = cos_a;
      dc[0][1] = sin_a;
      dc[1][0] = -sin_a;
      dc[1][1] = cos_a;
      dc[2][2] = 1;
      break;

    case kPitchType:
      dc[0][0] = cos_a;
      dc[0][2] = -sin_a;
      dc[1][1] = 1;
      dc[2][0] = sin_a;
      dc[2][2] = cos_a;
      break;

    case kRollType:
      dc[0][0] = 1;
      dc[1][1] = cos_a;
      dc[1][2] = sin_a;
      dc[2][1] = -sin_a;
      dc[2][2] = cos_a;
      break;

    default:
      break;
    }
  }
  // END OF LLH TO TCS HELPERS -------------------------------------------------
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NovatelToPoseNode>());
  rclcpp::shutdown();
  return 0;
}
