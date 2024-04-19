#include "loam_mapper/transform_provider.hpp"

#include "iostream"
#include "loam_mapper/date.h"
#include "loam_mapper/utils.hpp"

#include <Eigen/Geometry>

namespace loam_mapper
{
TransformProvider::TransformProvider(
  const boost::filesystem::path & pose_txt, double origin_x, double origin_y, double origin_z)
{
  io::LineReader lines(pose_txt.string());
  double line_number = 1;
  while (char * line = lines.next_line()) {
    if (line_number == 16) {  // 16 is the mission date line in the applanix export file
      mission_date += line;
      mission_date.erase(0, 20);  // erases 20 characters from 0.
      std::cout << "mission_date: " << mission_date << std::endl;
      break;
    }
    if (line_number == 29) {  // 29 is the first data line. UTC Time is wanted
      time_string += line;
      time_string.erase(11, 188);  // only the time left in the string
      break;
    }
    line_number++;
  }

  std::string temp_pose_file = "temp_pose_file.csv";
  {
    std::ifstream filein(pose_txt.string());  // File to read from
    std::ofstream fileout(temp_pose_file);    // Temporary file
    if (!filein || !fileout) {
      // throw exception
      throw(std::runtime_error("Error opening file"));
    }
    std::string line;
    bool header_is_found = false;
    size_t line_number_header = 0;
    size_t line_number_current = 0;

    while (std::getline(filein, line)) {
      line_number_current++;

      // fix the header line
      if (line.find("TIME,") != std::string::npos) {
        line_number_header = line_number_current;
        header_is_found = true;

        // separate line into tokens by comma
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;
        while (std::getline(ss, token, ',')) {
          // remove leading and trailing spaces
          token.erase(0, token.find_first_not_of(' '));
          token.erase(token.find_last_not_of(' ') + 1);
          // replace spaces with underscores
          std::replace(token.begin(), token.end(), ' ', '_');
          tokens.push_back(token);
        }
        // concatenate tokens with spaces
        std::string new_line;
        for (const auto & token : tokens) {
          new_line += token + ",";
        }
        // remove last comma
        new_line.pop_back();
        line = new_line;
      }
      // fix the data lines to be csv like
      if (line_number_current > line_number_header + 3 && header_is_found) {
        // replace multiple spaces with single space
        line.erase(
          std::unique(
            line.begin(), line.end(), [](char a, char b) { return isspace(a) && isspace(b); }),
          line.end());
        // replace spaces with commas
        std::replace(line.begin(), line.end(), '\t', ',');
        line.pop_back();
      }
      fileout << line << std::endl;
    }
  }

  io::CSVReader<26, io::trim_chars<','>> csv_global_pose(temp_pose_file);
  for (int i = 0; i < 24; ++i) csv_global_pose.next_line();

  // take only what you need
  csv_global_pose.read_header(
    //                line_char,
    io::ignore_no_column, "TIME", "DISTANCE", "EASTING", "NORTHING", "ORTHOMETRIC_HEIGHT",
    "LATITUDE", "LONGITUDE", "ELLIPSOID_HEIGHT", "ROLL", "PITCH", "HEADING", "EAST_VELOCITY",
    "NORTH_VELOCITY", "UP_VELOCITY", "X_ANGULAR_RATE", "Y_ANGULAR_RATE", "Z_ANGULAR_RATE",
    "X_ACCELERATION", "Y_ACCELERATION", "Z_ACCELERATION", "EAST_SD", "NORTH_SD", "HEIGHT_SD",
    "ROLL_SD", "PITCH_SD", "HEADING_SD");
  // Skip "units" line
  csv_global_pose.next_line();
  csv_global_pose.next_line();
  csv_global_pose.next_line();
  struct Transform
  {
    double utc_time;            // in seconds
    double distance;            // in meters
    double easting;             // in meters
    double northing;            // in meters
    double orthometric_height;  // in meters
    double latitude;            // in degrees
    double longitude;           // in degrees
    double ellipsoid_height;    // in meters
    double roll;                // in degrees
    double pitch;               // in degrees
    double heading;             // in degrees
    double east_vel;            // in Meter/Sec
    double north_vel;           // in Meter/Sec
    double up_vel;              // in Meter/Sec
    double x_angular_rate;      // in meters
    double y_angular_rate;      // in meters
    double z_angular_rate;      // in meters
    double x_acceleration;      // in meters
    double y_acceleration;      // in meters
    double z_acceleration;      // in meters

    double east_std;     // in meters
    double north_std;    // in meters
    double height_std;   // in meters
    double roll_std;     // in degrees
    double pitch_std;    // in degrees
    double heading_std;  // in degrees
  } in;

  try {
    data_line_number = csv_global_pose.get_file_line() + 1;
    while (csv_global_pose.read_row(
      in.utc_time, in.distance, in.easting, in.northing, in.orthometric_height, in.latitude,
      in.longitude, in.ellipsoid_height, in.roll, in.pitch, in.heading, in.east_vel, in.north_vel,
      in.up_vel, in.x_angular_rate, in.y_angular_rate, in.z_angular_rate, in.x_acceleration,
      in.y_acceleration, in.z_acceleration, in.east_std, in.north_std, in.height_std, in.roll_std,
      in.pitch_std, in.heading_std)) {
      Pose pose;
      pose.pose_with_covariance.pose.position.set__x(in.easting - origin_x);
      pose.pose_with_covariance.pose.position.set__y(in.northing - origin_y);
      pose.pose_with_covariance.pose.position.set__z(in.ellipsoid_height - origin_z);
      Eigen::AngleAxisd angle_axis_x(utils::Utils::deg_to_rad(in.roll), Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd angle_axis_y(utils::Utils::deg_to_rad(in.pitch), Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd angle_axis_z(
        utils::Utils::deg_to_rad(-in.heading), Eigen::Vector3d::UnitZ());

      Eigen::Quaterniond q = (angle_axis_z * angle_axis_y * angle_axis_x);
      pose.pose_with_covariance.pose.orientation.set__x(q.x());
      pose.pose_with_covariance.pose.orientation.set__y(q.y());
      pose.pose_with_covariance.pose.orientation.set__z(q.z());
      pose.pose_with_covariance.pose.orientation.set__w(q.w());

      auto segments = utils::Utils::string_to_vec_split_by(mission_date, '/');

      int days_raw = std::stoi(segments.at(0));
      int months_raw = std::stoi(segments.at(1));
      int years_raw = std::stoi(segments.at(2));

      date::year_month_day date_current_ = date::year{years_raw} / months_raw / days_raw;
      date::hh_mm_ss time_since_midnight =
        date::make_time(std::chrono::milliseconds(static_cast<uint64_t>(in.utc_time * 1000)));

      auto tp = date::sys_days(date_current_) + std::chrono::hours{time_since_midnight.hours()} +
                std::chrono::minutes{time_since_midnight.minutes()} +
                std::chrono::seconds{time_since_midnight.seconds()};

      pose.stamp_unix_seconds = std::chrono::seconds(tp.time_since_epoch()).count();
      pose.stamp_nanoseconds = std::chrono::nanoseconds(time_since_midnight.subseconds()).count();

      std::array<double, 6> variances{
        std::pow(in.north_std, 2), std::pow(in.east_std, 2),  std::pow(in.height_std, 2),
        std::pow(in.roll_std, 2),  std::pow(in.pitch_std, 2), std::pow(in.heading_std, 2),
      };

      //  0  1  2  3  4  5
      //  6  7  8  9  10 11
      //  12 13 14 15 16 17
      //  18 19 20 21 22 23
      //  24 25 26 27 28 29
      //  30 31 32 33 34 35
      //  fill diagonal with variances
      for (size_t i = 0; i < 6; ++i) {
        pose.pose_with_covariance.covariance.at(i * 7) = variances.at(i);
      }
      poses_.push_back(pose);
    }
  } catch (const std::exception & ex) {
    std::cerr << "Probably empty lines at the end of csv, no problems: " << ex.what() << std::endl;
  }
}

TransformProvider::Pose TransformProvider::get_pose_at(
  uint32_t stamp_unix_seconds, uint32_t stamp_nanoseconds)
{
  Pose pose_search;
  pose_search.stamp_unix_seconds = stamp_unix_seconds;
  pose_search.stamp_nanoseconds = stamp_nanoseconds;
  auto iter_result = std::lower_bound(
    poses_.begin(), poses_.end(), pose_search, [](const Pose & p1, const Pose & p2) {
      if (p1.stamp_unix_seconds == p2.stamp_unix_seconds) {
        return p1.stamp_nanoseconds < p2.stamp_nanoseconds;
      }
      return p1.stamp_unix_seconds < p2.stamp_unix_seconds;
    });
  size_t index = std::distance(poses_.begin(), iter_result);
  return poses_.at(index);
}

geometry_msgs::msg::PoseStamped TransformProvider::convert_to_ros_pose(TransformProvider::Pose pose) {
  geometry_msgs::msg::PoseStamped ros_pose;
  ros_pose.header.frame_id = "map";
  ros_pose.header.stamp.sec = pose.stamp_unix_seconds;
  ros_pose.header.stamp.nanosec = pose.stamp_nanoseconds;
  ros_pose.pose.position.set__x(pose.pose_with_covariance.pose.position.x);
  ros_pose.pose.position.set__y(pose.pose_with_covariance.pose.position.y);
  ros_pose.pose.position.set__z(pose.pose_with_covariance.pose.position.z);
  ros_pose.pose.orientation.set__x(pose.pose_with_covariance.pose.orientation.x);
  ros_pose.pose.orientation.set__y(pose.pose_with_covariance.pose.orientation.y);
  ros_pose.pose.orientation.set__z(pose.pose_with_covariance.pose.orientation.z);
  ros_pose.pose.orientation.set__w(pose.pose_with_covariance.pose.orientation.w);

  return ros_pose;
}

}  // namespace loam_mapper