#include <vector>
#include <map>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver/CustomPoint.h>

#include <filesystem>
#include "LasLoader.h"
#include "common/ImuLoader.h"

/*
  NANOS VERSION:
  -------------
  1) Writes IMU data as /livox/imu (sensor_msgs::Imu).
  2) Writes both a Livox CustomMsg (/livox/lidar) and a PointCloud2 (/livox/points),
     where the PointCloud2 has 7 fields:
       x (float32),
       y (float32),
       z (float32),
       intensity (float32),
       tag (uint8),
       line (uint8),
       timestamp (float64 in nanoseconds).

  Use this if your downstream SLAM is set to "livoxXYZRTLT_ns" and expects
  nanoseconds in the timestamp field.

  The CustomMsg offset_time is still microseconds (32-bit) to avoid overflow.

  Usage:
    ./laz_converter <directory> <output_bag> [--lines <number_of_lines>]
*/

static const double TIME_START  = 0.0;   // Shift all timestamps by this many seconds
static const int    NUM_POINT   = 19968; // Flush the CustomMsg after this many points
static const int    LINE_COUNT  = 4;     // How often we increment line_id

int main(int argc, char** argv) {
  ros::init(argc, argv, "laz_converter_ns");

  if (argc < 3) {
    std::cout << "Usage: " << argv[0] << " <directory> <output_bag>\n"
              << "  --lines <number_of_lines> (default 8)\n";
    return 1;
  }

  // Parse command-line args
  std::string directory   = argv[1];
  std::string output_bag  = argv[2];
  int number_of_lines     = 8; // default

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg.size() > 2 && arg.rfind("--", 0) == 0) {
      if (arg == "--lines") {
        number_of_lines = std::stoi(argv[++i]);
      } else {
        std::cerr << "Unknown option: " << arg << std::endl;
        return 1;
      }
    }
  }

  std::cout << "Input directory: " << directory << "\n"
            << "Output bag:      " << output_bag << "\n"
            << "Number of lines: " << number_of_lines << "\n";

  // Collect IMU .csv and LAZ .laz files
  std::vector<std::string> files_imu;
  std::vector<std::string> files_laz;
  for (auto &entry : std::filesystem::directory_iterator(directory)) {
    if (entry.path().extension() == ".csv") {
      files_imu.push_back(entry.path());
    } else if (entry.path().extension() == ".laz") {
      files_laz.push_back(entry.path());
    }
  }
  std::sort(files_imu.begin(), files_imu.end());
  std::sort(files_laz.begin(), files_laz.end());

  rosbag::Bag bag;
  bag.open(output_bag, rosbag::bagmode::Write);

  // ---------------------------------------------------------------
  // (1) Write IMU data => /livox/imu
  // ---------------------------------------------------------------
  for (auto &imu_fn : files_imu) {
    auto data = mandeye::load_imu(imu_fn, 0);
    for (auto &[ts, ang, acc] : data) {
      if (ts == 0.0) { 
        continue; 
      }
      sensor_msgs::Imu imu;
      imu.header.frame_id = "livox";
      imu.header.stamp.fromSec(TIME_START + ts);

      imu.angular_velocity.x    = ang[0];
      imu.angular_velocity.y    = ang[1];
      imu.angular_velocity.z    = ang[2];
      imu.linear_acceleration.x = acc[0];
      imu.linear_acceleration.y = acc[1];
      imu.linear_acceleration.z = acc[2];

      bag.write("/livox/imu", imu.header.stamp, imu);
    }
  }

  // ---------------------------------------------------------------
  // (2) Create both CustomMsg (/livox/lidar) & PointCloud2 (/livox/points)
  // ---------------------------------------------------------------

  // A) Livox CustomMsg
  livox_ros_driver::CustomMsg custom_msg;
  custom_msg.header.frame_id = "livox_frame";
  custom_msg.lidar_id        = 192;
  custom_msg.rsvd[0]         = 0;
  custom_msg.rsvd[1]         = 0;
  custom_msg.rsvd[2]         = 0;
  custom_msg.points.reserve(NUM_POINT);

  // B) PointCloud2
  sensor_msgs::PointCloud2 pc2_msg;
  pc2_msg.header.frame_id = "livox_frame";
  pc2_msg.height          = 1;
  pc2_msg.is_bigendian    = false;
  pc2_msg.is_dense        = true;

  // 7 fields: x,y,z,intensity,tag,line,timestamp
  // same offsets as before, except "timestamp" is float64
  // We'll store "timestamp" in *nanoseconds* as a double
  pc2_msg.fields.resize(7);

  // x
  pc2_msg.fields[0].name     = "x";
  pc2_msg.fields[0].offset   = 0;
  pc2_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32; // 7
  pc2_msg.fields[0].count    = 1;

  // y
  pc2_msg.fields[1].name     = "y";
  pc2_msg.fields[1].offset   = 4;
  pc2_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  pc2_msg.fields[1].count    = 1;

  // z
  pc2_msg.fields[2].name     = "z";
  pc2_msg.fields[2].offset   = 8;
  pc2_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  pc2_msg.fields[2].count    = 1;

  // intensity
  pc2_msg.fields[3].name     = "intensity";
  pc2_msg.fields[3].offset   = 12;
  pc2_msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  pc2_msg.fields[3].count    = 1;

  // tag
  pc2_msg.fields[4].name     = "tag";
  pc2_msg.fields[4].offset   = 16;
  pc2_msg.fields[4].datatype = sensor_msgs::PointField::UINT8; // 2
  pc2_msg.fields[4].count    = 1;

  // line
  pc2_msg.fields[5].name     = "line";
  pc2_msg.fields[5].offset   = 17;
  pc2_msg.fields[5].datatype = sensor_msgs::PointField::UINT8; // 2
  pc2_msg.fields[5].count    = 1;

  // timestamp (float64)
  pc2_msg.fields[6].name     = "timestamp";
  pc2_msg.fields[6].offset   = 18;
  pc2_msg.fields[6].datatype = sensor_msgs::PointField::FLOAT64; // 8
  pc2_msg.fields[6].count    = 1;

  // total size = 4 floats + 2 uint8 + 1 double = 26 bytes
  pc2_msg.point_step = 26;

  std::vector<livox_ros_driver::CustomPoint> custom_buffer;
  custom_buffer.reserve(NUM_POINT);

  std::vector<uint8_t> pc2_buffer;
  pc2_buffer.reserve(NUM_POINT * pc2_msg.point_step);

  auto flush_batch = [&](ros::Time flush_stamp) {
    if (custom_buffer.empty()) return;

    // 1) Write the CustomMsg
    custom_msg.header.stamp = flush_stamp;
    custom_msg.point_num    = custom_buffer.size();
    custom_msg.points       = custom_buffer;
    bag.write("/livox/lidar", custom_msg.header.stamp, custom_msg);
    custom_buffer.clear();

    // 2) Write the PointCloud2
    pc2_msg.header.stamp = flush_stamp;
    pc2_msg.width        = pc2_buffer.size() / pc2_msg.point_step;
    pc2_msg.row_step     = pc2_msg.width * pc2_msg.point_step;
    pc2_msg.data         = pc2_buffer;
    bag.write("/livox/points", pc2_msg.header.stamp, pc2_msg);
    pc2_buffer.clear();
  };

  bool first_in_batch  = true;
  uint64_t timebase_us = 0;
  int line_id          = 0;

  // For each .laz file
  for (auto &pcFile : files_laz) {
    auto points = mandeye::load(pcFile);
    for (auto &p : points) {
      if (p.timestamp == 0.0) {
        continue;
      }
      p.timestamp += TIME_START;
      ros::Time current_stamp;
      current_stamp.fromSec(p.timestamp);

      // If first in new batch, set timebase
      if (first_in_batch) {
        custom_msg.points.clear();
        pc2_buffer.clear();
        timebase_us = static_cast<uint64_t>(p.timestamp * 1e6);
        custom_msg.timebase = timebase_us;
        first_in_batch = false;
      }

      // 1) Fill the Livox CustomMsg
      livox_ros_driver::CustomPoint cp;
      uint64_t current_us = static_cast<uint64_t>(p.timestamp * 1e6);
      cp.offset_time      = static_cast<uint32_t>(current_us - timebase_us); // microseconds
      cp.reflectivity     = p.intensity;
      cp.x                = p.point.x();
      cp.y                = p.point.y();
      cp.z                = p.point.z();
      cp.tag              = 0;
      cp.line             = line_id;
      custom_buffer.push_back(cp);

      // 2) Fill the PointCloud2 (26 bytes/point)
      uint8_t buffer[26];
      float  *x_ptr   = reinterpret_cast<float*>(&buffer[0]);
      float  *y_ptr   = reinterpret_cast<float*>(&buffer[4]);
      float  *z_ptr   = reinterpret_cast<float*>(&buffer[8]);
      float  *i_ptr   = reinterpret_cast<float*>(&buffer[12]);
      uint8_t*tag_ptr = &buffer[16];
      uint8_t*ln_ptr  = &buffer[17];
      double *ts_ptr  = reinterpret_cast<double*>(&buffer[18]);

      *x_ptr    = static_cast<float>(p.point.x());
      *y_ptr    = static_cast<float>(p.point.y());
      *z_ptr    = static_cast<float>(p.point.z());
      *i_ptr    = static_cast<float>(p.intensity);
      *tag_ptr  = 0;       // or (uint8_t) p.tag
      *ln_ptr   = line_id; // or (uint8_t) p.line

      // Now store "timestamp" in *nanoseconds*:
      // p.timestamp is in seconds, so multiply by 1e9
      *ts_ptr = static_cast<double>(p.timestamp * 1.0e9);

      pc2_buffer.insert(pc2_buffer.end(), buffer, buffer + 26);

      if (++line_id >= LINE_COUNT) {
        line_id = 0;
        // Flush if we exceed NUM_POINT
        if (static_cast<int>(custom_buffer.size()) >= NUM_POINT) {
          flush_batch(current_stamp);
          first_in_batch = true;
        }
      }
    }
  }

  // Flush remainder
  if (!custom_buffer.empty()) {
    double last_time_ns = 0.0;
    if (!pc2_buffer.empty()) {
      // The last 8 bytes in pc2_buffer is the double 'timestamp'
      size_t offset = pc2_buffer.size() - 8;
      std::memcpy(&last_time_ns, &pc2_buffer[offset], sizeof(double));
    }
    // last_time_ns is in nanoseconds
    double last_secs = last_time_ns * 1e-9; 
    ros::Time final_stamp;
    final_stamp.fromSec(last_secs);
    flush_batch(final_stamp);
  }

  bag.close();
  ROS_INFO_STREAM("Done writing bag (timestamps in ns) to: " << output_bag);
  return 0;
}

