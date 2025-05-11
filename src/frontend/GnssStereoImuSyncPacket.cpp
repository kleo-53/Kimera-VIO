/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoImuSyncPacket.h
 * @brief  Class describing the minimum input for VIO to run
 * Contains a Stereo Frame with Imu data synchronized from last
 * Keyframe timestamp to the current stereo frame timestamp.
 * @author Antoni Rosinol
 */

#include "kimera-vio/frontend/GnssStereoImuSyncPacket.h"
// #include "kimera-vio/frontend/MonoImuSyncPacket.h"
#include <utility>
#include <optional>

namespace VIO {

GnssStereoImuSyncPacket::GnssStereoImuSyncPacket(
    const StereoFrame& stereo_frame,
    const ImuStampS& imu_stamps,
    const ImuAccGyrS& imu_accgyrs,
    const GnssStampS& gnss_stamps,
    const GnssPointS& gnss_points,
    const std::optional<gtsam::NavState> external_odometry,
    const ReinitPacket& reinit_packet)
    : FrontendInputPacketBase(
      stereo_frame.timestamp_,
      imu_stamps,
      imu_accgyrs,
      external_odometry),
      stereo_frame_(stereo_frame),
      gnss_stamps_(gnss_stamps),
      gnss_points_(gnss_points),
      reinit_packet_(reinit_packet) {
  // The timestamp of the last IMU measurement must correspond to the timestamp
  // of the stereo frame. In case there is no IMU measurement with exactly
  // the same timestamp as the stereo frame, the user should interpolate
  // IMU measurements to get a value at the time of the stereo_frame.
  CHECK_GT(imu_stamps_.cols(), 0);
  CHECK_EQ(stereo_frame_.timestamp_, imu_stamps_(imu_stamps_.cols() - 1));
  // LOG(WARNING) << "GNSS POSES SIZE IN SYNCPACKET" << gnss_points.size();
}

// GnssStereoImuSyncPacket::GnssStereoImuSyncPacket(
//     const StereoImuSyncPacket::UniquePtr&& sis_packet,
//     const Gnss& gnss_points)
//     : StereoImuSyncPacket(sis_packet->getStereoFrame(),
//                           sis_packet->getImuStamps(),
//                           sis_packet->getImuAccGyrs(),
//                           std::nullopt,
//                           sis_packet->getReinitPacket()),
//       gnss_points_(gnss_points) {}
      
void GnssStereoImuSyncPacket::print() const {
  LOG(INFO) << "Stereo Frame timestamp: " << stereo_frame_.timestamp_ << '\n'
            << "STAMPS IMU rows : \n"
            << imu_stamps_.rows() << '\n'
            << "STAMPS IMU cols : \n"
            << imu_stamps_.cols() << '\n'
            << "STAMPS IMU: \n"
            << imu_stamps_ << '\n'
            << "ACCGYR IMU rows : \n"
            << imu_accgyrs_.rows() << '\n'
            << "ACCGYR IMU cols : \n"
            << imu_accgyrs_.cols() << '\n'
            << "ACCGYR IMU: \n"
            << imu_accgyrs_;
  if (reinit_packet_.getReinitFlag() == true) {
    LOG(INFO) << "\nVIO Re-Initialized at: " << reinit_packet_.getReinitTime()
              << '\n'
              << "POSE : \n"
              << reinit_packet_.getReinitPose() << '\n'
              << "VELOCITY : \n"
              << reinit_packet_.getReinitVel() << '\n'
              << "BIAS : \n"
              << reinit_packet_.getReinitBias();
  }
  // if (gnss_points_) {
    LOG(INFO) << "GNSS Data is present.\n"
                << "GNSS Timestamp: " << gnss_stamps_.rows() << '\n'
                << "GNSS Position: " << gnss_points_.rows() << '\n';
  // } else {
    // LOG(INFO) << "No GNSS Data.\n";
  // }
      // MonoImuSyncPacket::print(); // TOD: надеюсь так можно
  //   LOG(INFO) << "Stereo Frame timestamp: " << getStereoFrame().timestamp_ << '\n'
  //       << "STAMPS IMU rows : \n"
  //       << getImuStamps().rows() << '\n'
  //       << "STAMPS IMU cols : \n"
  //       << getImuStamps().cols() << '\n'
  //       << "STAMPS IMU: \n"
  //       << getImuStamps() << '\n'
  //       << "ACCGYR IMU rows : \n"
  //       << getImuAccGyrs().rows() << '\n'
  //       << "ACCGYR IMU cols : \n"
  //       << getImuAccGyrs().cols() << '\n'
  //       << "ACCGYR IMU: \n"
  //       << getImuAccGyrs();
  //   if (getReinitPacket().getReinitFlag() == true) {
  //       LOG(INFO) << "\nVIO Re-Initialized at: " << getReinitPacket().getReinitTime()
  //           << '\n'
  //           << "POSE : \n"
  //           << getReinitPacket().getReinitPose() << '\n'
  //           << "VELOCITY : \n"
  //           << getReinitPacket().getReinitVel() << '\n'
  //           << "BIAS : \n"
  //           << getReinitPacket().getReinitBias();
  //   }
  }
}  // namespace VIO

