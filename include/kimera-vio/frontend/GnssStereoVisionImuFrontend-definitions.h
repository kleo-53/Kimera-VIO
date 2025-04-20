/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   GnssStereoVisionImuFrontend-definitions.h
 * @brief  Definitions for StereoVisionImuFrontend
 * @author Antoni Rosinol
 */

#pragma once

#include <optional>

#include "kimera-vio/frontend/GnssTypes.h"
#include "kimera-vio/frontend/StereoVisionImuFrontend-definitions.h"

// #include <kimera-vio/frontend/StereoVisionImuFrontend-definitions.h>
#include "kimera-vio/frontend/Gnss.h"
// #include "kimera-vio/common/vio_types.h"
// #include "kimera-vio/frontend/Tracker-definitions.h" 
// #include "kimera-vio/frontend/FrontendOutputPacketBase.h"
// #include "kimera-vio/frontend/StereoFrame.h"
// #include "kimera-vio/frontend/VisionImuFrontend-definitions.h"

namespace VIO {

  // using GnssPose = Eigen::Matrix<double, 3, 1>;

// struct GnssMeasurement {
//   GnssMeasurement() = default;
//   GnssMeasurement(const Timestamp& timestamp, const gtsam::Vector3& pose)
//     : timestamp_(timestamp), pose_(pose) {}
//   GnssMeasurement(Timestamp& timestamp, gtsam::Vector3& pose)
//     : timestamp_(std::move(timestamp)), pose_(std::move(pose)) {}

//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//   Timestamp timestamp_;
//   gtsam::Vector3 pose_;
// };

// using GnssStampS = Eigen::Matrix<Timestamp, 1, Eigen::Dynamic>;
// using GnssPoseS = Eigen::Matrix<double, 3, Eigen::Dynamic>;
//   struct GnssMeasurements {
//     public:
//      GnssMeasurements() = default;
//      GnssMeasurements(const GnssStampS& timestamps, const GnssPoseS& poses)
//          : timestamps_(timestamps), poses_(poses) {}
//      GnssMeasurements(GnssStampS&& timestamps, GnssPoseS&& poses)
//          : timestamps_(std::move(timestamps)), poses_(std::move(poses)) {}
   
//      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//      GnssStampS timestamps_;
//      GnssPoseS poses_;
//   };

struct GnssStereoFrontendOutput : public VIO::StereoFrontendOutput {
 public:
  KIMERA_POINTER_TYPEDEFS(GnssStereoFrontendOutput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(GnssStereoFrontendOutput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // GnssStereoFrontendOutput(
  //     const bool is_keyframe,
  //     const StatusStereoMeasurementsPtr& status_stereo_measurements,
  //     // const TrackingStatus& tracker_status,
  //     const gtsam::Pose3& relative_pose_body_stereo,
  //     const gtsam::Pose3& b_Pose_camL_rect,
  //     const gtsam::Pose3& b_Pose_camR_rect,
  //     const StereoFrame& stereo_frame_lkf,
  //     // Use rvalue reference: FrontendOutput owns pim now.
  //     const ImuFrontend::PimPtr& pim,
  //     const ImuAccGyrS& imu_acc_gyrs,
  //     const cv::Mat& feature_tracks,
  //     const DebugTrackerInfo& debug_tracker_info,
  //     const Gnss& nav_data)
  //     : StereoFrontendOutput(is_keyframe,
  //                            status_stereo_measurements,
  //                           //  tracker_status,
  //                            relative_pose_body_stereo,
  //                            b_Pose_camL_rect,
  //                            b_Pose_camR_rect,
  //                            stereo_frame_lkf,
  //                            pim,
  //                            imu_acc_gyrs,
  //                            feature_tracks,
  //                            debug_tracker_info),
  //       gnss_nav_data_(nav_data) {}
  GnssStereoFrontendOutput(const VIO::StereoFrontendOutput::UniquePtr& output,
    const Gnss& nav_data)
    : StereoFrontendOutput(
      output->is_keyframe_,
      output->status_stereo_measurements_,
      output->b_Pose_camL_rect_,
      output->b_Pose_camR_rect_,
      output->stereo_frame_lkf_,
      output->pim_,
      output->imu_acc_gyrs_,
      output->feature_tracks_,
      output->debug_tracker_info_),
      // output->lkf_body_Pose_kf_body_,
      // output->debug_tracker_info_),
      gnss_nav_data_(nav_data) {}

  virtual ~GnssStereoFrontendOutput() = default;

 public:
  // const StatusStereoMeasurementsPtr status_nav_measurements_;
  // const gtsam::Vector3 b_translate_;
  const Gnss gnss_nav_data_;
};

}  // namespace VIO
