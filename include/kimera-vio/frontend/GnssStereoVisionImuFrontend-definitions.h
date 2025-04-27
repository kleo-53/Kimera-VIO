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
#include "kimera-vio/frontend/Gnss.h"

namespace VIO {

struct GnssStereoFrontendOutput : public VIO::StereoFrontendOutput {
 public:
  KIMERA_POINTER_TYPEDEFS(GnssStereoFrontendOutput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(GnssStereoFrontendOutput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
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
  const Gnss gnss_nav_data_;
};

}  // namespace VIO
