/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VisionImuFrontend-definitions.h
 * @brief  Definitions for VisionImuFrontend
 * @author Marcus Abate
 * @author Igor Lovets
 */

#pragma once

#include <gtsam/navigation/NavState.h>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/frontend/FrontendType.h"

namespace VIO {

enum class FrontendType {
  //! Frontend that works with Mono camera and Imu
  kMonoImu = 0,
  //! Frontend that works with Stereo camera and Imu
  kStereoImu = 1,
  //! Frontend that works with RGB + Depth camera and Imu
  kRgbdImu = 2,
  //! Frontend that works with Gnss, Stereo camera and Imu
  kGnssStereoImu = 3,
// TODO(marcus): this could probably move somewhere else once refactor happens
};

struct ExternalOdomMeasurement {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ExternalOdomMeasurement() = default;

  ExternalOdomMeasurement(const Timestamp& timestamp,
                          const gtsam::NavState& odom_data)
      : timestamp_(timestamp), odom_data_(odom_data) {}

  Timestamp timestamp_;
  gtsam::NavState odom_data_;
};

}  // namespace VIO
