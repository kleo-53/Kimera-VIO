/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   GnssParams.cpp
 * @brief  Params for GnssFrontend.
 * @author Igor Lovets
 */

#include "kimera-vio/frontend/GnssParams.h"

#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>

#include "kimera-vio/utils/UtilsOpenCV.h"
#include "kimera-vio/utils/YamlParser.h"
#include "kimera-vio/common/vio_types.h"

namespace VIO {

GnssParams::GnssParams() : PipelineParams("GNSS params") {}

bool GnssParams::parseYAML(const std::string& filepath) {
  YamlParser yaml_parser(filepath);

  // std::vector<double> vector_pose;

  int n_rows = 0;
  yaml_parser.getNestedYamlParam("T_BS", "rows", &n_rows);
  CHECK_EQ(n_rows, 4u);
  int n_cols = 0;
  yaml_parser.getNestedYamlParam("T_BS", "cols", &n_cols);
  CHECK_EQ(n_cols, 4u);
  std::vector<double> vector_pose;
  // yaml_parser.getNestedYamlParam("T_BS", "data", &vector_pose);
  // body_Pose_prism_ = UtilsOpenCV::poseVectorToGtsamPose3(vector_pose);
  yaml_parser.getNestedYamlParam("T_BS", "data", &vector_pose);
  b_pose_gnss_ = UtilsOpenCV::poseVectorToGtsamPose3(vector_pose);
  // LOG_IF(FATAL, !b_pose_gnss.equals(gtsam::Pose3()))
    // << "parseGnssData: we expected identity b_pose_gnss: is everything ok?";

  // double rate_hz = 0.0;
  // yaml_parser.getYamlParam("rate_hz", &rate_hz);
  // CHECK_GT(rate_hz, 0.0);
  // nominal_sampling_time_s_ = 1.0 / rate_hz;
  // std::string times;
  yaml_parser.getYamlParam("timestamp", &timestamp_);
  // timestamp_ = std::stol(times);
  // timestamp_ = std::static_pointer_cast(*times);
  yaml_parser.getYamlParam("noise", &noise_);
  int window_tmp = 0;
  yaml_parser.getYamlParam("gnss_period_estimation_window", &window_tmp);
  gnss_period_estimation_window_ = static_cast<size_t>(window_tmp);
  // yaml_parser.getYamlParam("gnss_period_estimation_window", &gnss_period_estimation_window_);
  std::string per;
  yaml_parser.getYamlParam("period", &per);
  period_ = std::stol(per);

  return true;
}

void GnssParams::print() const {
  std::stringstream out;
  PipelineParams::print(out,
                        // "b_pose_: ",
                        // b_pose_gnss,
                        // "nominal_sampling_time_s_: ",
                        // nominal_sampling_time_s_,
                        "noise_: ",
                        noise_);
  LOG(INFO) << out.str();
}

bool GnssParams::equals(const PipelineParams& obj) const {
  const auto& rhs = static_cast<const GnssParams&>(obj);
  // clang-format off
  return b_pose_gnss_.equals(rhs.b_pose_gnss_) &&
      timestamp_ == rhs.timestamp_ &&
      noise_ == rhs.noise_ &&
      period_ == rhs.period_ &&
      gnss_period_estimation_window_ == rhs.gnss_period_estimation_window_; //&&
      // nominal_sampling_time_s_ == rhs.nominal_sampling_time_s_;
  // clang-format on
}

}  // namespace VIO
