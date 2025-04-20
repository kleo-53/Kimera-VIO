#pragma once

#include <Eigen/Core>
#include "kimera-vio/common/vio_types.h"

namespace VIO {

struct GnssMeasurement {
  GnssMeasurement() = default;
  GnssMeasurement(const Timestamp& timestamp, const gtsam::Vector3& pose)
    : timestamp_(timestamp), pose_(pose) {}
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Timestamp timestamp_;
  gtsam::Vector3 pose_;
};

using GnssStampS = Eigen::Matrix<Timestamp, 1, Eigen::Dynamic>;
using GnssPoseS = Eigen::Matrix<double, 3, Eigen::Dynamic>;

  struct GnssMeasurements {
    public:
     GnssMeasurements() = default;
     GnssMeasurements(const GnssStampS& timestamps, const GnssPoseS& poses)
         : timestamps_(timestamps), poses_(poses) {}
     GnssMeasurements(GnssStampS&& timestamps, GnssPoseS&& poses)
         : timestamps_(std::move(timestamps)), poses_(std::move(poses)) {}
   
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
     GnssStampS timestamps_;
     GnssPoseS poses_;
  };

}  // namespace VIO