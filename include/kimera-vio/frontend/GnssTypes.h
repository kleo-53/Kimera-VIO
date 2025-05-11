// #pragma once

// #include <Eigen/Core>
// #include "kimera-vio/common/vio_types.h"

// namespace VIO {

// struct GnssMeasurement {
//   GnssMeasurement() = default;
//   GnssMeasurement(const Timestamp& timestamp, const gtsam::Vector3& point)
//     : timestamp_(timestamp), point_(point) {}
  
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//   Timestamp timestamp_;
//   gtsam::Vector3 point_;
// };

// using GnssPoint = gtsam::Point3;
// using GnssStampS = Eigen::Matrix<Timestamp, 1, Eigen::Dynamic>;
// using GnssPointS = Eigen::Matrix<double, 3, Eigen::Dynamic>;

//   struct GnssMeasurements {
//     public:
//      GnssMeasurements() = default;
//      GnssMeasurements(const GnssStampS& timestamps, const GnssPointS& points)
//          : timestamps_(timestamps), points_(points) {}
//      GnssMeasurements(GnssStampS&& timestamps, GnssPointS&& points)
//          : timestamps_(std::move(timestamps)), points_(std::move(points)) {}
   
//      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//      GnssStampS timestamps_;
//      GnssPointS points_;
//   };

// }  // namespace VIO