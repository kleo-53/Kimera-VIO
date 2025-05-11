/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/*
 * PointPlaneFactor.cpp
 *
 *  Created on: Feb 20, 2018
 *      Author: Antoni Rosinol
 */

#include "kimera-vio/factors/GnssFactor.h"

using namespace std;

namespace gtsam {

    
GnssFactor::GnssFactor() {}

GnssFactor::~GnssFactor() {}

GnssFactor::GnssFactor(const Key& poseKey,
                       const Point3& measuredPosition,
                       const SharedNoiseModel& noiseModel)
    : Base(noiseModel, poseKey), 
      position_(measuredPosition) {}

      
      gtsam::Vector GnssFactor::evaluateError(
        const gtsam::Pose3& pose,
        boost::optional<gtsam::Matrix&> H) const {
  if (H) {
    *H = (Matrix(3,6) << Matrix3::Zero(), Matrix3::Identity()).finished();
  }
  return pose.translation() - position_;
}

gtsam::NonlinearFactor::shared_ptr GnssFactor::clone() const {
  return boost::make_shared<GnssFactor>(*this);
  return gtsam::NonlinearFactor::shared_ptr(new GnssFactor(*this));
}

void GnssFactor::print(const std::string& s,
  const KeyFormatter& keyFormatter) const {
    cout << s << "GNSS Factor on pose " << keyFormatter(this->key()) << "\n";
    this->noiseModel_->print("  noise model: ");
    traits<Point3>::Print(position_, "  measured position: ");
  }
  // Vector GnssFactor::evaluateError(const Pose3& pose, OptionalMatrixType H) const {
  //   if (H) {
  //     // Derivative of translation w.r.t Pose3 is [0 | I]
  //     *H = (Matrix(3,6) << Matrix3::Zero(), Matrix3::Identity()).finished();
  //   }
  //   return pose.translation() - position_;
  // }

}  // namespace gtsam
