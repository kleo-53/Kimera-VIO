/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RegularVioBackendParams.h
 * @brief  Class collecting the parameters of the Visual Inertial odometry
 * pipeline for the RegularVIO implementation.
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/backend/RegularVioBackendParams.h"
#include "kimera-vio/backend/GnssVioBackend-definitions.h"

namespace VIO {

class GnssVioBackendParams : public RegularVioBackendParams {
 public:
  KIMERA_POINTER_TYPEDEFS(GnssVioBackendParams);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GnssVioBackendParams();
  virtual ~GnssVioBackendParams() = default;

public:
  bool parseYAML(const std::string& filepath) override;

  bool equals(const BackendParams& vp2, double tol = 1e-8) const override;
  
  void print() const override;

  static GnssVioBackendParams safeCast(const BackendParams& params);

 public:
 GnssBackendModality backend_modality_ =
    GnssBackendModality::STRUCTURELESS_WITH_GNSS;
    
  double initialGnssPoseSigma_ = 0.0;
  double gnssNoiseSigma_ = 1.0;
  int gnssNormType_ = 0;     // 0: L2, 1: Huber, 2: Tukey
  double gnssNormParam_ = 0.0;

 protected:
  bool parseYAMLGnssVioBackendParams(const YamlParser& yaml_parser);
  bool equalsGnssVioBackendParams(const BackendParams& vp2, double tol = 1e-8) const;
  void printGnssVioBackendParams() const;
};

}  // namespace VIO