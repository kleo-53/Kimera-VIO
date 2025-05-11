/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RegularVioBackend.h
 * @brief  Derived class from VioBackend which enforces regularity constraints
 * on the factor graph.
 *
 * A. Rosinol, T. Sattler, M. Pollefeys, and L. Carlone. Incremental
 * Visual-Inertial 3D Mesh Generation with Structural Regularities. IEEE Intl.
 * Conf. on Robotics and Automation (ICRA), 2019
 *
 * @author Antoni Rosinol
 */

 #include "kimera-vio/backend/GnssVioBackend.h"
 #include "kimera-vio/factors/GnssFactor.h"

 namespace VIO {
 
 GnssVioBackend::GnssVioBackend(
     const Pose3& B_Pose_leftCamRect,
     const StereoCalibPtr& stereo_calibration,
     const BackendParams& backend_params,
     const ImuParams& imu_params,
     const BackendOutputParams& backend_output_params,
     const bool& log_output,
     std::optional<OdometryParams> odom_params)
     : RegularVioBackend(B_Pose_leftCamRect,
                         stereo_calibration,
                         backend_params,
                         imu_params,
                         backend_output_params,
                         log_output,
                         odom_params),
       gnss_vio_params_(GnssVioBackendParams::safeCast(backend_params)) {
        LOG(INFO) << "Using Gnss VIO Backend.\n";
        // gnss_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        //   (gtsam::Vector(3) << gnss_vio_params_.gnssNoiseSigma_, gnss_vio_params_.gnssNoiseSigma_, gnss_vio_params_.gnssNoiseSigma_).finished()
        // );
        auto base_model = gtsam::noiseModel::Isotropic::Sigma(
          3, gnss_vio_params_.gnssNoiseSigma_);
        selectNormType(&gnss_noise_,
                        base_model,
                        gnss_vio_params_.gnssNormType_,
                        gnss_vio_params_.gnssNormParam_);
       }
 
//  bool GnssVioBackend::addVisualInertialStateAndOptimize(
//      const Timestamp& timestamp_kf_nsec,
//      const StatusStereoMeasurements& status_smart_stereo_measurements_kf,
//      const gtsam::PreintegrationType& pim,
//      std::optional<gtsam::Pose3> odometry_body_pose,
//      std::optional<gtsam::Velocity3> odometry_vel,
//      const std::vector<gtsam::Point3>& gnss_points = {})  // GNSS
//  {
//   if (gnss_point) {
//     return addVisualInertialStateAndOptimize(
//         timestamp_kf_nsec,
//         status_smart_stereo_measurements_kf,
//         pim,
//         odometry_body_pose,
//         odometry_vel,
//         std::vector<gtsam::Point3>{*gnss_point});
//   } else {
//     return addVisualInertialStateAndOptimize(
//         timestamp_kf_nsec,
//         status_smart_stereo_measurements_kf,
//         pim,
//         odometry_body_pose,
//         odometry_vel,
//         {});
//   }
//  }

// BackendOutput::UniquePtr VioBackend::spinOnce(const BackendInput& input) {
//   LOG(INFO) << "IN GNSSVIOBACKEND::spinOnce";
//   if (VLOG_IS_ON(10)) {
//     input.print();
//   }

//   if (logger_) {
//     logger_->logBackendExtOdom(input);
//   }

//   bool backend_status = false;
//   const BackendState backend_state = backend_state_;
//   switch (backend_state) {
//     case BackendState::Bootstrap: {
//       initializeBackend(input);
//       LOG(INFO) << "INITIALIZED";
//       backend_status = true;
//       break;
//     }
//     case BackendState::Nominal: {
//       // Process data with VIO.
//       // if (!input.gnss_points_.empty()) {
//         backend_status = addVisualInertialStateAndOptimize(
//             input.timestamp_,
//             input.status_stereo_measurements_kf_,
//             input.pim_,
//             input.body_lkf_OdomPose_body_kf_,
//             input.body_kf_world_OdomVel_body_kf_,
//             input.gnss_points_);
//       // } else {
//       //   backend_status = addVisualInertialStateAndOptimize(
//       //       input.timestamp_,
//       //       input.status_smart_stereo_measurements_,
//       //       input.pim_,
//       //       input.odom_pose_,
//       //       input.odom_vel_);
//       // }
//       LOG(INFO) << "SPIN BACKEND";
//       break;
//     }
//     default: {
//       LOG(FATAL) << "Unrecognized Backend state.";
//       break;
//     }
//   }

//   // Fill ouput_payload (it will remain nullptr if the backend_status is not ok)
//   BackendOutput::UniquePtr output_payload = nullptr;
//   if (backend_status) {
//     // If Backend is doing ok, fill and return ouput_payload;
//     if (VLOG_IS_ON(10)) {
//       LOG(INFO) << "Latest Backend IMU bias is: ";
//       getLatestImuBias().print();
//       LOG(INFO) << "Prev kf Backend IMU bias is: ";
//       getImuBiasPrevKf().print();
//     }

//     // TODO(Toni): remove all of this.... It should be done in 3DVisualizer
//     // or in the Mesher depending on who needs what...
//     // Generate extra optional backend ouputs.
//     static const bool kOutputLmkMap =
//         backend_output_params_.output_map_lmk_ids_to_3d_points_in_time_horizon_;
//     static const bool kMinLmkObs =
//         backend_output_params_.min_num_obs_for_lmks_in_time_horizon_;
//     static const bool kOutputLmkTypeMap =
//         backend_output_params_.output_lmk_id_to_lmk_type_map_;
//     LmkIdToLmkTypeMap lmk_id_to_lmk_type_map;
//     PointsWithIdMap lmk_ids_to_3d_points_in_time_horizon;
//     if (kOutputLmkMap) {
//       // Generate this map only if requested, since costly.
//       // Also, if lmk type requested, fill lmk id to lmk type object.
//       // WARNING this also cleans the lmks inside the old_smart_factors map!
//       lmk_ids_to_3d_points_in_time_horizon =
//           getMapLmkIdsTo3dPointsInTimeHorizon(
//               smoother_->getFactors(),
//               kOutputLmkTypeMap ? &lmk_id_to_lmk_type_map : nullptr,
//               kMinLmkObs);
//     }

//     if (map_update_callback_) {
//       map_update_callback_(lmk_ids_to_3d_points_in_time_horizon);
//     } else {
//       LOG(FATAL) << "Did you forget to register the Map "
//                     "Update callback for at least the "
//                     "Frontend? Do so by using "
//                     "registerMapUpdateCallback function.";
//     }

//     // Create Backend Output Payload.
//     output_payload = std::make_unique<BackendOutput>(
//         VioNavStateTimestamped(
//             input.timestamp_,
//             (FLAGS_no_incremental_pose ? W_Pose_B_lkf_from_state_
//                                        : W_Pose_B_lkf_from_increments_),
//             W_Vel_B_lkf_,
//             imu_bias_lkf_),
//         // TODO(Toni): Make all below optional!!
//         state_,
//         smoother_->getFactors(),
//         getCurrentStateCovariance(),
//         curr_kf_id_,
//         landmark_count_,
//         debug_info_,
//         lmk_ids_to_3d_points_in_time_horizon,
//         lmk_id_to_lmk_type_map);

//     if (logger_) {
//       logger_->logBackendOutput(*output_payload);
//     }
//   }

//   return output_payload;
// }

//  bool GnssVioBackend::addVisualInertialStateAndOptimize(
//     const Timestamp& timestamp_kf_nsec,
//     const StatusStereoMeasurements& status_smart_stereo_measurements_kf,
//     const gtsam::PreintegrationType& pim,
//     std::optional<gtsam::Pose3> odometry_body_pose,
//     std::optional<gtsam::Velocity3> odometry_vel,
//     std::optional<std::vector<gtsam::Point3>> gnss_points)
//   {
//     LOG(INFO) << "IN GNSS BACKEND";
//     bool success = RegularVioBackend::addVisualInertialStateAndOptimize(
//       timestamp_kf_nsec,
//       status_smart_stereo_measurements_kf,
//       pim,
//       odometry_body_pose,
//       odometry_vel,
//       gnss_points);

//     if (!success || !gnss_points || gnss_points->empty()) {
//       LOG(WARNING) << "NO GNSS IN BACKEND";
//       return success;
//     }

//     gtsam::NonlinearFactorGraph gnss_factor_graph;
//     const FrameId frame_id = curr_kf_id_;
//     const gtsam::Symbol pose_key('x', frame_id);  // привязка к текущему кадру
    
//     for (const auto& gnss : *gnss_points) {
//       gnss_factor_graph.add(gtsam::GnssFactor(pose_key, gnss, gnss_noise_));
//       LOG(INFO) << "Adding GNSS factor at timestamp " << frame_id;
//     }
//     // LOG(INFO) << "Factors in graph BEFORE: " << smoother_->getFactors().size();
//     // smoother_->update(gnss_factor_graph, gtsam::Values());
//     // LOG(INFO) << "Factors in graph AFTER : " << smoother_->getFactors().size();
//     return true;
// }
 
void GnssVioBackend::beforeOptimizeHook(const Timestamp& ts, std::optional<std::vector<GnssPoint>> gnss_points) {
  if (!gnss_points || gnss_points->empty()) return;

  // gtsam::Point3 mean_gnss = std::accumulate(
  //   gnss_points->begin(), gnss_points->end(), gtsam::Point3(0, 0, 0)) /
  //   gnss_points->size();

  GnssPoint mean_gnss = gnss_points->back();

  addGnssFactor(curr_kf_id_, mean_gnss, &new_imu_prior_and_other_factors_);
  LOG(INFO) << "Added GNSS factor to keyframe " << curr_kf_id_;

  // BAD
  // for (const auto& gnss : *gnss_points) {
  //   addGnssFactor(curr_kf_id_, gnss, &new_imu_prior_and_other_factors_);
  //   LOG(INFO) << "Added GNSS factor to keyframe " << curr_kf_id_;
  // }
}

  void GnssVioBackend::addGnssFactor(
      const FrameId& frame_id,
      const GnssPoint& gnss_point,
      gtsam::NonlinearFactorGraph* graph) {
    CHECK_NOTNULL(graph);
    const gtsam::Symbol pose_key('x', frame_id);
    graph->add(gtsam::GnssFactor(pose_key, gnss_point, gnss_noise_));
  }

 /* -------------------------------------------------------------------------- */
// Output a noise model with a selected norm type:
// norm_type = 0: l-2.
// norm_type = 1: Huber.
// norm_type = 2: Tukey.
void GnssVioBackend::selectNormType(
  gtsam::SharedNoiseModel* noise_model_output,
  const gtsam::SharedNoiseModel& noise_model_input,
  const size_t& norm_type,
  const double& norm_type_parameter) {
CHECK_NOTNULL(noise_model_output);
switch (norm_type) {
  case 0: {
    VLOG(1) << "Using l-2 norm.";
    *noise_model_output = noise_model_input;
    break;
  }
  case 1: {
    VLOG(1) << "Using Huber norm, with parameter value: "
            << norm_type_parameter;
    *noise_model_output = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(
            norm_type_parameter,
            gtsam::noiseModel::mEstimator::Huber::Scalar),  // Default is
                                                            // Block
        noise_model_input);
    break;
  }
  case 2: {
    VLOG(1) << "Using Tukey norm, with parameter value: "
            << norm_type_parameter;
    *noise_model_output = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Tukey::Create(
            norm_type_parameter,
            gtsam::noiseModel::mEstimator::Tukey::Scalar),  // Default is
                                                            // Block
        noise_model_input);                                 // robust
    break;
  }
  default: {
    LOG(ERROR) << "Wrong norm_type passed...";
    break;
  }
}
}

 }  // namespace VIO
 