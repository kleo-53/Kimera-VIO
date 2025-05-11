/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoDataProviderModule.cpp
 * @brief  Pipeline Module that takes care of providing data to the VIO
 * pipeline.
 * @author Antoni Rosinol
 */

 #include "kimera-vio/dataprovider/GnssStereoDataProviderModule.h"

 #include <glog/logging.h>
 
 #include "kimera-vio/frontend/GnssStereoImuSyncPacket.h"
 #include "kimera-vio/frontend/Gnss.h"
 #include "kimera-vio/frontend/MonoImuSyncPacket.h"
 
 namespace VIO {
 
 GnssStereoDataProviderModule::GnssStereoDataProviderModule(
     OutputQueue* output_queue,
     const std::string& name_id,
     const bool& parallel_run,
     const StereoMatchingParams& stereo_matching_params,
     const GnssParams& gnss_params)
     : MonoDataProviderModule(output_queue,
                              name_id,
                              parallel_run),
       right_frame_queue_("data_provider_right_frame_queue"),
       stereo_matching_params_(stereo_matching_params),
       gnss_params_(gnss_params) {}
 
 GnssStereoDataProviderModule::InputUniquePtr
 GnssStereoDataProviderModule::getInputPacket() {
   //! Get left image + IMU data
   MonoImuSyncPacket::UniquePtr mono_imu_sync_packet =
       getMonoImuSyncPacket(false);
    if (!mono_imu_sync_packet) {
      LOG(WARNING) << "RETURN NULL: mono_imu_sync_packet is nullptr!"; // TOD: убрать
      return nullptr;
    }
    // LOG(INFO) << "MonoImuSyncPacket created! Timestamp: " << mono_imu_sync_packet->timestamp_;// TOD: убрать
 
   const Timestamp& timestamp = mono_imu_sync_packet->timestamp_;
   const FrameId& left_frame_id = mono_imu_sync_packet->frame_->id_;
 
   //! Get right image data
   // This search might not be successful if the data_provider did not push
   // to
   // the right queue (perhaps fast enough).
   Frame::UniquePtr right_frame_payload = nullptr;
   if (!MISO::syncQueue(timestamp, &right_frame_queue_, &right_frame_payload)) {
     // Dropping this message because of missing left/right stereo synced
     // frames.
     LOG(ERROR) << "Missing right frame for left frame with id " << left_frame_id
                << ", dropping this frame.";
     return nullptr;
   }
   CHECK(right_frame_payload);
   //! Retrieve IMU data.
  GnssMeasurements gnss_meas;
  FrameAction action = getTimeSyncedGnssMeasurements(timestamp, gnss_params_, &gnss_meas);
  switch (action) {
    case FrameAction::Use:
      break;
    case FrameAction::Wait:
      // cached_left_frame_ = std::move(left_frame_payload);
      // return nullptr;
      break;
    case FrameAction::Drop:
      // return nullptr;
      break;
  }

  if (gnss_meas.points_.cols() == 0 || gnss_meas.timestamps_.cols() == 0) {
    gnss_meas.timestamps_.resize(Eigen::NoChange, 0);
    gnss_meas.points_.resize(Eigen::NoChange, 0);
  }


  //  static int gnss_counter = 0;
  //    Gnss::UniquePtr gnss_data_payload = nullptr;
  //    do {
  //      auto gnss_shared = gnss_queue_.popBlocking();
  //      gnss_data_payload = std::move(*gnss_shared);
  //      if (!gnss_data_payload) {
  //        LOG(WARNING) << "Missing GNSS data for frame, continuing without it";
  //        // return;
  //        break;
  //      }
  //      gnss_counter++;
  //    } while (gnss_counter % 10 != 0);
   
    //  Gnss::UniquePtr gnss_data_payload = nullptr;
    //  auto a = MISO::syncQueue(timestamp, &gnss_queue_, &gnss_data_payload);
    //  LOG(INFO) << "AFTER SYNC QUEUE: " << a;
    // if (!a) {
      //  LOG(INFO) << "GNSS timestamp: " << gnss_data_payload->timestamp_;
      //  LOG(INFO) << "Frame timestamp: " << timestamp;
      //  LOG(INFO) << "Absolute difference: " 
              //  << fabs(gnss_data_payload->timestamp_ - timestamp);
      //  LOG(WARNING) << "Missing gnss data for frames, return";
      //  return nullptr;
      //  gnss_data_payload =
      //      std::make_unique<Gnss>(timestamp, gtsam::Vector3()); // TOD: тут было true
    //  }
    //  LOG(INFO) << "GNSS timestamp: " << gnss_data_payload->timestamp_;
    //  LOG(INFO) << "Frame timestamp: " << timestamp;
    //  LOG(INFO) << "Absolute difference: " 
            //  << fabs(gnss_data_payload->timestamp_ - timestamp);
    //  CHECK(gnss_data_payload);
    // if (cache_timestamp) {
      timestamp_last_frame_ = timestamp;
    // }
     if (!shutdown_) {
       auto packet = std::make_unique<GnssStereoImuSyncPacket>(
           StereoFrame(left_frame_id,
                       timestamp,
                       *mono_imu_sync_packet->frame_,  // this copies...
                       *right_frame_payload),          // this copies...
           // be given in PipelineParams.
           mono_imu_sync_packet->imu_stamps_,
           mono_imu_sync_packet->imu_accgyrs_,
           gnss_meas.timestamps_,
           gnss_meas.points_);
          //  std::nullopt,
          //  VIO::ReinitPacket());
          //  Gnss(gnss_data_payload->timestamp_,
          //        gnss_data_payload->nav_pos_));
      // }
       if (send_packet_) {
        //  LOG(INFO) << "GNSS PACKAGE SENT"; //TOD: убрать
         CHECK(vio_pipeline_callback_);
         vio_pipeline_callback_(std::move(packet));
       } else {
        // LOG(INFO) << "GNSS PACKAGE NOT SENT"; // TOD: убрать
         return packet;
       }
     }

   // Push the synced messages to the Frontend's input queue
   // TODO(Toni): should be a return like that, so that we pass the info to
   // the
   // queue... Right now we use a callback bcs otw I need to fix all
   // initialization which is a lot to be fixed.
   // return std::make_unique<StereoImuSyncPacket>(
   //    StereoFrame(
   //        left_frame_payload->id_,
   //        timestamp,
   //        *left_frame_payload,
   //        *right_frame_payload,
   //        stereo_matching_params_),  // TODO(Toni): these params should
   //                                   // be given in PipelineParams.
   //    imu_meas.timestamps_,
   //    imu_meas.acc_gyr_);
   return nullptr;
 }
 
 void GnssStereoDataProviderModule::shutdownQueues() {
   right_frame_queue_.shutdown();
   MonoDataProviderModule::shutdownQueues();
 }
 
 }  // namespace VIO
 