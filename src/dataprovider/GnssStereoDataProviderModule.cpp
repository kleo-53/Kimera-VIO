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
     const StereoMatchingParams& stereo_matching_params)
     : MonoDataProviderModule(output_queue,
                              name_id,
                              parallel_run),
       right_frame_queue_("data_provider_right_frame_queue"),
       gnss_queue_("data_provider_gnss_queue"),
       stereo_matching_params_(stereo_matching_params) {}
 
 GnssStereoDataProviderModule::InputUniquePtr
 GnssStereoDataProviderModule::getInputPacket() {
   //! Get left image + IMU data
   MonoImuSyncPacket::UniquePtr mono_imu_sync_packet =
       getMonoImuSyncPacket(false);
   if (!mono_imu_sync_packet) {
    LOG(INFO) << "RETURN WOUT PACKET";
     return nullptr;
   }
 
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
   timestamp_last_frame_ = timestamp;
   static int gnss_counter = 0;
     Gnss::UniquePtr gnss_data_payload = nullptr;
     do {
       auto gnss_shared = gnss_queue_.popBlocking();
       gnss_data_payload = std::move(*gnss_shared);
       if (!gnss_data_payload) {
         LOG(WARNING) << "Missing GNSS data for frame, continuing without it";
         // return;
         break;
       }
       gnss_counter++;
     } while (gnss_counter % 10 != 0);
   
     // Gnss::UniquePtr gnss_data_payload = nullptr;
     if (!MISO::syncQueue(timestamp, &gnss_queue_, &gnss_data_payload)) {
       LOG(INFO) << "GNSS timestamp: " << gnss_data_payload->timestamp_;
       LOG(INFO) << "Frame timestamp: " << timestamp;
       LOG(INFO) << "Absolute difference: " 
               << fabs(gnss_data_payload->timestamp_ - timestamp);
       LOG(INFO) << "Missing gnss data for frames, continue";
       // gnss_data_payload =
       //     std::make_unique<Gnss>(packet->timestamp_, gtsam::Vector3()); // TOD: тут было true
     }
     CHECK(gnss_data_payload);
     if (!shutdown_) {
         LOG(INFO) << "getInputPacket() called for GNSS pipeline";
       auto packet = std::make_unique<GnssStereoImuSyncPacket>(
           StereoFrame(left_frame_id,
                       timestamp,
                       *mono_imu_sync_packet->frame_,  // this copies...
                       *right_frame_payload),          // this copies...
           // be given in PipelineParams.
           mono_imu_sync_packet->imu_stamps_,
           mono_imu_sync_packet->imu_accgyrs_,
           std::nullopt,
           VIO::ReinitPacket(),
           Gnss(gnss_data_payload->timestamp_,
                 gnss_data_payload->nav_pos_));
       if (send_packet_) {
         LOG(INFO) << "GNSS PACKAGE SENT";
         CHECK(vio_pipeline_callback_);
         vio_pipeline_callback_(std::move(packet));
       } else {
        LOG(INFO) << "GNSS PACKAGE NOT SENT";
         return packet;
       }
     }
  //  if (!shutdown_) {
  //    auto packet = std::make_unique<GnssStereoImuSyncPacket>( // TOD: сюда мб gnss-данные(?)
  //        StereoFrame(left_frame_id,
  //                    timestamp,
  //                    *mono_imu_sync_packet->frame_,  // this copies...
  //                    *right_frame_payload),          // this copies...
  //        // be given in PipelineParams.
  //        mono_imu_sync_packet->imu_stamps_,
  //        mono_imu_sync_packet->imu_accgyrs_,);
  //    if (send_packet_) {
  //      CHECK(vio_pipeline_callback_);
  //      vio_pipeline_callback_(std::move(packet));
  //    } else {
  //      return packet;
  //    }
  //  }
 
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
   gnss_queue_.shutdown();
   MonoDataProviderModule::shutdownQueues();
 }
 
 }  // namespace VIO
 

// /* ----------------------------------------------------------------------------
//  * Copyright 2017, Massachusetts Institute of Technology,
//  * Cambridge, MA 02139
//  * All Rights Reserved
//  * Authors: Luca Carlone, et al. (see THANKS for the full author list)
//  * See LICENSE for the license information
//  * -------------------------------------------------------------------------- */

// /**
//  * @file   StereoDataProviderModule.cpp
//  * @brief  Pipeline Module that takes care of providing data to the VIO
//  * pipeline.
//  * @author Antoni Rosinol
//  */

// #include "kimera-vio/dataprovider/GnssStereoDataProviderModule.h"

// #include <optional>
// #include <glog/logging.h>
// #include "kimera-vio/frontend/Frame.h"
// #include "kimera-vio/frontend/GnssStereoImuSyncPacket.h"

// namespace VIO {

// GnssStereoDataProviderModule::GnssStereoDataProviderModule(
//     OutputQueue* output_queue,
//     const std::string& name_id,
//     const bool& parallel_run,
//     const StereoMatchingParams& stereo_matching_params)
//     : MonoDataProviderModule(output_queue,
//         name_id,
//         parallel_run),
//       gnss_queue_("data_provider_gnss_queue"),
//       right_frame_queue_("data_provider_right_frame_queue"),
//       stereo_matching_params_(stereo_matching_params) {}

// GnssStereoDataProviderModule::InputUniquePtr
// GnssStereoDataProviderModule::getInputPacket() {
//   //! Get left image + IMU data

//   MonoImuSyncPacket::UniquePtr mono_imu_sync_packet =
//       getMonoImuSyncPacket(false);
//   if (!mono_imu_sync_packet) {
//     return nullptr;
//   }

//   const Timestamp& timestamp = mono_imu_sync_packet->timestamp_;
//   const FrameId& left_frame_id = mono_imu_sync_packet->frame_->id_;

//   Frame::UniquePtr right_frame_payload = nullptr;
//   if (!MISO::syncQueue(timestamp, &right_frame_queue_, &right_frame_payload)) {
//     // Dropping this message because of missing left/right stereo synced
//     // frames.
//     LOG(ERROR) << "Missing right frame for left frame with id " << left_frame_id
//                << ", dropping this frame.";
//     return nullptr;
//   }
//   CHECK(right_frame_payload);
//   timestamp_last_frame_ = timestamp;

  
//   // send_packet_ = false;
//   // GnssStereoDataProviderModule::InputUniquePtr packet =
//   //     GnssStereoDataProviderModule::getInputPacket();
//   static int gnss_counter = 0;
//   Gnss::UniquePtr gnss_data_payload = nullptr;
//   do {
//     auto gnss_shared = gnss_queue_.popBlocking();
//     gnss_data_payload = std::move(*gnss_shared);
//     if (!gnss_data_payload) {
//       LOG(WARNING) << "Missing GNSS data for frame, continuing without it";
//       // return;
//       break;
//     }
//     gnss_counter++;
//   } while (gnss_counter % 10 != 0);

//   // Gnss::UniquePtr gnss_data_payload = nullptr;
//   if (!MISO::syncQueue(timestamp, &gnss_queue_, &gnss_data_payload)) {
//     LOG(INFO) << "GNSS timestamp: " << gnss_data_payload->timestamp_;
//     LOG(INFO) << "Frame timestamp: " << timestamp;
//     LOG(INFO) << "Absolute difference: " 
//             << fabs(gnss_data_payload->timestamp_ - timestamp);
//     LOG(INFO) << "Missing gnss data for frames, continue";
//     // gnss_data_payload =
//     //     std::make_unique<Gnss>(packet->timestamp_, gtsam::Vector3()); // TOD: тут было true
//   }
//   CHECK(gnss_data_payload);
//   if (!shutdown_) {
//       LOG(INFO) << "getInputPacket() called for GNSS pipeline";
//     auto packet = std::make_unique<GnssStereoImuSyncPacket>(
//         StereoFrame(left_frame_id,
//                     timestamp,
//                     *mono_imu_sync_packet->frame_,  // this copies...
//                     *right_frame_payload),          // this copies...
//         // be given in PipelineParams.
//         mono_imu_sync_packet->imu_stamps_,
//         mono_imu_sync_packet->imu_accgyrs_,
//         std::nullopt,
//         VIO::ReinitPacket(),
//         Gnss(gnss_data_payload->timestamp_,
//               gnss_data_payload->nav_pos_));
//     if (send_packet_) {
//       LOG(INFO) << "GNSS PACKAGE SENT";
//       CHECK(vio_pipeline_callback_);
//       vio_pipeline_callback_(std::move(packet));
//     } else {
//       return packet;
//     }
//   }

//   // Gnss data(gnss_data_payload->timestamp_,
//   //           gnss_data_payload->nav_pos_);
//   //           // ,
//   //           // gnss_data_payload->undefined_);

//   // // StereoImuSyncPacket::UniquePtr vio_packet =
//   // //     VIO::safeCast<FrontendInputPacketBase, StereoImuSyncPacket>(
//   // //         std::move(packet));
//   // // MonoImuSyncPacket::UniquePtr mono_imu_sync_packet =
//   // //  getMonoImuSyncPacket(false);
//   // // if (!mono_imu_sync_packet) {
//   // //   return nullptr;
//   // // }
//   // // Frame::UniquePtr right_frame_payload = nullptr;
//   // // const Timestamp& timestamp = mono_imu_sync_packet->timestamp_;
//   // // if (!MISO::syncQueue(timestamp, &right_frame_queue_, &right_frame_payload)) {
//   // //   // Dropping this message because of missing left/right stereo synced
//   // //   // frames.
//   // //   LOG(ERROR) << "Missing right frame for left frame with id " << left_frame_id
//   // //              << ", dropping this frame.";
//   // //   return nullptr;
//   // // }
//   // // CHECK(right_frame_payload);
//   // // auto packet = std::make_unique<StereoImuSyncPacket>(
//   // //   StereoFrame(mono_imu_sync_packet->frame_->id_,
//   // //               mono_imu_sync_packet->timestamp_,
//   // //               *mono_imu_sync_packet->frame_,  // this copies...
//   // //               *right_frame_payload),          // this copies...
//   // //   // be given in PipelineParams.
//   // //   mono_imu_sync_packet->imu_stamps_,
//   // //   mono_imu_sync_packet->imu_accgyrs_);
//   // // if (send_packet_) {
//   // //   CHECK(vio_pipeline_callback_);
//   // //   vio_pipeline_callback_(std::move(packet));
//   // // } else {
//   // //   return packet;
//   // // }

//   // // StereoImuSyncPacket* raw_ptr = dynamic_cast<StereoImuSyncPacket*>(packet.get());
//   // // CHECK(raw_ptr != nullptr) << "Failed to cast FrontendInputPacketBase to StereoImuSyncPacket";
//   // // StereoImuSyncPacket::UniquePtr vio_packet(raw_ptr);
//   // GnssStereoImuSyncPacket* stereo_packet = 
//   // dynamic_cast<GnssStereoImuSyncPacket*>(packet.get());
//   // CHECK_NOTNULL(stereo_packet);

//   // // 2. Забираем владение указателем
//   // packet.release();

//   // // 3. Создаем UniquePtr правильного типа
//   // GnssStereoImuSyncPacket::UniquePtr stereo_packet_ptr(stereo_packet);

//   // // 4. Теперь можем создать GnssStereoImuSyncPacket
//   // auto gnss_packet = std::make_unique<GnssStereoImuSyncPacket>(
//   //   std::move(stereo_packet_ptr),
//   //   data
//   // );
//   // // auto gnss_packet =
//       // std::make_unique<GnssStereoImuSyncPacket>(std::move(packet), data);

//   // if (!shutdown_) {
//   //   CHECK(vio_pipeline_callback_);
//   //   vio_pipeline_callback_(std::move(gnss_packet));
//   // }

//   return nullptr;
// }

// void GnssStereoDataProviderModule::shutdownQueues() {
//   gnss_queue_.shutdown();
//   right_frame_queue_.shutdown();
//   MonoDataProviderModule::shutdownQueues();
// }

// }  // namespace VIO
