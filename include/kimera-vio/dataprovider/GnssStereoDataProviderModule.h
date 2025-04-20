/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoDataProviderModule.h
 * @brief  Pipeline module that provides stereo data to the VIO pipeline.
 * @details Collects camera and IMU data, publishes StereoFrames via callback
 *          getInputPacket processes one stereo pair at a time, attempting to
 *          gather IMU data between the current stereo pair and the previous
 *          stereo pair.
 * output_queue is unused-- the resulting bundle (IMU + stereo, called a
 *          StereoImuSyncPacket) is published via registerVioPipelineCallback.
 * @author Antoni Rosinol
 */

 #pragma once

 #include <functional>
 #include <string>
 #include <utility>  // for move
 
 #include "kimera-vio/dataprovider/MonoDataProviderModule.h"
 #include "kimera-vio/frontend/StereoImuSyncPacket.h"
 #include "kimera-vio/frontend/StereoMatchingParams.h"
 #include "kimera-vio/pipeline/Pipeline-definitions.h"
 #include "kimera-vio/pipeline/PipelineModule.h"
 #include "kimera-vio/utils/Macros.h"
 #include "kimera-vio/frontend/Gnss.h"
 
 namespace VIO {
 
 class GnssStereoDataProviderModule : public MonoDataProviderModule {
  public:
   KIMERA_DELETE_COPY_CONSTRUCTORS(GnssStereoDataProviderModule);
   KIMERA_POINTER_TYPEDEFS(GnssStereoDataProviderModule);
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 
   GnssStereoDataProviderModule(OutputQueue* output_queue,
                            const std::string& name_id,
                            const bool& parallel_run,
                            const StereoMatchingParams& stereo_matching_params);
 
   ~GnssStereoDataProviderModule() override = default;
 
   inline OutputUniquePtr spinOnce(InputUniquePtr input) override {
     // Called by spin(), which also calls getInputPacket().
     // Data provider syncs and publishes input sensor information, which
     // is done at the level of getInputPacket. No other action needed.
     return input;
   }
 
   //! Callbacks to fill queues: they should be all lighting fast.
   inline void fillRightFrameQueue(Frame::UniquePtr right_frame) {
     CHECK(right_frame);
     right_frame_queue_.push(std::move(right_frame));
   }
   inline void fillRightFrameQueueBlockingIfFull(Frame::UniquePtr right_frame) {
     CHECK(right_frame);
     right_frame_queue_.pushBlockingIfFull(std::move(right_frame), 5u);
   }

   inline void fillGnssQueue(Gnss::UniquePtr gnss_data) {
      CHECK(gnss_data);
      gnss_queue_.push(std::move(gnss_data));
    }
 
  protected:
   //! The data synchronization function
   InputUniquePtr getInputPacket() override;
 
   std::atomic_bool send_packet_ = {true};
 
   //! Called when general shutdown of PipelineModule is triggered.
   void shutdownQueues() override;
 
   //! Checks if the module has work to do (should check input queues are empty)
   inline bool hasWork() const override {
     return MonoDataProviderModule::hasWork() || !right_frame_queue_.empty();
   }
 
  private:
   //! Input data
   ThreadsafeQueue<Frame::UniquePtr> right_frame_queue_;

   ThreadsafeQueue<Gnss::UniquePtr> gnss_queue_;
   // TODO(Toni): remove these below
   StereoMatchingParams stereo_matching_params_;
 };
 
 }  // namespace VIO
 

// /* ----------------------------------------------------------------------------
//  * Copyright 2017, Massachusetts Institute of Technology,
//  * Cambridge, MA 02139
//  * All Rights Reserved
//  * Authors: Luca Carlone, et al. (see THANKS for the full author list)
//  * See LICENSE for the license information
//  * -------------------------------------------------------------------------- */

// /**
//  * @file   StereoDataProviderModule.h
//  * @brief  Pipeline module that provides stereo data to the VIO pipeline.
//  * @details Collects camera and IMU data, publishes StereoFrames via callback
//  *          getInputPacket processes one stereo pair at a time, attempting to
//  *          gather IMU data between the current stereo pair and the previous
//  *          stereo pair.
//  * output_queue is unused-- the resulting bundle (IMU + stereo, called a
//  *          StereoImuSyncPacket) is published via registerVioPipelineCallback.
//  * @author Antoni Rosinol
//  */

// #pragma once

// #include <functional>
// #include <string>
// #include <utility>  // for move

// #include "kimera-vio/dataprovider/StereoDataProviderModule.h"
// #include "kimera-vio/frontend/Gnss.h"
// #include "kimera-vio/frontend/StereoImuSyncPacket.h"
// #include "kimera-vio/frontend/StereoMatchingParams.h"
// #include "kimera-vio/pipeline/Pipeline-definitions.h"
// #include "kimera-vio/pipeline/PipelineModule.h"
// #include "kimera-vio/utils/Macros.h"

// namespace VIO {

// class GnssStereoDataProviderModule : public MonoDataProviderModule {
//  public:
//   KIMERA_DELETE_COPY_CONSTRUCTORS(GnssStereoDataProviderModule);
//   KIMERA_POINTER_TYPEDEFS(GnssStereoDataProviderModule);
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//   GnssStereoDataProviderModule(
//       OutputQueue* output_queue,
//       const std::string& name_id,
//       const bool& parallel_run,
//       const StereoMatchingParams& stereo_matching_params);

//   ~GnssStereoDataProviderModule() override = default;

//   inline OutputUniquePtr spinOnce(InputUniquePtr input) override {
//     LOG(INFO) << "CALL GNSS SPINONCE";
//     // Called by spin(), which also calls getInputPacket().
//     // Data provider syncs and publishes input sensor information, which
//     // is done at the level of getInputPacket. No other action needed.
//     return input;
//   }

//   inline void fillRightFrameQueue(Frame::UniquePtr right_frame) {
//     CHECK(right_frame);
//     right_frame_queue_.push(std::move(right_frame));
//   }
//   inline void fillRightFrameQueueBlockingIfFull(Frame::UniquePtr right_frame) {
//     CHECK(right_frame);
//     right_frame_queue_.pushBlockingIfFull(std::move(right_frame), 5u);
//   }

//   inline void setGnssTimeShift(const double& gnss_time_shift_s) {
//     gnss_time_shift_ns_ = UtilsNumerical::SecToNsec(gnss_time_shift_s);
//   }

//   // inline void doCoarseGnssCameraTemporalSync() {
//   //   do_coarse_gnss_camera_temporal_sync_ = true;
//   // }

//   //! Callbacks to fill queues: they should be all lighting fast.
//   // inline void fillGnssQueue(Gnss::UniquePtr gnss_data) {
//   //   CHECK(gnss_data);
//   //   gnss_queue_.push(std::move(gnss_data));
//   // }

//  protected:
//   std::atomic<Timestamp> gnss_time_shift_ns_;
//   //! The data synchronization function
//   InputUniquePtr getInputPacket() override;

//   //! Called when general shutdown of PipelineModule is triggered.
//   void shutdownQueues() override;

//   std::atomic_bool send_packet_ = {true};

//   //! Checks if the module has work to do (should check input queues are empty)
//   inline bool hasWork() const override {
//     return MonoDataProviderModule::hasWork() || !right_frame_queue_.empty();
//   }

//  private:
//   //! Input data
//   ThreadsafeQueue<Gnss::UniquePtr> gnss_queue_;
//   //! Input data
//   ThreadsafeQueue<Frame::UniquePtr> right_frame_queue_;
//   // TODO(Toni): remove these below
//   StereoMatchingParams stereo_matching_params_;
// };

// }  // namespace VIO
