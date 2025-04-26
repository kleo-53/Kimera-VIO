/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoImuPipeline.h
 * @brief  Implements StereoVIO pipeline workflow.
 * @author Antoni Rosinol
 * @author Marcus Abate
 */

 #pragma once

 #include "kimera-vio/dataprovider/GnssStereoDataProviderModule.h"
 #include "kimera-vio/frontend/StereoCamera.h"
 #include "kimera-vio/pipeline/Pipeline.h"
 
 namespace VIO {
 
 class GnssStereoImuPipeline : public Pipeline {
  public:
   KIMERA_POINTER_TYPEDEFS(GnssStereoImuPipeline);
   KIMERA_DELETE_COPY_CONSTRUCTORS(GnssStereoImuPipeline);
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 
   /**
      * @brief StereoImuPipeline
      * @param params Vio parameters
      * @param visualizer Optional visualizer for visualizing 3D results
      * @param displayer Optional displayer for visualizing 2D results
      */
   GnssStereoImuPipeline(const VioParams& params,
                     Visualizer3D::UniquePtr&& visualizer = nullptr,
                     DisplayBase::UniquePtr&& displayer = nullptr,
                     PreloadedVocab::Ptr&& preloaded_vocab = nullptr);
 
   ~GnssStereoImuPipeline() = default;
 
  public:
   inline void fillRightFrameQueue(Frame::UniquePtr right_frame) {
     CHECK(data_provider_module_);
     CHECK(right_frame);
     LOG(INFO) << ">>> Got RIGHT frame: " << right_frame->timestamp_;
     // TODO(nathan) this is ugly
     dynamic_cast<GnssStereoDataProviderModule*>(data_provider_module_.get())
         ->fillRightFrameQueue(std::move(right_frame));
   }
 
   inline void fillRightFrameQueueBlockingIfFull(Frame::UniquePtr right_frame) {
     CHECK(data_provider_module_);
     CHECK(right_frame);
 
     // TODO(nathan) this is ugly
     dynamic_cast<GnssStereoDataProviderModule*>(data_provider_module_.get())
         ->fillRightFrameQueueBlockingIfFull(std::move(right_frame));
   }
 
  protected:
   //! Definition of sensor rig used
   StereoCamera::ConstPtr stereo_camera_;
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
//  * @file   GnssStereoImuPipeline.h
//  * @brief  Implements Gnss StereoVIO pipeline workflow.
//  * @author Igor Lovets
//  */

// #pragma once

// #include "kimera-vio/dataprovider/GnssStereoDataProviderModule.h"
// //#include "kimera-vio/frontend/StereoCamera.h"
// //#include "kimera-vio/pipeline/Pipeline.h"
// #include "kimera-vio/pipeline/Pipeline.h"
// #include "kimera-vio/loopclosure/LoopClosureDetector.h"
// #include "kimera-vio/frontend/StereoCamera.h"

// namespace VIO {

// class GnssStereoImuPipeline : public Pipeline {
//  public:
//   KIMERA_POINTER_TYPEDEFS(GnssStereoImuPipeline);
//   KIMERA_DELETE_COPY_CONSTRUCTORS(GnssStereoImuPipeline);
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//   /**
//    * @brief StereoImuPipeline
//    * @param params Vio parameters
//    * @param visualizer Optional visualizer for visualizing 3D results
//    * @param displayer Optional displayer for visualizing 2D results
//    */
//   GnssStereoImuPipeline(const VioParams& params,
//                         Visualizer3D::UniquePtr&& visualizer = nullptr,
//                         DisplayBase::UniquePtr&& displayer = nullptr,
//                         PreloadedVocab::Ptr&& preloaded_vocab = nullptr);

//   ~GnssStereoImuPipeline() = default;

//   public:
//   inline void fillRightFrameQueue(Frame::UniquePtr right_frame) {
//     CHECK(data_provider_module_);
//     CHECK(right_frame);

//     // TODO(nathan) this is ugly
//     dynamic_cast<GnssStereoDataProviderModule*>(data_provider_module_.get())
//         ->fillRightFrameQueue(std::move(right_frame));
//   }

//   inline void fillRightFrameQueueBlockingIfFull(Frame::UniquePtr right_frame) {
//     CHECK(data_provider_module_);
//     CHECK(right_frame);

//     // TODO(nathan) this is ugly
//     dynamic_cast<GnssStereoDataProviderModule*>(data_provider_module_.get())
//         ->fillRightFrameQueueBlockingIfFull(std::move(right_frame));
//   }


//  protected:
//   //! Definition of sensor rig used
//   StereoCamera::ConstPtr stereo_camera_;
// };

// }  // namespace VIO
