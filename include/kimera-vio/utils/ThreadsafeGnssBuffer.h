// // // #pragma once

// // // #include <algorithm>
// // // #include <atomic>
// // // #include <condition_variable>
// // // #include <memory>
// // // #include <mutex>
// // // #include <utility>

// // // #include <functional>
// // // #include <map>


// // // #include "kimera-vio/frontend/GnssStereoVisionImuFrontend-definitions.h"
// // // #include "kimera-vio/common/vio_types.h"
// // // #include "kimera-vio/utils/ThreadsafeTemporalBuffer.h"

// // // namespace VIO {

  
// // // namespace utils {
// // // using GnssStampS = Eigen::Matrix<Timestamp, 1, Eigen::Dynamic>;
// // // using GnssPoseS = Eigen::Matrix<double, 3, Eigen::Dynamic>;

// // // class ThreadsafeGnssBuffer {
// // //  public:
// // //   KIMERA_POINTER_TYPEDEFS(ThreadsafeGnssBuffer);


// // //   explicit ThreadsafeGnssBuffer(const Timestamp& buffer_length_ns)
// // //       : buffer_(buffer_length_ns), shutdown_(false) {}

// // //   ~ThreadsafeGnssBuffer() { shutdown(); }

// // //   void shutdown() {
// //   //     shutdown_ = true;
// //   //     cv_new_measurement_.notify_all();
// //   //   }
  
// //   //   size_t size() const { return buffer_.size(); }

// //   //   void clear() { buffer_.clear(); }
  
// //   //   inline void addMeasurement(const Timestamp& timestamp_ns, const VIO::GnssMeasurement& gnss_measurement);
  
// //   //   inline void addMeasurements(const GnssStampS& timestamps_nanoseconds,
// //   //     const GnssPoseS& gnss_measurements);
  
// //   //   bool getNewestGnssMeasurement(VIO::GnssMeasurement* value) {
// //     //     CHECK_NOTNULL(value);
// //     //     return buffer_.getNewestValue(value);
// //     //   }
    
// //     //   QueryResult getMeasurementAtTimestamp(const Timestamp& timestamp_ns,
// //     //                                         VIO::GnssMeasurement* result) const {
// // //     return buffer_.getValueAtTime(timestamp_ns, result)
// // //                ? QueryResult::kDataAvailable
// // //                : QueryResult::kDataNeverAvailable;
// // //               //  : isDataAvailableImpl(timestamp_ns);
// // //   }

// // //   QueryResult getDataBtwTimestamps(const Timestamp& timestamp_from,
// // //                                    const Timestamp& timestamp_to,
// // //                                    std::vector<VIO::GnssMeasurement>* measurements) {
// // //     return buffer_.getValuesBetweenTimes(
// //   //                timestamp_from, timestamp_to, &measurements, false)
// // //                ? QueryResult::kDataAvailable
// // //                : QueryResult::kDataNeverAvailable;
// // //               //  : isDataAvailableImpl(timestamp_to);
// // //     }

// // //     // QueryResult isDataAvailableImpl(const Timestamp& timestamp) const {
// //   //     //   if (shutdown_) return QueryResult::kQueueShutdown;
  
// //   //     //   if (!buffer_.hasDataAtOrAfterTimestamp(timestamp)) {
// //     //     //     return QueryResult::kDataNotYetAvailable;
// //     //     //   }
    
// //     //     //   if (!buffer_.hasDataAtOrBeforeTimestamp(timestamp)) {
// //       //     //     return QueryResult::kDataNeverAvailable;
// //       //     //   }
      
// //       //     // return QueryResult::kDataAvailable;
// //       //   // }
      
// //       //   private:
// //       //     typedef std::pair<const Timestamp, VIO::GnssMeasurement> BufferElement;
// //       //     typedef Eigen::aligned_allocator<BufferElement> BufferAllocator;
// //       //     typedef ThreadsafeTemporalBuffer<VIO::GnssMeasurement, BufferAllocator> Buffer;
      
// //       //     Buffer buffer_;
// //       //     mutable std::mutex m_buffer_;
// //       //     std::condition_variable cv_new_measurement_;
// //       //     std::atomic<bool> shutdown_;
// //       //   };
// //       // }
// //       // }  // namespace VIO
      
      
// //       // #include "./ThreadsafeGnssBuffer-inl.h"
      
      
// //       // kimera-vio/utils/ThreadsafeGnssBuffer.h
// //   #pragma once
      
// // #include <vector>
// // #include <utility>
// // #include <memory>

// // #include "kimera-vio/frontend/GnssTypes.h"
// // #include "kimera-vio/utils/ThreadsafeTemporalBuffer.h"
// // #include "kimera-vio/common/vio_types.h"

// // namespace VIO {
// // namespace utils {
  
// //   class ThreadsafeGnssBuffer {
// //     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// //     public:
// //   enum class QueryResult {
// //     kDataAvailable,
// //     kDataNotYetAvailable,
// //     kDataNeverAvailable,
// //     kQueueShutdown,
// //     kTooFewMeasurementsAvailable
// //     };
// //     typedef std::pair<const Timestamp, GnssMeasurement> BufferElement;
// //     typedef std::allocator<BufferElement> BufferAllocator;
// //     typedef ThreadsafeTemporalBuffer<GnssMeasurement, BufferAllocator> Buffer;

// //   explicit ThreadsafeGnssBuffer(int buffer_size = -1)
// //       : buffer_(buffer_size) {}

// //   size_t size() const { return buffer_.size(); }
// //   void clear() { buffer_.clear(); }

// //   void addMeasurement(const Timestamp& timestamp_ns, 
// //                      const GnssMeasurement& gnss_measurement);
  
// //   void addMeasurements(const GnssStampS& timestamps_nanoseconds,
// //                       const GnssPoseS& gnss_measurements);

// //   bool getNewestGnssMeasurement(GnssMeasurement* value);
  
// //   QueryResult getMeasurementAtTimestamp(const Timestamp& timestamp_ns,
// //                                       GnssMeasurement* result) const;
  
// //   QueryResult getDataBtwTimestamps(const Timestamp& timestamp_from,
// //                                  const Timestamp& timestamp_to,
// //                                  std::vector<GnssMeasurement>* measurements);

// //  private:
// //   Buffer buffer_;
// // };

// // }  // namespace utils
// // }  // namespace VIO

// /* ----------------------------------------------------------------------------
//  * Copyright 2017, Massachusetts Institute of Technology,
//  * Cambridge, MA 02139
//  * All Rights Reserved
//  * Authors: Luca Carlone, et al. (see THANKS for the full author list)
//  * See LICENSE for the license information
//  * -------------------------------------------------------------------------- */

// /**
//  * @file   ThreadsafeOdometryBuffer.h
//  * @brief  Threadsafe buffer to store additional odometry measurements
//  * @author Nathan Hughes
//  */

// //  #include <gtsam/navigation/NavState.h>

// #include "kimera-vio/common/vio_types.h"
//  #include "kimera-vio/utils/ThreadsafeTemporalBuffer.h"
//  #include "kimera-vio/utils/Macros.h"
//  #include "kimera-vio/frontend/GnssTypes.h"
 
//  namespace VIO {
 
// namespace utils {

//  class ThreadsafeGnssBuffer {
//   public:
//    KIMERA_POINTER_TYPEDEFS(ThreadsafeGnssBuffer);
 
//    enum class QueryResult {
//      kDataNotYetAvailable,
//      kDataNeverAvailable,
//      kDataAvailable,
//      kQueueShutdown,
//      kTooFewMeasurementsAvailable,
//    };
 
//    explicit ThreadsafeGnssBuffer(const Timestamp& buffer_length_ns)
//       : buffer_(buffer_length_ns), shutdown_(false) {}

//   ~ThreadsafeGnssBuffer() { shutdown(); }

//   /// Shutdown the queue and release all blocked waiters.
//   inline void shutdown();
//   inline size_t size() const;
//   inline void clear();

//   //  explicit ThreadsafeGnssBuffer(const Timestamp& buffer_length_ns);
 
//   //  void add(const Timestamp& time, const GnssMeasurement& gnss_meas);
 
//    ThreadsafeGnssBuffer::QueryResult getNearest(const Timestamp& timestamp); //,
//                                                     // GnssMeasurement* gnss_meas);

//   inline void addMeasurement(const Timestamp& timestamp_nanoseconds,
//     const GnssMeasurement& gnss_measurement);
//   inline void addMeasurements(const GnssStampS& timestamps_nanoseconds,
//       const GnssPoseS& gnss_measurements);
//   typedef std::pair<const Timestamp, GnssMeasurement> BufferElement;
//   typedef Eigen::aligned_allocator<BufferElement> BufferAllocator;
//   typedef ThreadsafeTemporalBuffer<GnssMeasurement, BufferAllocator> Buffer;
  
//   bool getNewestGnssMeasurement(GnssMeasurement* value);

//   private:
//   struct Gnss {
//     Timestamp timestamp;
//     const gtsam::Vector3 value;
//   };
//   Buffer buffer_;
  
//   std::condition_variable cv_new_measurement_;
//   // utils::ThreadsafeTemporalBuffer<Gnss> buffer_;
//    std::atomic<bool> shutdown_;
//  };
 
//  }  // namespace VIO
 
// }

// #include "kimera-vio/utils/ThreadsafeGnssBuffer-inl.h"

/********************************************************************************
 Copyright 2017 Autonomous Systems Lab, ETH Zurich, Switzerland

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*********************************************************************************/

/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ThreadsafeGnssBuffer.h
 * @brief  Threadsafe Imu Buffer with timestamp lookup.
 * @author Antoni Rosinol
 */

 #pragma once

 #include <algorithm>
 #include <atomic>
 #include <condition_variable>
 #include <memory>
 #include <mutex>
 #include <utility>
 
 #include <functional>
 #include <map>
 
 #include <Eigen/Dense>
 
 #include "kimera-vio/common/vio_types.h"
//  #include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"
 #include "kimera-vio/utils/ThreadsafeTemporalBuffer.h"
 #include "kimera-vio/frontend/GnssTypes.h"
 
 namespace VIO {
 
 namespace utils {
 
 /// \class ThreadsafeGnssBuffer
 /// This buffering class can be used to store a history of Gnss measurements. It
 /// allows to
 /// retrieve a list  of measurements up to a given timestamp. The data is stored
 /// in the order
 /// it is added. So make sure to add it in correct time-wise order.
 class ThreadsafeGnssBuffer {
  public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 
   enum class QueryResult {
     /// Query was successful and the data is available.
     kDataAvailable,
     /// The required data is not (yet) available. The query timestamp is above
     /// the last GNSS sample's time.
     kDataNotYetAvailable,
     /// The queried timestamp lies before the first GNSS sample in the buffer.
     /// This request will never succeed considering chronological ordering
     /// of the buffer input data.
     kDataNeverAvailable,
     /// Queue shutdown.
     kQueueShutdown,
     kTooFewMeasurementsAvailable
   };
 
   explicit ThreadsafeGnssBuffer(const Timestamp& buffer_length_ns)
       : buffer_(buffer_length_ns), shutdown_(false) {}
 
   ~ThreadsafeGnssBuffer() { shutdown(); }
 
   /// Shutdown the queue and release all blocked waiters.
   inline void shutdown();
   inline size_t size() const;
   inline void clear();
 
   /// Add GNSS measurement in GNSS frame.
   /// (Ordering: accelerations [m/s^2], angular velocities [rad/s])
   inline void addMeasurement(const Timestamp& timestamp_nanoseconds,
                              const GnssPose& gnss_measurement);
   inline void addMeasurements(const GnssStampS& timestamps_nanoseconds,
                               const GnssPoseS& gnss_measurements);
 
   // Get Gnss data strictly between Timestamps.
   // Example: content: 2 3 4 5
   //      getGnssDataStrictlyBtwTiemstamps(2, 5, ...) returns elements at 3, 4.
   // Alternatively, the user might ask for the lower bound as well.
   //      getGnssDataStrictlyBtwTiemstamps(2, 5, ..., true)
   // returns elements at 2, 3, 4.
   // by setting the parameter get_lower_bound to true.
   QueryResult getGnssDataBtwTimestamps(const Timestamp& timestamp_ns_from,
                                       const Timestamp& timestamp_ns_to,
                                       GnssStampS* gnss_timestamps,
                                       GnssPoseS* gnss_measurements,
                                       bool get_lower_bound = false);
 
   /// \brief Return a list of the Gnss measurements between the specified
   /// timestamps. The Gnss values get interpolated if the queried timestamp does
   /// not match a measurement.
   /// @param[in] timestamp_from Try to get the Gnss measurements from this time
   /// [ns].
   /// @param[in] timestamp_to Try to get the Gnss measurements up this time [ns].
   /// @param[out] gnss_timestamps List of timestamps. [ns]
   /// @param[out] gnss_measurements List of Gnss measurements. (Order: [acc,
   /// gyro])
   /// @return The return code signals if the buffer does not contain data
   /// up to the requested timestamp.
   /// In this case the output matrices will be of size 0.
   QueryResult getGnssDataInterpolatedBorders(const Timestamp& timestamp_from,
                                             const Timestamp& timestamp_to,
                                             GnssStampS* gnss_timestamps,
                                             GnssPoseS* gnss_measurements);
 
   /// \brief Return a list of the Gnss measurements between the specified
   /// timestamps. ONLY the Gnss newest value gets interpolated (if the queried
   /// timestamp does not match a measurement, otw no interpolation).
   /// The oldest Gnss value is always an actual measurement
   /// (aka no interpolation for the lower border, ever).
   /// @param[in] timestamp_from Get the Gnss measurements from this time
   /// [ns].
   /// @param[in] timestamp_to Try to get the Gnss measurements up this time [ns].
   /// @param[out] gnss_timestamps List of timestamps. [ns]
   /// @param[out] gnss_measurements List of Gnss measurements. (Order: [acc,
   /// gyro])
   /// @return The return code signals if the buffer does not contain data
   /// surrounding the requested timestamps
   /// (check function isDataAvailableUpToImpl).
   /// In this case the output matrices will be of size 0.
   QueryResult getGnssDataInterpolatedUpperBorder(
       const Timestamp& timestamp_ns_from,
       const Timestamp& timestamp_ns_to,
       GnssStampS* gnss_timestamps,
       GnssPoseS* gnss_measurements);
 
   // Interpolates an Gnss measurement at timestamp by taking previous and
   // posterior measurements to the given timestamp.
   // WARNING: the user must make sure the buffer has enough elements.
   // This can be done be checked using the isDataAvailableUpToImpl function.
   void interpolateValueAtTimestamp(const Timestamp& timestamp_ns,
                                    GnssPose* interpolated_gnss_measurement);
 
   /// Try to pop the requested Gnss measurements for the duration of
   /// wait_timeout_nanoseconds.
   /// If the requested data is still not available when timeout has been
   /// reached, the method
   /// will return false and no data will be removed from the buffer.
   QueryResult getGnssDataInterpolatedBordersBlocking(
       const Timestamp& timestamp_ns_from,
       const Timestamp& timestamp_ns_to,
       const Timestamp& wait_timeout_nanoseconds,
       GnssStampS* gnss_timestamps,
       GnssPoseS* gnss_measurements);
 
   /// Get the most recently pushed Gnss measurement
   /// Returns true unless the buffer is empty
   bool getNewestGnssMeasurement(GnssMeasurement* value);
 
   /// Linear interpolation between two gnss measurements.
   static void linearInterpolate(const Timestamp& x0,
                                 const GnssPose& y0,
                                 const Timestamp& x1,
                                 const GnssPose& y1,
                                 const Timestamp& x,
                                 GnssPose* y);
 
  private:
   /// TODO I think this comment is deprecated:
   /// Is data available up to this timestamp? Note this function does not lock
   /// the buffers, the caller must hold the lock.
   /// TODO I believe it should be:
   /// a) checks that the requested interval of data is a subset of the data in
   ///  the Gnss buffer, returns:
   ///       Ex. Gnss buffer holds timestamps 2 3 4 5, then
   ///       i) Query timestamp_ns_to > 5 (e.g. 7)
   ///           , returns kDataNotYetAvailable.
   ///       ii) Query from timestamp < 2 (e.g. 0) or timestamp to < 2 (e.g. 1)
   ///           , returns kDataNeverAvailable.
   ///       iii) Query from timestamp 3 to 4, returns kDataAvailable.
   /// And I don't think there is a need to lock any mutex, as buffer_ is already
   /// threadsafe...
   QueryResult isDataAvailableUpToImpl(const Timestamp& timestamp_ns_from,
                                       const Timestamp& timestamp_ns_to) const;
 
   typedef std::pair<const Timestamp, GnssMeasurement> BufferElement;
   typedef Eigen::aligned_allocator<BufferElement> BufferAllocator;
   typedef ThreadsafeTemporalBuffer<GnssMeasurement, BufferAllocator> Buffer;
 
   Buffer buffer_;
   mutable std::mutex m_buffer_;
   std::condition_variable cv_new_measurement_;
   std::atomic<bool> shutdown_;
   std::atomic<bool> have_warned_user_;
 };
 
 }  // namespace utils
 
 }  // namespace VIO
 
 #include "./ThreadsafeGnssBuffer-inl.h"
 