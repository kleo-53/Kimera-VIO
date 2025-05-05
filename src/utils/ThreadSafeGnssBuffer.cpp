// /* ----------------------------------------------------------------------------
//  * Copyright 2017, Massachusetts Institute of Technology,
//  * Cambridge, MA 02139
//  * All Rights Reserved
//  * Authors: Luca Carlone, et al. (see THANKS for the full author list)
//  * See LICENSE for the license information
//  * -------------------------------------------------------------------------- */

// /**
//  * @file   ThreadsafeOdometryBuffer.cpp
//  * @brief  Threadsafe buffer to store additional odometry measurements
//  * @author Nathan Hughes
//  */

//  #include "kimera-vio/utils/ThreadsafeGnssBuffer.h"
//  #include "kimera-vio/utils/UtilsNumerical.h"
// //  #include "kimera-vio/frontend/Gnss.h"
 
//  namespace VIO {
    
// namespace utils {
 
// //  ThreadsafeGnssBuffer::ThreadsafeGnssBuffer(const Timestamp& buffer_length_ns)
// //      : shutdown_(false), buffer_(buffer_length_ns) {}
 
// //  void ThreadsafeGnssBuffer::add(const Timestamp& time,
// //                                     const GnssMeasurement& gnss_meas) {
// //    Gnss to_add;
// //    to_add.timestamp_ = time;
// //    to_add.value = gnss_meas;
// //    buffer_.addValue(time, to_add);
// //  }

//  bool ThreadsafeGnssBuffer::getNewestGnssMeasurement(GnssMeasurement* value) {
//     CHECK_NOTNULL(value);
//     return buffer_.getNewestValue(value);
//   }
 
//  ThreadsafeGnssBuffer::QueryResult ThreadsafeGnssBuffer::getNearest(
//      const Timestamp& timestamp){  //, GnssMeasurement* /) {
//     //  gtsam::NavState* odometry) {
//    if (buffer_.empty()) {
//      return QueryResult::kDataNotYetAvailable;
//    }
 
//    GnssMeasurement oldest;
//    CHECK(buffer_.getOldestValue(&oldest))
//        << "odomety buffer failed to get oldest value";
//    if (oldest.timestamp_ > timestamp) {
//      return QueryResult::kDataNeverAvailable;
//    }
 
//    GnssMeasurement newest;
//    CHECK(buffer_.getNewestValue(&newest))
//        << "odometry buffer failed to get newest value";
 
//    if (newest.timestamp_ < timestamp) {
//      return QueryResult::kDataNotYetAvailable;
//    }
 
//    GnssMeasurement m_nearest;
//    buffer_.getNearestValueToTime(timestamp, &m_nearest);
//    VLOG(10) << "Found odom measurement with t="
//             << UtilsNumerical::NsecToSec(m_nearest.timestamp_) << " which is "
//             << UtilsNumerical::NsecToSec(m_nearest.timestamp_ - timestamp)
//             << " seconds away from t=" << UtilsNumerical::NsecToSec(timestamp);
 
// //    *odometry = m_nearest.value;
//    return QueryResult::kDataAvailable;
//  }

// } // namespace utils
 
//  }  // namespace VIO
 

/*******************************************************************************
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
*******************************************************************************/

/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ThreadsafeGnssBuffer.cpp
 * @brief  Threadsafe Gnss Buffer with timestamp lookup.
 * @author Antoni Rosinol
 */

 #include <chrono>
 #include <iostream>
 
 #include <algorithm>
 
 #include <gflags/gflags.h>
 #include <glog/logging.h>
 
 #include "kimera-vio/utils/ThreadsafeGnssBuffer.h"
 #include "kimera-vio/utils/Timer.h"
 
 namespace VIO {
 
 namespace utils {
 
 template <template <typename, typename> class Container, typename Type>
 using Aligned = Container<Type, Eigen::aligned_allocator<Type>>;
 
 ThreadsafeGnssBuffer::QueryResult ThreadsafeGnssBuffer::isDataAvailableUpToImpl(
     const Timestamp& timestamp_ns_from,
     const Timestamp& timestamp_ns_to) const {
   CHECK_LT(timestamp_ns_from, timestamp_ns_to);
 
   if (shutdown_) {
     return QueryResult::kQueueShutdown;
   }
 
   if (buffer_.empty()) {
     return QueryResult::kDataNotYetAvailable;
   }
 
   GnssMeasurement value;
   if (buffer_.getNewestValue(&value) && value.timestamp_ < timestamp_ns_to) {
     // This is triggered if the timestamp_ns_to requested exceeds the newest
     // IMU measurement, meaning that there is data still to arrive to reach the
     // requested point in time.
     return QueryResult::kDataNotYetAvailable;
   }
 
   if (buffer_.getOldestValue(&value) && timestamp_ns_from < value.timestamp_) {
     // This is triggered if the user requests data previous to the oldest IMU
     // measurement present in the buffer, meaning that there is missing data
     // from the timestamp_ns_from requested to the oldest stored timestamp.
     return QueryResult::kDataNeverAvailable;
   }
   return QueryResult::kDataAvailable;
 }
 
 bool ThreadsafeGnssBuffer::getNewestGnssMeasurement(GnssMeasurement* value) {
   CHECK_NOTNULL(value);
   return buffer_.getNewestValue(value);
 }
 
 void ThreadsafeGnssBuffer::linearInterpolate(const Timestamp& t0,
                                             const GnssPose& y0,
                                             const Timestamp& t1,
                                             const GnssPose& y1,
                                             const Timestamp& t,
                                             GnssPose* y) {
   CHECK_NOTNULL(y);
   CHECK_LE(t0, t);
   CHECK_LE(t, t1);
   *y = t0 == t1 ? y0 :  // y0 if t0 == t1, interpolate otherwise:
            y0 + (y1 - y0) * static_cast<double>(t - t0) /
                     static_cast<double>(t1 - t0);
 }
 
 ThreadsafeGnssBuffer::QueryResult ThreadsafeGnssBuffer::getGnssDataBtwTimestamps(
     const Timestamp& timestamp_ns_from,
     const Timestamp& timestamp_ns_to,
     GnssStampS* gnss_timestamps,
     GnssPoseS* gnss_measurements,
     bool get_lower_bound) {
   CHECK_NOTNULL(gnss_timestamps);
   CHECK_NOTNULL(gnss_measurements);
   DCHECK_LT(timestamp_ns_from, timestamp_ns_to);
   QueryResult query_result =
       isDataAvailableUpToImpl(timestamp_ns_from, timestamp_ns_to);
   if (query_result != QueryResult::kDataAvailable) {
     gnss_timestamps->resize(Eigen::NoChange, 0);
     gnss_measurements->resize(Eigen::NoChange, 0);
     return query_result;
   }
 
   // TODO can we avoid copying and have the temporal buffer store pairs of
   // timestamps/accgyr data?
   // Copy the data with timestamp_up_to <= timestamps_buffer from the buffer.
   Aligned<std::vector, GnssMeasurement> between_values;
   CHECK(buffer_.getValuesBetweenTimes(timestamp_ns_from, timestamp_ns_to,
                                       &between_values, get_lower_bound));
 
   if (between_values.empty()) {
     LOG(WARNING) << "No GNSS measurements available strictly between time "
                  << timestamp_ns_from << "[ns] and " << timestamp_ns_to
                  << "[ns].";
     gnss_timestamps->resize(Eigen::NoChange, 0);
     gnss_measurements->resize(Eigen::NoChange, 0);
     return QueryResult::kTooFewMeasurementsAvailable;
   }
 
   const size_t num_measurements = between_values.size();
   gnss_timestamps->resize(Eigen::NoChange, num_measurements);
   gnss_measurements->resize(Eigen::NoChange, num_measurements);
 
   for (size_t idx = 0u; idx < num_measurements; ++idx) {
     (*gnss_timestamps)(idx) = between_values[idx].timestamp_;
     (*gnss_measurements).col(idx) = between_values[idx].pose_;
   }
 
   return query_result;
 }
 
 ThreadsafeGnssBuffer::QueryResult
 ThreadsafeGnssBuffer::getGnssDataInterpolatedUpperBorder(
     const Timestamp& timestamp_ns_from,
     const Timestamp& timestamp_ns_to,
     GnssStampS* gnss_timestamps,
     GnssPoseS* gnss_measurements) {
   CHECK_NOTNULL(gnss_timestamps);
   CHECK_NOTNULL(gnss_measurements);
   DCHECK_LT(timestamp_ns_from, timestamp_ns_to);
   // Get data.
   QueryResult query_result = getGnssDataBtwTimestamps(
       timestamp_ns_from, timestamp_ns_to, gnss_timestamps, gnss_measurements,
       true);  // Get lower bound.
   // Early exit if there is no data.
   if (query_result != QueryResult::kDataAvailable) {
     gnss_timestamps->resize(Eigen::NoChange, 0);
     gnss_measurements->resize(Eigen::NoChange, 0);
     return query_result;
   }
 
   // Interpolate upper border.
   GnssPose interpolated_upper_border;
   interpolateValueAtTimestamp(timestamp_ns_to, &interpolated_upper_border);
 
   DCHECK_EQ(gnss_timestamps->rows(), 1);
   DCHECK_EQ(gnss_measurements->rows(), 6);
   // The last measurement will correspond to the interpolated data.
   const size_t num_measurements = gnss_timestamps->cols() + 1u;
   gnss_timestamps->conservativeResize(Eigen::NoChange, num_measurements);
   gnss_measurements->conservativeResize(Eigen::NoChange, num_measurements);
   // Append upper border.
   gnss_timestamps->rightCols<1>()(0) = timestamp_ns_to;
   gnss_measurements->rightCols<1>() = interpolated_upper_border;
 
   return query_result;
 }
 
 ThreadsafeGnssBuffer::QueryResult
 ThreadsafeGnssBuffer::getGnssDataInterpolatedBorders(
     const Timestamp& timestamp_ns_from,
     const Timestamp& timestamp_ns_to,
     GnssStampS* gnss_timestamps,
     GnssPoseS* gnss_measurements) {
   CHECK_NOTNULL(gnss_timestamps);
   CHECK_NOTNULL(gnss_measurements);
   // Get data.
   GnssStampS gnss_timestamps_tmp;
   GnssPoseS gnss_measurements_tmp;
   QueryResult query_result =
       getGnssDataBtwTimestamps(timestamp_ns_from, timestamp_ns_to,
                               &gnss_timestamps_tmp, &gnss_measurements_tmp);
 
   // Early exit if there is no data.
   if (query_result != QueryResult::kDataAvailable) {
     gnss_timestamps->resize(Eigen::NoChange, 0);
     gnss_measurements->resize(Eigen::NoChange, 0);
     return query_result;
   }
   if (gnss_measurements_tmp.cols() == 0) {
    gnss_timestamps->resize(Eigen::NoChange, 0);
    gnss_measurements->resize(Eigen::NoChange, 0);
    return QueryResult::kTooFewMeasurementsAvailable;
  }
 
   // Interpolate lower border.
   GnssPose interpolated_lower_border;
   interpolateValueAtTimestamp(timestamp_ns_from, &interpolated_lower_border);
   // Interpolate upper border.
   GnssPose interpolated_upper_border;
   interpolateValueAtTimestamp(timestamp_ns_to, &interpolated_upper_border);
 
   DCHECK_EQ(gnss_timestamps->rows(), 1);
   DCHECK_EQ(gnss_measurements->rows(), 6);
   // The first and last measurements will correspond to the interpolated data.
   const size_t num_measurements = gnss_timestamps_tmp.cols() + 2u;
   gnss_timestamps->resize(Eigen::NoChange, num_measurements);
   gnss_measurements->resize(Eigen::NoChange, num_measurements);
   // Prepend lower border.
   gnss_timestamps->leftCols<1>()(0) = timestamp_ns_from;
   gnss_measurements->leftCols<1>() = interpolated_lower_border;
   // Add measurements.
   gnss_timestamps->middleCols(1, gnss_timestamps_tmp.cols()) = gnss_timestamps_tmp;
   gnss_measurements->middleCols(1, gnss_measurements_tmp.cols()) =
       gnss_measurements_tmp;
   // Append upper border.
   gnss_timestamps->rightCols<1>()(0) = timestamp_ns_to;
   gnss_measurements->rightCols<1>() = interpolated_upper_border;
 
   return query_result;
 }
 
 void ThreadsafeGnssBuffer::interpolateValueAtTimestamp(
     const Timestamp& timestamp_ns,
     GnssPose* interpolated_gnss_measurement) {
   CHECK_NOTNULL(interpolated_gnss_measurement);
   Timestamp pre_border_timestamp, post_border_timestamp;
   GnssMeasurement pre_border_value, post_border_value;
   CHECK(buffer_.getValueAtOrBeforeTime(timestamp_ns, &pre_border_timestamp,
                                        &pre_border_value))
       << "The GNSS buffer seems not to contain measurements at or before time: "
       << timestamp_ns;
   CHECK_EQ(pre_border_timestamp, pre_border_value.timestamp_);
   CHECK(buffer_.getValueAtOrAfterTime(timestamp_ns, &post_border_timestamp,
                                       &post_border_value))
       << "The GNSS buffer seems not to contain measurements at or after time: "
       << timestamp_ns;
   CHECK_EQ(post_border_timestamp, post_border_value.timestamp_);
   linearInterpolate(pre_border_value.timestamp_,
                     pre_border_value.pose_,
                     post_border_value.timestamp_,
                     post_border_value.pose_,
                     timestamp_ns,
                     interpolated_gnss_measurement);
 }
 
 ThreadsafeGnssBuffer::QueryResult
 ThreadsafeGnssBuffer::getGnssDataInterpolatedBordersBlocking(
     const Timestamp& timestamp_ns_from,
     const Timestamp& timestamp_ns_to,
     const Timestamp& wait_timeout_nanoseconds,
     GnssStampS* gnss_timestamps,
     GnssPoseS* gnss_measurements) {
   CHECK_NOTNULL(gnss_timestamps);
   CHECK_NOTNULL(gnss_measurements);
 
   // Wait for the GNSS buffer to contain the required measurements within a
   // timeout.
   auto tic = Timer::tic();
   QueryResult query_result;
   {
     std::unique_lock<std::mutex> lock(m_buffer_);
     while ((query_result =
                 isDataAvailableUpToImpl(timestamp_ns_from, timestamp_ns_to)) !=
            QueryResult::kDataAvailable) {
       cv_new_measurement_.wait_for(
           lock, std::chrono::nanoseconds(wait_timeout_nanoseconds));
 
       if (shutdown_) {
         gnss_timestamps->resize(Eigen::NoChange, 0);
         gnss_measurements->resize(Eigen::NoChange, 0);
         return QueryResult::kQueueShutdown;
       }
 
       // Check if we hit the max. time allowed to wait for the required data.
       auto toc = Timer::toc<std::chrono::nanoseconds>(tic);
       if (toc.count() >= wait_timeout_nanoseconds) {
         gnss_timestamps->resize(Eigen::NoChange, 0);
         gnss_measurements->resize(Eigen::NoChange, 0);
         LOG(WARNING) << "Timeout reached while trying to get the requested "
                      << "GNSS data. Requested range: " << timestamp_ns_from
                      << " to " << timestamp_ns_to << ".";
         if (query_result == QueryResult::kDataNotYetAvailable) {
           LOG(WARNING) << "The relevant GNSS data is not yet available.";
         } else if (query_result == QueryResult::kDataNeverAvailable) {
           LOG(WARNING) << "The relevant GNSS data will never be available. "
                        << "Either the buffer is too small or a sync issue "
                        << "occurred.";
         } else {
           LOG(FATAL) << "Unknown query result error.";
         }
         return query_result;
       }
     }
   }
   return getGnssDataInterpolatedBorders(timestamp_ns_from, timestamp_ns_to,
                                        gnss_timestamps, gnss_measurements);
 }
 
 }  // namespace utils
 
 }  // namespace VIO
 