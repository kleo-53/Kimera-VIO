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
                                             const GnssPoint& y0,
                                             const Timestamp& t1,
                                             const GnssPoint& y1,
                                             const Timestamp& t,
                                             GnssPoint* y) {
   CHECK_NOTNULL(y);
   CHECK_LE(t0, t);
   CHECK_LE(t, t1);
   *y = t0 == t1 ? y0 :  // y0 if t0 == t1, interpolate otherwise:
            y0 + (y1 - y0) * static_cast<double>(t - t0) /
                     static_cast<double>(t1 - t0);
 }

 ThreadsafeGnssBuffer::QueryResult ThreadsafeGnssBuffer::getInterpolatedValueWithFixedPeriod(
                            const Timestamp& timestamp_ns,
                            const uint64_t gnss_period_ns,
                            GnssStampS* gnss_timestamps,
                            GnssPointS* gnss_points) const {
  CHECK_NOTNULL(gnss_timestamps);
  CHECK_NOTNULL(gnss_points);
  // std::lock_guard<std::mutex> lock(m_buffer_);

  if (buffer_.empty()) {
    LOG(WARNING) << "GNSS buffer is empty.";
    return QueryResult::kTooFewMeasurementsAvailable;
  }

  GnssMeasurement oldest;
   CHECK(buffer_.getOldestValue(&oldest))
       << "Gnss buffer failed to get oldest value";
   if (oldest.timestamp_ > timestamp_ns) {
     return QueryResult::kDataNeverAvailable;
   }
  //  GnssMeasurement newest;
  //  CHECK(buffer_.getNewestValue(&newest))
  //      << "Gnss buffer failed to get newest value";
  //  if (newest.timestamp_ < timestamp) {
  //    return QueryResult::kDataNotYetAvailable;
  //  }
  size_t index = static_cast<size_t>((timestamp_ns - oldest.timestamp_) / gnss_period_ns);
  Timestamp ts_lower = oldest.timestamp_ + index * gnss_period_ns;
  Timestamp ts_upper = ts_lower + gnss_period_ns;
  // GnssMeasurement meas_lower, meas_upper;
  // if (!buffer_.getValueAtTime(ts_lower, &meas_lower) ||
  //     !buffer_.getValueAtTime(ts_upper, &meas_upper)) {
  //   LOG(WARNING) << "Failed to retrieve GNSS points at fixed interval.";
  //   return false;
  // }
   Timestamp pre_border_timestamp, post_border_timestamp;
   GnssMeasurement pre_border_value, post_border_value;
   if (!buffer_.getValueAtOrBeforeTime(ts_lower, &pre_border_timestamp,
                                        &pre_border_value)) {
       LOG(WARNING) << "The GNSS buffer seems not to contain points at or before time: "
       << ts_lower;
       return QueryResult::kTooFewMeasurementsAvailable;
   }
   CHECK_EQ(pre_border_timestamp, pre_border_value.timestamp_);
   if (!buffer_.getValueAtOrAfterTime(ts_upper, &post_border_timestamp,
                                       &post_border_value)) {
      LOG(WARNING) << "The GNSS buffer seems not to contain points at or after time: "
      << ts_upper;
      return QueryResult::kTooFewMeasurementsAvailable;
    }
   CHECK_EQ(post_border_timestamp, post_border_value.timestamp_);
   GnssPoint interpolated_gnss_point;
   linearInterpolate(pre_border_value.timestamp_,
                     pre_border_value.point_,
                     post_border_value.timestamp_,
                     post_border_value.point_,
                     timestamp_ns,
                     &interpolated_gnss_point);

  // linearInterpolate(oldest.timestamp_,
  //                   oldest.point_,
  //                   newest.timestamp_,
  //                   newest.point_,
  //                   timestamp_ns,
  //                   &interpolated);

  gnss_timestamps->resize(Eigen::NoChange, 1u);
  gnss_points->resize(Eigen::NoChange, 1u);
  gnss_timestamps->rightCols<1>()(0) = timestamp_ns;
  gnss_points->rightCols<1>() = interpolated_gnss_point;
  return QueryResult::kDataAvailable;
}
 
 ThreadsafeGnssBuffer::QueryResult ThreadsafeGnssBuffer::getGnssDataBtwTimestamps(
     const Timestamp& timestamp_ns_from,
     const Timestamp& timestamp_ns_to,
     GnssStampS* gnss_timestamps,
     GnssPointS* gnss_points,
     bool get_lower_bound) {
   CHECK_NOTNULL(gnss_timestamps);
   CHECK_NOTNULL(gnss_points);
   DCHECK_LT(timestamp_ns_from, timestamp_ns_to);
   QueryResult query_result =
       isDataAvailableUpToImpl(timestamp_ns_from, timestamp_ns_to);
   if (query_result != QueryResult::kDataAvailable) {
     gnss_timestamps->resize(Eigen::NoChange, 0);
     gnss_points->resize(Eigen::NoChange, 0);
     return query_result;
   }
 
   // TODO can we avoid copying and have the temporal buffer store pairs of
   // timestamps/accgyr data?
   // Copy the data with timestamp_up_to <= timestamps_buffer from the buffer.
   Aligned<std::vector, GnssMeasurement> between_values;
   CHECK(buffer_.getValuesBetweenTimes(timestamp_ns_from, timestamp_ns_to,
                                       &between_values, get_lower_bound));
 
   if (between_values.empty()) {
     LOG(WARNING) << "No GNSS points available strictly between time "
                  << timestamp_ns_from << "[ns] and " << timestamp_ns_to
                  << "[ns].";
     gnss_timestamps->resize(Eigen::NoChange, 0);
     gnss_points->resize(Eigen::NoChange, 0);
     return QueryResult::kTooFewMeasurementsAvailable;
   }
 
   const size_t num_points = between_values.size();
   gnss_timestamps->resize(Eigen::NoChange, num_points);
   gnss_points->resize(Eigen::NoChange, num_points);
 
   for (size_t idx = 0u; idx < num_points; ++idx) {
     (*gnss_timestamps)(idx) = between_values[idx].timestamp_;
     (*gnss_points).col(idx) = between_values[idx].point_;
   }
 
   return query_result;
 }
 
 ThreadsafeGnssBuffer::QueryResult
 ThreadsafeGnssBuffer::getGnssDataInterpolatedUpperBorder(
     const Timestamp& timestamp_ns_from,
     const Timestamp& timestamp_ns_to,
     GnssStampS* gnss_timestamps,
     GnssPointS* gnss_points) {
   CHECK_NOTNULL(gnss_timestamps);
   CHECK_NOTNULL(gnss_points);
   DCHECK_LT(timestamp_ns_from, timestamp_ns_to);
   // Get data.
   QueryResult query_result = getGnssDataBtwTimestamps(
       timestamp_ns_from, timestamp_ns_to, gnss_timestamps, gnss_points,
       true);  // Get lower bound.
   // Early exit if there is no data.
   if (query_result != QueryResult::kDataAvailable) {
     gnss_timestamps->resize(Eigen::NoChange, 0);
     gnss_points->resize(Eigen::NoChange, 0);
     return query_result;
   }
 
   // Interpolate upper border.
   GnssPoint interpolated_upper_border;
   interpolateValueAtTimestamp(timestamp_ns_to, &interpolated_upper_border);
 
   DCHECK_EQ(gnss_timestamps->rows(), 1);
   DCHECK_EQ(gnss_points->rows(), 6);
   // The last point will correspond to the interpolated data.
   const size_t num_points = gnss_timestamps->cols() + 1u;
   gnss_timestamps->conservativeResize(Eigen::NoChange, num_points);
   gnss_points->conservativeResize(Eigen::NoChange, num_points);
   // Append upper border.
   gnss_timestamps->rightCols<1>()(0) = timestamp_ns_to;
   gnss_points->rightCols<1>() = interpolated_upper_border;
 
   return query_result;
 }
 
 ThreadsafeGnssBuffer::QueryResult
 ThreadsafeGnssBuffer::getGnssDataInterpolatedBorders(
     const Timestamp& timestamp_ns_from,
     const Timestamp& timestamp_ns_to,
     GnssStampS* gnss_timestamps,
     GnssPointS* gnss_points) {
   CHECK_NOTNULL(gnss_timestamps);
   CHECK_NOTNULL(gnss_points);
   // Get data.
   GnssStampS gnss_timestamps_tmp;
   GnssPointS gnss_points_tmp;
   QueryResult query_result =
       getGnssDataBtwTimestamps(timestamp_ns_from, timestamp_ns_to,
                               &gnss_timestamps_tmp, &gnss_points_tmp, true);
 
   // Early exit if there is no data.
   if (query_result != QueryResult::kDataAvailable) {
     gnss_timestamps->resize(Eigen::NoChange, 0);
     gnss_points->resize(Eigen::NoChange, 0);
     return query_result;
   }
 
   // Interpolate lower border.
   GnssPoint interpolated_lower_border;
   interpolateValueAtTimestamp(timestamp_ns_from, &interpolated_lower_border);
   // Interpolate upper border.
   GnssPoint interpolated_upper_border;
   interpolateValueAtTimestamp(timestamp_ns_to, &interpolated_upper_border);
 
   DCHECK_EQ(gnss_timestamps->rows(), 1);
   DCHECK_EQ(gnss_points->rows(), 6);
   // The first and last points will correspond to the interpolated data.
   const size_t num_points = gnss_timestamps_tmp.cols() + 2u;
   gnss_timestamps->resize(Eigen::NoChange, num_points);
   gnss_points->resize(Eigen::NoChange, num_points);
   // Prepend lower border.
   gnss_timestamps->leftCols<1>()(0) = timestamp_ns_from;
   gnss_points->leftCols<1>() = interpolated_lower_border;
   // Add points.
   gnss_timestamps->middleCols(1, gnss_timestamps_tmp.cols()) = gnss_timestamps_tmp;
   gnss_points->middleCols(1, gnss_points_tmp.cols()) =
       gnss_points_tmp;
   // Append upper border.
   gnss_timestamps->rightCols<1>()(0) = timestamp_ns_to;
   gnss_points->rightCols<1>() = interpolated_upper_border;
 
   return query_result;
 }
 
 void ThreadsafeGnssBuffer::interpolateValueAtTimestamp(
     const Timestamp& timestamp_ns,
     GnssPoint* interpolated_gnss_point) {
   CHECK_NOTNULL(interpolated_gnss_point);
   Timestamp pre_border_timestamp, post_border_timestamp;
   GnssMeasurement pre_border_value, post_border_value;
   CHECK(buffer_.getValueAtOrBeforeTime(timestamp_ns, &pre_border_timestamp,
                                        &pre_border_value))
       << "The GNSS buffer seems not to contain points at or before time: "
       << timestamp_ns;
   CHECK_EQ(pre_border_timestamp, pre_border_value.timestamp_);
   CHECK(buffer_.getValueAtOrAfterTime(timestamp_ns, &post_border_timestamp,
                                       &post_border_value))
       << "The GNSS buffer seems not to contain points at or after time: "
       << timestamp_ns;
   CHECK_EQ(post_border_timestamp, post_border_value.timestamp_);
   linearInterpolate(pre_border_value.timestamp_,
                     pre_border_value.point_,
                     post_border_value.timestamp_,
                     post_border_value.point_,
                     timestamp_ns,
                     interpolated_gnss_point);
 }
 
 ThreadsafeGnssBuffer::QueryResult
 ThreadsafeGnssBuffer::getGnssDataInterpolatedBordersBlocking(
     const Timestamp& timestamp_ns_from,
     const Timestamp& timestamp_ns_to,
     const Timestamp& wait_timeout_nanoseconds,
     GnssStampS* gnss_timestamps,
     GnssPointS* gnss_points) {
   CHECK_NOTNULL(gnss_timestamps);
   CHECK_NOTNULL(gnss_points);
 
   // Wait for the GNSS buffer to contain the required points within a
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
         gnss_points->resize(Eigen::NoChange, 0);
         return QueryResult::kQueueShutdown;
       }
 
       // Check if we hit the max. time allowed to wait for the required data.
       auto toc = Timer::toc<std::chrono::nanoseconds>(tic);
       if (toc.count() >= wait_timeout_nanoseconds) {
         gnss_timestamps->resize(Eigen::NoChange, 0);
         gnss_points->resize(Eigen::NoChange, 0);
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
                                        gnss_timestamps, gnss_points);
 }
 
 }  // namespace utils
 
 }  // namespace VIO
 