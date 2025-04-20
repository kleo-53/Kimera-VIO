// #pragma once

// #include <algorithm>
// #include <atomic>
// #include <condition_variable>
// #include <memory>
// #include <mutex>
// #include <utility>

// #include <functional>
// #include <map>


// #include "kimera-vio/frontend/GnssStereoVisionImuFrontend-definitions.h"
// #include "kimera-vio/common/vio_types.h"
// #include "kimera-vio/utils/ThreadsafeTemporalBuffer.h"

// namespace VIO {

  
// namespace utils {
// using GnssStampS = Eigen::Matrix<Timestamp, 1, Eigen::Dynamic>;
// using GnssPoseS = Eigen::Matrix<double, 3, Eigen::Dynamic>;

// class ThreadsafeGnssBuffer {
//  public:
//   KIMERA_POINTER_TYPEDEFS(ThreadsafeGnssBuffer);


//   explicit ThreadsafeGnssBuffer(const Timestamp& buffer_length_ns)
//       : buffer_(buffer_length_ns), shutdown_(false) {}

//   ~ThreadsafeGnssBuffer() { shutdown(); }

//   void shutdown() {
  //     shutdown_ = true;
  //     cv_new_measurement_.notify_all();
  //   }
  
  //   size_t size() const { return buffer_.size(); }

  //   void clear() { buffer_.clear(); }
  
  //   inline void addMeasurement(const Timestamp& timestamp_ns, const VIO::GnssMeasurement& gnss_measurement);
  
  //   inline void addMeasurements(const GnssStampS& timestamps_nanoseconds,
  //     const GnssPoseS& gnss_measurements);
  
  //   bool getNewestGnssMeasurement(VIO::GnssMeasurement* value) {
    //     CHECK_NOTNULL(value);
    //     return buffer_.getNewestValue(value);
    //   }
    
    //   QueryResult getMeasurementAtTimestamp(const Timestamp& timestamp_ns,
    //                                         VIO::GnssMeasurement* result) const {
//     return buffer_.getValueAtTime(timestamp_ns, result)
//                ? QueryResult::kDataAvailable
//                : QueryResult::kDataNeverAvailable;
//               //  : isDataAvailableImpl(timestamp_ns);
//   }

//   QueryResult getDataBtwTimestamps(const Timestamp& timestamp_from,
//                                    const Timestamp& timestamp_to,
//                                    std::vector<VIO::GnssMeasurement>* measurements) {
//     return buffer_.getValuesBetweenTimes(
  //                timestamp_from, timestamp_to, &measurements, false)
//                ? QueryResult::kDataAvailable
//                : QueryResult::kDataNeverAvailable;
//               //  : isDataAvailableImpl(timestamp_to);
//     }

//     // QueryResult isDataAvailableImpl(const Timestamp& timestamp) const {
  //     //   if (shutdown_) return QueryResult::kQueueShutdown;
  
  //     //   if (!buffer_.hasDataAtOrAfterTimestamp(timestamp)) {
    //     //     return QueryResult::kDataNotYetAvailable;
    //     //   }
    
    //     //   if (!buffer_.hasDataAtOrBeforeTimestamp(timestamp)) {
      //     //     return QueryResult::kDataNeverAvailable;
      //     //   }
      
      //     // return QueryResult::kDataAvailable;
      //   // }
      
      //   private:
      //     typedef std::pair<const Timestamp, VIO::GnssMeasurement> BufferElement;
      //     typedef Eigen::aligned_allocator<BufferElement> BufferAllocator;
      //     typedef ThreadsafeTemporalBuffer<VIO::GnssMeasurement, BufferAllocator> Buffer;
      
      //     Buffer buffer_;
      //     mutable std::mutex m_buffer_;
      //     std::condition_variable cv_new_measurement_;
      //     std::atomic<bool> shutdown_;
      //   };
      // }
      // }  // namespace VIO
      
      
      // #include "./ThreadsafeGnssBuffer-inl.h"
      
      
      // kimera-vio/utils/ThreadsafeGnssBuffer.h
      #pragma once
      
      #include <vector>
      #include <utility>
      #include <memory>
      
      #include "kimera-vio/frontend/GnssTypes.h"
      #include "kimera-vio/utils/ThreadsafeTemporalBuffer.h"
      #include "kimera-vio/common/vio_types.h"
      
      namespace VIO {
        namespace utils {
          
          class ThreadsafeGnssBuffer {
            public:
            enum class QueryResult {
              kDataAvailable,
              kDataNotYetAvailable,
              kDataNeverAvailable,
              kQueueShutdown
            };
            typedef std::pair<const Timestamp, GnssMeasurement> BufferElement;
            typedef std::allocator<BufferElement> BufferAllocator;
            typedef ThreadsafeTemporalBuffer<GnssMeasurement, BufferAllocator> Buffer;

  explicit ThreadsafeGnssBuffer(int buffer_size = -1)
      : buffer_(buffer_size) {}

  size_t size() const { return buffer_.size(); }
  void clear() { buffer_.clear(); }

  void addMeasurement(const Timestamp& timestamp_ns, 
                     const GnssMeasurement& gnss_measurement);
  
  void addMeasurements(const GnssStampS& timestamps_nanoseconds,
                      const GnssPoseS& gnss_measurements);

  bool getNewestGnssMeasurement(GnssMeasurement* value);
  
  QueryResult getMeasurementAtTimestamp(const Timestamp& timestamp_ns,
                                      GnssMeasurement* result) const;
  
  QueryResult getDataBtwTimestamps(const Timestamp& timestamp_from,
                                 const Timestamp& timestamp_to,
                                 std::vector<GnssMeasurement>* measurements);

 private:
  Buffer buffer_;
};

}  // namespace utils
}  // namespace VIO