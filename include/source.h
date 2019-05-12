#ifndef PRJ_SOURCE_H_
#define PRJ_SOURCE_H_

// test third party headers
// opencv
#include <opencv2/opencv.hpp>
// Eigen
#include <Eigen/Core>
// GLOG
#include <glog/logging.h>
// ceres
#include <ceres/ceres.h>
// cuda
#include <cuda_runtime.h>
// pangolin
#include <pangolin/pangolin.h>
// pcl is so much

namespace prj {
class Source {
 private:
    /* data */
 public:
    Source(int g = 0);
    ~Source();
};

}  // namespace prj
#endif  // PRJ_SOURCE_H_