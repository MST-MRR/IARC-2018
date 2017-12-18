#include <iostream>

#include "opencv2/core/cuda.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudaarithm.hpp"
#include "opencv2/cudafilters.hpp"

using namespace std;
using namespace cv;
using namespace cv::cuda;

const int BLOCK_SIZE = 256;

__global__ void in_range_worker(const cv::cuda::PtrStepSz<uchar3> src, cv::cuda::PtrStepSzb dst, int lb0, int ub0, int lb1, int ub1, int lb2, int ub2) 
{
  const int n = src.rows*src.cols;
  int index = blockIdx.x * blockDim.x + threadIdx.x;
  int stride = blockDim.x * gridDim.x;
  int x = 0, y = 0;

  for (int i = index; i < n; i += stride)
  {
    x = i % src.cols;
    y = i / src.cols;
    uchar3 v = src(y, x);
    dst(y, x) = (v.x >= lb0 && v.x <= ub0 && v.y >= lb1 && v.y <= ub1 && v.z >= lb2 && v.z <= ub2)*255;
  }
}

void in_range(cv::cuda::GpuMat &src, cv::cuda::GpuMat &dst, const cv::Scalar &lower, const cv::Scalar &upper) 
{
  const int n = src.rows*src.cols;
  const int num_blocks = (n + BLOCK_SIZE - 1)/BLOCK_SIZE;
  
  in_range_worker<<<num_blocks, BLOCK_SIZE>>>(src, dst, lower[0], upper[0], lower[1], upper[1], lower[2], upper[2]);
  cudaDeviceSynchronize();
}