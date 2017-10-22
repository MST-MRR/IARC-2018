#include <vector>
#include <iostream>
#include <iterator>
#include <algorithm>
#include <string>
#include <chrono>
#include <boost/python.hpp>

#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/cuda.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudaarithm.hpp"
#include "opencv2/cudafilters.hpp"

using namespace std;
using namespace cv;
using namespace cv::cuda;

namespace py = boost::python;

#include "conversion.hpp"

const string TEST_FILE_PATH("../mrrdt_vision/mrrdt_vision/data/2017-09-15 20:43:47.724821.jpg");
// lower bound for the red roomba's flap in YCrCb space
const Scalar RED_YCRCB_LOWER_BOUND = Scalar(0, 156, 107);
// upper bound for the red roomba's flap in YCrCb space
const Scalar RED_YCRCB_UPPER_BOUND = Scalar(255, 255, 255);
// lower bound for the green roomba's flap in LAB space
const Scalar GREEN_LAB_LOWER_BOUND = Scalar(0, 0, 127);
// upper bound for the green roomba's flap in LAB space
const Scalar GREEN_LAB_UPPER_BOUND = Scalar(94, 123, 250);

void in_range(cv::cuda::GpuMat &src, cv::cuda::GpuMat &dst, const cv::Scalar &lower, const cv::Scalar &upper);

PyObject* threshold_image_for_roombas(PyObject* img)
{
    Mat cpu_img;
    pyopencv_to(img, cpu_img);
    GpuMat gpu_img(cpu_img);
    GpuMat lab_img, ycrcb_img, combined_mask;

    Ptr<cuda::Filter> filter = cuda::createGaussianFilter(gpu_img.type(), gpu_img.type(), Size(11, 11), 0);
    filter->apply(gpu_img, gpu_img);
    
    cuda::cvtColor(gpu_img, lab_img, COLOR_BGR2Lab);
    cuda::cvtColor(gpu_img, ycrcb_img, COLOR_BGR2YCrCb);
    GpuMat green_threshold = GpuMat(lab_img.rows, lab_img.cols, CV_8UC1);
    GpuMat red_threshold = GpuMat(ycrcb_img.rows, ycrcb_img.cols, CV_8UC1);
    
    in_range(lab_img, green_threshold, GREEN_LAB_LOWER_BOUND, GREEN_LAB_UPPER_BOUND);
    in_range(ycrcb_img, red_threshold, RED_YCRCB_LOWER_BOUND, RED_YCRCB_UPPER_BOUND);
    cuda::bitwise_or(red_threshold, green_threshold, combined_mask);
    combined_mask.download(cpu_img);

    return pyopencv_from(cpu_img);
}

static void init()
{
    Py_Initialize();
    import_array();
}

BOOST_PYTHON_MODULE(threshold_gpu)
{
    init();
    py::def("threshold_image_for_roombas", threshold_image_for_roombas);
}