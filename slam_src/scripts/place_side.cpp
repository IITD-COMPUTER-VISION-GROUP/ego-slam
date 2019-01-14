#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

int main(int argc, char **argv)
{
    cv::Mat im1(500,300,CV_8UC1);
    cv::Mat im2(500,300,CV_8UC1);
    im1 = cv::Scalar(10);
    im2 = cv::Scalar(200);
    cv::Size sz1 = im1.size();
    cv::Size sz2 = im2.size();
    cv::Mat im3(sz1.height, sz1.width+sz2.width, CV_8UC1);
    cv::Mat left(im3, cv::Rect(0, 0, sz1.width, sz1.height));
    im1.copyTo(left);
    cv::Mat right(im3, cv::Rect(sz1.width, 0, sz2.width, sz2.height));
    im2.copyTo(right);
    cv::imshow("im3", im3);
    cv::waitKey(0);
    return 0;
}
