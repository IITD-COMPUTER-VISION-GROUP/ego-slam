#ifndef CORRESPONDANCE_H
#define CORRESPONDANCE_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <iostream>

struct corr {
  int frame_1;
  int frame_2;
  std::vector<int> unique_id;
  std::vector<cv::Point2f> p1;
  std::vector<cv::Point2f> p2; 
  std::vector<cv::Vec3b> col;
  cv::Point2f delta;

  corr() {};

  corr(int f1, int f2) {
    frame_1 = f1;
    frame_2 = f2;
  }

  corr(const corr& other) {
    frame_1 = other.frame_1;
    frame_2 = other.frame_2;
    unique_id = other.unique_id;
    p1 = other.p1;
    p2 = other.p2;
    delta = other.delta;
    col = other.col;
    assert (p1.size() == p2.size());
    assert (p1.size() == col.size());
  }
};

corr CompressCorr(corr &init, corr &final);

void GetGoodPoints(std::vector<cv::Point2f> &prevtracking,
  std::vector<cv::Point2f> &inversetracking, 
  std::vector<uchar> &status,
  std::vector<uchar> &statusinv);

void ChangeCenterSubtracted(corr &p, int cx, int cy);

void CalculateDelta(corr &c);

bool WithinRange(corr &c);

#endif
