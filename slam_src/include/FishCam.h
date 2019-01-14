#ifndef FISHOCAM_H
#define FISHOCAM_H

#include "undistort.h"
#include <string>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <cassert>
#include <opencv2/core/core.hpp>

class FishOcam
{
public:
  const static int maxPolLength = 64;
  double pol[maxPolLength];    // the polynomial coefficients: pol[0] + x"pol[1] + x^2*pol[2] + ... + x^(N-1)*pol[N-1]
  int length_pol;                // length of polynomial
  double invpol[maxPolLength]; // the coefficients of the inverse polynomial
  int length_invpol;             // length of inverse polynomial
  double xc;         // row coordinate of the center
  double yc;         // column coordinate of the center
  double c;          // affine parameter
  double d;          // affine parameter
  double e;          // affine parameter
  int width;         // image width
  int height;        // image height
  int hout;          // Image Height out
  int wout;          // Image Width out
  double invdet;
  double focal;
  CVec2Img warp;     // Warping matrix

  FishOcam() {}
  
  void init(const std::string & calibFile) {
    FILE *f = fopen(calibFile.c_str(), "rb");

    const int maxBuf = 1024;
    char buf[maxBuf];

    //Read polynomial coefficients
    fgets(buf, maxBuf, f);
    fscanf(f, "\n");
    fscanf(f, "%d", &length_pol);
    for (int i = 0; i < length_pol; i++) {
      fscanf(f, " %lf", &pol[i]);
    }

    //Read inverse polynomial coefficients
    fscanf(f, "\n");
    fgets(buf, maxBuf, f);
    fscanf(f, "\n");
    fscanf(f, "%d", &length_invpol);
    for (int i = 0; i < length_invpol; i++) {
      fscanf(f, " %lf", &invpol[i]);
    }

    //Read center coordinates
    fscanf(f, "\n");
    fgets(buf, maxBuf, f);
    fscanf(f, "\n");
    fscanf(f, "%lf %lf\n", &xc, &yc);

    //Read affine coefficients
    fgets(buf, maxBuf, f);
    fscanf(f, "\n");
    fscanf(f, "%lf %lf %lf\n", &c, &d, &e);

    //Read image size
    fgets(buf, maxBuf, f);
    fscanf(f, "\n");
    fscanf(f, "%d %d", &height, &width);

    fclose(f);

    invdet = 1 / (c - d*e); // 1/det(A), where A = [c,d;e,1] as in the Matlab file
    wout = width;
    createPerspectiveWarp(width, height, width, true);
  }

  CVec3d cam2world(const CVec2d & f) {
    double xp = invdet*((f.x - xc) - d*(f.y - yc));
    double yp = invdet*(-e*(f.x - xc) + c*(f.y - yc));

    double r = sqrt(xp*xp + yp*yp); //distance [pixels] of  the point from the image center
    double zp = pol[0];
    double r_i = 1;
    int i;

    for (i = 1; i < length_pol; i++) {
      r_i *= r;
      zp += r_i*pol[i];
    }

    //normalize to unit norm
    double invnorm = 1 / sqrt(xp*xp + yp*yp + zp*zp);

    CVec3d res;
    res.x = invnorm*xp;
    res.y = invnorm*yp;
    res.z = invnorm*zp;
    return res;
  }

  CVec2d world2cam(const CVec3d & p) {
    double norm = sqrt(p.x*p.x + p.y*p.y);
    double theta = atan(p.z / norm);

    CVec2d res;

    if (norm != 0) {
      double invnorm = 1 / norm;
      double t = theta;
      double rho = invpol[0];
      double t_i = 1;

      for (int i = 1; i < length_invpol; i++) {
        t_i *= t;
        rho += t_i*invpol[i];
      }

      double x = p.x*invnorm*rho;
      double y = p.y*invnorm*rho;

      res.x = x*c + y*d + xc;
      res.y = x*e + y + yc;
    }
    else {
      res.x = xc;
      res.y = yc;
    }
    return res;
  }

  void createPerspectiveWarp(const int win, const int hin, const int wout, const bool crop) {
    CVec3d wbase, wright, wup;

    if (crop) {
      CVec3d wcenter = cam2world(CVec2d(hin / 2.0, win / 2.0));

      wright = cam2world(CVec2d(hin / 2.0, win));
      wright = (wright / (wright*wcenter) - wcenter).Unit();

      wup = wcenter.Cross(wright);

      CVec3d w0 = cam2world(CVec2d(0, win / 2.0));
      w0 = w0 / (w0*wcenter);

      CVec3d w1 = cam2world(CVec2d(hin, win / 2.0));
      w1 = w1 / (w1*wcenter);

      CVec3d w2 = cam2world(CVec2d(hin / 2.0, 0));
      w2 = w2 / (w2*wcenter);

      CVec3d w3 = cam2world(CVec2d(hin / 2.0, win));
      w3 = w3 / (w3*wcenter);

      double extright = std::min(fabs((w2 - wcenter)*wright), fabs((w3 - wcenter)*wright));
      double extup = std::min(fabs((w0 - wcenter)*wup), fabs((w1 - wcenter)*wup));

      wbase = wcenter - wright*extright - wup*extup;
      wright = wright * (2.0*extright);
      wup = wup * (2.0*extup);

      hout = wout*extup / extright;
    }
    else {
      CVec3d wcenter = cam2world(CVec2d(hin / 2.0, win / 2.0));

      wright = cam2world(CVec2d(hin / 2.0, win));
      wright = (wright / (wright*wcenter) - wcenter).Unit();

      wup = wcenter.Cross(wright);

      CVec3d w0 = cam2world(CVec2d(0, 0));
      w0 = w0 / (w0*wcenter);

      CVec3d w1 = cam2world(CVec2d(0, win));
      w1 = w1 / (w1*wcenter);

      CVec3d w2 = cam2world(CVec2d(hin, win));
      w2 = w2 / (w2*wcenter);

      CVec3d w3 = cam2world(CVec2d(hin, 0));
      w3 = w3 / (w3*wcenter);

      double extright = std::max(std::max(abs((w0 - wcenter)*wright), abs((w1 - wcenter)*wright)), std::max(abs((w2 - wcenter)*wright), abs((w3 - wcenter)*wright)));
      double extup = std::max(std::max(abs((w0 - wcenter)*wup), abs((w1 - wcenter)*wup)), std::max(abs((w2 - wcenter)*wup), abs((w3 - wcenter)*wup)));

      wbase = wcenter - wright*extright - wup*extup;
      wright = wright * (2.0*extright);
      wup = wup * (2.0*extup);

      hout = wout*extup / extright;
    }

    warp.Create(wout, hout);

    for (int y = 0; y < hout; y++) {
      for (int x = 0; x < wout; x++) {
        CVec3d p = wbase;
        CVec3d temp1;
        temp1.x = (wright.x)*x / (wout - 1.0);
        temp1.y = (wright.y)*x / (wout - 1.0);
        temp1.z = (wright.z)*x / (wout - 1.0);

        CVec3d temp2;
        temp2.x = (wup.x)*y / (hout - 1.0);
        temp2.y = (wup.y)*y / (hout - 1.0);
        temp2.z = (wup.z)*y / (hout - 1.0);

        p.x += temp1.x + temp2.x;
        p.y += temp1.y + temp2.y;
        p.z += temp1.z + temp2.z;

        CVec2d f = world2cam(p);
        if (f.x < 0) {
          f.x = 0;
        }
        if (f.y < 0) {
          f.y = 0;
        }

        if (f.x > hin - 1) {
          f.x = hin - 1;
        }
        if (f.y > win - 1) {
          f.y = win -1;
        }

        warp.arr[y][x].x = f.y;
        warp.arr[y][x].y = f.x;
        assert(warp.arr[y][x].y <= hin -1 );
        assert(warp.arr[y][x].y >= 0);
        assert(warp.arr[y][x].x <= win -1);
        assert(warp.arr[y][x].x >= 0);
      }
    }
    double hfov = acos((wbase + wup / 2).Unit() * (wbase + wup / 2 + wright).Unit());
    double vfov = acos((wbase + wright / 2).Unit() * (wbase + wright / 2 + wup).Unit());
    focal = (wout > hout ?( 0.5 *wout )/ tan(hfov / 2) : (0.5 *hout) / tan(vfov / 2));
  }

  void WarpImage(cv::Mat &input, cv::Mat&output) {
    output = cv::Mat(cv::Size(wout, hout), CV_8UC3);
    assert(output.rows == warp.rows);
    assert(output.cols == warp.cols);
    for (int i = 0; i < warp.rows; i++) {
      for (int j = 0; j < warp.cols; j++) {
        CVec2f tmp = warp.arr[i][j];
        output.at<cv::Vec3b>(i, j) = input.at<cv::Vec3b>(tmp.y, tmp.x);
      }
    }
  }
};

#endif
