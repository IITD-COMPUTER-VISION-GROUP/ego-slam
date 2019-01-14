#ifndef UNDISTORT_H
#define UNDISTORT_H

#include <cmath>

class CVec2d {
public:
  double x;
  double y;
  
  CVec2d() {
    x = 0.0;
    y = 0.0;
  }
  
  CVec2d(double p, double q) {
    x = p;
    y = q;
  }
  
  ~CVec2d() {};
  
  double Norm() {
    return(sqrt(x*x + y*y));
  }

  CVec2d Unit() {
    double norm = sqrt(x*x + y*y);
    CVec2d tmp(x / norm, y / norm);
    return tmp;
  }
};

class CVec2f {
public:
  double x;
  double y;
  
  CVec2f() {
    x = 0.0;
    y = 0.0;
  }
  
  CVec2f(double p, double q) {
    x = p;
    y = q;
  }
  
  ~CVec2f() {};
  
  double Norm() {
    return(sqrt(x*x + y*y));
  }

  CVec2f Unit() {
    double norm = sqrt(x*x + y*y);
    CVec2f tmp(x / norm, y / norm);
    return tmp;
  }
};

class CVec3d {
public:
  double x;
  double y;
  double z;
  
  CVec3d() {
    x = 0.0;
    y = 0.0;
    z = 0.0;
  }
  
  CVec3d(double p, double q, double r) {
    x = p;
    y = q;
    z = r;
  }
  
  ~CVec3d() {};
  
  double Norm() {
    return(sqrt(x*x + y*y + z*z));
  }

  CVec3d Unit() {
    double norm = sqrt(x*x + y*y + z*z);
    CVec3d tmp(x/norm, y/norm, z/norm);
    return tmp;
  }

  CVec3d Cross(CVec3d v1) {
    CVec3d tmp;
    tmp.x = y*v1.z - z*v1.y; // u2v3 - u3v2
    tmp.y = z*v1.x - x*v1.z; //u3v1 - u1v3
    tmp.z = x*v1.y - y*v1.x; //u1*v2 - u2*v1
    return tmp;
  }

  double operator*(const CVec3d& rhs) // compound assignment (does not need to be a member,
  {                           // but often is, to modify the private members)
                /* addition of rhs to *this takes place here */
    double val;
    val = this->x*rhs.x + this->y*rhs.y + this->z*rhs.z;
    return val;
  }

  CVec3d operator/(const double rhs) // compound assignment (does not need to be a member,
  {                           // but often is, to modify the private members)
                /* addition of rhs to *this takes place here */
    CVec3d val;
    val.x = this->x / rhs;
    val.y = this->y / rhs;
    val.z= this->z / rhs;
    return val;
  }

  CVec3d operator*(const double rhs) // compound assignment (does not need to be a member,
  {                           // but often is, to modify the private members)
                /* addition of rhs to *this takes place here */
    CVec3d val;
    val.x = this->x * rhs;
    val.y = this->y * rhs;
    val.z = this->z * rhs;
    return val;
  }

  CVec3d operator+(const CVec3d& rhs) // compound assignment (does not need to be a member,
  {                           // but often is, to modify the private members)
                /* addition of rhs to *this takes place here */
    CVec3d val;
    val.x = this->x + rhs.x;
    val.y = this->y + rhs.y;
    val.z = this->z + rhs.z;
    return val;
  }

  CVec3d operator-(const CVec3d& rhs) // compound assignment (does not need to be a member,
  {                           // but often is, to modify the private members)
                /* addition of rhs to *this takes place here */
    CVec3d val;
    val.x = this->x - rhs.x;
    val.y = this->y - rhs.y;
    val.z = this->z - rhs.z;
    return val;
  }
};

class CVec2Img {
public:
  int cols;
  int rows;
  CVec2f **arr;

  CVec2Img(){}
  
  CVec2Img(int w, int h) {
    cols = w;
    rows = h;

    arr = new CVec2f*[h];
    for (int i = 0; i < h; i++)
      arr[i] = new CVec2f[w];
  }

  ~CVec2Img() { }
  
  void Create(int w, int h) {
    cols = w;
    rows = h;

    arr = new CVec2f*[h];
    for (int i = 0; i < h; i++) {
      arr[i] = new CVec2f[w];
    }
  }
};

#endif
