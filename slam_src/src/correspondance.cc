#include "correspondance.h"

corr CompressCorr(corr &init, corr &final) {
  assert(init.frame_2 == final.frame_1);
  corr compressed(init.frame_1, final.frame_2);
  int finalcorr=0;
  int limitfinal = final.p1.size();
  for (int i=0; (i<init.p1.size()) && (finalcorr<limitfinal); i++) {
    if (init.p2[i]==final.p1[finalcorr]) {
      compressed.unique_id.push_back(init.unique_id[i]);
      compressed.p1.push_back(init.p1[i]);
      compressed.p2.push_back(final.p2[finalcorr]);
      compressed.col.push_back(init.col[i]);
      finalcorr++;
    }
  }
  compressed.delta = init.delta + final.delta;
  return compressed;
}

void GetGoodPoints(std::vector<cv::Point2f> &prevtracking,
  std::vector<cv::Point2f> &inversetracking, 
  std::vector<uchar> &status,
  std::vector<uchar> &statusinv) {
  int tot, removed;
  tot = prevtracking.size();
  removed=0;
  for (int i=0; i<prevtracking.size(); i++)  {
    if (status[i]==0) {
      continue;
    }
    if (statusinv[i] == 0) {
      status[i] = 0;
      continue;
    }
    status[i]=0;
    cv::Point2f temmpPoint = inversetracking[i]-prevtracking[i];
    float magnitude = (temmpPoint.x)*(temmpPoint.x) + (temmpPoint.y)*(temmpPoint.y);
    if (magnitude<=0.1) {
      status[i]=1;
    } else {
      removed++;
    }
  }
  // std::cout << "Removed " << removed << " points from " << tot << " points\n";
  assert (removed < tot);
}

void ChangeCenterSubtracted(corr &p, int cx, int cy) {
  for (int i=0; i<p.p1.size(); i++) {
    p.p1[i].x -= cx;
    p.p1[i].y -= cy; 
    p.p2[i].x -= cx;
    p.p2[i].y -= cy; 
  }
}

void CalculateDelta(corr &c) {
  c.delta = cv::Point2f(0,0);
  assert(c.p1.size() == c.p2.size());
  for (int i=0; i< c.p1.size(); i++) {
    c.delta += c.p2[i] - c.p1[i];
  }
  if (c.p1.size() > 0) {
    c.delta.x /= c.p1.size();
    c.delta.y /= c.p1.size();
  }
}

bool WithinRange(corr &c) {
  float val = c.delta.dot(c.delta);
  return (val < 500);
}
