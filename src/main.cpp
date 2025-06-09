#include <iostream>
#include <iomanip>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
// #include "matplotlibcpp.h"

// using namespace std;
// namespace plt = matplotlibcpp;
using namespace Eigen;

// tim nearest neighbor
void findNearestNeighbors(const std::vector<Vector2d>& source, const std::vector<Vector2d>& target, std::vector<int>& indices) {
  indices.clear();
  for(const auto&p : source) {
    double min_distance = 1e9;
    int idx = 0;
    for(int i = 0; i<target.size(); i++) {
      double distance = (p - target[i]).squaredNorm();
        if(distance < min_distance) {
          idx = i;
          min_distance = distance;
        }
    }
    indices.push_back(idx);
  }
}

// ap dung phep bien doi
void applyTransformation(const Matrix2d &rot, const Vector2d &transl, std::vector<Vector2d> &dst) {
  for(auto& p: dst) {
    p = rot*p + transl;
  }
}

// tim phep bien doi tot nhat
void findTransformation(std::vector<Vector2d> &source, std::vector<Vector2d> &target, 
                        std::vector<int> indices, Matrix2d &R, Vector2d &t) {
  // tinh tam diem (centroid)
  Vector2d source_cen, target_cen;
  int n = source_cen.size();
  for(int i = 0; i < n; i++) {
    source_cen = source_cen + source[i];
    target_cen += target[indices[i]]; // trong truong hop so diem o 2 vector la khac nhau
  }
  source_cen /=n;
  target_cen /=n;

  Matrix2d H = Matrix2d::Zero();
  for(int i = 0; i<n; i++) {
    Vector2d ps = source[i] - source_cen;
    Vector2d pt = target[indices[i]] - target_cen;
    H += ps * pt.transpose();
  }
  
  JacobiSVD<Matrix2d> svd(H, ComputeFullU | ComputeFullV);
  R = svd.matrixV() * svd.matrixU().transpose();

  // ensure R is a rotation matrix
  if(R.determinant() < 0) {
    R.col(1) *= -1;
  }

  t = target_cen - R * source_cen;
}

int main() {
  // tao diem ban dau 
  std::vector<Vector2d> origin = {{1.0, 0.0}, {3.0,3.0}, {0.0,1.0}, {0.0,0.0}};

  // tao diem sau khi da xoay
  std::vector<Vector2d> transformed = origin;

  Matrix2d R_init = Rotation2Dd(45 * M_PI / 180).toRotationMatrix();
  Vector2d t_init;
  t_init << 0.5, -0.3;

  applyTransformation(R_init, t_init, transformed);


  for(int i = 0; i<100; i++) {
    std::vector<int> indices;
    findNearestNeighbors(transformed, origin, indices);
    Matrix2d R;
    Vector2d t;
    findTransformation(transformed, origin, indices, R, t);
    applyTransformation(R,t, transformed);
  }
  for(auto p: transformed){
    std::cout << std::fixed << std::setprecision(4) << p[0] << " " << p[1] << std::endl;  
  }
  return 0;
}