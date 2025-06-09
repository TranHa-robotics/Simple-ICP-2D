#include <iostream>
#include <iomanip>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

// using namespace std;
using namespace Eigen;

void findNearestNeighbors(const std::vector<Vector2d>& source, const std::vector<Vector2d>& dest, std::vector<int>& indices) {
  indices.clear();
  for(const auto&p : source) {
    double min_distance = 1e9;
    int idx = 0;
    for(int i = 0; i<dest.size(); i++) {
      double distance = (p - dest[i]).squaredNorm();
        if(distance < min_distance) {
          idx = i;
          min_distance = distance;
        }
    }
    indices.push_back(idx);
  }
}
void applyTransformation(const Matrix2d &rot, const Vector2d &transl, std::vector<Vector2d> &dst) {
  for(auto& p: dst) {
    p = rot*p + transl;
  }
}

int main() {
  // tao diem ban dau 
  std::vector<Vector2d> origin = {{1.0, 0.0}, {1.0,1.0}, {0.0,1.0}, {0.0,0.0}};

  // tao diem sau khi da xoay
  std::vector<Vector2d> transformed = origin;

  Matrix2d R_init = Rotation2Dd(90 * M_PI / 180).toRotationMatrix();
  Vector2d t_init;
  t_init << 0.5, -0.3;

  applyTransformation(R_init, t_init, transformed);
  for(auto p: transformed){
    std::cout << std::fixed << std::setprecision(4) << p[0] << " " << p[1] << std::endl;  
  }

  // for(int i = 0; i<20; i++) {
    std::vector<int> indices;
    findNearestNeighbors(origin, transformed, indices);
    for(auto i : indices)
      std::cout << i << " " << std::endl;
  // }
  return 0;
}