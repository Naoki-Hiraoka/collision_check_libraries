#include <mesh_boolean_libigl/mesh_boolean_libigl.h>

#include <iostream>
#include <sys/time.h>


void sample1 (){
  Eigen::MatrixXd cube(3,8);
  cube.col(0) = Eigen::Vector3d(0,0,0);
  cube.col(1) = Eigen::Vector3d(1,0,0);
  cube.col(2) = Eigen::Vector3d(1,1,0);
  cube.col(3) = Eigen::Vector3d(0,1,0);
  cube.col(4) = Eigen::Vector3d(0,0,1);
  cube.col(5) = Eigen::Vector3d(1,0,1);
  cube.col(6) = Eigen::Vector3d(1,1,1);
  cube.col(7) = Eigen::Vector3d(0,1,1);

  Eigen::MatrixXd polygon(3,6);
  polygon.col(0) = Eigen::Vector3d(-1,0,0.5);
  polygon.col(1) = Eigen::Vector3d(2,0,0.5);
  polygon.col(2) = Eigen::Vector3d(0.5,1,0.5);
  for(int i=0;i<3;i++){
    polygon.col(3+i) = polygon.col(i) + Eigen::Vector3d(0,0,0.001);
  }

  Eigen::MatrixXd out;

  struct timeval prevTime;
  gettimeofday(&prevTime, NULL);
  bool solved = mesh_boolean_libigl::intersection(cube, polygon, out,false);
  struct timeval currentTime;
  gettimeofday(&currentTime, NULL);
  std::cerr << solved << " " << out.rows() << "x"<<out.cols() << " " << (currentTime.tv_sec - prevTime.tv_sec) + (currentTime.tv_usec - prevTime.tv_usec) * 1e-6 << "[s]" <<std::endl;
  std::cerr <<out<<std::endl;;
}

void sample2 (){
  Eigen::MatrixXd cube(3,8);
  cube.col(0) = Eigen::Vector3d(0,0,0);
  cube.col(1) = Eigen::Vector3d(1,0,0);
  cube.col(2) = Eigen::Vector3d(1,1,0);
  cube.col(3) = Eigen::Vector3d(0,1,0);
  cube.col(4) = Eigen::Vector3d(0,0,1);
  cube.col(5) = Eigen::Vector3d(1,0,1);
  cube.col(6) = Eigen::Vector3d(1,1,1);
  cube.col(7) = Eigen::Vector3d(0,1,1);

  Eigen::MatrixXd polygon(3,6);
  polygon.col(0) = Eigen::Vector3d(-1,0,1.5);
  polygon.col(1) = Eigen::Vector3d(2,0,1.5);
  polygon.col(2) = Eigen::Vector3d(0.5,1,1.5);
  for(int i=0;i<3;i++){
    polygon.col(3+i) = polygon.col(i) + Eigen::Vector3d(0,0,0.001);
  }

  Eigen::MatrixXd out;

  struct timeval prevTime;
  gettimeofday(&prevTime, NULL);
  bool solved = mesh_boolean_libigl::intersection(cube, polygon, out,false);
  struct timeval currentTime;
  gettimeofday(&currentTime, NULL);
  std::cerr << solved << " " << out.rows() << "x"<<out.cols() << " " << (currentTime.tv_sec - prevTime.tv_sec) + (currentTime.tv_usec - prevTime.tv_usec) * 1e-6 << "[s]" <<std::endl;
  std::cerr <<out<<std::endl;;
}

void sample3 (){
  Eigen::MatrixXd sphere(3,16*8);
  int idx = 0;
  for(int i=0;i<16;i++){
    for(int j=0;j<8;j++){
      sphere.col(idx) = Eigen::Vector3d(std::cos(M_PI/8*i),std::sin(M_PI/8*i),std::cos(M_PI/8*j));
      idx++;
    }
  }

  Eigen::MatrixXd polygon(3,8);
  polygon.col(0) = Eigen::Vector3d(-1,-1,0);
  polygon.col(1) = Eigen::Vector3d(-1,1,0);
  polygon.col(2) = Eigen::Vector3d(1,1,0);
  polygon.col(3) = Eigen::Vector3d(1,-1,0);
  for(int i=0;i<4;i++){
    polygon.col(4+i) = polygon.col(i) + Eigen::Vector3d(0,0,0.001);
  }

  Eigen::MatrixXd out;

  struct timeval prevTime;
  gettimeofday(&prevTime, NULL);
  bool solved = mesh_boolean_libigl::intersection(sphere, polygon, out,false);
  struct timeval currentTime;
  gettimeofday(&currentTime, NULL);
  std::cerr << solved << " " << out.rows() << "x"<<out.cols() << " " << (currentTime.tv_sec - prevTime.tv_sec) + (currentTime.tv_usec - prevTime.tv_usec) * 1e-6 << "[s]" <<std::endl;
  std::cerr <<out<<std::endl;;
}


int main(){
  sample1();
  sample2();
  sample3();

  return 0;
}

