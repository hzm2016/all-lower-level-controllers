#include <iostream>
// #include <Eigen/Dense>
#include <ArduinoEigenDense.h>

int main() {
    Eigen::Matrix2d mat;
    mat << 1, 2,
           3, 4;
    std::cout << "Matrix:\n" << mat << std::endl;

    Eigen::Matrix2d inv = mat.inverse();
    std::cout << "Inverse:\n" << inv << std::endl;

    return 0;
} 

