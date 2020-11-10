#include <gtest/gtest.h>
#include <iostream>
#include <Eigen/Geometry>
#include <random>
#include "../vtolDynamicsSim.hpp"

TEST(VtolDynamicsSim, calculateWind){
    VtolDynamicsSim test;
    Eigen::Vector3d wind_mean_velocity;
    double wind_variance;

    wind_mean_velocity = Eigen::Vector3d(0, 10, 0);
    wind_variance = 0.0;
    test.setWindParameter(wind_mean_velocity, wind_variance);
    std::cout << test.calculateWind();

    ASSERT_TRUE(test.calculateWind() == Eigen::Vector3d(0, 10, 0));
}

TEST(VtolDynamicsSim, calculateRotationMatrix){
    VtolDynamicsSim test;
    Eigen::Vector3d eulerAngles;
    Eigen::Matrix3d rotatedMatrix;

    eulerAngles = Eigen::Vector3d(0, 0, 0);
    rotatedMatrix.setIdentity();
    test.setEulerAngles(eulerAngles);
    std::cout << test.calculateRotationMatrix();

    ASSERT_TRUE(test.calculateRotationMatrix() == rotatedMatrix);
}

int main(int argc, char *argv[]){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}