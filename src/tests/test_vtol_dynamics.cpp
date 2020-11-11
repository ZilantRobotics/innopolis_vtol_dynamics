#include <gtest/gtest.h>
#include <iostream>
#include <Eigen/Geometry>
#include <random>
#include "../vtolDynamicsSim.hpp"

TEST(VtolDynamicsSim, calculateWind){
    VtolDynamicsSim vtolDynamicsSim;
    Eigen::Vector3d wind_mean_velocity;
    double wind_variance;

    wind_mean_velocity = Eigen::Vector3d(0, 10, 0);
    wind_variance = 0.0;
    vtolDynamicsSim.setWindParameter(wind_mean_velocity, wind_variance);
    ASSERT_TRUE(vtolDynamicsSim.calculateWind() == Eigen::Vector3d(0, 10, 0));
}

TEST(VtolDynamicsSim, calculateRotationMatrix){
    VtolDynamicsSim vtolDynamicsSim;
    Eigen::Vector3d eulerAngles;
    Eigen::Matrix3d rotatedMatrix;

    eulerAngles = Eigen::Vector3d(0, 0, 0);
    rotatedMatrix.setIdentity();
    vtolDynamicsSim.setEulerAngles(eulerAngles);
    ASSERT_TRUE(vtolDynamicsSim.calculateRotationMatrix() == rotatedMatrix);

    eulerAngles = Eigen::Vector3d(15 * 3.14 / 180, 0, 0);
    rotatedMatrix << 1.0000e+00,    1.7453e-08,     -1.7453e-08,
                    -1.2341e-08,    9.6593e-01,     2.5882e-01,
                     2.1376e-08,    -2.5882e-01,    9.6593e-01;
    vtolDynamicsSim.setEulerAngles(eulerAngles);
    ASSERT_TRUE(vtolDynamicsSim.calculateRotationMatrix() == rotatedMatrix);

    eulerAngles = Eigen::Vector3d(0, 15 * 3.14 / 180, 0);
    rotatedMatrix << 9.6593e-01,    1.6859e-08,     -2.5882e-01,
                    -1.2936e-08,    1.0000e+00,     1.6859e-08,
                     2.5882e-01,    -1.2936e-08,    9.6593e-01;
    vtolDynamicsSim.setEulerAngles(eulerAngles);
    ASSERT_TRUE(vtolDynamicsSim.calculateRotationMatrix() == rotatedMatrix);

    eulerAngles = Eigen::Vector3d(0, 0, 15 * 3.14 / 180);
    rotatedMatrix << 9.6593e-01,    2.5882e-01,     -1.7453e-08,
                    -2.5882e-01,    9.6593e-01,     1.7453e-08,
                     2.1376e-08,    -1.2341e-08,    1.0000e+00;
    vtolDynamicsSim.setEulerAngles(eulerAngles);
    ASSERT_TRUE(vtolDynamicsSim.calculateRotationMatrix() == rotatedMatrix);
}

TEST(VtolDynamicsSim, calculateAnglesOfAtack){
    VtolDynamicsSim vtolDynamicsSim;
    Eigen::Vector3d airSpeed;

    airSpeed = Eigen::Vector3d(1, 2, 3);
    ASSERT_TRUE((vtolDynamicsSim.calculateAnglesOfAtack(airSpeed) - 1.2490) < 0.001);

    airSpeed = Eigen::Vector3d(0, 0, 0);
    ASSERT_TRUE((vtolDynamicsSim.calculateAnglesOfAtack(airSpeed) - 0) < 0.001);
}

TEST(VtolDynamicsSim, calculateAnglesOfSideslip){
    VtolDynamicsSim vtolDynamicsSim;
    Eigen::Vector3d airSpeed;

    airSpeed = Eigen::Vector3d(1, 2, 3);
    ASSERT_TRUE((vtolDynamicsSim.calculateAnglesOfSideslip(airSpeed) - 0.56394) < 0.001);

    airSpeed = Eigen::Vector3d(0, 0, 0);
    ASSERT_TRUE((vtolDynamicsSim.calculateAnglesOfSideslip(airSpeed) - 0) < 0.001);
}

int main(int argc, char *argv[]){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}