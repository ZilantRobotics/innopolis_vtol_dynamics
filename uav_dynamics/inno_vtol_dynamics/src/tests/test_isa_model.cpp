/**
 * @file test_isa_model.cpp
 * @author Dmitry Ponomarev
 */
#include <gtest/gtest.h>
#include <cmath>
#include <iostream>
#include <geographiclib_conversions/geodetic_conv.hpp>
#include "sensors_isa_model.hpp"
#include <ros/ros.h>


TEST(IsaModel, diffPressureHpaMax){
    Eigen::Vector3d gpsPosition(55.7544426, 48.742684, 0),
                    linVelNed(100, 0, 0),
                    enuPosition(0, 0, 0);
    geodetic_converter::GeodeticConverter geodeticConverter;
    geodeticConverter.initialiseReference(gpsPosition[0], gpsPosition[1], gpsPosition[2]);
    geodeticConverter.enu2Geodetic(enuPosition[0], enuPosition[1], enuPosition[2],
                                    &gpsPosition[0], &gpsPosition[1], &gpsPosition[2]);
    double expectedDiffPressureHpa = 61.25;

    float temperatureKelvin, absPressureHpa, diffPressureHpa;
    SensorModelISA::EstimateAtmosphere(gpsPosition, linVelNed,
                                        temperatureKelvin, absPressureHpa, diffPressureHpa);
    EXPECT_NEAR(diffPressureHpa, expectedDiffPressureHpa, 0.01);
}
TEST(IsaModel, diffPressureHpaZero){
    Eigen::Vector3d gpsPosition(55.7544426, 48.742684, 0),
                    linVelNed(0, 0, 0),
                    enuPosition(0, 0, 0);
    geodetic_converter::GeodeticConverter geodeticConverter;
    geodeticConverter.initialiseReference(gpsPosition[0], gpsPosition[1], gpsPosition[2]);
    geodeticConverter.enu2Geodetic(enuPosition[0], enuPosition[1], enuPosition[2],
                                    &gpsPosition[0], &gpsPosition[1], &gpsPosition[2]);
    double expectedDiffPressureHpa = 0;

    float temperatureKelvin, absPressureHpa, diffPressureHpa;
    SensorModelISA::EstimateAtmosphere(gpsPosition, linVelNed,
                                        temperatureKelvin, absPressureHpa, diffPressureHpa);
    EXPECT_NEAR(diffPressureHpa, expectedDiffPressureHpa, 0.01);
}
TEST(IsaModel, diffPressureHpaMin){
    Eigen::Vector3d gpsPosition(55.7544426, 48.742684, 0),
                    linVelNed(-100, 0, 0),
                    enuPosition(0, 0, 0);
    geodetic_converter::GeodeticConverter geodeticConverter;
    geodeticConverter.initialiseReference(gpsPosition[0], gpsPosition[1], gpsPosition[2]);
    geodeticConverter.enu2Geodetic(enuPosition[0], enuPosition[1], enuPosition[2],
                                    &gpsPosition[0], &gpsPosition[1], &gpsPosition[2]);
    double expectedDiffPressureHpa = 61.25;

    float temperatureKelvin, absPressureHpa, diffPressureHpa;
    SensorModelISA::EstimateAtmosphere(gpsPosition, linVelNed,
                                        temperatureKelvin, absPressureHpa, diffPressureHpa);
    EXPECT_NEAR(diffPressureHpa, expectedDiffPressureHpa, 0.01);
}


int main(int argc, char *argv[]){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "isa_model_test");
    return RUN_ALL_TESTS();
}
