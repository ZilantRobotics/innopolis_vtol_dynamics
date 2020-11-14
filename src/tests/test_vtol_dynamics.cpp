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

TEST(VtolDynamicsSim, findRowForPolynomial){
    VtolDynamicsSim vtolDynamicsSim;
    Eigen::MatrixXd table(8, 8);
    table << 5, -2.758e-11, 8.139e-09, 1.438e-07, -3.095e-05, -0.0003512, 0.05557, 0.4132,
            10, -3.934e-11, 8.204e-09, 1.935e-07, -3.075e-05, -0.0004209, 0.0552,  0.4438,
            15, -5.464e-11, 7.747e-09, 2.369e-07, -2.918e-05, -0.0004564, 0.05447, 0.4545,
            20, -5.087e-11, 7.803e-09, 2.267e-07, -2.926e-05, -0.0004493, 0.05435, 0.4525,
            25, -5.489e-11, 7.949e-09, 2.428e-07, -2.975e-05, -0.0004656, 0.05472, 0.4578,
            30, -4.749e-11, 7.778e-09, 2.219e-07, -2.926e-05, -0.0004567, 0.05433, 0.4599,
            35, -5.911e-11, 7.879e-09, 2.574e-07, -2.961e-05, -0.0004838, 0.05458, 0.4637,
            40, -5.911e-11, 7.879e-09, 2.574e-07, -2.961e-05, -0.0004838, 0.05458, 0.4637;
    ASSERT_TRUE(vtolDynamicsSim.findRowForPolynomial(table, -1) + 1 == 1);
    ASSERT_TRUE(vtolDynamicsSim.findRowForPolynomial(table, 10) + 1 == 1);
    ASSERT_TRUE(vtolDynamicsSim.findRowForPolynomial(table, 10.1) + 1 == 2);
    ASSERT_TRUE(vtolDynamicsSim.findRowForPolynomial(table, 15.1) + 1 == 3);
    ASSERT_TRUE(vtolDynamicsSim.findRowForPolynomial(table, 34.9) + 1 == 6);
    ASSERT_TRUE(vtolDynamicsSim.findRowForPolynomial(table, 35.1) + 1 == 7);
    ASSERT_TRUE(vtolDynamicsSim.findRowForPolynomial(table, 39.9) + 1 == 7);
    ASSERT_TRUE(vtolDynamicsSim.findRowForPolynomial(table, 40.1) + 1 == 7);
    ASSERT_TRUE(vtolDynamicsSim.findRowForPolynomial(table, 50.0) + 1 == 7);
}

TEST(VtolDynamicsSim, calculateCLPolynomial){
    VtolDynamicsSim vtolDynamicsSim;
    vtolDynamicsSim.init();
    Eigen::VectorXd calculatedpolynomialCoeffs(7);
    Eigen::VectorXd expectedPolynomialCoeffs(7);
    Eigen::VectorXd diff(7);
    auto isZeroComparator = [](double a) { return a < 0.00001;};

    expectedPolynomialCoeffs << -3.9340e-11, 8.2040e-09, 1.9350e-07, -3.0750e-05, -4.2090e-04, 0.055200, 0.44380;
    vtolDynamicsSim.calculateCLPolynomial(10, calculatedpolynomialCoeffs);
    diff = expectedPolynomialCoeffs - calculatedpolynomialCoeffs;
    ASSERT_TRUE(std::for_each(&diff[0], &diff[6], isZeroComparator));

    expectedPolynomialCoeffs << -1.5820e-11, 8.0740e-09, 9.4100e-08, -3.1150e-05, -2.8150e-04, 0.055940, 0.38260;
    vtolDynamicsSim.calculateCLPolynomial(0, calculatedpolynomialCoeffs);
    diff = expectedPolynomialCoeffs - calculatedpolynomialCoeffs;
    ASSERT_TRUE(std::for_each(&diff[0], &diff[6], isZeroComparator));

    expectedPolynomialCoeffs << 7.7000e-12, 7.9440e-09, -5.3000e-09, -3.1550e-05, -1.4210e-04, 0.056680, 0.32140;
    vtolDynamicsSim.calculateCLPolynomial(-10, calculatedpolynomialCoeffs);
    diff = expectedPolynomialCoeffs - calculatedpolynomialCoeffs;
    ASSERT_TRUE(std::for_each(&diff[0], &diff[6], isZeroComparator));

    expectedPolynomialCoeffs << -5.9110e-11, 7.8790e-09, 2.5740e-07, -2.9610e-05, -4.8380e-04, 0.054580, 0.46370;
    vtolDynamicsSim.calculateCLPolynomial(45, calculatedpolynomialCoeffs);
    diff = expectedPolynomialCoeffs - calculatedpolynomialCoeffs;
    ASSERT_TRUE(std::for_each(&diff[0], &diff[6], isZeroComparator));
}


TEST(VtolDynamicsSim, polyval){
    VtolDynamicsSim vtolDynamicsSim;
    Eigen::VectorXd poly(7);
    double value;

    poly << 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7;
    value = 0.5;
    ASSERT_TRUE(std::abs(vtolDynamicsSim.polyval(poly, value) - 3.1859) < 0.001);
}

TEST(VtolDynamicsSim, griddata){
    VtolDynamicsSim vtolDynamicsSim;
    vtolDynamicsSim.init();
    Eigen::MatrixXd x(1, 3);
    Eigen::MatrixXd y(1, 4);
    Eigen::MatrixXd f(4, 3);
    double x_value;
    double y_value;
    double result;

    x << 1, 2, 3;
    y << 2, 3, 4, 5;
    f << 2.5, 3.0, 3.5,
         3.0, 3.5, 4.0,
         3.5, 4.0, 4.5,
         4.0, 4.5, 5.0;

    x_value = 2.25;
    y_value = 3.75;
    ASSERT_TRUE(std::abs(vtolDynamicsSim.griddata(x, y, f, x_value, y_value) - 4.0) < 0.001);

    x_value = 1.1;
    y_value = 4.75;
    result = vtolDynamicsSim.griddata(x, y, f, x_value, y_value);
    ASSERT_TRUE(std::abs(result - 3.9250) < 0.001);
}

TEST(VtolDynamicsSim, calculateCSRudder){
    VtolDynamicsSim vtolDynamicsSim;
    vtolDynamicsSim.init();
    double rudder_position;
    double airspeed;
    double result;

    rudder_position = 15;
    airspeed = 10;
    result = vtolDynamicsSim.calculateCSRudder(rudder_position, airspeed);
    ASSERT_TRUE(std::abs(result - 0.028345) < 0.001);
}

TEST(VtolDynamicsSim, calculateAerodynamics){
    VtolDynamicsSim vtolDynamicsSim;
    vtolDynamicsSim.init();
    Eigen::Vector3d diff;
    Eigen::Vector3d expectedResult;
    auto isZeroComparator = [](double a) {return a < 0.0001;};

    Eigen::Vector3d airspeed(0.000001, -9.999999, 0.000001);
    double dynamicPressure = 44.399991;
    double AoA = 0.958191;
    double AoS = -1.570796;
    double aileron_pos = 0.000000;
    double elevator_pos = 0.000000;
    double rudder_pos = 0.000000;
    Eigen::Vector3d Faero;
    Eigen::Vector3d Maero;
    double Cmx_a;
    double Cmy_e;
    double Cmz_r;

    expectedResult = Eigen::Vector3d(0.000001, 29.513404, -0.000006);
    vtolDynamicsSim.calculateAerodynamics(airspeed, dynamicPressure, AoA, AoS, aileron_pos, elevator_pos, rudder_pos, Faero, Maero, Cmx_a, Cmy_e, Cmz_r);
    diff = expectedResult - Faero;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
}

TEST(VtolDynamicsSim, search){
    VtolDynamicsSim vtolDynamicsSim;
    Eigen::MatrixXd matrix(6, 1);

    matrix << 1, 2, 4, 7, 9, 11;
}

int main(int argc, char *argv[]){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}