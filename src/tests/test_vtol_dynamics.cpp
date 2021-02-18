#include <gtest/gtest.h>
#include <iostream>
#include <Eigen/Geometry>
#include <random>
#include <geographiclib_conversions/geodetic_conv.hpp>
#include "../sensors_isa_model.hpp"
#include "../vtolDynamicsSim.hpp"

TEST(InnoVtolDynamicsSim, calculateWind){
    InnoVtolDynamicsSim vtolDynamicsSim;
    Eigen::Vector3d wind_mean_velocity;
    double wind_variance;

    wind_mean_velocity = Eigen::Vector3d(0, 10, 0);
    wind_variance = 0.0;
    vtolDynamicsSim.setWindParameter(wind_mean_velocity, wind_variance);
    ASSERT_TRUE(vtolDynamicsSim.calculateWind() == Eigen::Vector3d(0, 10, 0));
}

TEST(InnoVtolDynamicsSim, calculateAnglesOfAtack){
    InnoVtolDynamicsSim vtolDynamicsSim;
    Eigen::Vector3d airSpeed;
    double result, expected;

    std::vector<std::pair<Eigen::Vector3d, double>> dataset;
    dataset.push_back((std::make_pair(Eigen::Vector3d(0, 0, 0),     0.0)));

    dataset.push_back((std::make_pair(Eigen::Vector3d(10, 1, 1),    0.099669)));
    dataset.push_back((std::make_pair(Eigen::Vector3d(1, 10, 1),    0.785398)));
    dataset.push_back((std::make_pair(Eigen::Vector3d(1, 1, 10),    1.471128)));
    dataset.push_back((std::make_pair(Eigen::Vector3d(1, 2, 3),     1.2490)));

    dataset.push_back((std::make_pair(Eigen::Vector3d(-10, 1, 1),   3.041924)));
    dataset.push_back((std::make_pair(Eigen::Vector3d(-1, 10, 1),   2.356194)));
    dataset.push_back((std::make_pair(Eigen::Vector3d(-1, 1, 10),   1.670465)));
    dataset.push_back((std::make_pair(Eigen::Vector3d(-1, 2, 3),    1.892547)));

    dataset.push_back((std::make_pair(Eigen::Vector3d(10, 1, -1),   -0.099669)));
    dataset.push_back((std::make_pair(Eigen::Vector3d(1, 10, -1),   -0.785398)));
    dataset.push_back((std::make_pair(Eigen::Vector3d(1, 1, -10),   -1.471128)));
    dataset.push_back((std::make_pair(Eigen::Vector3d(1, 2, -3),    -1.249046)));

    for(auto pair : dataset){
        result = vtolDynamicsSim.calculateAnglesOfAtack(pair.first);
        ASSERT_TRUE(abs(result - pair.second) < 0.001);
    }
}

TEST(InnoVtolDynamicsSim, calculateAnglesOfSideslip){
    InnoVtolDynamicsSim vtolDynamicsSim;
    Eigen::Vector3d airSpeed;
    double result, expected;

    std::vector<std::pair<Eigen::Vector3d, double>> dataset;
    dataset.push_back((std::make_pair(Eigen::Vector3d(0, 0, 0),     0.0)));

    dataset.push_back((std::make_pair(Eigen::Vector3d(10, 1, 1),    0.099177)));
    dataset.push_back((std::make_pair(Eigen::Vector3d(1, 10, 1),    1.430307)));
    dataset.push_back((std::make_pair(Eigen::Vector3d(1, 1, 10),    0.099177)));
    dataset.push_back((std::make_pair(Eigen::Vector3d(1, 2, 3),     0.563943)));

    dataset.push_back((std::make_pair(Eigen::Vector3d(10, -1, 1),   -0.099177)));
    dataset.push_back((std::make_pair(Eigen::Vector3d(1, -10, 1),   -1.430307)));
    dataset.push_back((std::make_pair(Eigen::Vector3d(1, -1, 10),   -0.099177)));
    dataset.push_back((std::make_pair(Eigen::Vector3d(1, -2, 3),    -0.563943)));

    dataset.push_back((std::make_pair(Eigen::Vector3d(10, 1, -1),   0.099177)));
    dataset.push_back((std::make_pair(Eigen::Vector3d(1, 10, -1),   1.430307)));
    dataset.push_back((std::make_pair(Eigen::Vector3d(1, 1, -10),   0.099177)));
    dataset.push_back((std::make_pair(Eigen::Vector3d(1, 2, -3),    0.563943)));

    for(auto pair : dataset){
        result = vtolDynamicsSim.calculateAnglesOfSideslip(pair.first);
        ASSERT_TRUE(abs(result - pair.second) < 0.001);
    }
}

TEST(InnoVtolDynamicsSim, findRow){
    InnoVtolDynamicsSim vtolDynamicsSim;
    Eigen::MatrixXd table(8, 8);
    table << 5, -2.758e-11, 8.139e-09, 1.438e-07, -3.095e-05, -0.0003512, 0.05557, 0.4132,
            10, -3.934e-11, 8.204e-09, 1.935e-07, -3.075e-05, -0.0004209, 0.0552,  0.4438,
            15, -5.464e-11, 7.747e-09, 2.369e-07, -2.918e-05, -0.0004564, 0.05447, 0.4545,
            20, -5.087e-11, 7.803e-09, 2.267e-07, -2.926e-05, -0.0004493, 0.05435, 0.4525,
            25, -5.489e-11, 7.949e-09, 2.428e-07, -2.975e-05, -0.0004656, 0.05472, 0.4578,
            30, -4.749e-11, 7.778e-09, 2.219e-07, -2.926e-05, -0.0004567, 0.05433, 0.4599,
            35, -5.911e-11, 7.879e-09, 2.574e-07, -2.961e-05, -0.0004838, 0.05458, 0.4637,
            40, -5.911e-11, 7.879e-09, 2.574e-07, -2.961e-05, -0.0004838, 0.05458, 0.4637;
    ASSERT_EQ(vtolDynamicsSim.findRow(table, -1) + 1,   1);
    ASSERT_EQ(vtolDynamicsSim.findRow(table, 10) + 1,   1);
    ASSERT_EQ(vtolDynamicsSim.findRow(table, 10.1) + 1, 2);
    ASSERT_EQ(vtolDynamicsSim.findRow(table, 15.1) + 1, 3);
    ASSERT_EQ(vtolDynamicsSim.findRow(table, 34.9) + 1, 6);
    ASSERT_EQ(vtolDynamicsSim.findRow(table, 35.1) + 1, 7);
    ASSERT_EQ(vtolDynamicsSim.findRow(table, 39.9) + 1, 7);
    ASSERT_EQ(vtolDynamicsSim.findRow(table, 40.1) + 1, 7);
    ASSERT_EQ(vtolDynamicsSim.findRow(table, 50.0) + 1, 7);
}

TEST(InnoVtolDynamicsSim, calculateCLPolynomial){
    InnoVtolDynamicsSim vtolDynamicsSim;
    vtolDynamicsSim.init();
    Eigen::VectorXd calculatedpolynomialCoeffs(7);
    Eigen::VectorXd expectedPolynomialCoeffs(7);
    Eigen::VectorXd diff(7);
    auto isZeroComparator = [](double a) { return abs(a) < 0.00001;};

    expectedPolynomialCoeffs << -3.9340e-11, 8.2040e-09, 1.9350e-07, -3.0750e-05,
                                -4.2090e-04, 0.055200, 0.44380;
    vtolDynamicsSim.calculateCLPolynomial(10, calculatedpolynomialCoeffs);
    diff = expectedPolynomialCoeffs - calculatedpolynomialCoeffs;
    ASSERT_TRUE(std::for_each(&diff[0], &diff[6], isZeroComparator));

    expectedPolynomialCoeffs << -1.5820e-11, 8.0740e-09, 9.4100e-08, -3.1150e-05,
                                -2.8150e-04, 0.055940, 0.38260;
    vtolDynamicsSim.calculateCLPolynomial(0, calculatedpolynomialCoeffs);
    diff = expectedPolynomialCoeffs - calculatedpolynomialCoeffs;
    ASSERT_TRUE(std::for_each(&diff[0], &diff[6], isZeroComparator));

    expectedPolynomialCoeffs << 7.7000e-12, 7.9440e-09, -5.3000e-09, -3.1550e-05,
                                -1.4210e-04, 0.056680, 0.32140;
    vtolDynamicsSim.calculateCLPolynomial(-10, calculatedpolynomialCoeffs);
    diff = expectedPolynomialCoeffs - calculatedpolynomialCoeffs;
    ASSERT_TRUE(std::for_each(&diff[0], &diff[6], isZeroComparator));

    expectedPolynomialCoeffs << -5.9110e-11, 7.8790e-09, 2.5740e-07, -2.9610e-05,
                                -4.8380e-04, 0.054580, 0.46370;
    vtolDynamicsSim.calculateCLPolynomial(45, calculatedpolynomialCoeffs);
    diff = expectedPolynomialCoeffs - calculatedpolynomialCoeffs;
    ASSERT_TRUE(std::for_each(&diff[0], &diff[6], isZeroComparator));
}

TEST(InnoVtolDynamicsSim, DISABLED_calculateLiftForce){
    InnoVtolDynamicsSim vtolDynamicsSim;
    vtolDynamicsSim.init();
    auto isZeroComparator = [](double a) { return abs(a) < 0.00001;};

    Eigen::VectorXd polynomialCoeffs(7);
    Eigen::Vector3d FL, airspeed;
    double CL, dynamicPressure, AoA_deg, airspeedNorm, AoA_rad;

    airspeedNorm = 10;
    for(AoA_deg = -45; AoA_deg <= 45; AoA_deg += 5){
        AoA_rad = AoA_deg / 180 * 3.1415;
        airspeed << airspeedNorm * cos(AoA_rad), 0, airspeedNorm * sin(AoA_rad);
        dynamicPressure = vtolDynamicsSim.calculateDynamicPressure(airspeed.norm());
        vtolDynamicsSim.calculateCLPolynomial(airspeedNorm, polynomialCoeffs);
        CL = vtolDynamicsSim.polyval(polynomialCoeffs, AoA_deg);
        FL = 0.5 * dynamicPressure * (Eigen::Vector3d(0, 1, 0).cross(airspeed.normalized())) * CL;
        std::cout << AoA_deg << " FL = " << FL.transpose() << ", " << airspeed.norm() << std::endl;
    }
}

TEST(InnoVtolDynamicsSim, DISABLED_estimate_atmosphere){
    InnoVtolDynamicsSim vtolDynamicsSim;
    vtolDynamicsSim.init();

    Eigen::Vector3d gpsPosition, linVelNed, enuPosition;
    float temperatureKelvin, absPressureHpa, diffPressureHpa;

    geodetic_converter::GeodeticConverter geodeticConverter;
    geodeticConverter.initialiseReference(55.7544426, 48.742684, 0);

    for(double pose = 0; pose <= 100; pose += 10){
        enuPosition << 0, 0, pose;
        gpsPosition << 55.7544426, 48.742684, 0;
        linVelNed << 0, 0, 0;
        geodeticConverter.enu2Geodetic(enuPosition[0], enuPosition[1], enuPosition[2],
                                       &gpsPosition[0], &gpsPosition[1], &gpsPosition[2]);
        SensorModelISA::EstimateAtmosphere(gpsPosition, linVelNed,
                                           temperatureKelvin, absPressureHpa, diffPressureHpa);
        std::cout << pose << ": " << temperatureKelvin << ", " << gpsPosition[2] << ", " << absPressureHpa << ", " << diffPressureHpa << std::endl;
    }
}

TEST(InnoVtolDynamicsSim, polyval){
    InnoVtolDynamicsSim vtolDynamicsSim;
    Eigen::VectorXd poly(7);
    double value;

    poly << 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7;
    value = 0.5;
    ASSERT_TRUE(std::abs(vtolDynamicsSim.polyval(poly, value) - 3.1859) < 0.001);
}

TEST(InnoVtolDynamicsSim, griddata){
    InnoVtolDynamicsSim vtolDynamicsSim;
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

TEST(InnoVtolDynamicsSim, calculateCSRudder){
    InnoVtolDynamicsSim vtolDynamicsSim;
    vtolDynamicsSim.init();

    struct DataSet{
        double rudder_position;
        double airspeed;
        double expected;
    };
    std::vector<DataSet> data_set;
    double result;

    data_set.push_back({.rudder_position=0,     .airspeed=5,    .expected=-1.5009e-04});
    data_set.push_back({.rudder_position=0,     .airspeed=5.1,  .expected=-1.2303e-04});
    data_set.push_back({.rudder_position=0,     .airspeed=8.5,  .expected=5.9762e-04});
    data_set.push_back({.rudder_position=0,     .airspeed=8.66025,  .expected=6.0903e-04});
    data_set.push_back({.rudder_position=0,     .airspeed=10,   .expected=7.0445e-04});
    data_set.push_back({.rudder_position=0,     .airspeed=20,   .expected=9.2322e-04});
    data_set.push_back({.rudder_position=0,     .airspeed=40,   .expected=-0.0013107});
    data_set.push_back({.rudder_position=-20,   .airspeed=5,    .expected=-0.034155});
    data_set.push_back({.rudder_position=0,     .airspeed=5,    .expected=-1.5009e-04});
    data_set.push_back({.rudder_position=20,    .airspeed=5,    .expected=0.037053});

    for(auto test_case : data_set){
        result = vtolDynamicsSim.calculateCSRudder(test_case.rudder_position, test_case.airspeed);
        ASSERT_TRUE(std::abs(result - test_case.expected) < 0.001);
    }
}

TEST(InnoVtolDynamicsSim, calculateCSBeta){
    InnoVtolDynamicsSim vtolDynamicsSim;
    vtolDynamicsSim.init();

    struct DataSet{
        double aos_degree;
        double airspeed;
        double expected;
    };
    std::vector<DataSet> data_set;
    double result;

    data_set.push_back({.aos_degree=0,      .airspeed=5,        .expected=-0.0032540});
    data_set.push_back({.aos_degree=0,      .airspeed=10,       .expected=-0.0040036});
    data_set.push_back({.aos_degree=0,      .airspeed=15,       .expected=-0.0037597});
    data_set.push_back({.aos_degree=0,      .airspeed=20,       .expected=-0.0033221});


    for(auto test_case : data_set){
        result = vtolDynamicsSim.calculateCSBeta(test_case.aos_degree, test_case.airspeed);
        ASSERT_TRUE(std::abs(result - test_case.expected) < 0.0000001);
    }
}

TEST(InnoVtolDynamicsSim, DISABLED_calculateCmxAileron){
    InnoVtolDynamicsSim vtolDynamicsSim;
    vtolDynamicsSim.init();
    double Cmx_aileron, airspeedNorm, aileron_pos, dynamicPressure;
    double characteristicLength = 1.5;

    airspeedNorm = 20;
    dynamicPressure = vtolDynamicsSim.calculateDynamicPressure(airspeedNorm);
    for(aileron_pos = -2e1; aileron_pos <= 2e1; aileron_pos += 4){
        Cmx_aileron = vtolDynamicsSim.calculateCmyElevator(aileron_pos, airspeedNorm);
        Cmx_aileron *= 0.5 * dynamicPressure * characteristicLength;
        std::cout << aileron_pos << " Cmx_aileron = " << Cmx_aileron << std::endl;
    }
}

TEST(InnoVtolDynamicsSim, calculateAerodynamics){
    InnoVtolDynamicsSim vtolDynamicsSim;
    vtolDynamicsSim.init();
    Eigen::Vector3d diff, expectedResult, Faero, Maero;
    double Cmx_a, Cmy_e, Cmz_r;
    auto isZeroComparator = [](double a) {return abs(a) < 0.001;};

    Eigen::Vector3d airspeed(0.000001, -9.999999, 0.000001);
    double AoA = 0.958191;
    double AoS = -1.570796;
    double aileron_pos = 0.000000;
    double elevator_pos = 0.000000;
    double rudder_pos = 0.000000;

    Eigen::Vector3d extectedFaero(-4.8133e-07, 2.9513e+01, -6.0493e-06);
    Eigen::Vector3d extectedMaero(0.21470, 0.69480, -0.31633);

    vtolDynamicsSim.calculateAerodynamics(airspeed, AoA, AoS, aileron_pos, elevator_pos, rudder_pos,
                                          Faero, Maero);
    diff = extectedFaero - Faero;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
    diff = extectedMaero - Maero;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
}

TEST(InnoVtolDynamicsSim, calculateAerodynamicsCaseAileron){
    InnoVtolDynamicsSim vtolDynamicsSim;
    vtolDynamicsSim.init();
    Eigen::Vector3d diff, expectedResult, Faero, Maero;
    double Cmx_a, Cmy_e, Cmz_r;
    auto isZeroComparator = [](double a) {return abs(a) < 0.02;};

    Eigen::Vector3d airspeed(5, 5, 5);
    double AoA = 0.1;
    double AoS = 0.1;
    double aileron_pos = 0.5;
    double elevator_pos = 0.0;
    double rudder_pos = 0.0;

    Eigen::Vector3d extectedFaero(7.4133, -4.3077, -6.6924);
    Eigen::Vector3d extectedMaero(0.333818, 1.754507, -0.037038);

    vtolDynamicsSim.calculateAerodynamics(airspeed, AoA, AoS, aileron_pos, elevator_pos, rudder_pos,
                                          Faero, Maero);
    diff = extectedFaero - Faero;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
    diff = extectedMaero - Maero;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
}

TEST(InnoVtolDynamicsSim, calculateAerodynamicsCaseElevator){
    InnoVtolDynamicsSim vtolDynamicsSim;
    vtolDynamicsSim.init();
    Eigen::Vector3d diff, expectedResult, Faero, Maero;
    double Cmx_a, Cmy_e, Cmz_r;
    auto isZeroComparator = [](double a) {return abs(a) < 0.02;};

    Eigen::Vector3d airspeed(5, 5, 5);
    double AoA = 0.1;
    double AoS = 0.1;
    double aileron_pos = 0.0;
    double elevator_pos = 5;
    double rudder_pos = 0.0;

    Eigen::Vector3d extectedFaero(7.4133, -4.3077, -6.6924);
    Eigen::Vector3d extectedMaero(0.190243, 1.220935, -0.037038);

    vtolDynamicsSim.calculateAerodynamics(airspeed, AoA, AoS, aileron_pos, elevator_pos, rudder_pos,
                                          Faero, Maero);
    diff = extectedFaero - Faero;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
    diff = extectedMaero - Maero;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
}

TEST(InnoVtolDynamicsSim, calculateAerodynamicsAoA){
    InnoVtolDynamicsSim vtolDynamicsSim;
    vtolDynamicsSim.init();
    Eigen::Vector3d diff, expectedResult, Faero, Maero;
    double Cmx_a, Cmy_e, Cmz_r;
    auto isZeroComparator = [](double a) {return abs(a) < 0.04;};

    Eigen::Vector3d airspeed(5, 5, 5);
    double AoA = 27.0 * 3.1415 / 180.0;
    double AoS = 0;
    double aileron_pos = 0.0;
    double elevator_pos = 0.0;
    double rudder_pos = 0.0;

    Eigen::Vector3d extectedFaero(6.0625, -7.7260, -17.5536);
    Eigen::Vector3d extectedMaero(0.16512, 1.26568, -0.11093);

    vtolDynamicsSim.calculateAerodynamics(airspeed, AoA, AoS, aileron_pos, elevator_pos, rudder_pos,
                                          Faero, Maero);
    diff = extectedFaero - Faero;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
    diff = extectedMaero - Maero;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
}

TEST(InnoVtolDynamicsSim, calculateAerodynamicsRealCase){
    InnoVtolDynamicsSim vtolDynamicsSim;
    vtolDynamicsSim.init();
    Eigen::Vector3d diff, expectedResult, Faero, Maero;
    double Cmx_a, Cmy_e, Cmz_r;
    auto isZeroComparator = [](double a) {return abs(a) < 0.02;};

    Eigen::Vector3d airspeed(2.93128, 0.619653, 0.266774);
    double AoA = 45 * 3.1415 / 180.0;
    double AoS = 11.8888 * 3.1415 / 180.0;
    double aileron_pos = 0.0;
    double elevator_pos = 0.0;
    double rudder_pos = 0.0;

    Eigen::Vector3d extectedFaero(-2.28665, -0.92928, -2.66499);
    Eigen::Vector3d extectedMaero(0.017652, 0.074924, -0.024468);

    vtolDynamicsSim.calculateAerodynamics(airspeed, AoA, AoS, aileron_pos, elevator_pos, rudder_pos,
                                          Faero, Maero);
    diff = extectedFaero - Faero;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
    diff = extectedMaero - Maero;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
}

TEST(InnoVtolDynamicsSim, thruster){
    InnoVtolDynamicsSim vtolDynamicsSim;
    vtolDynamicsSim.init();
    double thrust, torque;

    vtolDynamicsSim.thruster(500.004648, thrust, torque);
    ASSERT_TRUE(abs(thrust - 15.8930) < 0.001);
    ASSERT_TRUE(abs(torque - 0.27273) < 0.00001);

    vtolDynamicsSim.thruster(134.254698, thrust, torque);
    ASSERT_TRUE(abs(thrust - 3.590800) < 0.0001);
    ASSERT_TRUE(abs(torque - 0.013696) < 0.000001);
}

/**
 * @note In InnoDynamics the altitude is directed to the bottom, but in this simulator
 * it is directed to the top, so we perform invertion.
 */
void calculateNewState(double dt,
                    std::vector<double> actuators,
                    Eigen::Vector3d Maero,
                    Eigen::Vector3d Faero,
                    Eigen::Vector3d initialLinearVelocity,
                    Eigen::Vector3d initialAngularVelocity,
                    Eigen::Vector3d initialPosition,
                    Eigen::Quaterniond initialAttitude,
                    Eigen::Vector3d& expectedAngAccel,
                    Eigen::Vector3d& expectedLinAccel,
                    Eigen::Vector3d& angularAcceleration,
                    Eigen::Vector3d& linearAcceleration){
    InnoVtolDynamicsSim vtolDynamicsSim;
    vtolDynamicsSim.init();
    vtolDynamicsSim.setInitialVelocity(initialLinearVelocity, initialAngularVelocity);
    vtolDynamicsSim.setInitialPosition(initialPosition, initialAttitude);

    vtolDynamicsSim.calculateNewState(Maero, Faero, actuators, dt);
    angularAcceleration = vtolDynamicsSim.getAngularAcceleration();
    linearAcceleration = vtolDynamicsSim.getLinearAcceleration();
}

TEST(InnoVtolDynamicsSim, calculateNewStateFirstCaseOnlyAttitude){
    double dt = 0.002500;
    std::vector<double> actuators{0, 0, 0, 0, 0, 0, 0, 0};
    Eigen::Quaterniond initAttitude(1, 0.2, 0.10, 0.05);
    Eigen::Vector3d angAccel, linAccel, diff,
                    Faero(0.0, 0.0, 0.0),
                    Maero(0.0, 0.0, 0.0),
                    initialLinVel(0, 0, 0),
                    initAngVel(0, 0, 0),
                    initPose(0, 0, 0),
                    expectedAngAccel(0.0, 0.0, 0.0),
                    expectedLinAccel(2.5377e-16, -5.0753e-16, 9.8066e+00);
    auto isZeroComparator = [](double a) {return abs(a) < 1e-04;};

    calculateNewState(dt, actuators,  Maero, Faero,
                      initialLinVel, initAngVel, initPose, initAttitude,
                      expectedAngAccel, expectedLinAccel,
                      angAccel, linAccel);

    diff = expectedAngAccel - angAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
    diff = expectedLinAccel - linAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
}

TEST(InnoVtolDynamicsSim, calculateNewStateSecondCaseOnlyAngularVelocity){
    double dt = 0.002500;
    std::vector<double> actuators{0, 0, 0, 0, 0, 0, 0, 0};
    Eigen::Quaterniond initAttitude(1, 0.00, 0.00, 0.00);
    Eigen::Vector3d angAccel, linAccel, diff,
                    Faero(0.0, 0.0, 0.0),
                    Maero(0.0, 0.0, 0.0),
                    initialLinVel(0, 0, 0),
                    initAngVel(0.3, 0.2, 0.1),
                    initPose(0, 0, 0),
                    expectedAngAccel(-1.9719e-02,   2.9589e-02,     -8.3459e-04),
                    expectedLinAccel(9.9127e-19,    1.9825e-18,     9.8066e+00);
    auto isZeroComparator = [](double a) {return abs(a) < 6e-05;};

    calculateNewState(dt, actuators,  Maero, Faero,
                      initialLinVel, initAngVel, initPose, initAttitude,
                      expectedAngAccel, expectedLinAccel,
                      angAccel, linAccel);

    // std::cout << angAccel.transpose() << std::endl;
    // std::cout << linAccel.transpose() << std::endl;
    diff = expectedAngAccel - angAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
    diff = expectedLinAccel - linAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
}

TEST(InnoVtolDynamicsSim, calculateNewStateThirdCaseOnlyFaero){
    double dt = 0.002500;
    std::vector<double> actuators{0, 0, 0, 0, 0, 0, 0, 0};
    Eigen::Quaterniond initAttitude(1, 0.00, 0.00, 0.00);
    Eigen::Vector3d angAccel, linAccel, diff,
                    Faero(5.7448e-01, 2.9513e+01, 6.1333e-01),
                    Maero(0.0, 0.0, 0.0),
                    initialLinVel(0, 0, 0),
                    initAngVel(0.0, 0.0, 0.0),
                    initPose(0, 0, 0),
                    expectedAngAccel(0.0,       0.0,        0.0),
                    expectedLinAccel(0.082069,  4.216143,   9.894269);
    auto isZeroComparator = [](double a) {return abs(a) < 6e-05;};

    calculateNewState(dt, actuators,  Maero, Faero,
                      initialLinVel, initAngVel, initPose, initAttitude,
                      expectedAngAccel, expectedLinAccel,
                      angAccel, linAccel);

    diff = expectedAngAccel - angAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
    diff = expectedLinAccel - linAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
}

TEST(InnoVtolDynamicsSim, calculateNewStateFourthCaseOnlyMaero){
    double dt = 0.002500;
    std::vector<double> actuators{0, 0, 0, 0, 0, 0, 0, 0};
    Eigen::Quaterniond initAttitude(1, 0.00, 0.00, 0.00);
    Eigen::Vector3d angAccel, linAccel, diff,
                    Faero(0.0, 0.0, 0.0),
                    Maero(-0.214696, -0.694801, -0.316328),
                    initialLinVel(0, 0, 0),
                    initAngVel(0.0, 0.0, 0.0),
                    initPose(0, 0, 0),
                    expectedAngAccel(-0.34251,      -1.07821,       -0.25057),
                    expectedLinAccel(7.7443e-21,    -3.8722e-21,    9.8066e+00);
    auto isZeroComparator = [](double a) {return abs(a) < 6e-05;};

    calculateNewState(dt, actuators,  Maero, Faero,
                      initialLinVel, initAngVel, initPose, initAttitude,
                      expectedAngAccel, expectedLinAccel,
                      angAccel, linAccel);

    diff = expectedAngAccel - angAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
    diff = expectedLinAccel - linAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
}

TEST(InnoVtolDynamicsSim, calculateNewStateFifthCaseOnlyCopterMotorsWithEqualPower){
    double dt = 0.002500;
    std::vector<double> actuators{700, 700, 700, 700, 0, 0, 0, 0};
    Eigen::Quaterniond initAttitude(1, 0.00, 0.00, 0.00);
    Eigen::Vector3d angAccel, linAccel, diff,
                    Faero(0.0, 0.0, 0.0),
                    Maero(0.0, 0.0, 0.0),
                    initialLinVel(0, 0, 0),
                    initAngVel(0.0, 0.0, 0.0),
                    initPose(0, 0, 0),
                    expectedAngAccel(0.00000,       0.00000,        0.00000),
                    expectedLinAccel(0.00000,       0.00000,        -6.36769);
    auto isZeroComparator = [](double a) {return abs(a) < 6e-05;};

    calculateNewState(dt, actuators,  Maero, Faero,
                      initialLinVel, initAngVel, initPose, initAttitude,
                      expectedAngAccel, expectedLinAccel,
                      angAccel, linAccel);

    diff = expectedAngAccel - angAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
    diff = expectedLinAccel - linAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
}

TEST(InnoVtolDynamicsSim, calculateNewStateSixthCaseOnlyCopterMotorsWithNotEqualPower){
    double dt = 0.002500;
    std::vector<double> actuators{700, 680, 660, 640, 0, 0, 0, 0};
    Eigen::Quaterniond initAttitude(1, 0.00, 0.00, 0.00);
    Eigen::Vector3d angAccel, linAccel, diff,
                    Faero(0.0, 0.0, 0.0),
                    Maero(0.0, 0.0, 0.0),
                    initialLinVel(0, 0, 0),
                    initAngVel(0.0, 0.0, 0.0),
                    initPose(0, 0, 0),
                    expectedAngAccel(0.13933,       1.48111,        0.10723),
                    expectedLinAccel(-1.3753e-04,   1.2938e-05,     -5.0505e+00);
    auto isZeroComparator = [](double a) {return abs(a) < 6e-05;};

    calculateNewState(dt, actuators,  Maero, Faero,
                      initialLinVel, initAngVel, initPose, initAttitude,
                      expectedAngAccel, expectedLinAccel,
                      angAccel, linAccel);

    diff = expectedAngAccel - angAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
    diff = expectedLinAccel - linAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
}

TEST(InnoVtolDynamicsSim, calculateNewStateSeventhCaseOnlyICE){
    double dt = 0.002500;
    std::vector<double> actuators{0, 0, 0, 0, 500, 0, 0, 0};
    Eigen::Quaterniond initAttitude(1, 0.00, 0.00, 0.00);
    Eigen::Vector3d angAccel, linAccel, diff,
                    Faero(0.0, 0.0, 0.0),
                    Maero(0.0, 0.0, 0.0),
                    initialLinVel(0, 0, 0),
                    initAngVel(0.0, 0.0, 0.0),
                    initPose(0, 0, 0),
                    expectedAngAccel(-0.43508,      0.00000,        0.00000),
                    expectedLinAccel(2.2705e+00,    3.8722e-21,     9.8066e+00);
    auto isZeroComparator = [](double a) {return abs(a) < 6e-05;};

    calculateNewState(dt, actuators,  Maero, Faero,
                      initialLinVel, initAngVel, initPose, initAttitude,
                      expectedAngAccel, expectedLinAccel,
                      angAccel, linAccel);

    diff = expectedAngAccel - angAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
    diff = expectedLinAccel - linAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
}

TEST(InnoVtolDynamicsSim, calculateNewStateEightComplexWithoutInitialAttitude){
    double dt = 0.002500;
    std::vector<double> actuators{600, 550, 450, 500, 650, 0, 0, 0};
    Eigen::Quaterniond initAttitude(1, 0, 0, 0);

    Eigen::Vector3d angAccel, linAccel, diff,
                    Faero(15.0, 10.0, 5.0),
                    Maero(5.0, 10.0, 15.0),
                    initialLinVel(15, 3, 1),
                    initAngVel(0.5, 0.4, 0.3),
                    initPose(0, 0, 10),
                    expectedAngAccel(5.0598, 16.2287, 11.9625),
                    expectedLinAccel(5.60908, 1.44474, 0.80233);
    auto isZeroComparator = [](double a) {return abs(a) < 1e-03;};

    calculateNewState(dt, actuators,  Maero, Faero,
                      initialLinVel, initAngVel, initPose, initAttitude,
                      expectedAngAccel, expectedLinAccel,
                      angAccel, linAccel);

    // std::cout << angAccel.transpose() << std::endl;
    // std::cout << linAccel.transpose() << std::endl;
    diff = expectedAngAccel - angAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
    diff = expectedLinAccel - linAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
}

TEST(InnoVtolDynamicsSim, calculateNewStateEightComplexFull){
    double dt = 0.002500;
    std::vector<double> actuators{600, 550, 450, 500, 650, 4, 7, 11};
    Eigen::Quaterniond initAttitude(0.9833, 0.1436, 0.106, 0.03427);    // rpy = [0.3, 0.2, 0.1]

    Eigen::Vector3d angAccel, linAccel, diff,
                    Faero(15.0, 10.0, 5.0),
                    Maero(5.0, 10.0, 15.0),
                    initialLinVel(15, 3, 1),
                    initAngVel(0.5, 0.4, 0.3),
                    initPose(0, 0, 10),
                    expectedAngAccel(5.0598, 16.2287, 11.9625),
                    expectedLinAccel(3.45031, 4.40765, 0.68005);
    auto isZeroComparator = [](double a) {return abs(a) < 1e-03;};

    calculateNewState(dt, actuators,  Maero, Faero,
                      initialLinVel, initAngVel, initPose, initAttitude,
                      expectedAngAccel, expectedLinAccel,
                      angAccel, linAccel);

    // std::cout << angAccel.transpose() << std::endl;
    // std::cout << linAccel.transpose() << std::endl;
    diff = expectedAngAccel - angAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
    diff = expectedLinAccel - linAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
}

int main(int argc, char *argv[]){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    return RUN_ALL_TESTS();
}
