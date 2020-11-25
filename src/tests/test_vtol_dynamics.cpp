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

TEST(VtolDynamicsSim, findRow){
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

TEST(VtolDynamicsSim, calculateCLPolynomial){
    VtolDynamicsSim vtolDynamicsSim;
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
        std::cout << "- " << test_case.rudder_position << ", " << test_case.airspeed << std::endl;
        std::cout << "- " << test_case.expected << ", " << result << std::endl << std::endl;

        ASSERT_TRUE(std::abs(result - test_case.expected) < 0.001);
    }
}

TEST(VtolDynamicsSim, calculateCSBeta){
    VtolDynamicsSim vtolDynamicsSim;
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
        std::cout << "- " << test_case.aos_degree << ", " << test_case.airspeed << std::endl;
        std::cout << "- " << test_case.expected << ", " << result << std::endl << std::endl;

        ASSERT_TRUE(std::abs(result - test_case.expected) < 0.0000001);
    }
}

TEST(VtolDynamicsSim, calculateAerodynamics){
    VtolDynamicsSim vtolDynamicsSim;
    vtolDynamicsSim.init();
    Eigen::Vector3d diff, expectedResult, Faero, Maero;
    double Cmx_a, Cmy_e, Cmz_r;
    auto isZeroComparator = [](double a) {return abs(a) < 0.001;};

    Eigen::Vector3d airspeed(0.000001, -9.999999, 0.000001);
    double dynamicPressure = 44.399991;
    double AoA = 0.958191;
    double AoS = -1.570796;
    double aileron_pos = 0.000000;
    double elevator_pos = 0.000000;
    double rudder_pos = 0.000000;

    Eigen::Vector3d extectedFaero(-4.8133e-07, 2.9513e+01, -6.0493e-06);
    Eigen::Vector3d extectedMaero(0.21470, 0.69480, -0.31633);
    double extectedCmx_a = 0.011426;
    double extectedCmy_e = 0.0049566;
    double extectedCmz_r = 0.0022183;

    vtolDynamicsSim.calculateAerodynamics(airspeed,
                                          dynamicPressure,
                                          AoA,
                                          AoS,
                                          aileron_pos,
                                          elevator_pos,
                                          rudder_pos,
                                          Faero, Maero, Cmx_a, Cmy_e, Cmz_r);
    diff = extectedFaero - Faero;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
    diff = extectedMaero - Maero;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
    ASSERT_TRUE(isZeroComparator(extectedCmx_a - Cmx_a));
    ASSERT_TRUE(isZeroComparator(extectedCmy_e - Cmy_e));
    ASSERT_TRUE(isZeroComparator(extectedCmz_r - Cmz_r));
}

TEST(VtolDynamicsSim, calculateAerodynamicsCaseAileron){
    VtolDynamicsSim vtolDynamicsSim;
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
    double dynamicPressure = vtolDynamicsSim.calculateDynamicPressure(airspeed.norm());

    Eigen::Vector3d extectedFaero(7.4133, -4.3077, -6.6924);
    Eigen::Vector3d extectedMaero(0.333818, 1.754507, -0.037038);
    double extectedCmx_a = 0.011498;
    double extectedCmy_e = 0.0050693;
    double extectedCmz_r = 0.0019689;

    vtolDynamicsSim.calculateAerodynamics(airspeed,
                                          dynamicPressure,
                                          AoA,
                                          AoS,
                                          aileron_pos,
                                          elevator_pos,
                                          rudder_pos,
                                          Faero, Maero, Cmx_a, Cmy_e, Cmz_r);
    diff = extectedFaero - Faero;

    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
    diff = extectedMaero - Maero;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
    ASSERT_TRUE(isZeroComparator(extectedCmx_a - Cmx_a));
    ASSERT_TRUE(isZeroComparator(extectedCmy_e - Cmy_e));
    ASSERT_TRUE(isZeroComparator(extectedCmz_r - Cmz_r));
}

TEST(VtolDynamicsSim, thruster){
    VtolDynamicsSim vtolDynamicsSim;
    vtolDynamicsSim.init();
    double thrust, torque, kf, km;

    vtolDynamicsSim.thruster(500.004648, thrust, torque, kf, km);
    ASSERT_TRUE(abs(thrust -    15.893) <       0.001);
    ASSERT_TRUE(abs(torque -    0.27273) <      0.00001);
    ASSERT_TRUE(abs(kf -        6.3572e-05) <   0.0001e-05);
    ASSERT_TRUE(abs(km -        1.0909e-06) <   0.0001e-06);


    vtolDynamicsSim.thruster(134.254698, thrust, torque, kf, km);
    ASSERT_TRUE(abs(thrust -    3.5908) <       0.0001);
    ASSERT_TRUE(abs(torque -    0.013696) <     0.000001);
    ASSERT_TRUE(abs(kf -        1.9922e-04) <   0.0001e-04);
    ASSERT_TRUE(abs(km -        7.5984e-07) <   0.0001e-07);
}

TEST(VtolDynamicsSim, calculateNewStateFirstCaseOnlyAttitude){
    VtolDynamicsSim vtolDynamicsSim;
    vtolDynamicsSim.init();
    double thrust, torque, kf, km, dt;
    Eigen::Vector3d Maero, Faero, angAccel, expectedAngAccel, linAccel, expectedLinAccel, diff;
    auto isZeroComparator = [](double a) {return abs(a) < 6e-05;};

    dt = 0.002500;
    std::vector<double> actuators{0, 0, 0, 0, 0, 0, 0, 0};
    Faero << 0.0, 0.0, 0.0;
    Maero << 0.0, 0.0, 0.0;
    vtolDynamicsSim.setInitialVelocity(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0.0, 0.0, 0.0));
    vtolDynamicsSim.setInitialPosition(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0.2, 0.10, 0.05));

    /**
     * @note In InnoDynamics the altitude is directed to the bottom, but in this simulator
     * it is directed to the top, so we perform invertion.
     * - expectedAngAccel x and y were multiplied by -1
     * - expectedLinAccel.z was multiplied by -1
     */
    expectedAngAccel << 0.0, 0.0, 0.0;
    expectedLinAccel << 2.5377e-16, -5.0753e-16, -9.8066e+00;

    vtolDynamicsSim.calculateNewState(Maero, Faero, actuators, dt);
    angAccel = vtolDynamicsSim.getAngularAcceleration();
    linAccel = vtolDynamicsSim.getLinearAcceleration();
    diff = expectedAngAccel - angAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
    diff = expectedLinAccel - linAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
}

TEST(VtolDynamicsSim, calculateNewStateSecondCaseOnlyAngularVelocity){
    VtolDynamicsSim vtolDynamicsSim;
    vtolDynamicsSim.init();
    double thrust, torque, kf, km, dt;
    Eigen::Vector3d Maero, Faero, angAccel, expectedAngAccel, linAccel, expectedLinAccel, diff;
    auto isZeroComparator = [](double a) {return abs(a) < 6e-05;};

    dt = 0.002500;
    std::vector<double> actuators{0, 0, 0, 0, 0, 0, 0, 0};
    Faero << 0.0, 0.0, 0.0;
    Maero << 0.0, 0.0, 0.0;
    vtolDynamicsSim.setInitialVelocity(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0.3, 0.2, 0.1));
    vtolDynamicsSim.setInitialPosition(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0.0, 0.0, 0.0));

    /**
     * @note In InnoDynamics the altitude is directed to the bottom, but in this simulator
     * it is directed to the top, so we perform invertion.
     * - expectedAngAccel x and y were multiplied by -1
     * - expectedLinAccel.z was multiplied by -1
     */
    expectedAngAccel << 1.9719e-02, -2.9589e-02, -8.3459e-04;
    expectedLinAccel << 9.9127e-19, 1.9825e-18, -9.8066e+00;

    vtolDynamicsSim.calculateNewState(Maero, Faero, actuators, dt);
    angAccel = vtolDynamicsSim.getAngularAcceleration();
    linAccel = vtolDynamicsSim.getLinearAcceleration();
    diff = expectedAngAccel - angAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
    diff = expectedLinAccel - linAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
}

TEST(VtolDynamicsSim, calculateNewStateThirdCaseOnlyFaero){
    VtolDynamicsSim vtolDynamicsSim;
    vtolDynamicsSim.init();
    double thrust, torque, kf, km, dt;
    Eigen::Vector3d Maero, Faero, angAccel, expectedAngAccel, linAccel, expectedLinAccel, diff;
    auto isZeroComparator = [](double a) {return abs(a) < 4.3e-07;};

    dt = 0.002500;
    std::vector<double> actuators{0, 0, 0, 0, 0, 0, 0, 0};
    Faero << 5.7448e-01, 2.9513e+01, -6.1333e-01;
    Maero << 0.0, 0.0, 0.0;
    vtolDynamicsSim.setInitialVelocity(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0.0, 0.0, 0.0));
    vtolDynamicsSim.setInitialPosition(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0.0, 0.0, 0.0));

    /**
     * @note In InnoDynamics the altitude is directed to the bottom, but in this simulator
     * it is directed to the top, so we perform invertion.
     * - expectedAngAccel x and y were multiplied by -1
     * - expectedLinAccel.z was multiplied by -1
     */
    expectedAngAccel << 0.0, 0.0, 0.0;
    expectedLinAccel << 0.082069, 4.216143, -9.719031;

    vtolDynamicsSim.calculateNewState(Maero, Faero, actuators, dt);
    angAccel = vtolDynamicsSim.getAngularAcceleration();
    linAccel = vtolDynamicsSim.getLinearAcceleration();
    diff = expectedAngAccel - angAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
    diff = expectedLinAccel - linAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
}

TEST(VtolDynamicsSim, calculateNewStateFourthCaseOnlyMaero){
    VtolDynamicsSim vtolDynamicsSim;
    vtolDynamicsSim.init();
    double thrust, torque, kf, km, dt;
    Eigen::Vector3d Maero, Faero, angAccel, expectedAngAccel, linAccel, expectedLinAccel, diff;
    auto isZeroComparator = [](double a) {return abs(a) < 5.1e-05;};

    dt = 0.002500;
    std::vector<double> actuators{0, 0, 0, 0, 0, 0, 0, 0};
    Faero << 0.0, 0.0, 0.0;
    Maero << 0.214696, 0.694801, -0.316328;
    vtolDynamicsSim.setInitialVelocity(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0.0, 0.0, 0.0));
    vtolDynamicsSim.setInitialPosition(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0.0, 0.0, 0.0));

    /**
     * @note In InnoDynamics the altitude is directed to the bottom, but in this simulator
     * it is directed to the top, so we perform invertion.
     * - expectedAngAccel x and y were multiplied by -1
     * - expectedLinAccel.z was multiplied by -1
     */
    expectedAngAccel << -0.34251, -1.07821, -0.25057;
    expectedLinAccel << -7.7443e-21, 3.8722e-21, -9.8066e+00;

    vtolDynamicsSim.calculateNewState(Maero, Faero, actuators, dt);
    angAccel = vtolDynamicsSim.getAngularAcceleration();
    linAccel = vtolDynamicsSim.getLinearAcceleration();
    diff = expectedAngAccel - angAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
    diff = expectedLinAccel - linAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
}

TEST(VtolDynamicsSim, calculateNewStateSixthCaseOnlyCopterMotorsWithEqualPower){
    VtolDynamicsSim vtolDynamicsSim;
    vtolDynamicsSim.init();
    double thrust, torque, kf, km, dt;
    Eigen::Vector3d Maero, Faero, angAccel, expectedAngAccel, linAccel, expectedLinAccel, diff;
    auto isZeroComparator = [](double a) {return abs(a) < 6.74158e-07;};

    dt = 0.002500;
    std::vector<double> actuators{700, 700, 700, 700, 0, 0, 0, 0};
    Faero << 0.0, 0.0, 0.0;
    Maero << 0.0, 0.0, 0.0;
    vtolDynamicsSim.setInitialVelocity(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0.0, 0.0, 0.0));
    vtolDynamicsSim.setInitialPosition(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0.0, 0.0, 0.0));

    /**
     * @note In InnoDynamics the altitude is directed to the bottom, but in this simulator
     * it is directed to the top, so we perform invertion.
     * - expectedAngAccel x and y were multiplied by -1
     * - expectedLinAccel.z was multiplied by -1
     */
    expectedAngAccel << 0.0, 0.0, 0.0;
    expectedLinAccel << 0.00000, 0.00000, 6.36769;
    vtolDynamicsSim.calculateNewState(Maero, Faero, actuators, dt);
    angAccel = vtolDynamicsSim.getAngularAcceleration();
    linAccel = vtolDynamicsSim.getLinearAcceleration();
    diff = expectedAngAccel - angAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
    diff = expectedLinAccel - linAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
}

TEST(VtolDynamicsSim, calculateNewStateSeventhCaseOnlyCopterMotorsWithNotEqualPower){
    VtolDynamicsSim vtolDynamicsSim;
    vtolDynamicsSim.init();
    double thrust, torque, kf, km, dt;
    Eigen::Vector3d Maero, Faero, angAccel, expectedAngAccel, linAccel, expectedLinAccel, diff;
    auto isZeroComparator = [](double a) {return abs(a) < 4.24861e-05;};

    dt = 0.002500;
    std::vector<double> actuators{700, 680, 660, 640, 0, 0, 0, 0};
    Faero << 0.0, 0.0, 0.0;
    Maero << 0.0, 0.0, 0.0;
    vtolDynamicsSim.setInitialVelocity(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0.0, 0.0, 0.0));
    vtolDynamicsSim.setInitialPosition(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0.0, 0.0, 0.0));

    /**
     * @note In InnoDynamics the altitude is directed to the bottom, but in this simulator
     * it is directed to the top, so we perform invertion.
     * - expectedAngAccel x and y were multiplied by -1
     * - expectedLinAccel.z was multiplied by -1
     */
    expectedAngAccel << -0.13933, -1.48111, 0.10723;
    expectedLinAccel << -1.3753e-04, 1.2938e-05, 5.0505e+00;

    vtolDynamicsSim.calculateNewState(Maero, Faero, actuators, dt);
    angAccel = vtolDynamicsSim.getAngularAcceleration();
    linAccel = vtolDynamicsSim.getLinearAcceleration();
    diff = expectedAngAccel - angAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
    diff = expectedLinAccel - linAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
}

TEST(VtolDynamicsSim, calculateNewStateEighthCaseOnlyICE){
    VtolDynamicsSim vtolDynamicsSim;
    vtolDynamicsSim.init();
    double thrust, torque, kf, km, dt;
    Eigen::Vector3d Maero, Faero, angAccel, expectedAngAccel, linAccel, expectedLinAccel, diff;
    auto isZeroComparator = [](double a) {return abs(a) < 5.1e-05;};

    dt = 0.002500;
    std::vector<double> actuators{0, 0, 0, 0, 500, 0, 0, 0};
    Faero << 0.0, 0.0, 0.0;
    Maero << 0.0, 0.0, 0.0;
    vtolDynamicsSim.setInitialVelocity(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0.0, 0.0, 0.0));
    vtolDynamicsSim.setInitialPosition(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0.0, 0.0, 0.0));

    expectedAngAccel << -0.43508, 0.00000, 0.00000;
    expectedLinAccel << 2.2705e+00, 3.8722e-21, 9.8066e+00;

    /**
     * @note In InnoDynamics the altitude is directed to the bottom, but in this simulator
     * it is directed to the top, so we perform invertion.
     */
    expectedAngAccel[0] *= -1;
    expectedAngAccel[1] *= -1;
    expectedLinAccel[2] *= -1;

    vtolDynamicsSim.calculateNewState(Maero, Faero, actuators, dt);
    angAccel = vtolDynamicsSim.getAngularAcceleration();
    linAccel = vtolDynamicsSim.getLinearAcceleration();
    diff = expectedAngAccel - angAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
    diff = expectedLinAccel - linAccel;
    ASSERT_TRUE(std::all_of(&diff[0], &diff[3], isZeroComparator));
}


int main(int argc, char *argv[]){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
