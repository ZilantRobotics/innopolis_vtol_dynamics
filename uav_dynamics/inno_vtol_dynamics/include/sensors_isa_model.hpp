/**
 * @file sensors_isa_model.hpp
 * @author ponomarevda96@gmail.com
 */

#ifndef SENSORS_ISA_MODEL_HPP
#define SENSORS_ISA_MODEL_HPP

#include <Eigen/Geometry>

namespace SensorModelISA
{

    void EstimateAtmosphere(const Eigen::Vector3d& gpsPosition, const Eigen::Vector3d& linVelNed,
                        float& temperatureKelvin, float& absPressureHpa, float& diffPressureHpa){
        const float PRESSURE_MSL_HPA = 1013.250f;
        const float TEMPERATURE_MSL_KELVIN = 288.0f;
        const float RHO_MSL = 1.225f;
        const float LAPSE_TEMPERATURE_RATE = 1 / 152.4;

        float alt_msl = gpsPosition.z();

        temperatureKelvin = TEMPERATURE_MSL_KELVIN - LAPSE_TEMPERATURE_RATE * alt_msl;
        float pressureRatio = powf((TEMPERATURE_MSL_KELVIN/temperatureKelvin), 5.256f);
        const float densityRatio = powf((TEMPERATURE_MSL_KELVIN/temperatureKelvin), 4.256f);
        float rho = RHO_MSL / densityRatio;
        absPressureHpa = PRESSURE_MSL_HPA / pressureRatio;
        diffPressureHpa = 0.005f * rho * linVelNed.norm() * linVelNed.norm();
    }

}

#endif  // SENSORS_ISA_MODEL_HPP
