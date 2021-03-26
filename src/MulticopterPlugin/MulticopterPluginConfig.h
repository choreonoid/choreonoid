/**
   @author Japan Atomic Energy Agency
*/

#pragma once

namespace Multicopter{

    const std::string YAML_BODY_TAG = "MultiCopterTargetBody";
    const std::string YAML_BODY_CUTOFF_DISTANCE_TAG = "cutoffDistance";
    const std::string YAML_BODY_NORM_MIDDLE_VALUE_TAG = "normMiddleValue";

    const std::string YAML_LINK_TAG = "MultiCopterTargetLink";
    const std::string YAML_LINK_APPLYFORCE_TAG = "applyForce";
    const std::string YAML_LINK_DENSITY_TAG = "density";
    const std::string YAML_LINK_BUOYANCY_CENTER_TAG = "centerOfBuoyancy";
    const std::string YAML_LINK_ADDITIONAL_MASS_TAG = "additionalMassCoef";
    const std::string YAML_LINK_ADDITIONAL_INERTIA_TAG = "additionalInertiaMatrix";

    static const char* YAML_ROTOR_DEVICE_TAG = "RotorDevice";
    static const char* YAML_ROTOR_NAME = "name";
    static const char* YAML_ROTOR_POSITION = "position";
    static const char* YAML_ROTOR_DIRECTION = "direction";
    static const char* YAML_ROTOR_VALUE_RANGE = "value_range";
    static const char* YAML_ROTOR_VALUE_RANGE_OLD = "valueRange";
    static const char* YAML_ROTOR_TORQUE_RANGE = "torque_range";
    static const char* YAML_ROTOR_TORQUE_RANGE_OLD = "torqueRange";

    static const char* YAML_EFFECT_TAG = "effect";
    static const char* YAML_EFFECT_TAG_OLD = "effectParameter";
    static const char* YAML_WALL_DISTANCE = "wall_distance";
    static const char* YAML_WALL_DISTANCE_OLD = "wallDistance";
    static const char* YAML_WALL_NORM_MIDDLE_VALUE = "wall_norm_middle_value";
    static const char* YAML_WALL_NORM_MIDDLE_VALUE_OLD = "wallNormMiddleValue";
    static const char* YAML_WALL_MAX_RATE = "wall_max_rate";
    static const char* YAML_WALL_MAX_RATE_OLD = "wallMaxRate";
    static const char* YAML_GROUND_DISTANCE = "ground_distance";
    static const char* YAML_GROUND_DISTANCE_OLD = "groundDistance";
    static const char* YAML_GROUND_NORM_MIDDLE_VALUE = "ground_norm_middle_value";
    static const char* YAML_GROUND_NORM_MIDDLE_VALUE_OLD = "groundNormMiddleValue";
    static const char* YAML_GROUND_MAX_RATE = "ground_max_rate";
    static const char* YAML_GROUND_MAX_RATE_OLD = "groundMaxRate";

    const std::string AIR_DEFINITION_FILE_TAG = "AirEnvironment";
    const std::string AIR_DEFINITION_FILE_VERSION ="1.0.0";

    const std::string MULTICOPTER_DENSITY="Fluid Density[kg/m^3]";
    const std::string MULTICOPTER_VISCOSITY="Viscosity[Pa*s]";
    const std::string MULTICOPTER_VELOCITY ="Fluid Velocity[m/s]";
    const std::string MULTICOPTER_AIRDEFINITION="Air Definition File";
    const std::string MULTICOPTER_WALLEFFECT="Wall Effect";
    const std::string MULTICOPTER_GROUNDEFFECT="Ground Effect";
    const std::string MULTICOPTER_OUTPUT="Output Parameter";
    const std::string MULTICOPTER_TIMESTEP="Output Time Step[s]";

    const std::string MULTICOPTER_LOG_HEADER = std::string("Time[s],BodyName,LinkName,Position-X[m],Position-Y[m],Position-Z[m],"
                                                      "Velocity-X[m/s],Velocity-Y[m/s],Velocity-Z[m/s],"
                                                      "Acceleration-X[m/s^2],Acceleration-Y[m/s^2],Acceleration-Z[m/s^2],"
                                                      "AngularVelocity-X[rad/s],AngularVelocity-Y[rad/s],AngularVelocity-Z[rad/s],"
                                                      "AngularAcceleration-X[rad/s^2],AngularAcceleration-Y[rad/s^2],AngularAcceleration-Z[rad/s^2],"
                                                      "BuoyancyForce-X[N],BuoyancyForce-Y[N],BuoyancyForce-Z[N],"
                                                      "SurfaceForce-X[N],SurfaceForce-Y[N],SurfaceForce-Z[N],"
                                                      "AddMassForce-X[N],AddMassForce-Y[N],AddMassForce-Z[N],"
                                                      "AddInertiaTorque-X,AddInertiaTorque-Y,AddInertiaTorque-Z");

}
