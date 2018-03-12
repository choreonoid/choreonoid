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

    const std::string YAML_ROTOR_DEVICE_TAG = "RotorDevice";
    const std::string YAML_ROTOR_NAME = "name";
    const std::string YAML_ROTOR_POSITION = "position";
    const std::string YAML_ROTOR_DIRECTION = "direction";
    const std::string YAML_ROTOR_VALUE_RANGE = "valueRange";
    const std::string YAML_ROTOR_TORQUE_RANGE = "torqueRange";

    const std::string YAML_EFFECT_TAG = "effectParameter";
    const std::string YAML_WALL_DISTANCE = "wallDistance";
    const std::string YAML_WALL_NORM_MIDDLE_VALUE = "wallNormMiddleValue";
    const std::string YAML_WALL_MAX_RATE = "wallMaxRate";
    const std::string YAML_GROUND_DISTANCE = "groundDistance";
    const std::string YAML_GROUND_NORM_MIDDLE_VALUE = "groundNormMiddleValue";
    const std::string YAML_GROUND_MAX_RATE = "groundMaxRate";

    const std::string AIR_DEFINITION_FILE_TAG = "AirEnvironment";
    const std::string AIR_DEFINITION_FILE_VERSION ="1.0.0";

    const std::string MULTICOPTER_DENSITY="Fluid Density[kg/m^3]";
    const std::string MULTICOPTER_VISCOSITY="Viscosity[Pa･s]";
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
