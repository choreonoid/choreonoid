#ifndef CNOID_BASE_DISPLAY_VALUE_FORMAT_H
#define CNOID_BASE_DISPLAY_VALUE_FORMAT_H

#include <cnoid/Signal>
#include <cnoid/EigenTypes>
#include "exportdecl.h"

namespace cnoid {

/**
   \todo Move this class to the Util module and make it thread-safe.
*/
class CNOID_EXPORT DisplayValueFormat
{
public:
    static DisplayValueFormat* instance() { return &instance_; }

    DisplayValueFormat(const DisplayValueFormat& org) = delete;

    enum LengthUnit { Meter, Millimeter };
    void setLengthUnit(LengthUnit unitType) { lengthUnit_ = unitType; }
    int lengthUnit() const { return lengthUnit_; }
    bool isMeter() const { return lengthUnit_ == Meter; }
    bool isMillimeter() const { return lengthUnit_ == Millimeter; }
    double toDisplayLength(double meter) const {
        return (lengthUnit_ == Meter) ? meter : meter * 1000.0;
    }
    double toMeter(double displayLength) const {
        return (lengthUnit_ == Meter) ? displayLength : displayLength / 1000.0;
    }
    double ratioToDisplayLength() const {
        return (lengthUnit_ == Meter) ? 1.0 : 1000.0;
    }
    void setMeterDecimals(int decimals) { meterDecimals_ = decimals; }
    int meterDecimals() const { return meterDecimals_; }
    void setMillimeterDecimals(int decimals){ millimeterDecimals_ = decimals; }
    int millimeterDecimals() const { return millimeterDecimals_; }
    void setLengthDecimals(int decimals);
    int lengthDecimals() const {
        return (lengthUnit_ == Meter) ? meterDecimals_ : millimeterDecimals_;
    }
    void setLengthDecimalsForcedMode(bool on) { isLengthDecimalsForcedMode_ = on; }
    bool isLengthDecimalsForcedMode() const { return isLengthDecimalsForcedMode_; }
    void setMeterStep(double step){ meterStep_ = step; }
    double meterStep() const { return meterStep_; }
    void setMillimeterStep(double step){ millimeterStep_ = step; }
    double millimeterStep() const { return millimeterStep_; }
    void setLengthStep(double step);
    double lengthStep() const {
        return (lengthUnit_ == Meter) ? meterStep_ : millimeterStep_;
    }
    void setLengthStepForcedMode(bool on){ isLengthStepForcedMode_ = on; }
    bool isLengthStepForcedMode() const { return isLengthStepForcedMode_; }

    enum AngleUnit { Degree, Radian };
    void setAngleUnit(AngleUnit unitType) { angleUnit_ = unitType; }
    int angleUnit() const { return angleUnit_; }
    bool isDegree() const { return angleUnit_ == Degree; }
    void setDegreeDecimals(int decimals) { degreeDecimals_ = decimals; }
    int degreeDecimals() const { return degreeDecimals_; }
    void setRadianDecimals(int decimals) { radianDecimals_ = decimals; }
    int radianDecimals() const { return radianDecimals_; }
    void setAngleDecimals(int decimals);
    int angleDecimals() const {
        return (angleUnit_ == Degree) ? degreeDecimals_ : radianDecimals_;
    }
    void setAngleDecimalsForcedMode(bool on) { isAngleDecimalsForcedMode_ = on; }
    bool isAngleDecimalsForcedMode() const { return isAngleDecimalsForcedMode_; }
    void setDegreeStep(double step) { degreeStep_ = step; }
    double degreeStep() const { return degreeStep_; }
    void setRadianStep(double step) { radianStep_ = step; }
    double radianStep() const { return radianStep_; }
    void setAngleStep(double step);
    double angleStep() const {
        return (angleUnit_ == Degree) ? degreeStep_ : radianStep_;
    }
    void setAngleStepForcedMode(bool on) { isAngleStepForcedMode_ = on; }
    bool isAngleStepForcedMode() const { return isAngleStepForcedMode_; }

    enum CoordinateSystem { RightHanded, LeftHanded };
    void setCoordinateSystem(CoordinateSystem system) { coordinateSystem_ = system; }
    int coordinateSystem() const { return coordinateSystem_; }
    bool isLeftHandedCoordinateSystem() const { return coordinateSystem_ == LeftHanded; }
    bool isRightHandedCoordinateSystem() const { return coordinateSystem_ == RightHanded; }

    void updateToDisplayCoordPosition(Vector3& p) const {
        if(isLeftHandedCoordinateSystem()){
            p.y() = -p.y();
        }
    }
    void updateToRightHandedPosition(Vector3& p) const {
        if(isLeftHandedCoordinateSystem()){
            p.y() = -p.y();
        }
    }
    void updateToDisplayCoordRotation(Matrix3& R) const {
        if(isLeftHandedCoordinateSystem()){
            R.transposeInPlace();
        }
    }
    void updateToDisplayCoordRpy(Vector3& rpy) const {
        if(isLeftHandedCoordinateSystem()){
            rpy.x() = -rpy.x();
            rpy.z() = -rpy.z();
        }
    }
    void updateToRightHandedRpy(Vector3& rpy) const {
        if(isLeftHandedCoordinateSystem()){
            rpy.x() = -rpy.x();
            rpy.z() = -rpy.z();
        }
    }
    void updateToDisplayCoordRotation(Quaterniond& q) const {
        if(isLeftHandedCoordinateSystem()){
            q.x() = -q.x();
            q.z() = -q.z();
        }
    }
    void updateToRightHandedRotation(Quaterniond& q) const {
        if(isLeftHandedCoordinateSystem()){
            q.x() = -q.x();
            q.z() = -q.z();
        }
    }

    void updateToDisplayPosition(Vector3& p) const {
        p *= ratioToDisplayLength();
        updateToDisplayCoordPosition(p);
    }

    void notifyFormatChange();
    SignalProxy<void()> sigFormatChanged() { return sigFormatChanged_; }

    void restoreConfiguration();

private:
    DisplayValueFormat();

    static DisplayValueFormat instance_;

    LengthUnit lengthUnit_;
    int meterDecimals_;
    int millimeterDecimals_;
    double meterStep_;
    double millimeterStep_;
    bool isLengthDecimalsForcedMode_;
    bool isLengthStepForcedMode_;
    AngleUnit angleUnit_;
    int degreeDecimals_;
    int radianDecimals_;
    double degreeStep_;
    double radianStep_;
    bool isAngleDecimalsForcedMode_;
    bool isAngleStepForcedMode_;
    int coordinateSystem_;

    Signal<void()> sigFormatChanged_;
};

}

#endif
