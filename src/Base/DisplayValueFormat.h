#ifndef CNOID_BASE_DISPLAY_VALUE_FORMAT_H
#define CNOID_BASE_DISPLAY_VALUE_FORMAT_H

#include <cnoid/Referenced>
#include <cnoid/Signal>
#include <cnoid/EigenTypes>
#include "exportdecl.h"

namespace cnoid {

/**
   \todo Move this class to the Util module and make it thread-safe.
*/
class CNOID_EXPORT DisplayValueFormat : public Referenced
{
public:
    static DisplayValueFormat* master() { return master_; }

    [[deprecated("Use master")]]
    static DisplayValueFormat* instance() { return master_; }

    DisplayValueFormat();
    DisplayValueFormat(const DisplayValueFormat& org);

    enum LengthUnit { Meter, Millimeter, Kilometer };
    void setLengthUnit(LengthUnit unitType) { lengthUnit_ = unitType; }
    int lengthUnit() const { return lengthUnit_; }
    bool isMeter() const { return lengthUnit_ == Meter; }
    bool isMillimeter() const { return lengthUnit_ == Millimeter; }
    bool isKilometer() const { return lengthUnit_ == Kilometer; }
    double toDisplayLength(double meter) const {
        if(lengthUnit_ == Meter) return meter;
        if(lengthUnit_ == Millimeter) return meter * 1000.0;
        return meter / 1000.0;  // Kilometer
    }
    double toMeter(double displayLength) const {
        if(lengthUnit_ == Meter) return displayLength;
        if(lengthUnit_ == Millimeter) return displayLength / 1000.0;
        return displayLength * 1000.0;  // Kilometer
    }
    double ratioToDisplayLength() const {
        if(lengthUnit_ == Meter) return 1.0;
        if(lengthUnit_ == Millimeter) return 1000.0;
        return 0.001;  // Kilometer
    }
    const char* lengthUnitSymbol() const {
        if(lengthUnit_ == Meter) return "m";
        if(lengthUnit_ == Millimeter) return "mm";
        return "km";
    }
    void setMeterDecimals(int decimals) { meterDecimals_ = decimals; }
    int meterDecimals() const { return meterDecimals_; }
    void setMillimeterDecimals(int decimals){ millimeterDecimals_ = decimals; }
    int millimeterDecimals() const { return millimeterDecimals_; }
    void setKilometerDecimals(int decimals){ kilometerDecimals_ = decimals; }
    int kilometerDecimals() const { return kilometerDecimals_; }
    void setLengthDecimals(int decimals);
    int lengthDecimals() const {
        if(lengthUnit_ == Meter) return meterDecimals_;
        if(lengthUnit_ == Millimeter) return millimeterDecimals_;
        return kilometerDecimals_;
    }
    void setLengthDecimalsForcedMode(bool on) { isLengthDecimalsForcedMode_ = on; }
    bool isLengthDecimalsForcedMode() const { return isLengthDecimalsForcedMode_; }
    void setMeterStep(double step){ meterStep_ = step; }
    double meterStep() const { return meterStep_; }
    void setMillimeterStep(double step){ millimeterStep_ = step; }
    double millimeterStep() const { return millimeterStep_; }
    void setKilometerStep(double step){ kilometerStep_ = step; }
    double kilometerStep() const { return kilometerStep_; }
    void setLengthStep(double step);
    double lengthStep() const {
        if(lengthUnit_ == Meter) return meterStep_;
        if(lengthUnit_ == Millimeter) return millimeterStep_;
        return kilometerStep_;
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
    static ref_ptr<DisplayValueFormat> master_;

    LengthUnit lengthUnit_;
    int meterDecimals_;
    int millimeterDecimals_;
    int kilometerDecimals_;
    double meterStep_;
    double millimeterStep_;
    double kilometerStep_;
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

typedef ref_ptr<DisplayValueFormat> DisplayValueFormatPtr;

}

#endif
