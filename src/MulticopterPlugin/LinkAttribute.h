/**
   @author Japan Atomic Energy Agency
*/

#pragma once

namespace Multicopter {

class LinkAttribute
{
public:

    LinkAttribute(){
        _isNull = true;
        _hasBuoyancyCen = false;
        _logMode = true;
    }

    bool isNull() const{
        return _isNull;
    }

    double cutoffDistance() const{        
        return _cutoffDist;
    }

    void setCutoffDistance(double dist){
        _cutoffDist = dist;
    }

    double normMiddleValue() const{
        return _normMidVal;
    }

    void setNormMiddleValue(double normMidVal){
        _normMidVal = normMidVal;
    }

    double density() const{
        return _den;
    }

    void setDensity(double den){
        _den = den;
    }

    Eigen::Vector3d centerOfBuoyancy() const{
        return _buoyancyCen;
    }

    void setCenterOfBuoyancy(const Eigen::Vector3d& pos){
        _hasBuoyancyCen = true;
        _buoyancyCen = pos;
    }

    bool hasCenterOfBuoyancy() const{
        return _hasBuoyancyCen;
    }

    double additionalMassCoef() const{
        return _addMass;
    }

    void setAdditionalMassCoef(double coef){
        _addMass = coef;
    }

    Eigen::Matrix3d additionalInertiaMatrix() const{
        return _addInertia;
    }

    void setAdditionalInertiaMatrix(const Eigen::Matrix3d& iner){
        _addInertia = iner;
    }

    void checkDone(){
        _isNull = false;
    }

    std::vector<bool> linkForceApplyFlgAry() const{
        return _linkForceApplyFlagAry;
    }

    void setLinkForceApplyFlgAry(std::vector<bool> applyFlgAry){
        _linkForceApplyFlagAry=applyFlgAry;
    }

    int effectMode() const{
        return _effectMode;
    }

    void setEffectMode(int effectMode){
        if(effectMode<0 || 2 < effectMode )effectMode=0;
        _effectMode=effectMode;
    }

    bool logMode() const{
        return _logMode;
    }

    void setLogMode(bool logMode){
        _logMode = logMode;
    }

    boost::optional<cnoid::CollisionDetector::GeometryHandle> getGeometryHandle() const{
        return _handle;
    }

    void setGeometryHandle(boost::optional<cnoid::CollisionDetector::GeometryHandle> handle){
        _handle=handle;
    }

protected:

private:
    bool _isNull;
    double _cutoffDist;
    double _normMidVal;
    double _den;
    bool _hasBuoyancyCen;
    Eigen::Vector3d _buoyancyCen;
    double _addMass;
    Eigen::Matrix3d _addInertia;
    std::vector<bool> _linkForceApplyFlagAry;
    int _effectMode;
    bool _logMode;
    boost::optional<cnoid::CollisionDetector::GeometryHandle> _handle;
};

}
