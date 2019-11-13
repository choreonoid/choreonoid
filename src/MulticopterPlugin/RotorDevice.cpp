/**
   @author Japan Atomic Energy Agency
*/

#include "MulticopterPluginHeader.h"


using namespace std;
using namespace cnoid;
using namespace Multicopter;

bool readRotorDevice(YAMLBodyLoader& loader, Mapping& node)
{
    RotorDevicePtr rotorDevice= new RotorDevice;
    Eigen::Vector3d a;
    Eigen::Vector2d b;
    double v;
    bool flg=true;

    if(read(node,YAML_ROTOR_POSITION,a)){
        rotorDevice->setPosition(a);
    }else{
        flg=false;
        UtilityImpl::printSomethingWrongAtRotor(YAML_ROTOR_POSITION);
    }
    if(read(node,YAML_ROTOR_DIRECTION,a)){
        rotorDevice->setDirection(a);
    }else{
        flg=false;
        UtilityImpl::printSomethingWrongAtRotor(YAML_ROTOR_DIRECTION);
    }
    if(read(node,YAML_ROTOR_VALUE_RANGE,b)){
        rotorDevice->setValueRange(b[0],b[1]);
    }else{
        flg=false;
        UtilityImpl::printSomethingWrongAtRotor(YAML_ROTOR_VALUE_RANGE);
    }
    if(read(node,YAML_ROTOR_TORQUE_RANGE,b)){
        rotorDevice->setTorqueRange(b[0],b[1]);
    }else{
        flg=false;
        UtilityImpl::printSomethingWrongAtRotor(YAML_ROTOR_TORQUE_RANGE);
    }

#ifdef ENABLE_MULTICOPTER_PLUGIN_DEBUG
    if(node.read("value",v)) rotorDevice->setValue(v);
    if(node.read("torque",v)) rotorDevice->setTorque(v);
#endif


    Mapping* effectMap = UtilityImpl::mapping(&node,YAML_EFFECT_TAG);

    if( effectMap != nullptr ){
        if(effectMap->read(YAML_WALL_DISTANCE,v)){
            rotorDevice->setWallEffectDistance(v);
        }else{
            flg=false;
            UtilityImpl::printSomethingWrongAtRotor(YAML_WALL_DISTANCE);
        }

        if(effectMap->read(YAML_WALL_NORM_MIDDLE_VALUE,v)){
            rotorDevice->setWallEffectNormMiddleValue(v);
        }else{
            flg=false;
            UtilityImpl::printSomethingWrongAtRotor(YAML_WALL_NORM_MIDDLE_VALUE);
        }

        if(effectMap->read(YAML_WALL_MAX_RATE,v)){
            rotorDevice->setWallEffectMaxRate(v);
        }else{
            flg=false;
            UtilityImpl::printSomethingWrongAtRotor(YAML_WALL_MAX_RATE);
        }

        if(effectMap->read(YAML_GROUND_DISTANCE,v)){
            rotorDevice->setGroundEffectDistance(v);
        }else{
            flg=false;
            UtilityImpl::printSomethingWrongAtRotor(YAML_GROUND_DISTANCE);
        }

        if(effectMap->read(YAML_GROUND_NORM_MIDDLE_VALUE,v)){
            rotorDevice->setGroundEffectNormMiddleValue(v);
        }else{
            flg=false;
            UtilityImpl::printSomethingWrongAtRotor(YAML_GROUND_NORM_MIDDLE_VALUE);
        }

        if(effectMap->read(YAML_GROUND_MAX_RATE,v)){
            rotorDevice->setGroundEffectMaxRate(v);
        }else{
            flg=false;
            UtilityImpl::printSomethingWrongAtRotor(YAML_GROUND_MAX_RATE);
        }
    }

    rotorDevice->setActive(flg);
    return loader.readDevice(rotorDevice, node);
}

struct TypeRegistration
{
    TypeRegistration() {
        YAMLBodyLoader::addNodeType(YAML_ROTOR_DEVICE_TAG, readRotorDevice);
    }
} registration;


double
RotorDevice::value() const
{
    return _val;
}

void
RotorDevice::setValue(const double val)
{
    if( _minVal > val ){
        _val = _minVal;
    }
    else if( val > _maxVal ){
        _val = _maxVal;
    }
    else{
        _val = val;
    }
}

void
RotorDevice::valueRange(double& min, double& max) const
{
    min = _minVal;
    max = _maxVal;
}

void
RotorDevice::setValueRange(const double min, const double max)
{
    _minVal = min;
    _maxVal = max;
}

double
RotorDevice::torque()const{
    return _torque;
}

void
RotorDevice::setTorque(const double torque){
    if( _torqueMinVal > torque ){
        _torque = _torqueMinVal;
    }
    else if( torque > _torqueMaxVal ){
        _torque = _torqueMaxVal;
    }
    else{
        _torque = torque;
    }
}

void
RotorDevice::torqueRange(double& min, double& max) const
{
    min = _torqueMinVal;
    max = _torqueMaxVal;
}

void
RotorDevice::setTorqueRange(const double min, const double max)
{
    _torqueMinVal = min;
    _torqueMaxVal = max;
}

double
RotorDevice::applyValueRange(const double val){
    if( _minVal > val ){
        return _minVal;
    }
    else if( val > _maxVal ){
        return _maxVal;
    }
    else{
        return val;
    }
}

double
RotorDevice::applyTorqueRange(const double val){
    if( _torqueMinVal > val ){
        return _torqueMinVal;
    }
    else if( val > _torqueMaxVal ){
        return _torqueMaxVal;
    }
    else{
        return val;
    }
}

Eigen::Vector3d
RotorDevice::direction() const{
    return _dir;
}

void
RotorDevice::setDirection(const Eigen::Vector3d& dir)
{
    _dir = dir;
    _dir.normalize();
}

Eigen::Vector3d
RotorDevice::position() const{
    return _pos;
}

void
RotorDevice::setPosition(const Eigen::Vector3d& pos){
    _pos = pos;
    localTranslation() = _pos;
}

RotorDevice::RotorDevice()
{

    _on= true;
    _pos = Eigen::Vector3d(0.0, 0.0, 0.0);
    _dir = Eigen::Vector3d(0.0, 0.0, 1.0);
    _val = 0.0;
    _minVal = -100.0;
    _maxVal = +100.0;
    _torque = 0.0;
    _torqueMinVal=-100;
    _torqueMaxVal=100;
    _wallEffectDistance=0;
    _wallEffectNormMiddleValue=0;
    _wallEffectMaxRate=0;
    _groundEffectDistance=0;
    _groundEffectNormMiddleValue=0;
    _groundEffectMaxRate=0;

}

const char* RotorDevice::typeName()
{
    return "RotorDevice";
}

void RotorDevice::copyStateFrom(const RotorDevice& other)
{
    _on = other._on;
    _pos = other._pos;
    _dir = other._dir;
    _val = other._val;
    _minVal = other._minVal;
    _maxVal = other._maxVal;
    _torque = other._torque;
    _torqueMinVal=other._torqueMinVal;
    _torqueMaxVal=other._torqueMaxVal;
    _wallEffectDistance=other._wallEffectDistance;
    _wallEffectNormMiddleValue=other._wallEffectNormMiddleValue;
    _wallEffectMaxRate=other._wallEffectMaxRate;
    _groundEffectDistance=other._groundEffectDistance;
    _groundEffectNormMiddleValue=other._groundEffectNormMiddleValue;
    _groundEffectMaxRate=other._groundEffectMaxRate;

}

void RotorDevice::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(RotorDevice)){
        throw std::invalid_argument("Type mismatch in the RotorDevice::copyStateFrom function");
    }
    copyStateFrom(static_cast<const RotorDevice&>(other));
}

RotorDevice::RotorDevice(const RotorDevice& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyStateFrom(org);
}

DeviceState* RotorDevice::cloneState() const
{
    return new RotorDevice(*this, false);
}

Referenced* RotorDevice::doClone(CloneMap*) const
{
    return new RotorDevice(*this);
}

void RotorDevice::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(RotorDevice))){
        Device::forEachActualType(func);
    }
}

int RotorDevice::stateSize() const
{
    return 19;
}

const double* RotorDevice::readState(const double* buf)
{
    _on = (buf[0] > 0.0) ? true : false;
    _pos = Eigen::Map<const Eigen::Vector3d>(&(buf[1]));
    _dir = Eigen::Map<const Eigen::Vector3d>(&(buf[4]));
    _val = buf[7];
    _minVal = buf[8];
    _maxVal = buf[9];
    _torque = buf[10];
    _torqueMinVal = buf[11];
    _torqueMaxVal = buf[12];
    _wallEffectDistance = buf[13];
    _wallEffectNormMiddleValue = buf[14];
    _wallEffectMaxRate = buf[15];
    _groundEffectDistance = buf[16];
    _groundEffectNormMiddleValue = buf[17];
    _groundEffectMaxRate = buf[18];
    return buf + stateSize();
}

double* RotorDevice::writeState(double* out_buf) const
{
    out_buf[0] = (_on == true) ? 1.0 : 0.0;
    memcpy(out_buf+1, _pos.data(), sizeof(double)*3);
    memcpy(out_buf+4, _dir.data(), sizeof(double)*3);
    out_buf[7] = _val;
    out_buf[8] = _minVal;
    out_buf[9] = _maxVal;
    out_buf[10] = _torque;
    out_buf[11] = _torqueMinVal;
    out_buf[12] = _torqueMaxVal;
    out_buf[13] = _wallEffectDistance;
    out_buf[14] = _wallEffectNormMiddleValue;
    out_buf[15] = _wallEffectMaxRate;
    out_buf[16] = _groundEffectDistance;
    out_buf[17] = _groundEffectNormMiddleValue;
    out_buf[18] = _groundEffectMaxRate;

    return out_buf + stateSize();
}

double RotorDevice::wallEffectDistance() const
{
    return _wallEffectDistance;
}

void RotorDevice::setWallEffectDistance(double wallEffectDistance)
{
    _wallEffectDistance = wallEffectDistance;
}

double RotorDevice::wallEffectNormMiddleValue() const
{
    return _wallEffectNormMiddleValue;
}

void RotorDevice::setWallEffectNormMiddleValue(double wallEffectNormMiddleValue)
{
    _wallEffectNormMiddleValue = wallEffectNormMiddleValue;
}

double RotorDevice::wallEffectMaxRate() const
{
    return _wallEffectMaxRate;
}

void RotorDevice::setWallEffectMaxRate(double wallEffectMaxRate)
{
    _wallEffectMaxRate = wallEffectMaxRate;
}

double RotorDevice::groundEffectDistance() const
{
    return _groundEffectDistance;
}

void RotorDevice::setGroundEffectDistance(double groundEffectDistance)
{
    _groundEffectDistance = groundEffectDistance;
}

double RotorDevice::groundEffectNormMiddleValue() const
{
    return _groundEffectNormMiddleValue;
}

void RotorDevice::setGroundEffectNormMiddleValue(double groundEffectNormMiddleValue)
{
    _groundEffectNormMiddleValue = groundEffectNormMiddleValue;
}

double RotorDevice::groundEffectMaxRate() const
{
    return _groundEffectMaxRate;
}

void RotorDevice::setGroundEffectMaxRate(double groundEffectMaxRate)
{
    _groundEffectMaxRate = groundEffectMaxRate;
}

void RotorDevice::showParameter(){

    if(_on)std::cout<<"T"<<std::endl;
    else std::cout<<"F"<<std::endl;
    std::cout<<_pos<<std::endl;
    std::cout<<_dir<<std::endl;
    std::cout<<_val<<std::endl;
    std::cout<<_minVal<<std::endl;
    std::cout<<_maxVal<<std::endl;
    std::cout<<_torque<<std::endl;
    std::cout<<_torqueMinVal<<std::endl;
    std::cout<<_torqueMaxVal<<std::endl;
    std::cout<<_wallEffectDistance<<std::endl;
    std::cout<<_wallEffectNormMiddleValue<<std::endl;
    std::cout<<_wallEffectMaxRate<<std::endl;
    std::cout<<_groundEffectDistance<<std::endl;
    std::cout<<_groundEffectNormMiddleValue<<std::endl;
    std::cout<<_groundEffectMaxRate<<std::endl;
    std::cout<<std::endl;
}
