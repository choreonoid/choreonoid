/**
   @author Japan Atomic Energy Agency
*/

#pragma once

namespace Multicopter{
class SimulationManager;

class FluidEnvironment
{
friend class SimulationManager;
public:

    class FluidValue{
    public:
        bool isFluid;
        double density;
        Eigen::Vector3d velocity;
        double viscosity;

        friend FluidValue operator+(const FluidValue& lhs, const FluidValue& rhs){
            FluidEnvironment::FluidValue tmp;
            tmp.density = lhs.density+rhs.density;
            tmp.velocity = lhs.velocity+rhs.velocity;
            tmp.viscosity = lhs.viscosity+rhs.viscosity;
            return tmp;            
        }
                
        friend FluidValue operator-(const FluidValue& lhs, const FluidValue& rhs){
            FluidEnvironment::FluidValue tmp;
            tmp.density = lhs.density-rhs.density;
            tmp.velocity = lhs.velocity-rhs.velocity;
            tmp.viscosity = lhs.viscosity-rhs.viscosity;
            return tmp;            
        }

        friend FluidValue operator*(const FluidValue& lhs, const double rhs){
            FluidEnvironment::FluidValue tmp;
            tmp.density = lhs.density*rhs;
            tmp.velocity = lhs.velocity*rhs;
            tmp.viscosity = lhs.viscosity*rhs;
            return tmp;            
        }
    };

    bool get(const Eigen::Vector3d& pos, FluidValue& val) const;

    bool load(const std::string& fileName);

    Boxd boundary() const{
        return _bnds;
    }

    bool isNull() const{
        return _isNull;
    }
        
protected:

    FluidEnvironment();

    ~FluidEnvironment();

    bool isFluidCell(int x, int y, int z) const;

    void string2Vector3i(const std::string& line, Eigen::Vector3i& ret);

    bool stringArray2Data(const std::vector<std::string>& strAry, Eigen::Vector3i& idx, FluidValue& val);

private:
    bool _isNull;
    Eigen::Vector3d _orgPos;
    Eigen::Vector3d _gridLen;
    Eigen::Vector3i _gridSize;
    Boxd _bnds;
    Array3<FluidValue> _valAry;

};
}

