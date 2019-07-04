/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_EXTRA_BODY_STATE_ACCESSOR_H
#define CNOID_BODY_EXTRA_BODY_STATE_ACCESSOR_H

#include <cnoid/Referenced>
#include <cnoid/Array2D>
#include <cnoid/EigenTypes>
#include <cnoid/Signal>
#include <cnoid/stdx/variant>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ExtraBodyStateAccessor : public Referenced
{
    static int elementSizes[];
    Signal<void()> sigStateChanged_;
        
public:

    class Angle {
        double angle;
    public:
        Angle(double rad) : angle(rad) { }
        double value() const { return angle; }
        //operator double() const { return angle; }
    };
        
    struct None { } none;

    enum Attribute {
        NORMAL = 0,
        WARNING = 1 << 0,
        STRONG_WARNING = WARNING | 1 << 1
    };

    enum { BOOL, INT, DOUBLE, ANGLE, STRING, VECTOR3, VECTORX, NONE };

    class Value {
        /**
           Vector2 cannot be contained in variant because it is an Eigen type which must be aligned
           but the variant type does not seem to ahieve that for its elements
        */
        stdx::variant<bool, int, double, Angle, std::string, Vector3f, VectorX, None> value;
        int attr;
    public:
        Value() : attr(0) { }
        Value& operator=(const Value& rhs) { value = rhs.value; attr = rhs.attr; return *this; }
        template<typename T> Value& operator=(const T& rhs) { value = rhs; return *this; }
        bool getBool() const { return stdx::get<bool>(value); }
        int getInt() const { return stdx::get<int>(value); }
        double getDouble() const { return stdx::get<double>(value); }
        double getAngle() const { return stdx::get<Angle>(value).value(); }
        void setAngle(double rad) { value = Angle(rad); }
        const std::string& getString() const { return stdx::get<std::string>(value); }
        const Vector3f& getVector3f() const { return stdx::get<Vector3f>(value); }
        Vector3 getVector3() const { return stdx::get<Vector3f>(value).cast<Vector3::Scalar>(); }
        const VectorX& getVectorX() const { return stdx::get<VectorX>(value); }
            
        int which() const { return stdx::get_variant_index(value); }
        void setAttribute(int attribute) { attr = attribute; }
        int attribute() const { return attr; }
    };

    ExtraBodyStateAccessor();
    ExtraBodyStateAccessor(const ExtraBodyStateAccessor& org);
    virtual ~ExtraBodyStateAccessor();

    static int getNumValueElements(const Value& v){
        int n = elementSizes[v.which()];
        if(n < 0){
            n = v.getVectorX().size();
        }
        return n;
    }

    virtual int getNumStateItems() const = 0;
    virtual int getNumJointStateItems() const = 0;
    virtual const char* getStateItemName(int stateIndex) const = 0;
    //! Translated version of getStateItemName()
    virtual const char* getStateItemLabel(int stateIndex) const = 0;
    virtual const char* getJointStateItemName(int jointStateIndex) const = 0;
    //! Translated version of getJointStateItemName()
    virtual const char* getJointStateItemLabel(int jointStateIndex) const = 0;

    virtual void getState(std::vector<Value>& out_state) const = 0;

    /**
       This function returns false by default.
       The function which updates the state values and returns true
       should be implementedin a inherited class.
    */
    virtual bool setState(const std::vector<Value>& state) const;

    /**
       @return out_jointState 2D array of (jointIndex, itemIndex)
    */
    virtual void getJointState(Array2D<Value>& out_jointState) const = 0;

    /**
       This function returns false by default.
       The function which updates the state values and returns true
       should be implementedin the inherited class.
    */
    virtual bool setJointState(const Array2D<Value>& jointState) const;

    SignalProxy<void()> sigStateChanged() { return sigStateChanged_; }
    void notifyStateChange() { sigStateChanged_(); }
};

template<> inline ExtraBodyStateAccessor::Value&
ExtraBodyStateAccessor::Value::operator=(const Vector3& rhs) { value = Vector3f(rhs.cast<float>()); return *this; }

typedef ref_ptr<ExtraBodyStateAccessor> ExtraBodyStateAccessorPtr;
};

#endif
