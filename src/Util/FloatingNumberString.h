/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_FLOATING_NUMBER_STRING_H
#define CNOID_UTIL_FLOATING_NUMBER_STRING_H

#include <string>
#include <fmt/format.h>

#ifdef _MSC_VER
#define INFINITY (DBL_MAX+DBL_MAX)
#define NAN (INFINITY-INFINITY)
#else
#include <cmath>
#endif

namespace cnoid {

class FloatingNumberString
{
    double v;
    std::string s;

public:
    FloatingNumberString() {
        v = 0.0;
        s = "0";
    }
            
    FloatingNumberString(const std::string& value)
        : s(value) {
        char* p;
        double nv = strtod(value.c_str(), &p);
        if(p != value.c_str()){
            v = nv;
            s = value;
        }
    }

    FloatingNumberString(double value) {
        operator=(value);
    }

    bool set(const std::string& value){
        char* p;
        double nv = strtod(value.c_str(), &p);
        if(p != value.c_str()){
            v = nv;
            s = value;
            return true;
        }
        return false;
    }
        
        
    FloatingNumberString& operator=(const FloatingNumberString& rhs){
        s = rhs.s;
        v = rhs.v;
        return *this;
    }

    FloatingNumberString& operator=(const std::string& rhs){
        set(rhs);
        return *this;
    }

    FloatingNumberString& operator=(double rhs){
        v = rhs;
        s = fmt::format("{:g}", rhs);
        return *this;
    }

    bool setPositiveValue(const std::string& value){
        char* p;
        double nv = strtod(value.c_str(), &p);
        if(p != value.c_str() && nv > 0.0){
            v = nv;
            s = value;
            return true;
        }
        return false;
    }

    bool setNonNegativeValue(const std::string& value){
        char* p;
        double nv = strtod(value.c_str(), &p);
        if(p != value.c_str() && nv >= 0.0){
            v = nv;
            s = value;
            return true;
        }
        return false;
    }
        
    operator std::string() const {
        return s;
    }

    const std::string& string() const {
        return s;
    }

    double value() const {
        return v;
    }
};
            
}

#endif
