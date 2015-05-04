/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_EIGEN_ARCHIVE_H
#define CNOID_UTIL_EIGEN_ARCHIVE_H

#include "ValueTree.h"
#include "EigenUtil.h"
#include <boost/function.hpp>

namespace cnoid {

template<typename Derived>
bool read(const Mapping& mapping, const std::string& key, Eigen::MatrixBase<Derived>& x)
{
    const Listing& s = *mapping.findListing(key);
    if(s.isValid()){
        const int nr = x.rows();
        const int nc = x.cols();
        const int n = s.size();
        int index = 0;
        if(n > 0){
            for(int i=0; i < nr; ++i){
                for(int j=0; j < nc; ++j){
                    x(i, j) = s[index++].toDouble();
                    if(index == n){
                        break;
                    }
                }
            }
        }
        return (index == nr * nc);
    }
    return false;
}


template<typename Derived>
void readEx(const Mapping& mapping, const std::string& key, Eigen::MatrixBase<Derived>& x)
{
    if(!read(mapping, key, x)){
        mapping.throwKeyNotFoundException(key);
    }
}
    
template<typename Derived>
Listing& write(Mapping& mapping, const std::string& key, const Eigen::MatrixBase<Derived>& x)
{
    Listing& s = *mapping.createFlowStyleListing(key);
    s.setDoubleFormat("%.9g");
    const int nr = x.rows();
    const int nc = x.cols();
    if(nc == 1){
        for(int i=0; i < nr; ++i){
            s.append(x(i));
        }
    } else {
        for(int i=0; i < nr; ++i){
            s.appendLF();
            for(int j=0; j < nc; ++j){
                s.append(x(i, j));
            }
        }
    }
    return s;
}

template<typename Scalar>
bool read(const Mapping& mapping, const std::string& key, Eigen::AngleAxis<Scalar>& r)
{
    const Listing& s = *mapping.findListing(key);
    if(s.isValid() && s.size() == 4){
        r.axis() << s[0].toDouble(), s[1].toDouble(), s[2].toDouble();
        r.angle() = s[3].toDouble();
        return true;
    }
    return false;
}

template<typename Scalar>
Listing& write(Mapping& mapping, const std::string& key, const Eigen::AngleAxis<Scalar>& r)
{
    Listing& s = *mapping.createFlowStyleListing(key);
    s.setDoubleFormat("%.9g");
    s.append(r.axis()[0]);
    s.append(r.axis()[1]);
    s.append(r.axis()[2]);
    s.append(r.angle());
    return s;
}

inline bool read(const Mapping& mapping, const std::string& key, boost::function<void(Vector3&)> setterFunc)
{
    Vector3 x;
    if(read(mapping, key, x)){
        setterFunc(x);
        return true;
    }
    return false;
}

}

#endif
