/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_EIGEN_ARCHIVE_H
#define CNOID_UTIL_EIGEN_ARCHIVE_H

#include "ValueTree.h"
#include "EigenUtil.h"
#include <fmt/format.h>
#include <functional>
#include "exportdecl.h"

namespace cnoid {

template<typename Derived>
void read(const Listing& listing, Eigen::MatrixBase<Derived>& x)
{
    const int nr = x.rows();
    const int nc = x.cols();
    if(listing.size() != nr * nc){
        listing.throwException(
            fmt::format("A {0} x {1} matrix / vector value is expected", nr, nc));
    }
    int index = 0;
    for(int i=0; i < nr; ++i){
        for(int j=0; j < nc; ++j){
            x(i, j) = listing[index++].toDouble();
        }
    }
}


template<typename Derived>
bool read(const Mapping& mapping, const std::string& key, Eigen::MatrixBase<Derived>& x)
{
    const Listing& s = *mapping.findListing(key);
    if(!s.isValid()){
        return false;
    }
    read(s, x);
    return true;
}


template<typename Scalar, int Dim, int Mode>
bool read(const Mapping& mapping, const std::string& key, Eigen::Transform<Scalar, Dim, Mode>& T)
{
    return read(mapping, key, T.matrix());
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


template<typename Scalar, int Dim, int Mode>
Listing& write(Mapping& mapping, const std::string& key, const Eigen::Transform<Scalar, Dim, Mode>& T)
{
    return write(mapping, key, T.matrix());
}


CNOID_EXPORT bool read(const Mapping& mapping, const std::string& key, Eigen::AngleAxisd& r);
CNOID_EXPORT bool read(const Mapping& mapping, const std::string& key, Eigen::AngleAxisf& r);

CNOID_EXPORT Listing& write(Mapping& mapping, const std::string& key, const Eigen::AngleAxisd& r);
CNOID_EXPORT Listing& write(Mapping& mapping, const std::string& key, const Eigen::AngleAxisf& r);

CNOID_EXPORT bool read(const Mapping& mapping, const std::string& key, std::function<void(const Eigen::Vector3d& value)> setterFunc);

}

#endif
