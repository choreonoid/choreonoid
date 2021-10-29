/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_EIGEN_ARCHIVE_H
#define CNOID_UTIL_EIGEN_ARCHIVE_H

#include "ValueTree.h"
#include "EigenUtil.h"
#include <fmt/format.h>
#include <functional>
#include <initializer_list>
#include "exportdecl.h"

namespace cnoid {

template<typename Derived>
void readEx(const Listing* listing, Eigen::MatrixBase<Derived>& x)
{
    const int nr = x.rows();
    const int nc = x.cols();
    if(listing->size() != nr * nc){
        listing->throwException(
            fmt::format("A {0} x {1} matrix / vector value is expected", nr, nc));
    }
    int index = 0;
    for(int i=0; i < nr; ++i){
        for(int j=0; j < nc; ++j){
            x(i, j) = (*listing)[index++].toDouble();
        }
    }
}


template<typename Derived>
void readEx(const Listing& listing, Eigen::MatrixBase<Derived>& x)
{
    readEx(&listing, x);
}


template<typename Derived>
bool read(const Mapping* mapping, const std::string& key, Eigen::MatrixBase<Derived>& x)
{
    auto s = mapping->findListing(key);
    if(!s->isValid()){
        return false;
    }
    readEx(s, x);
    return true;
}


template<typename Derived>
bool read(const Mapping* mapping, std::initializer_list<const char*> keys, Eigen::MatrixBase<Derived>& x)
{
    for(auto& key : keys){
        if(read(mapping, key, x)){
            return true;
        }
    }
    return false;
}


template<typename Derived>
bool read(const Mapping& mapping, const std::string& key, Eigen::MatrixBase<Derived>& x)
{
    return read(&mapping, key, x);
}


template<typename Derived>
bool read(const Mapping& mapping, std::initializer_list<const char*> keys, Eigen::MatrixBase<Derived>& x)
{
    return read(&mapping, keys, x);
}


template<typename Scalar, int Dim, int Mode>
bool read(const Mapping* mapping, const std::string& key, Eigen::Transform<Scalar, Dim, Mode>& T)
{
    return read(mapping, key, T.matrix());
}


template<typename Scalar, int Dim, int Mode>
bool read(const Mapping* mapping, std::initializer_list<const char*> keys, Eigen::Transform<Scalar, Dim, Mode>& T)
{
    auto M = T.matrix();
    for(auto& key : keys){
        if(read(*mapping, key, M)){
            return true;
        }
    }
    return false;
}


template<typename Scalar, int Dim, int Mode>
bool read(const Mapping& mapping, const std::string& key, Eigen::Transform<Scalar, Dim, Mode>& T)
{
    return read(&mapping, key, T.matrix());
}


template<typename Scalar, int Dim, int Mode>
bool read(const Mapping& mapping, std::initializer_list<const char*> keys, Eigen::Transform<Scalar, Dim, Mode>& T)
{
    return read(&mapping, keys, T.matrix());
}


template<typename Derived>
void readEx(const Mapping* mapping, const std::string& key, Eigen::MatrixBase<Derived>& x)
{
    if(!read(mapping, key, x)){
        mapping->throwKeyNotFoundException(key);
    }
}


template<typename Derived>
void readEx(const Mapping* mapping, std::initializer_list<const char*> keys, Eigen::MatrixBase<Derived>& x)
{
    for(auto& key : keys){
        if(read(mapping, key, x)){
            return;
        }
    }
    if(keys.begin() != keys.end()){
        mapping->throwKeyNotFoundException(*keys.begin());
    }
}


template<typename Derived>
void readEx(const Mapping& mapping, const std::string& key, Eigen::MatrixBase<Derived>& x)
{
    readEx(&mapping, key, x);
}

    
template<typename Derived>
Listing* write(Mapping* mapping, const std::string& key, const Eigen::MatrixBase<Derived>& x)
{
    auto s = mapping->createFlowStyleListing(key);
    const int nr = x.rows();
    const int nc = x.cols();
    if(nc == 1){
        for(int i=0; i < nr; ++i){
            s->append(x(i));
        }
    } else {
        for(int i=0; i < nr; ++i){
            s->appendLF();
            for(int j=0; j < nc; ++j){
                s->append(x(i, j));
            }
        }
    }
    return s;
}


template<typename Derived>
Listing& write(Mapping& mapping, const std::string& key, const Eigen::MatrixBase<Derived>& x)
{
    return *write(&mapping, key, x);
}


template<typename Scalar, int Dim, int Mode>
Listing* write(Mapping* mapping, const std::string& key, const Eigen::Transform<Scalar, Dim, Mode>& T)
{
    return write(mapping, key, T.matrix());
}


template<typename Scalar, int Dim, int Mode>
Listing& write(Mapping& mapping, const std::string& key, const Eigen::Transform<Scalar, Dim, Mode>& T)
{
    return *write(&mapping, key, T.matrix());
}

/*
  The following reading functions read angle values written in degrees unless isForcedRadianMode of
  the container list is true.
*/
CNOID_EXPORT bool readAngleAxis(const Mapping* mapping, const std::string& key, Eigen::AngleAxisd& aa);
CNOID_EXPORT bool readAngleAxis(const Mapping* mapping, std::initializer_list<const char*> keys, Eigen::AngleAxisd& aa);
CNOID_EXPORT bool readAngleAxis(const Mapping& mapping, const std::string& key, Eigen::AngleAxisd& aa);
CNOID_EXPORT bool readDegreeAngleAxis(const Mapping& mapping, const std::string& key, Eigen::AngleAxisd& aa);
CNOID_EXPORT bool readRadianAngleAxis(const Mapping& mapping, const std::string& key, Eigen::AngleAxisd& aa);

CNOID_EXPORT Listing* writeDegreeAngleAxis(Mapping* mapping, const std::string& key, const Eigen::AngleAxisd& aa);
CNOID_EXPORT Listing& writeDegreeAngleAxis(Mapping& mapping, const std::string& key, const Eigen::AngleAxisd& aa);
CNOID_EXPORT Listing* writeRadianAngleAxis(Mapping* mapping, const std::string& key, const Eigen::AngleAxisd& aa);

[[deprecated]]
CNOID_EXPORT bool read(
    const Mapping& mapping, const std::string& key, std::function<void(const Eigen::Vector3d& value)> setterFunc);

[[deprecated]]
CNOID_EXPORT Listing* writeAngleAxis(Mapping* mapping, const std::string& key, const Eigen::AngleAxisd& aa);
[[deprecated]]
CNOID_EXPORT Listing& writeAngleAxis(Mapping& mapping, const std::string& key, const Eigen::AngleAxisd& aa);
                                           
[[deprecated("Use readAngleAxis")]]
CNOID_EXPORT bool read(const Mapping& mapping, const std::string& key, Eigen::AngleAxisd& aa);
[[deprecated("Use writeDegreeAngleAxis")]]
CNOID_EXPORT Listing& write(Mapping& mapping, const std::string& key, const Eigen::AngleAxisd& aa);

}

#endif
