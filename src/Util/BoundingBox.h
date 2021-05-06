/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_BOUNDING_BOX_H
#define CNOID_UTIL_BOUNDING_BOX_H

#include "EigenTypes.h"
#include <iosfwd>
#include "exportdecl.h"

namespace cnoid {

class BoundingBoxf;

class CNOID_EXPORT BoundingBox
{
public:
    BoundingBox();
    BoundingBox(const Vector3& min, const Vector3& max);
    BoundingBox(const BoundingBox& org);
    BoundingBox(const BoundingBoxf& org);

    bool operator==(const BoundingBox& rhs) const {
        return (min_ == rhs.min_) && (max_ == rhs.max_);
    }
    bool operator!=(const BoundingBox& rhs) const {
        return !operator==(rhs);
    }

    void set(const Vector3& min, const Vector3& max);
    void clear();

    bool empty() const { return empty_; }
    explicit operator bool() const { return !empty_; }
    const Vector3& min() const { return min_; }
    const Vector3& max() const { return max_; }
    Vector3 center() const;
    Vector3 size() const;
    double boundingSphereRadius() const;
        
    void expandBy(const BoundingBox& bbox);
    void expandBy(double x, double y, double z);
    void expandBy(const Vector3& v){ expandBy(v.x(), v.y(), v.z()); }

    void transform(const Affine3& T);
    void scale(double s);

private:
    Vector3 min_;
    Vector3 max_;
    bool empty_;
};

CNOID_EXPORT std::ostream& operator<<(std::ostream& os, const BoundingBox& bb);

/**
   float type version of the BoundingBox class
*/
class CNOID_EXPORT BoundingBoxf
{
public:
    BoundingBoxf();
    BoundingBoxf(const Vector3f& min, const Vector3f& max);
    BoundingBoxf(const BoundingBoxf& org);
    BoundingBoxf(const BoundingBox& org);
        
    bool operator==(const BoundingBoxf& rhs) const {
        return (min_ == rhs.min_) && (max_ == rhs.max_);
    }
    bool operator!=(const BoundingBoxf& rhs) const {
        return !operator==(rhs);
    }

    void set(const Vector3f& min, const Vector3f& max);
    void clear();

    bool empty() const { return empty_; }
    explicit operator bool() const { return !empty_; }
    const Vector3f& min() const { return min_; }
    const Vector3f& max() const { return max_; }
    Vector3f center() const;
    Vector3f size() const;
    float boundingSphereRadius() const;
        
    void expandBy(const BoundingBoxf& bbox);
    void expandBy(const BoundingBox& bbox);
    void expandBy(float x, float y, float z);
    void expandBy(const Vector3f& v){ expandBy(v.x(), v.y(), v.z()); }

    void transform(const Affine3f& T);
    void scale(float f);

private:
    bool empty_;
    Vector3f min_;
    Vector3f max_;
};

CNOID_EXPORT std::ostream& operator<<(std::ostream& os, const BoundingBoxf& bb);

}    

#endif
