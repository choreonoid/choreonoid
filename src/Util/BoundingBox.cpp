/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "BoundingBox.h"

using namespace cnoid;


BoundingBox::BoundingBox()
{
    clear();
}

        
BoundingBox::BoundingBox(const Vector3& min, const Vector3& max)
{
    set(min, max);
}


BoundingBox::BoundingBox(const BoundingBox& org)
{
    min_ = org.min_;
    max_ = org.max_;
    empty_ = org.empty_;
}


BoundingBox::BoundingBox(const BoundingBoxf& org)
{
    min_ = org.min().cast<Vector3::Scalar>();
    max_ = org.max().cast<Vector3::Scalar>();
    empty_ = org.empty();
}


void BoundingBox::set(const Vector3& min, const Vector3& max)
{
    min_ = min;
    max_ = max;
    empty_ = (min.x() >= max.x()) && (min.y() >= max.y()) && (min.z() >= max.z());
}


void BoundingBox::clear()
{
    empty_ = true;
    min_.setConstant(std::numeric_limits<Vector3::Scalar>::max());
    max_.setConstant(-std::numeric_limits<Vector3::Scalar>::max());
}


Vector3 BoundingBox::center() const
{
    return (min_ + max_) / 2.0;
}


Vector3 BoundingBox::size() const
{
    if(empty_){
        return Vector3::Zero();
    } else {
        return (max_ - min_);
    }
}


double BoundingBox::boundingSphereRadius() const
{
    if(empty_){
        return 0.0;
    } else {
        return (max_ - center()).norm();
    }
}


void BoundingBox::expandBy(double x, double y, double z)
{
    if(x < min_.x()){
        min_.x() = x;
    }
    if(x > max_.x()){
        max_.x() = x;
    }
    if(y < min_.y()){
        min_.y() = y;
    }
    if(y > max_.y()){
        max_.y() = y;
    }
    if(z < min_.z()){
        min_.z() = z;
    }
    if(z > max_.z()){
        max_.z() = z;
    }
    if(empty_){
        empty_ = (min_.x() >= max_.x()) && (min_.y() >= max_.y()) && (min_.z() >= max_.z());
    }
}


void BoundingBox::expandBy(const BoundingBox& bbox)
{
    if(!bbox.empty()){
        if(bbox.min().x() < min_.x()){
            min_.x() = bbox.min().x();
        }
        if(bbox.max().x() > max_.x()){
            max_.x() = bbox.max().x();
        }
        if(bbox.min().y() < min_.y()){
            min_.y() = bbox.min().y();
        }
        if(bbox.max().y() > max_.y()){
            max_.y() = bbox.max().y();
        }
        if(bbox.min().z() < min_.z()){
            min_.z() = bbox.min().z();
        }
        if(bbox.max().z() > max_.z()){
            max_.z() = bbox.max().z();
        }
        if(empty_){
            empty_ = (min_.x() >= max_.x()) && (min_.y() >= max_.y()) && (min_.z() >= max_.z());
        }
    }
}


void BoundingBox::transform(const Affine3& T)
{
    if(!empty()){
        const Vector3 p1 = min_;
        const Vector3 p2 = max_;
        clear();
        expandBy(T * Vector3(p1.x(), p1.y(), p1.z()));
        expandBy(T * Vector3(p1.x(), p2.y(), p1.z()));
        expandBy(T * Vector3(p2.x(), p1.y(), p1.z()));
        expandBy(T * Vector3(p2.x(), p2.y(), p1.z()));
        expandBy(T * Vector3(p1.x(), p1.y(), p2.z()));
        expandBy(T * Vector3(p1.x(), p2.y(), p2.z()));
        expandBy(T * Vector3(p2.x(), p1.y(), p2.z()));
        expandBy(T * Vector3(p2.x(), p2.y(), p2.z()));
    }
}


std::ostream& cnoid::operator<<(std::ostream& os, const BoundingBox& bb)
{
    os << "(" << bb.min().x() << ", " << bb.min().y() << ", " << bb.min().z() << ") - (";
    os << bb.max().x() << ", " << bb.max().y() << ", " << bb.max().z() << ")";
    return os;
}


BoundingBoxf::BoundingBoxf()
{
    clear();
}

        
BoundingBoxf::BoundingBoxf(const Vector3f& min, const Vector3f& max)
{
    set(min, max);
}


BoundingBoxf::BoundingBoxf(const BoundingBoxf& org)
{
    min_ = org.min_;
    max_ = org.max_;
    empty_ = org.empty_;
}


BoundingBoxf::BoundingBoxf(const BoundingBox& org)
{
    min_ = org.min().cast<Vector3f::Scalar>();
    max_ = org.max().cast<Vector3f::Scalar>();
    empty_ = org.empty();
}


void BoundingBoxf::set(const Vector3f& min, const Vector3f& max)
{
    min_ = min;
    max_ = max;
    empty_ = (min.x() >= max.x()) && (min.y() >= max.y()) && (min.z() >= max.z());
}


void BoundingBoxf::clear()
{
    empty_ = true;
    min_.setConstant(std::numeric_limits<Vector3f::Scalar>::max());
    max_.setConstant(-std::numeric_limits<Vector3f::Scalar>::max());
}


Vector3f BoundingBoxf::center() const
{
    return (min_ + max_) / 2.0f;
}


Vector3f BoundingBoxf::size() const
{
    if(empty_){
        return Vector3f::Zero();
    } else {
        return (max_ - min_);
    }
}


float BoundingBoxf::boundingSphereRadius() const
{
    return (max_ - center()).norm();
}


void BoundingBoxf::expandBy(float x,  float y, float z)
{
    if(x < min_.x()){
        min_.x() = x;
    }
    if(x > max_.x()){
        max_.x() = x;
    }
    if(y < min_.y()){
        min_.y() = y;
    }
    if(y > max_.y()){
        max_.y() = y;
    }
    if(z < min_.z()){
        min_.z() = z;
    }
    if(z > max_.z()){
        max_.z() = z;
    }
    if(empty_){
        empty_ = (min_.x() >= max_.x()) && (min_.y() >= max_.y()) && (min_.z() >= max_.z());
    }
}


void BoundingBoxf::expandBy(const BoundingBoxf& bbox)
{
    if(!bbox.empty()){
        if(bbox.min().x() < min_.x()){
            min_.x() = bbox.min().x();
        }
        if(bbox.max().x() > max_.x()){
            max_.x() = bbox.max().x();
        }
        if(bbox.min().y() < min_.y()){
            min_.y() = bbox.min().y();
        }
        if(bbox.max().y() > max_.y()){
            max_.y() = bbox.max().y();
        }
        if(bbox.min().z() < min_.z()){
            min_.z() = bbox.min().z();
        }
        if(bbox.max().z() > max_.z()){
            max_.z() = bbox.max().z();
        }
        if(empty_){
            empty_ = (min_.x() >= max_.x()) && (min_.y() >= max_.y()) && (min_.z() >= max_.z());
        }
    }
}


void BoundingBoxf::transform(const Affine3f& T)
{
    if(!empty()){
        const Vector3f p1 = min_;
        const Vector3f p2 = max_;
        clear();
        expandBy(T * Vector3f(p1.x(), p1.y(), p1.z()));
        expandBy(T * Vector3f(p1.x(), p2.y(), p1.z()));
        expandBy(T * Vector3f(p2.x(), p1.y(), p1.z()));
        expandBy(T * Vector3f(p2.x(), p2.y(), p1.z()));
        expandBy(T * Vector3f(p1.x(), p1.y(), p2.z()));
        expandBy(T * Vector3f(p1.x(), p2.y(), p2.z()));
        expandBy(T * Vector3f(p2.x(), p1.y(), p2.z()));
        expandBy(T * Vector3f(p2.x(), p2.y(), p2.z()));
    }
}


std::ostream& cnoid::operator<<(std::ostream& os, const BoundingBoxf& bb)
{
    os << "(" << bb.min().x() << ", " << bb.min().y() << ", " << bb.min().z() << ") - (";
    os << bb.max().x() << ", " << bb.max().y() << ", " << bb.max().z() << ")";
    return os;
}
