#ifndef CNOID_POSITION_TAG_H
#define CNOID_POSITION_TAG_H

#include "Referenced.h"
#include "EigenTypes.h"
#include "exportdecl.h"

namespace cnoid {

class Mapping;

class CNOID_EXPORT PositionTag : public Referenced
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PositionTag();
    PositionTag(const Isometry3& T);
    PositionTag(const Vector3& location);
    PositionTag(const PositionTag& org);

    PositionTag& operator=(const PositionTag& rhs);

    Isometry3::ConstTranslationPart translation() const {
        return position_.translation();
    }
    template<typename Derived>
    void setTranslation(const Eigen::MatrixBase<Derived>& p) {
        position_.translation() = p.template cast<Isometry3::Scalar>();
    }

    bool hasAttitude() const { return hasAttitude_; }

    void clearAttitude() {
        position_.linear().setIdentity();
        hasAttitude_ = false;
    }

    const Isometry3& position() const { return position_; }

    template<class Scalar, int Mode, int Options>
        void setPosition(const Eigen::Transform<Scalar, 3, Mode, Options>& p) {
        position_ = p.template cast<Isometry3::Scalar>();
        hasAttitude_ = true;
    }

    Isometry3::ConstLinearPart rotation() const { return position_.linear(); }

    template<typename Derived>
        void setRotation(const Eigen::MatrixBase<Derived>& R) {
        position_.linear() = R.template cast<Isometry3::Scalar>();
        hasAttitude_ = true;
    }
    
    bool read(const Mapping* archive);
    bool write(Mapping* archive) const;

private:
    Isometry3 position_;
    bool hasAttitude_;
};

typedef ref_ptr<PositionTag> PositionTagPtr;

}

#endif
