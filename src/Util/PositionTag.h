#ifndef CNOID_POSITION_TAG_H
#define CNOID_POSITION_TAG_H

#include "Referenced.h"
#include "EigenTypes.h"
#include "exportdecl.h"

namespace cnoid {

class Mapping;
class ArchiveSession;

class CNOID_EXPORT PositionTag : public Referenced
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PositionTag();
    PositionTag(const Position& T);
    PositionTag(const Vector3& location);
    PositionTag(const PositionTag& org);

    Position::ConstTranslationPart translation() const {
        return position_.translation();
    }
    template<typename Derived>
    void setTranslation(const Eigen::MatrixBase<Derived>& p) {
        position_.translation() = p.template cast<Position::Scalar>();
    }

    bool hasAttitude() const { return hasAttitude_; }

    void clearAttitude() {
        position_.linear().setIdentity();
        hasAttitude_ = false;
    }

    const Position& position() const { return position_; }

    template<class Scalar, int Mode, int Options>
        void setPosition(const Eigen::Transform<Scalar, 3, Mode, Options>& p) {
        position_ = p.template cast<Position::Scalar>();
        hasAttitude_ = true;
    }

    Position::ConstLinearPart rotation() const { return position_.linear(); }

    template<typename Derived>
        void setRotation(const Eigen::MatrixBase<Derived>& R) {
        position_.linear() = R.template cast<Position::Scalar>();
        hasAttitude_ = true;
    }
    
    bool read(const Mapping* archive, ArchiveSession* session);
    bool write(Mapping* archive, ArchiveSession* session) const;

private:
    Position position_;
    bool hasAttitude_;
};

typedef ref_ptr<PositionTag> PositionTagPtr;

}

#endif
