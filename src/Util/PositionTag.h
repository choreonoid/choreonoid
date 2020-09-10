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
    PositionTag(const Vector3& location);
    PositionTag(const PositionTag& org);

    Position::TranslationPart translation() {
        return position_.translation();
    }
    template<typename Derived>
    void setTranslation(const Eigen::MatrixBase<Derived>& p) {
        position_.translation() = p.template cast<Position::Scalar>();
    }

    bool read(const Mapping* archive);
    void write(Mapping* archive) const;

private:
    Position position_;
    bool hasAttitude_;
};

typedef ref_ptr<PositionTag> PositionTagPtr;

}

#endif
