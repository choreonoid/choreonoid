#ifndef CNOID_UTIL_ABSTRACT_SCENE_LOADER_H
#define CNOID_UTIL_ABSTRACT_SCENE_LOADER_H

#include "SceneGraph.h"
#include <iosfwd>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT AbstractSceneLoader
{
public:
    AbstractSceneLoader();
    virtual ~AbstractSceneLoader();

    virtual void setMessageSink(std::ostream& os);
    virtual void setDefaultDivisionNumber(int n);
    virtual void setDefaultCreaseAngle(double theta);

    enum LengthUnitType { Meter, Millimeter, Inch, NumLengthUnitTypes };
    virtual void setLengthUnitHint(LengthUnitType hint);
    LengthUnitType lengthUnitHint() const { return lengthUnitHint_; }
    
    enum UpperAxisType { Z_Upper, Y_Upper, NumUpperAxisTypes };
    virtual void setUpperAxisHint(UpperAxisType hint);
    UpperAxisType upperAxisHint() const { return upperAxisHint_; }

    void clearHintsForLoading();
    void restoreLengthUnitAndUpperAxisHints(Mapping* metadata);

    virtual SgNode* load(const std::string& filename) = 0;

protected:
    SgNode* insertTransformNodesToAdjustLengthUnitAndUpperAxis(SgNode* node);
    SgNode* insertTransformNodeToAdjustUpperAxis(SgNode* node);
    void storeLengthUnitAndUpperAxisHintsAsMetadata(SgObject* object);

private:
    LengthUnitType lengthUnitHint_;
    UpperAxisType upperAxisHint_;
};

}

#endif
