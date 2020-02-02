/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_BODY_LOADER_H
#define CNOID_BODY_BODY_LOADER_H

#include "AbstractBodyLoader.h"
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class BodyLoaderImpl;

class CNOID_EXPORT BodyLoader : public AbstractBodyLoader
{
public:
    static bool registerLoader(const std::string& extension, std::function<AbstractBodyLoaderPtr()> factory);
        
    BodyLoader();
    ~BodyLoader();
    virtual void setMessageSink(std::ostream& os);
    virtual void setVerbose(bool on);
    virtual void setShapeLoadingEnabled(bool on);
    virtual void setDefaultDivisionNumber(int n);
    virtual void setDefaultCreaseAngle(double theta);

    enum LengthUnit { Meter, Millimeter, Inch, NumLengthUnitIds };
    enum UpperAxis { Z, Y, NumUpperAxisIds };
    void setMeshImportHint(LengthUnit unit, UpperAxis axis);
    
    virtual bool load(Body* body, const std::string& filename);
    Body* load(const std::string& filename);
    AbstractBodyLoaderPtr lastActualBodyLoader() const;

private:
    BodyLoaderImpl* impl;
};

}

#endif
