#ifndef CNOID_SDF_BODY_LOADER_SDF_BODY_LOADER_H
#define CNOID_SDF_BODY_LOADER_SDF_BODY_LOADER_H

#include <cnoid/AbstractBodyLoader>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SDFBodyLoader : public AbstractBodyLoader
{
public:
    SDFBodyLoader();
    ~SDFBodyLoader();
    virtual void setMessageSink(std::ostream& os) override;
    virtual void setDefaultDivisionNumber(int n) override;
    virtual void setDefaultCreaseAngle(double theta) override;
    virtual bool load(Body* body, const std::string& filename) override;

private:
    class Impl;
    Impl* impl;
};

};  // namespace cnoid

#endif
