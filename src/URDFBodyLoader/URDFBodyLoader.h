#ifndef CNOID_URDF_BODY_LOADER_URDF_BODY_LOADER_H
#define CNOID_URDF_BODY_LOADER_URDF_BODY_LOADER_H

#include "exportdecl.h"

#include <cnoid/AbstractBodyLoader>

namespace cnoid {

class CNOID_EXPORT URDFBodyLoader : public AbstractBodyLoader
{
public:
    URDFBodyLoader();
    ~URDFBodyLoader();
    void setMessageSink(std::ostream& os) override;
    virtual bool load(Body* body, const std::string& filename) override;

private:
    class Impl;
    Impl* impl;
};

};  // namespace cnoid

#endif
