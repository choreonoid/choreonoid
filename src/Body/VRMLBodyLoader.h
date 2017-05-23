/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_VRML_BODY_LOADER_H
#define CNOID_BODY_VRML_BODY_LOADER_H

#include "AbstractBodyLoader.h"
#include <cnoid/VRML>
#include "exportdecl.h"

namespace cnoid {

class Link;
class VRMLBodyLoaderImpl;
  
class CNOID_EXPORT VRMLBodyLoader : public AbstractBodyLoader
{
public:
    VRMLBodyLoader();
    ~VRMLBodyLoader();
    virtual void setMessageSink(std::ostream& os);
    virtual void setVerbose(bool on);
    virtual void enableShapeLoading(bool on);
    virtual void setDefaultDivisionNumber(int n);
    virtual bool load(Body* body, const std::string& filename);
    VRMLNodePtr getOriginalNode(Link* link);

private:
    VRMLBodyLoaderImpl* impl;
};

}

#endif
