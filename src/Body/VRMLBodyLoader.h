/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_VRML_BODY_LOADER_H_INCLUDED
#define CNOID_BODY_VRML_BODY_LOADER_H_INCLUDED

#include "AbstractBodyLoader.h"
#include <cnoid/VRML>
#include "exportdecl.h"

namespace cnoid {

class VRMLBodyLoaderImpl;
  
class CNOID_EXPORT VRMLBodyLoader : public AbstractBodyLoader
{
public:
    VRMLBodyLoader();
    ~VRMLBodyLoader();
    virtual const char* format() const;
    virtual void setMessageSink(std::ostream& os);
    virtual void setVerbose(bool on);
    virtual void enableShapeLoading(bool on);
    virtual void setDefaultDivisionNumber(int n);
    virtual bool load(BodyPtr body, const std::string& filename);
    virtual VRMLNodePtr retriveOriginalNode(Link* link);

private:
    VRMLBodyLoaderImpl* impl;
};
}

#endif
