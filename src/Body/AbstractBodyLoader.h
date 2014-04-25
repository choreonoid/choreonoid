/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_ABSTRACT_BODY_LOADER_H_INCLUDED
#define CNOID_BODY_ABSTRACT_BODY_LOADER_H_INCLUDED

#include "Body.h"
#include <iosfwd>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT AbstractBodyLoader
{
public:
    AbstractBodyLoader();
    virtual ~AbstractBodyLoader();
    virtual const char* format() const = 0;
    virtual void setMessageSink(std::ostream& os);
    virtual void setVerbose(bool on);
    virtual void setShapeLoadingEnabled(bool on);
    virtual void setDefaultDivisionNumber(int n);
    virtual void setDefaultCreaseAngle(double theta);
    virtual bool load(BodyPtr body, const std::string& filename) = 0;
};

typedef boost::shared_ptr<AbstractBodyLoader> AbstractBodyLoaderPtr;
}

#endif
