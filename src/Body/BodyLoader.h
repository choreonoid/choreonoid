/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_BODY_LOADER_H_INCLUDED
#define CNOID_BODY_BODY_LOADER_H_INCLUDED

#include "AbstractBodyLoader.h"
#include <boost/function.hpp>
#include "exportdecl.h"

namespace cnoid {

class BodyLoaderImpl;

class CNOID_EXPORT BodyLoader : public AbstractBodyLoader
{
public:
    static bool registerLoader(const std::string& extension, boost::function<AbstractBodyLoaderPtr()> factory);
        
    BodyLoader();
    ~BodyLoader();
    virtual const char* format() const;
    virtual void setMessageSink(std::ostream& os);
    virtual void setVerbose(bool on);
    virtual void enableShapeLoading(bool on);
    virtual void setDefaultDivisionNumber(int n);
    virtual void setDefaultCreaseAngle(double theta);
    virtual bool load(BodyPtr body, const std::string& filename);
    BodyPtr load(const std::string& filename);
    AbstractBodyLoaderPtr lastActualBodyLoader() const;

private:
    BodyLoaderImpl* impl;
};
}

#endif
