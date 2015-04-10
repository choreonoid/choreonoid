/*!
 * @brief Defines the minimum processing for performing pasing file for STL. 
 * @author Hisashi Ikari
 */

#ifndef CNOID_BODY_COLLADA_BODY_LOADER_H_INCLUDED
#define CNOID_BODY_COLLADA_BODY_LOADER_H_INCLUDED

#include "AbstractBodyLoader.h"
#include "exportdecl.h"

namespace cnoid
{
class ColladaBodyLoaderImpl;

class CNOID_EXPORT ColladaBodyLoader : public AbstractBodyLoader
{
public:
    ColladaBodyLoader();
    ~ColladaBodyLoader();
    virtual const char* format() const;
    virtual void setMessageSink(std::ostream& os);
    virtual void setVerbose(bool on);
    virtual void enableShapeLoading(bool on);
    virtual void setDefaultDivisionNumber(int n);
    virtual bool load(BodyPtr body, const std::string& filename);
    virtual VRMLNodePtr retriveOriginalNode(Link* link);

private:
    ColladaBodyLoaderImpl* impl;
};

}; // end of namespace

#endif
