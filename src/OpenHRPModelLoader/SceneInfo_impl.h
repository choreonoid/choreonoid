/*!
  @file
  @author Shizuko Hattori
*/

#ifndef CNOID_OPENHRP_MODELLOADER_SCENEINFO_IMPL_H
#define CNOID_OPENHRP_MODELLOADER_SCENEINFO_IMPL_H

#include <cnoid/corba/OpenHRPModelLoader/ModelLoader.hh>
#include "ShapeSetInfo_impl.h"

namespace cnoid {

class SceneInfo_impl :
    public virtual POA_OpenHRP::SceneInfo,
    public virtual ShapeSetInfo_impl
{
public:
		
    SceneInfo_impl(PortableServer::POA_ptr poa);
    virtual ~SceneInfo_impl();

    virtual char* url() override;
    virtual OpenHRP::TransformedShapeIndexSequence* shapeIndices() override;

    void load(const std::string& filename);
    void setMessageSink(std::ostream& os_) { os = &os_; };

private:
    std::ostream* os;
    std::string url_;
    OpenHRP::TransformedShapeIndexSequence shapeIndices_;
};

}
#endif
