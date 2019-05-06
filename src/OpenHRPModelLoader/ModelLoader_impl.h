/*!
  @file
  @author Shizuko Hattori
*/

#ifndef CNOID_OPENHRP_MODELLOADER_MODELLOADER_IMPL_H
#define CNOID_OPENHRP_MODELLOADER_MODELLOADER_IMPL_H

#include <cnoid/corba/OpenHRPModelLoader/ModelLoader.hh>
#include <string>
#include <map>

namespace cnoid {

class ModelLoader_impl : public POA_OpenHRP::ModelLoader,
                         public PortableServer::RefCountServantBase
{
    CORBA::ORB_var orb;
    PortableServer::POA_var poa;
    std::ostream* os;

    typedef std::map<std::string, POA_OpenHRP::BodyInfo*> UrlToBodyInfoMap;
    UrlToBodyInfoMap urlToBodyInfoMap;
    typedef std::map<POA_OpenHRP::BodyInfo*, time_t> BodyFileUpdateTimeMap;
    BodyFileUpdateTimeMap bodyFileUpdateTimeMap;

    POA_OpenHRP::BodyInfo* loadBodyInfoFromModelFile(const std::string& filename);

  public:

    ModelLoader_impl(CORBA::ORB_ptr orb, PortableServer::POA_ptr poa);
    virtual ~ModelLoader_impl();

    virtual PortableServer::POA_ptr _default_POA() override;
    virtual OpenHRP::BodyInfo_ptr getBodyInfo(const char* url) override;
    virtual OpenHRP::BodyInfo_ptr getBodyInfoEx(const char* url, const OpenHRP::ModelLoader::ModelLoadOption& option) override;
    virtual OpenHRP::BodyInfo_ptr loadBodyInfo(const char* url) override;
    virtual OpenHRP::BodyInfo_ptr loadBodyInfoEx(const char* url, const OpenHRP::ModelLoader::ModelLoadOption& option) override;
    virtual OpenHRP::SceneInfo_ptr loadSceneInfo(const char* url) override;
    virtual void clearData() override;

    void shutdown();
    void setMessageSink(std::ostream& os_) { os = &os_; };
};

}

#endif
