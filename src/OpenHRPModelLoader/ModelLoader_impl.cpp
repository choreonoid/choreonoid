/*!
  @file
  @author Shizuko Hattori
*/

#include "ModelLoader_impl.h"
#include "BodyInfo_impl.h"
#include "SceneInfo_impl.h"
#include <boost/filesystem.hpp>
#include <iostream>

using namespace std;
using namespace cnoid;
using namespace OpenHRP;
namespace filesystem = boost::filesystem;

namespace {

//Copy from VRMLParser.cpp
string removeURLScheme(string url)
{
    static const string fileProtocolHeader1("file://");
    static const string fileProtocolHeader2("file:");

    size_t pos = url.find(fileProtocolHeader1);
    if(0 == pos){
        url.erase(0, fileProtocolHeader1.size());
    } else {
        size_t pos = url.find(fileProtocolHeader2);
        if(0 == pos) {
            url.erase(0, fileProtocolHeader2.size());
        }
    }

    // Remove the first slash if the successive part is a Windows drive letter
    if(url.find(":") == 2 && url.find("/") ==0){
        url.erase(0, 1);
    }

    return url;
}

}


ModelLoader_impl::ModelLoader_impl(CORBA::ORB_ptr orb, PortableServer::POA_ptr poa)
    : orb(CORBA::ORB::_duplicate(orb)),
      poa(PortableServer::POA::_duplicate(poa))
{
    os = &cout;
}


ModelLoader_impl::~ModelLoader_impl()
{
    clearData();
}


POA_OpenHRP::BodyInfo* ModelLoader_impl::loadBodyInfoFromModelFile(const string& filename)
{
    BodyInfo_impl* bodyInfoImpl = new BodyInfo_impl(poa);
    bodyInfoImpl->setMessageSink(*os);
    try{
        bodyInfoImpl->loadModelFile(filename);
    }catch(OpenHRP::ModelLoader::ModelLoaderException& ex){
        *os << ex.description << endl;
        throw;
    }

    urlToBodyInfoMap[filename] = bodyInfoImpl;
    const filesystem::path path(filename);
    const time_t last_update = filesystem::last_write_time(path);
    bodyFileUpdateTimeMap[bodyInfoImpl] = last_update;

    return bodyInfoImpl;
}


BodyInfo_ptr ModelLoader_impl::getBodyInfo(const char* url)
{
    const string filename = removeURLScheme(string(url));
    UrlToBodyInfoMap::iterator p = urlToBodyInfoMap.find(filename);
    if(p != urlToBodyInfoMap.end() ){
        const filesystem::path path(filename);
        const time_t last_update = filesystem::last_write_time(path);
        if(last_update == bodyFileUpdateTimeMap[p->second]){
            return p->second->_this();
        }
    }

    return loadBodyInfo(url);
}


BodyInfo_ptr ModelLoader_impl::getBodyInfoEx(const char* url, const OpenHRP::ModelLoader::ModelLoadOption& option)
{
    return getBodyInfo(url);
}


PortableServer::POA_ptr ModelLoader_impl::_default_POA()
{
    return PortableServer::POA::_duplicate(poa);
}


void ModelLoader_impl::clearData()
{
    urlToBodyInfoMap.clear();
    bodyFileUpdateTimeMap.clear();
}


BodyInfo_ptr ModelLoader_impl::loadBodyInfo(const char* url)
{
    const string filename = removeURLScheme(string(url));
    POA_OpenHRP::BodyInfo* bodyInfo = loadBodyInfoFromModelFile(filename);
    return bodyInfo->_this();
}


BodyInfo_ptr ModelLoader_impl::loadBodyInfoEx(const char* url, const OpenHRP::ModelLoader::ModelLoadOption& option)
{
    return loadBodyInfo(url);
}


SceneInfo_ptr ModelLoader_impl::loadSceneInfo(const char* url)
{
    const string filename = removeURLScheme(string(url));
    SceneInfo_impl* sceneInfoImpl = new SceneInfo_impl(poa);
    sceneInfoImpl->setMessageSink(*os);
    try {
            sceneInfoImpl->load(filename);
    }
    catch(OpenHRP::ModelLoader::ModelLoaderException& ex){
        *os << ex.description << endl;
        throw;
    }
    return sceneInfoImpl->_this();
}


void ModelLoader_impl::shutdown()
{
    clearData();
    orb->shutdown(false);
}
