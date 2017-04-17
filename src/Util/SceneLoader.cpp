/*!
  @author Shin'ichiro Nakaoka
*/

#include "SceneLoader.h"
#include <cnoid/NullOut>
#include <cnoid/FileUtil>
#include <boost/format.hpp>
#include <mutex>
#include <map>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

typedef function<AbstractSceneLoaderPtr()> LoaderFactory;
typedef map<string, LoaderFactory> LoaderFactoryMap;
LoaderFactoryMap loaderFactoryMap;
mutex loaderFactoryMapMutex;

}

namespace cnoid {

class SceneLoaderImpl
{
public:
    ostream* os;
    typedef map<string, AbstractSceneLoaderPtr> LoaderMap;
    LoaderMap loaderMap;
    int defaultDivisionNumber;
    double defaultCreaseAngle;

    SceneLoaderImpl();
    SgNode* load(const std::string& filename);
};

}


void SceneLoader::registerLoader(const std::string& extension, std::function<AbstractSceneLoaderPtr()> factory)
{
    lock_guard<mutex> lock(loaderFactoryMapMutex);
    loaderFactoryMap[extension] = factory;
}


SceneLoader::SceneLoader()
{
    impl = new SceneLoaderImpl;
}


SceneLoaderImpl::SceneLoaderImpl()
{
    os = &nullout();
    defaultDivisionNumber = -1;
    defaultCreaseAngle = -1.0;
}


SceneLoader::~SceneLoader()
{
    delete impl;
}


SgNode* SceneLoader::load(const std::string& filename)
{
    return impl->load(filename);
}


SgNode* SceneLoaderImpl::load(const std::string& filename)
{
    SgNode* node = 0;
    boost::filesystem::path filepath(filename);
    string ext = getExtension(filepath);

    auto iter = loaderMap.find(ext);

    AbstractSceneLoaderPtr loader;
    
    LoaderMap::iterator p = loaderMap.find(ext);
    if(p != loaderMap.end()){
        loader = p->second;
    } else {
        lock_guard<mutex> lock(loaderFactoryMapMutex);
        LoaderFactoryMap::iterator q = loaderFactoryMap.find(ext);
        if(q != loaderFactoryMap.end()){
            LoaderFactory factory = q->second;
            loader = factory();
            loaderMap[ext] = loader;
        }
    }
    
    if(!loader){
        (*os) << str(boost::format(_("The file format of \"%1%\" is not supported by the scene loader."))
                     % getFilename(filepath)) << endl;
    } else {
        loader->setMessageSink(*os);
        if(defaultDivisionNumber > 0){
            loader->setDefaultDivisionNumber(defaultDivisionNumber);
        }
        if(defaultCreaseAngle >= 0.0){
            loader->setDefaultCreaseAngle(defaultCreaseAngle);
        }
        node = loader->load(filename);
    }

    return node;
}
