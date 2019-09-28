/*!
  @author Shin'ichiro Nakaoka
*/

#include "SceneLoader.h"
#include "NullOut.h"
#include "FileUtil.h"
#include <fmt/format.h>
#include <mutex>
#include <map>
#include <algorithm>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

typedef shared_ptr<AbstractSceneLoader> AbstractSceneLoaderPtr;
typedef function<AbstractSceneLoaderPtr()> LoaderFactory;
typedef map<string, int> LoaderIdMap;
LoaderIdMap loaderIdMap;
vector<LoaderFactory> loaderFactories;
mutex loaderMutex;

}

namespace cnoid {

class SceneLoaderImpl
{
public:
    ostream* os_;
    ostream& os() { return *os_; }
    typedef map<int, AbstractSceneLoaderPtr> LoaderMap;
    LoaderMap loaders;
    int defaultDivisionNumber;
    double defaultCreaseAngle;

    SceneLoaderImpl();
    AbstractSceneLoaderPtr findLoader(string ext);
    SgNode* load(const std::string& filename, bool* out_isSupportedFormat);
};

}


void SceneLoader::registerLoader(const char* extensions, std::function<AbstractSceneLoaderPtr()> factory)
{
    vector<string> extensionArray;
    const char* str = extensions;;
    do {
        const char* begin = str;
        while(*str != ';' && *str) ++str;
        extensionArray.push_back(string(begin, str));
    } while(0 != *str++);
    
    {
        lock_guard<mutex> lock(loaderMutex);
        const int id = loaderFactories.size();
        loaderFactories.push_back(factory);
        for(size_t i=0; i < extensionArray.size(); ++i){
            loaderIdMap[extensionArray[i]] = id;
        }
    }
}


std::string SceneLoader::availableFileExtensions()
{
    string extensions;
    for(auto iter = loaderIdMap.begin(); iter != loaderIdMap.end(); ++iter){
        const string& extension = iter->first;
        if(!extensions.empty()){
            extensions += ";";
        }
        extensions += extension;
    }
    return extensions;
}


SceneLoader::SceneLoader()
{
    impl = new SceneLoaderImpl;
}


SceneLoaderImpl::SceneLoaderImpl()
{
    os_ = &nullout();
    defaultDivisionNumber = -1;
    defaultCreaseAngle = -1.0;
}


SceneLoader::~SceneLoader()
{
    delete impl;
}


void SceneLoader::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
}


void SceneLoader::setDefaultDivisionNumber(int n)
{
    impl->defaultDivisionNumber = n;
}


void SceneLoader::setDefaultCreaseAngle(double theta)
{
    impl->defaultCreaseAngle = theta;
}


AbstractSceneLoaderPtr SceneLoaderImpl::findLoader(string ext)
{
    AbstractSceneLoaderPtr loader;
    
    lock_guard<mutex> lock(loaderMutex);

    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    auto p = loaderIdMap.find(ext);
    if(p != loaderIdMap.end()){
        const int loaderId = p->second;
        auto q = loaders.find(loaderId);
        if(q == loaders.end()){
            loader = loaderFactories[loaderId]();
            loaders[loaderId] = loader;
        } else {
            loader = q->second;
        }
    }

    return loader;
}


SgNode* SceneLoader::load(const std::string& filename)
{
    return impl->load(filename, nullptr);
}


SgNode* SceneLoader::load(const std::string& filename, bool& out_isSupportedFormat)
{
    out_isSupportedFormat = false;
    return impl->load(filename, &out_isSupportedFormat);
}


SgNode* SceneLoaderImpl::load(const std::string& filename, bool* out_isSupportedFormat)
{
    stdx::filesystem::path filepath(filename);

    string ext = getExtension(filepath);
    if(ext.empty()){
        os() << fmt::format(_("The file format of \"{}\" is unknown because it lacks a file name extension."),
                getFilename(filepath)) << endl;
        return nullptr;
    }

    SgNode* node = nullptr;
    auto loader = findLoader(ext);
    if(!loader){
        if(!out_isSupportedFormat){
            os() << fmt::format(_("The file format of \"{}\" is not supported by the scene loader."),
                                getFilename(filepath)) << endl;
        }
    } else {
        if(out_isSupportedFormat){
            *out_isSupportedFormat = true;
        }
        loader->setMessageSink(os());
        if(defaultDivisionNumber > 0){
            loader->setDefaultDivisionNumber(defaultDivisionNumber);
        }
        if(defaultCreaseAngle >= 0.0){
            loader->setDefaultCreaseAngle(defaultCreaseAngle);
        }
        node = loader->load(filename);
        os().flush();
    }

    return node;
}
