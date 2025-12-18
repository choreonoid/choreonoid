#include "SceneLoader.h"
#include "NullOut.h"
#include "UTF8.h"
#include "Format.h"
#include <filesystem>
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

// Use function-local static to avoid static initialization order fiasco
LoaderIdMap& getExtensionToLoaderIdMap()
{
    static LoaderIdMap instance;
    return instance;
}

// Use function-local static to avoid static initialization order fiasco
vector<LoaderFactory>& getLoaderFactories()
{
    static vector<LoaderFactory> instance;
    return instance;
}

// Use function-local static to avoid static initialization order fiasco
mutex& getLoaderMutex()
{
    static mutex instance;
    return instance;
}

// Use function-local static to avoid static initialization order fiasco
Signal<void(const std::vector<std::string>& extensions)>& getSigAvailableFileExtensionsAdded()
{
    static Signal<void(const std::vector<std::string>& extensions)> instance;
    return instance;
}

}

namespace cnoid {

class SceneLoader::Impl
{
public:
    SceneLoader* self;
    ostream* os_;
    ostream& os() { return *os_; }
    typedef map<int, AbstractSceneLoaderPtr> LoaderMap;
    LoaderMap loaders;
    std::shared_ptr<AbstractSceneLoader> actualSceneLoaderOnLastLoading;
    int defaultDivisionNumber;
    double defaultCreaseAngle;

    Impl(SceneLoader* impl);
    AbstractSceneLoaderPtr findLoader(string ext);
    SgNode* load(const std::string& filename, bool* out_isSupportedFormat);
};

}


void SceneLoader::registerLoader
(const std::vector<std::string>& extensions, std::function<std::shared_ptr<AbstractSceneLoader>()> factory)
{
    if(!extensions.empty()){
        lock_guard<mutex> lock(getLoaderMutex());
        const int id = getLoaderFactories().size();
        getLoaderFactories().push_back(factory);
        for(auto& ext : extensions){
            getExtensionToLoaderIdMap()[ext] = id;
        }
        getSigAvailableFileExtensionsAdded()(extensions);
    }
}


void SceneLoader::registerLoader(const char* extension, std::function<AbstractSceneLoaderPtr()> factory)
{
    if(extension){
        lock_guard<mutex> lock(getLoaderMutex());
        const int id = getLoaderFactories().size();
        getLoaderFactories().push_back(factory);
        getExtensionToLoaderIdMap()[extension] = id;
        getSigAvailableFileExtensionsAdded()({ extension });
    }
}


std::vector<std::string> SceneLoader::availableFileExtensions()
{
    lock_guard<mutex> lock(getLoaderMutex());
    vector<string> extensions;
    extensions.reserve(getExtensionToLoaderIdMap().size());
    for(auto& kv : getExtensionToLoaderIdMap()){
        extensions.push_back(kv.first);
    }
    return extensions;
}


SignalProxy<void(const std::vector<std::string>& extensions)> SceneLoader::sigAvailableFileExtensionsAdded()
{
    return getSigAvailableFileExtensionsAdded();
}


SceneLoader::SceneLoader()
{
    impl = new Impl(this);
}


SceneLoader::Impl::Impl(SceneLoader* self)
    : self(self)
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


AbstractSceneLoaderPtr SceneLoader::Impl::findLoader(string ext)
{
    AbstractSceneLoaderPtr loader;
    
    lock_guard<mutex> lock(getLoaderMutex());

    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    auto p = getExtensionToLoaderIdMap().find(ext);
    if(p != getExtensionToLoaderIdMap().end()){
        const int loaderId = p->second;
        auto q = loaders.find(loaderId);
        if(q == loaders.end()){
            loader = getLoaderFactories()[loaderId]();
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


SgNode* SceneLoader::Impl::load(const std::string& filename, bool* out_isSupportedFormat)
{
    std::filesystem::path filepath(fromUTF8(filename));

    string ext = filepath.extension().string();
    if(ext.empty()){
        os() << formatR(_("The file format of \"{}\" is unknown because it lacks a file name extension."),
                        toUTF8(filepath.filename().string())) << endl;
        return nullptr;
    }
    ext = ext.substr(1); // remove the dot

    SgNode* node = nullptr;
    auto loader = findLoader(ext);
    if(!loader){
        if(!out_isSupportedFormat){
            os() << formatR(_("The file format of \"{}\" is not supported by the scene loader."),
                            toUTF8(filepath.filename().string())) << endl;
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
        loader->setLengthUnitHint(self->lengthUnitHint());
        loader->setUpperAxisHint(self->upperAxisHint());
        
        node = loader->load(filename);
        
        actualSceneLoaderOnLastLoading = loader;
        os().flush();
    }

    return node;
}


std::shared_ptr<AbstractSceneLoader> SceneLoader::actualSceneLoaderOnLastLoading()
{
    return impl->actualSceneLoaderOnLastLoading;
}


