/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SceneRenderer.h"
#include "SceneDrawables.h"
#include "SceneCameras.h"
#include "SceneLights.h"
#include "SceneEffects.h"
#include "EigenUtil.h"
#include <cnoid/stdx/variant>
#include <set>
#include <unordered_map>
#include <mutex>

using namespace std;
using namespace cnoid;

namespace {

std::mutex extensionMutex;
set<SceneRenderer*> renderers;
vector<std::function<void(SceneRenderer* renderer)>> extendFunctions;

std::mutex propertyKeyMutex;
int propertyKeyCount = 0;
std::unordered_map<string, int> propertyKeyMap;

struct PreproNode
{
    enum { GROUP, TRANSFORM, PREPROCESSED, LIGHT, FOG, CAMERA };
    stdx::variant<SgGroup*, SgTransform*, SgPreprocessed*, SgLight*, SgFog*, SgCamera*> node;
    SgNode* base;
    PreproNode* parent;
    PreproNode* child;
    PreproNode* next;

    template<class T>
    PreproNode(T* n) : parent(0), child(0), next(0) {
        setNode(n);
    }
    
    template<class T> void setNode(T* n){
        node = n;
        base = n;
    }

    ~PreproNode() {
        if(child) delete child;
        if(next) delete next;
    }
};

class PreproTreeExtractor
{
    PolymorphicSceneNodeFunctionSet functions;
    PreproNode* node;
    bool found;

public:
    PreproTreeExtractor();
    PreproNode* apply(SgNode* node);
    void visitGroup(SgGroup* group);
};

}

namespace cnoid {

SceneRenderer::PropertyKey::PropertyKey(const std::string& key)
{
    std::lock_guard<std::mutex> guard(propertyKeyMutex);

    auto iter = propertyKeyMap.find(key);
    if(iter != propertyKeyMap.end()){
        id = iter->second;
    } else {
        id = propertyKeyCount;
        propertyKeyMap.insert(make_pair(key, id));
        propertyKeyCount++;
    }
}

class SceneRenderer::Impl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    SceneRenderer* self;
    string name;
    bool builtinFlagToUpdatePreprocessedNodeTree;
    bool* pFlagToUpdatePreprocessedNodeTree;
    std::unique_ptr<PreproNode> preproTree;

    struct CameraInfo
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        CameraInfo() : camera(0), M(Isometry3::Identity()), node(0) { }
        CameraInfo(SgCamera* camera, const Isometry3& M, PreproNode* node)
            : camera(camera), M(M), node(node)
        { }
        
        SgCamera* camera;

        // I want to use 'T' here, but that causes a compile error (c2327) for VC++2010.
        // 'T' is also used as a template parameter in a template code of Eigen,
        // and it seems that the name of a template parameter and this member conflicts.
        // So I changed the name to 'M'.
        // This behavior of VC++ seems stupid!!!
        Isometry3 M;
        PreproNode* node;
    };

    typedef vector<CameraInfo, Eigen::aligned_allocator<CameraInfo>> CameraInfoArray;
    CameraInfoArray cameras1;
    CameraInfoArray cameras2;
    CameraInfoArray* cameras;
    CameraInfoArray* prevCameras;

    Isometry3 I;

    bool camerasChanged;
    bool currentCameraRemoved;
    bool isCurrentCameraAutoRestorationMode;
    bool isPreferredCameraCurrent;
    int currentCameraIndex;
    SgCamera* currentCamera;
    vector<SgNodePath> cameraPaths;
    vector<string> preferredCurrentCameraPathStrings;
    Signal<void()> sigCamerasChanged;
    Signal<void()> sigCurrentCameraChanged;
        
    struct LightInfo
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        LightInfo() : light(0) { }
        LightInfo(SgLight* light, const Isometry3& M) : light(light), M(M) { }
        SgLight* light;
        Isometry3 M;
    };
    vector<LightInfo, Eigen::aligned_allocator<LightInfo>> lights;

    SgLightPtr headLight;
    std::set<SgLightPtr> defaultLights;
    bool additionalLightsEnabled;

    vector<SgFogPtr> fogs;
    bool isFogEnabled;

    std::mutex newExtensionMutex;
    vector<std::function<void(SceneRenderer* renderer)>> newExtendFunctions;

    typedef stdx::variant<bool, int, double> PropertyValue;
    vector<PropertyValue> properties;
    
    Impl(SceneRenderer* self);

    void extractPreproNodes();
    void extractPreproNodeIter(PreproNode* node, const Affine3& T);
    void updateCameraPaths();
    void setCurrentCamera(int index);
    bool setCurrentCamera(SgCamera* camera);
    bool getSimplifiedCameraPathStrings(int cameraIndex, std::vector<std::string>& out_pathStrings);
    void onExtensionAdded(std::function<void(SceneRenderer* renderer)> func);

    template<class ValueType>
    void setProperty(SceneRenderer::PropertyKey key, ValueType value){
        const int id = key.id;
        if(id >= static_cast<int>(properties.size())){
            properties.resize(id + 1);
        }
        properties[id] = value;
    }

    template<class ValueType>
    ValueType property(SceneRenderer::PropertyKey key, int which, ValueType defaultValue){
        const int id = key.id;
        if(id >= static_cast<int>(properties.size())){
            properties.resize(id + 1);
            return defaultValue;
        }
        const PropertyValue& value = properties[id];
        if(stdx::get_variant_index(value) == which){
            return stdx::get<ValueType>(value);
        }
        return defaultValue;
    }    
};

}


SceneRenderer::SceneRenderer()
{
    impl = new Impl(this);
    std::lock_guard<std::mutex> guard(extensionMutex);
    renderers.insert(this);
}


SceneRenderer::Impl::Impl(SceneRenderer* self)
    : self(self)
{
    builtinFlagToUpdatePreprocessedNodeTree = true;
    pFlagToUpdatePreprocessedNodeTree = &builtinFlagToUpdatePreprocessedNodeTree;

    cameras = &cameras1;
    prevCameras = &cameras2;
    isCurrentCameraAutoRestorationMode = false;
    isPreferredCameraCurrent = false;
    currentCameraIndex = -1;
    currentCamera = nullptr;
    I.setIdentity();

    headLight = new SgDirectionalLight();
    headLight->setAmbientIntensity(0.0f);
    additionalLightsEnabled = true;

    isFogEnabled = true;
}


SceneRenderer::~SceneRenderer()
{
    std::lock_guard<std::mutex> guard(extensionMutex);
    renderers.erase(this);
    delete impl;
}


void SceneRenderer::setName(const std::string& name)
{
    impl->name = name;
}


const std::string& SceneRenderer::name() const
{
    return impl->name;
}


void SceneRenderer::clearScene()
{
    SgTmpUpdate update;
    scene()->clearChildren(update);
}


void SceneRenderer::render()
{
    doRender();
}


bool SceneRenderer::pick(int x, int y)
{
    bool result = doPick(x, y);
    return result;
}


bool SceneRenderer::doPick(int /* x */, int /* y */)
{
    return false;
}


bool SceneRenderer::isRenderingPickingImage() const
{
    return false;
}


void SceneRenderer::setProperty(PropertyKey key, bool value)
{
    impl->setProperty(key, value);
}


void SceneRenderer::setProperty(PropertyKey key, int value)
{
    impl->setProperty(key, value);
}


void SceneRenderer::setProperty(PropertyKey key, double value)
{
    impl->setProperty(key, value);
}


bool SceneRenderer::property(PropertyKey key, bool defaultValue) const
{
    return impl->property(key, 0, defaultValue);
}


int SceneRenderer::property(PropertyKey key, int defaultValue) const
{
    return impl->property(key, 1, defaultValue);
}


double SceneRenderer::property(PropertyKey key, double defaultValue) const
{
    return impl->property(key, 2, defaultValue);
}


void SceneRenderer::setFlagVariableToUpdatePreprocessedNodeTree(bool& flag)
{
    impl->pFlagToUpdatePreprocessedNodeTree = &flag;
}


void SceneRenderer::extractPreprocessedNodes()
{
    impl->extractPreproNodes();
}


void SceneRenderer::Impl::extractPreproNodes()
{
    if(*pFlagToUpdatePreprocessedNodeTree){
        PreproTreeExtractor extractor;
        preproTree.reset(extractor.apply(self->sceneRoot()));
        *pFlagToUpdatePreprocessedNodeTree = false;
        builtinFlagToUpdatePreprocessedNodeTree = true;
    }

    std::swap(cameras, prevCameras);
    cameras->clear();
    camerasChanged = false;
    currentCameraRemoved = true;
    
    lights.clear();
    fogs.clear();

    if(preproTree){
        extractPreproNodeIter(preproTree.get(), Affine3::Identity());
    }

    if(!camerasChanged){
        if(cameras->size() != prevCameras->size()){
            camerasChanged = true;
        }
    }
    if(camerasChanged){
        if(currentCameraRemoved){
            currentCameraIndex = 0;
            if(isPreferredCameraCurrent){
                isPreferredCameraCurrent = false;
            }
        }
        cameraPaths.clear();
        sigCamerasChanged();
    }

    bool isCurrentCameraUpdated = false;
    if(isCurrentCameraAutoRestorationMode){
        if(!isPreferredCameraCurrent && !preferredCurrentCameraPathStrings.empty()){
            if(self->setCurrentCameraPath(preferredCurrentCameraPathStrings)){
                isPreferredCameraCurrent = true;
                isCurrentCameraUpdated = true;
            }
        }
    }

    if(!isCurrentCameraUpdated){
        setCurrentCamera(currentCameraIndex);
    }
}


void SceneRenderer::Impl::extractPreproNodeIter(PreproNode* node, const Affine3& T)
{
    switch(stdx::get_variant_index(node->node)){

    case PreproNode::GROUP:
        for(PreproNode* childNode = node->child; childNode; childNode = childNode->next){
            extractPreproNodeIter(childNode, T);
        }
        break;
        
    case PreproNode::TRANSFORM:
    {
        SgTransform* transform = stdx::get<SgTransform*>(node->node);
        Affine3 T1;
        transform->getTransform(T1);
        const Affine3 T2 = T * T1;
        for(PreproNode* childNode = node->child; childNode; childNode = childNode->next){
            extractPreproNodeIter(childNode, T2);
        }
    }
    break;
        
    case PreproNode::PREPROCESSED:
        // call additional functions
        break;

    case PreproNode::LIGHT:
    {
        SgLight* light = stdx::get<SgLight*>(node->node);
        if(additionalLightsEnabled || defaultLights.find(light) != defaultLights.end()){
            lights.push_back(LightInfo(light, convertToIsometryWithOrthonormalization(T)));
        }
        break;
    }

    case PreproNode::FOG:
    {
        SgFog* fog = stdx::get<SgFog*>(node->node);
        fogs.push_back(fog);
        break;
    }

    case PreproNode::CAMERA:
    {
        SgCamera* camera = stdx::get<SgCamera*>(node->node);
        size_t index = cameras->size();
        if(!camerasChanged){
            if(index >= prevCameras->size() || camera != (*prevCameras)[index].camera){
                camerasChanged = true;
            }
        }
        if(camera == currentCamera){
            currentCameraRemoved = false;
            currentCameraIndex = cameras->size();
        }
        cameras->push_back(CameraInfo(camera, convertToIsometryWithOrthonormalization(T), node));
    }
    break;

    default:
        break;
    }
}


namespace {

PreproTreeExtractor::PreproTreeExtractor()
{
    functions.setFunction<SgGroup>(
        [&](SgNode* node){ visitGroup(static_cast<SgGroup*>(node)); });

    functions.setFunction<SgSwitchableGroup>(
        [&](SgSwitchableGroup* group){
            if(group->isTurnedOn()){
                visitGroup(group);
            }
        });

    functions.setFunction<SgTransform>(
        [&](SgTransform* transform){
            visitGroup(transform);
            if(node){
                node->setNode(transform);
            }
        });

    functions.setFunction<SgPreprocessed>(
        [&](SgPreprocessed* preprocessed){
            node = new PreproNode(preprocessed);
            found = true;
        });

    functions.setFunction<SgLight>(
        [&](SgLight* light){
            node = new PreproNode(light);
            found = true;
        });

    functions.setFunction<SgFog>(
        [&](SgFog* fog){
            node = new PreproNode(fog);
            found = true;
        });

    functions.setFunction<SgCamera>(
        [&](SgCamera* camera){
            node = new PreproNode(camera);
            found = true;
        });
    
    functions.updateDispatchTable();
}


PreproNode* PreproTreeExtractor::apply(SgNode* snode)
{
    node = nullptr;
    found = false;
    functions.dispatch(snode);
    return node;
}


void PreproTreeExtractor::visitGroup(SgGroup* group)
{
    bool foundInSubTree = false;

    PreproNode* self = new PreproNode(group);

    for(SgGroup::const_reverse_iterator p = group->rbegin(); p != group->rend(); ++p){
        
        node = nullptr;
        found = false;

        functions.dispatch(*p);
        
        if(node){
            if(found){
                node->parent = self;
                node->next = self->child;
                self->child = node;
                foundInSubTree = true;
            } else {
                delete node;
            }
        }
    }
            
    found = foundInSubTree;

    if(found){
        node = self;
    } else {
        delete self;
        node = nullptr;
    }
}

}


int SceneRenderer::numCameras() const
{
    return impl->cameras->size();
}


SgCamera* SceneRenderer::camera(int index)
{
    return dynamic_cast<SgCamera*>(cameraPath(index).back());
}


const SgNodePath& SceneRenderer::cameraPath(int index) const
{
    if(impl->cameraPaths.empty()){
        impl->updateCameraPaths();
    }
    return impl->cameraPaths[index];
}


void SceneRenderer::Impl::updateCameraPaths()
{
    SgNodePath tmpPath;
    const int n = cameras->size();
    cameraPaths.resize(n);
    
    for(int i=0; i < n; ++i){
        CameraInfo& info = (*cameras)[i];
        tmpPath.clear();
        PreproNode* node = info.node;
        while(node){
            tmpPath.push_back(node->base);
            node = node->parent;
        }
        if(!tmpPath.empty()){
            tmpPath.pop_back(); // remove the root node
            SgNodePath& path = cameraPaths[i];
            path.resize(tmpPath.size());
            std::copy(tmpPath.rbegin(), tmpPath.rend(), path.begin());
        }
    }
}


const Isometry3& SceneRenderer::cameraPosition(int index) const
{
    if(index < static_cast<int>(impl->cameras->size())){
        return (*(impl->cameras))[index].M;
    } else {
        return impl->I;
    }
}


SignalProxy<void()> SceneRenderer::sigCamerasChanged() const
{
    return impl->sigCamerasChanged;
}


void SceneRenderer::setCurrentCamera(int index)
{
    impl->setCurrentCamera(index);
}


void SceneRenderer::Impl::setCurrentCamera(int index)
{
    SgCamera* newCamera = nullptr;
    if(index >= 0 && index < static_cast<int>(cameras->size())){
        newCamera = (*cameras)[index].camera;
    }
    if(newCamera && newCamera != currentCamera){
        currentCameraIndex = index;
        currentCamera = newCamera;
        if(isCurrentCameraAutoRestorationMode){
            getSimplifiedCameraPathStrings(index, preferredCurrentCameraPathStrings);
            isPreferredCameraCurrent = true;
        }
        sigCurrentCameraChanged();
    }
}


bool SceneRenderer::setCurrentCamera(SgCamera* camera)
{
    return impl->setCurrentCamera(camera);
}


bool SceneRenderer::Impl::setCurrentCamera(SgCamera* camera)
{
    if(camera != currentCamera){
        for(size_t i=0; i < cameras->size(); ++i){
            if((*cameras)[i].camera == camera){
                setCurrentCamera(i);
                return true;
            }
        }
    }
    return false;
}


SgCamera* SceneRenderer::currentCamera() const
{
    return impl->currentCamera;
}


int SceneRenderer::currentCameraIndex() const
{
    return impl->currentCameraIndex;
}


const Isometry3& SceneRenderer::currentCameraPosition() const
{
    if(impl->currentCameraIndex >= 0){
        return (*(impl->cameras))[impl->currentCameraIndex].M;
    } else {
        return impl->I;
    }
}


SignalProxy<void()> SceneRenderer::sigCurrentCameraChanged()
{
    return impl->sigCurrentCameraChanged;
}


std::vector<std::string> SceneRenderer::simplifiedCameraPathStrings(int cameraIndex)
{
    std::vector<std::string> pathStrings;
    impl->getSimplifiedCameraPathStrings(cameraIndex, pathStrings);
    return pathStrings;
}


bool SceneRenderer::getSimplifiedCameraPathStrings(int cameraIndex, std::vector<std::string>& out_pathStrings)
{
    return impl->getSimplifiedCameraPathStrings(cameraIndex, out_pathStrings);
}


bool SceneRenderer::Impl::getSimplifiedCameraPathStrings(int cameraIndex, std::vector<std::string>& out_pathStrings)
{
    out_pathStrings.clear();

    int n = cameras->size();
    if(cameraIndex < n){
        const SgNodePath& path = self->cameraPath(cameraIndex);
        const string& name = path.back()->name();
        if(!name.empty()){
            size_t n = path.size() - 1;
            for(size_t i=0; i < n; ++i){
                const string& element = path[i]->name();
                if(!element.empty()){
                    out_pathStrings.push_back(element);
                    break;
                }
            }
            out_pathStrings.push_back(name);
        }
    }
    return !out_pathStrings.empty();

}


/**
   @return Camera index, or -1 if the path is not found.
*/
int SceneRenderer::findCameraPath(const std::vector<std::string>& simplifiedPathStrings)
{
    int index = -1;
    
    if(!simplifiedPathStrings.empty()){
        vector<int> candidates;
        const string& name = simplifiedPathStrings.back();
        const int n = numCameras();
        for(int i=0; i < n; ++i){
            const SgNodePath& path = cameraPath(i);
            if(path.back()->name() == name){
                candidates.push_back(i);
            }
        }
        if(candidates.size() == 1){
            index = candidates.front();

        } else if(candidates.size() >= 2){
            if(simplifiedPathStrings.size() == 1){
                index = candidates.front();
            } else {
                const string& owner = simplifiedPathStrings.front();
                for(size_t i=0; i < candidates.size(); ++i){
                    const SgNodePath& path = cameraPath(i);
                    if(path.front()->name() == owner){
                        index = i;
                        break;
                    }
                }
            }
        }
    }

    return index;
}


bool SceneRenderer::setCurrentCameraPath(const std::vector<std::string>& simplifiedPathStrings)
{
    int index = findCameraPath(simplifiedPathStrings);
    if(index >= 0){
        setCurrentCamera(index);
        if(impl->isCurrentCameraAutoRestorationMode){
            impl->preferredCurrentCameraPathStrings = simplifiedPathStrings;
            impl->isPreferredCameraCurrent = true;
        }
        return true;
    }
    return false;
}


void SceneRenderer::setCurrentCameraAutoRestorationMode(bool on)
{
    impl->isCurrentCameraAutoRestorationMode = on;
    impl->preferredCurrentCameraPathStrings.clear();
    impl->isPreferredCameraCurrent = false;
}


int SceneRenderer::numLights() const
{
    return impl->lights.size();
}


void SceneRenderer::getLightInfo(int index, SgLight*& out_light, Isometry3& out_position) const
{
    if(index < static_cast<int>(impl->lights.size())){
        const Impl::LightInfo& info = impl->lights[index];
        out_light = info.light;
        out_position = info.M;
    } else {
        out_light = nullptr;
    }
}


SgLight* SceneRenderer::headLight()
{
    return impl->headLight;
}


void SceneRenderer::setHeadLight(SgLight* light)
{
    impl->headLight = light;
}


void SceneRenderer::setAsDefaultLight(SgLight* light)
{
    impl->defaultLights.insert(light);
}


void SceneRenderer::unsetDefaultLight(SgLight* light)
{
    impl->defaultLights.erase(light);
}


void SceneRenderer::enableAdditionalLights(bool on)
{
    impl->additionalLightsEnabled = on;
}

void SceneRenderer::enableFog(bool on)
{
    impl->isFogEnabled = on;
}


bool SceneRenderer::isFogEnabled() const
{
    return impl->isFogEnabled;
}


int SceneRenderer::numFogs() const
{
    return impl->fogs.size();
}


SgFog* SceneRenderer::fog(int index) const
{
    return impl->fogs[index];
}


void SceneRenderer::addExtension(std::function<void(SceneRenderer* renderer)> func)
{
    {
        std::lock_guard<std::mutex> guard(extensionMutex);
        extendFunctions.push_back(func);
    }
    for(SceneRenderer* renderer : renderers){
        renderer->impl->onExtensionAdded(func);
    }
}


void SceneRenderer::applyExtensions()
{
    std::lock_guard<std::mutex> guard(extensionMutex);
    for(size_t i=0; i < extendFunctions.size(); ++i){
        extendFunctions[i](this);
    }
}


void SceneRenderer::Impl::onExtensionAdded(std::function<void(SceneRenderer* renderer)> func)
{
    std::lock_guard<std::mutex> guard(newExtensionMutex);
    newExtendFunctions.push_back(func);
}


bool SceneRenderer::applyNewExtensions()
{
    std::lock_guard<std::mutex> guard(impl->newExtensionMutex);
    if(!impl->newExtendFunctions.empty()){
        for(size_t i=0; i < impl->newExtendFunctions.size(); ++i){
            impl->newExtendFunctions[i](this);
        }
        impl->newExtendFunctions.clear();
        return true;
    }
    return false;
}
