/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SceneRenderer.h"
#include "SceneDrawables.h"
#include "SceneCameras.h"
#include "SceneLights.h"
#include "SceneEffects.h"
#include "PolymorphicFunctionSet.h"
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
    PolymorphicFunctionSet<SgNode> functions;
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

class SceneRendererImpl
{
public:
    SceneRenderer* self;
    string name;
    bool isRendering;
    bool doPreprocessedNodeTreeExtraction;
    Signal<void()> sigRenderingRequest;
    std::unique_ptr<PreproNode> preproTree;

    struct CameraInfo
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        CameraInfo() : camera(0), M(Affine3::Identity()), node(0) { }
        CameraInfo(SgCamera* camera, const Affine3& M, PreproNode* node)
            : camera(camera), M(M), node(node) { }
        SgCamera* camera;

        // I want to use 'T' here, but that causes a compile error (c2327) for VC++2010.
        // 'T' is also used as a template parameter in a template code of Eigen,
        // and it seems that the name of a template parameter and this member conflicts.
        // So I changed the name to 'M'.
        // This behavior of VC++ seems stupid!!!
        Affine3 M;
        PreproNode* node;
    };

    typedef vector<CameraInfo, Eigen::aligned_allocator<CameraInfo>> CameraInfoArray;
    CameraInfoArray cameras1;
    CameraInfoArray cameras2;
    CameraInfoArray* cameras;
    CameraInfoArray* prevCameras;

    bool camerasChanged;
    bool currentCameraRemoved;
    int currentCameraIndex;
    SgCamera* currentCamera;
    vector<SgNodePath> cameraPaths;
    Signal<void()> sigCamerasChanged;
    Signal<void()> sigCurrentCameraChanged;
        
    struct LightInfo
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        LightInfo() : light(0) { }
        LightInfo(SgLight* light, const Affine3& M) : light(light), M(M) { } 
        SgLight* light;
        Affine3 M;
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
    
    SceneRendererImpl(SceneRenderer* self);

    void extractPreproNodes();
    void extractPreproNodeIter(PreproNode* node, const Affine3& T);
    void updateCameraPaths();
    void setCurrentCamera(int index, bool doRenderingRequest);
    bool setCurrentCamera(SgCamera* camera);
    void onExtensionAdded(std::function<void(SceneRenderer* renderer)> func);

    template<class ValueType> void setProperty(SceneRenderer::PropertyKey key, ValueType value){
        const int id = key.id;
        if(id >= static_cast<int>(properties.size())){
            properties.resize(id + 1);
        }
        properties[id] = value;
    }

    template<class ValueType> ValueType property(SceneRenderer::PropertyKey key, int which, ValueType defaultValue){
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
    impl = new SceneRendererImpl(this);
    std::lock_guard<std::mutex> guard(extensionMutex);
    renderers.insert(this);
}


SceneRendererImpl::SceneRendererImpl(SceneRenderer* self)
    : self(self)
{
    isRendering = false;
    doPreprocessedNodeTreeExtraction = true;

    cameras = &cameras1;
    prevCameras = &cameras2;
    currentCameraIndex = -1;
    currentCamera = 0;

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
    scene()->clearChildren(true);
}


Signal<void()>& SceneRenderer::sigRenderingRequest()
{
    return impl->sigRenderingRequest;
}


void SceneRenderer::onSceneGraphUpdated(const SgUpdate& update)
{
    if(update.action() & (SgUpdate::ADDED | SgUpdate::REMOVED)){
        impl->doPreprocessedNodeTreeExtraction = true;
    }
    if(!impl->isRendering){
        impl->sigRenderingRequest();
    }
}


void SceneRenderer::render()
{
    impl->isRendering = true;
    doRender();
    impl->isRendering = false;
}


bool SceneRenderer::pick(int x, int y)
{
    impl->isRendering = true;
    bool result = doPick(x, y);
    impl->isRendering = false;
    return result;
}


bool SceneRenderer::doPick(int /* x */, int /* y */)
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


void SceneRenderer::extractPreprocessedNodes()
{
    impl->extractPreproNodes();
}


void SceneRendererImpl::extractPreproNodes()
{
    if(doPreprocessedNodeTreeExtraction){
        PreproTreeExtractor extractor;
        preproTree.reset(extractor.apply(self->sceneRoot()));
        doPreprocessedNodeTreeExtraction = false;
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
        }
        cameraPaths.clear();
        sigCamerasChanged();
    }

    setCurrentCamera(currentCameraIndex, false);
}


void SceneRendererImpl::extractPreproNodeIter(PreproNode* node, const Affine3& T)
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
            lights.push_back(LightInfo(light, T));
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
        cameras->push_back(CameraInfo(camera, T, node));
    }
    break;

    default:
        break;
    }
}


PreproTreeExtractor::PreproTreeExtractor()
{
    functions.setFunction<SgGroup>(
        [&](SgNode* node){ visitGroup(static_cast<SgGroup*>(node)); });

    functions.setFunction<SgSwitch>(
        [&](SgSwitch* node){
            if(node->isTurnedOn()){
                functions.dispatchAs<SgGroup>(node);
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
    node = 0;
    found = false;
    functions.dispatch(snode);
    return node;
}


void PreproTreeExtractor::visitGroup(SgGroup* group)
{
    bool foundInSubTree = false;

    PreproNode* self = new PreproNode(group);

    for(SgGroup::const_reverse_iterator p = group->rbegin(); p != group->rend(); ++p){
        
        node = 0;
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
        node = 0;
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


const std::vector<SgNode*>& SceneRenderer::cameraPath(int index) const
{
    if(impl->cameraPaths.empty()){
        impl->updateCameraPaths();
    }
    return impl->cameraPaths[index];
}


void SceneRendererImpl::updateCameraPaths()
{
    vector<SgNode*> tmpPath;
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
            vector<SgNode*>& path = cameraPaths[i];
            path.resize(tmpPath.size());
            std::copy(tmpPath.rbegin(), tmpPath.rend(), path.begin());
        }
    }
}


SignalProxy<void()> SceneRenderer::sigCamerasChanged() const
{
    return impl->sigCamerasChanged;
}


void SceneRenderer::setCurrentCamera(int index)
{
    impl->setCurrentCamera(index, true);
}


void SceneRendererImpl::setCurrentCamera(int index, bool doRenderingRequest)
{
    SgCamera* newCamera = 0;
    if(index >= 0 && index < static_cast<int>(cameras->size())){
        newCamera = (*cameras)[index].camera;
    }
    if(newCamera && newCamera != currentCamera){
        currentCameraIndex = index;
        currentCamera = newCamera;
        sigCurrentCameraChanged();
        if(doRenderingRequest){
            sigRenderingRequest();
        }
    }
}


bool SceneRenderer::setCurrentCamera(SgCamera* camera)
{
    return impl->setCurrentCamera(camera);
}


bool SceneRendererImpl::setCurrentCamera(SgCamera* camera)
{
    if(camera != currentCamera){
        for(size_t i=0; i < cameras->size(); ++i){
            if((*cameras)[i].camera == camera){
                setCurrentCamera(i, true);
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


const Affine3& SceneRenderer::currentCameraPosition() const
{
    if(impl->currentCameraIndex >= 0){
        return (*(impl->cameras))[impl->currentCameraIndex].M;
    } else {
        static Affine3 I = Affine3::Identity();
        return I;
    }
}


SignalProxy<void()> SceneRenderer::sigCurrentCameraChanged()
{
    return impl->sigCurrentCameraChanged;
}


bool SceneRenderer::getSimplifiedCameraPathStrings(int index, std::vector<std::string>& out_pathStrings)
{
    out_pathStrings.clear();
    
    if(index < numCameras()){
        const SgNodePath& path = cameraPath(index);
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
            const vector<SgNode*>& path = cameraPath(i);
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
                    const vector<SgNode*>& path = cameraPath(i);
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
        return true;
    }
    return false;
}


int SceneRenderer::numLights() const
{
    return impl->lights.size();
}


void SceneRenderer::getLightInfo(int index, SgLight*& out_light, Affine3& out_position) const
{
    if(index < static_cast<int>(impl->lights.size())){
        const SceneRendererImpl::LightInfo& info = impl->lights[index];
        out_light = info.light;
        out_position = info.M;
    } else {
        out_light = 0;
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


void SceneRendererImpl::onExtensionAdded(std::function<void(SceneRenderer* renderer)> func)
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
