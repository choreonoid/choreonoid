/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SCENE_RENDERER_H
#define CNOID_UTIL_SCENE_RENDERER_H

#include "SceneGraph.h"
#include "PolymorphicFunctionSet.h"
#include "exportdecl.h"

namespace cnoid {

class SceneRendererImpl;

class CNOID_EXPORT SceneRenderer
{
public:
    SceneRenderer();
    virtual ~SceneRenderer();

    void setName(const std::string& name);
    const std::string& name() const;

    static void addExtension(std::function<void(SceneRenderer* renderer)> func);
    virtual void applyExtensions();
    virtual bool applyNewExtensions();

    virtual SgGroup* sceneRoot() = 0;
    virtual SgGroup* scene() = 0;
    virtual void clearScene();

    typedef PolymorphicFunctionSet<SgNode> NodeFunctionSet;

    virtual NodeFunctionSet* renderingFunctions() = 0;
    virtual void renderCustomGroup(SgGroup* group, std::function<void()> traverseFunction) = 0;
    virtual void renderCustomTransform(SgTransform* transform, std::function<void()> traverseFunction) = 0;

    virtual void renderNode(SgNode* node) = 0;

    int numCameras() const;
    SgCamera* camera(int index);
    const SgNodePath& cameraPath(int index) const;
    virtual const Affine3& currentCameraPosition() const;
    SignalProxy<void()> sigCamerasChanged() const;
    SgCamera* currentCamera() const;
    int currentCameraIndex() const;
    void setCurrentCamera(int index);
    bool setCurrentCamera(SgCamera* camera);
    SignalProxy<void()> sigCurrentCameraChanged();
    bool getSimplifiedCameraPathStrings(int index, std::vector<std::string>& out_pathStrings);
    int findCameraPath(const std::vector<std::string>& simplifiedPathStrings);
    bool setCurrentCameraPath(const std::vector<std::string>& simplifiedPathStrings);

    int numLights() const;
    void getLightInfo(int index, SgLight*& out_light, Affine3& out_position) const;
    void setAsDefaultLight(SgLight* light);
    void unsetDefaultLight(SgLight* light);
    SgLight* headLight();
    void setHeadLight(SgLight* light);
    void enableAdditionalLights(bool on);

    void enableFog(bool on);
    bool isFogEnabled() const;
    int numFogs() const;
    SgFog* fog(int index) const;

    virtual const Affine3& currentModelTransform() const = 0;
    virtual const Matrix4& projectionMatrix() const = 0;

    /**
       This function updates the information on preprocessed nodes such as
       cameras, lights, and fogs.
    */
    virtual void extractPreprocessedNodes();
    
    void render();
    bool pick(int x, int y);
    
    virtual void flush() = 0;

    Signal<void()>& sigRenderingRequest();

    class CNOID_EXPORT PropertyKey {
        int id;
    public:
        PropertyKey(const std::string& key);
        friend class SceneRendererImpl;
    };
    
    void setProperty(PropertyKey key, bool value);
    void setProperty(PropertyKey key, int value);
    void setProperty(PropertyKey key, double value);
    bool property(PropertyKey key, bool defaultValue) const;
    int property(PropertyKey key, int defaultValue) const;
    double property(PropertyKey key, double defaultValue) const;

protected:
    virtual void doRender() = 0;
    virtual bool doPick(int x, int y);
    virtual void onSceneGraphUpdated(const SgUpdate& update);

private:
    SceneRendererImpl* impl;
};

}

#endif
