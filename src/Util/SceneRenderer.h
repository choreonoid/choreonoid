#ifndef CNOID_UTIL_SCENE_RENDERER_H
#define CNOID_UTIL_SCENE_RENDERER_H

#include "SceneGraph.h"
#include "PolymorphicSceneNodeFunctionSet.h"
#include <optional>
#include <vector>
#include <cmath>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SceneRenderer
{
    class Impl;

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

    //! \deprecated. Use PolymorphicSceneNodeFunctionSet
    typedef PolymorphicSceneNodeFunctionSet NodeFunctionSet;

    virtual PolymorphicSceneNodeFunctionSet* renderingFunctions() = 0;
    virtual void renderCustomGroup(SgGroup* group, const std::function<void()>& traverseFunction) = 0;
    virtual void renderCustomTransform(SgTransform* transform, const std::function<void()>& traverseFunction) = 0;
    virtual void renderNode(SgNode* node) = 0;

    void setTintColor(std::optional<Vector3f> color) {
        tintColor_ = color;
        onTintColorChanged();
    }
    void clearTintColor() {
        tintColor_ = std::nullopt;
    }
    std::optional<Vector3f> tintColor() const {
        return tintColor_;
    }

    typedef std::function<SgNode*(SgNode* targetNode)> NodeDecorationFunction;
    virtual void addNodeDecoration(SgNode* targetNode, NodeDecorationFunction func, int id) = 0;
    virtual void clearNodeDecorations(int id) = 0;
    
    int numCameras() const;
    SgCamera* camera(int index);
    const SgNodePath& cameraPath(int index) const;
    const Isometry3& cameraPosition(int index) const;
    const Isometry3& currentCameraPosition() const;
    SignalProxy<void()> sigCameraListChanged() const;
    [[deprecated("Use sigCameraListChanged")]]
    SignalProxy<void()> sigCamerasChanged() const;
    SgCamera* currentCamera() const;
    int currentCameraIndex() const;
    void setCurrentCamera(int index);
    bool setCurrentCamera(SgCamera* camera);
    SignalProxy<void()> sigCurrentCameraSelectionChanged();
    [[deprecated("Use sigCurrentCameraSelectionChanged")]]
    SignalProxy<void()> sigCurrentCameraChanged();
    std::vector<std::string> simplifiedCameraPathStrings(int cameraIndex);
    bool getSimplifiedCameraPathStrings(int cameraIndex, std::vector<std::string>& out_pathStrings);
    int findCameraPath(const std::vector<std::string>& simplifiedPathStrings);
    bool setCurrentCameraPath(const std::vector<std::string>& simplifiedPathStrings);
    void setCurrentCameraAutoRestorationMode(bool on);

    SgLight* headLight();
    void setHeadLight(SgLight* light);
    SgLight* worldLight();
    void setWorldLight(SgLight* light);
    SgPosTransform* worldLightTransform();

    void enableAdditionalLights(bool on);
    int numAdditionalLights() const;
    [[deprecated("Use numAdditionalLights")]]
    int numLights() const { return numAdditionalLights(); }
    void getLightInfo(int index, SgLight*& out_light, Isometry3& out_position) const;

    void enableFog(bool on);
    bool isFogEnabled() const;
    int numFogs() const;
    SgFog* fog(int index) const;

    const std::vector<SgVisibilityProcessorPtr>& visibilityProcessors();

    virtual const Affine3& currentModelTransform() const = 0;
    virtual const Matrix4& projectionMatrix() const = 0;
    virtual const Matrix4& viewProjectionMatrix() const = 0;

    virtual Vector3 project(const Vector3& p) const = 0;
    virtual double projectedPixelSizeRatio(const Vector3& position) const = 0;
    virtual bool unproject(double x, double y, double z, Vector3& out_projected) const = 0;

    /**
       This function updates the information on preprocessed nodes such as
       cameras, lights, and fogs.
    */
    void extractPreprocessedNodes();

    /*
       This flag variable is used to check if the tree of preprocessed nodes
       must be updated when the extractPreprocessedNodes function is executed.
       You can skip the redundant update process by setting false to the flag
       when the tree is not changed.
    */
    void setFlagVariableToUpdatePreprocessedNodeTree(bool& flag);
    
    void render();
    bool pick(int x, int y);

    virtual bool isRenderingPickingImage() const;
    
    class CNOID_EXPORT PropertyKey {
        int id;
    public:
        PropertyKey(const std::string& key);
        friend class SceneRenderer;
    };

    void setProperty(PropertyKey key, bool value);
    void setProperty(PropertyKey key, int value);
    void setProperty(PropertyKey key, double value);

    template<typename T>
    inline T property(PropertyKey key, T defaultValue) const {
        static_assert(
            std::is_same<T, bool>::value ||
            std::is_same<T, int>::value ||
            std::is_same<T, double>::value,
            "Property type must be bool, int, or double"
        );
        if (key.id >= static_cast<int>(properties_.size())) {
            return defaultValue;
        }
        double val = properties_[key.id];
        if (std::isnan(val)) {
            return defaultValue;
        }
        return static_cast<T>(val);
    }

protected:
    virtual void doRender() = 0;
    virtual bool doPick(int x, int y);
    virtual void onTintColorChanged();

private:
    void setPropertyImpl(PropertyKey key, double value);

    Impl* impl;
    std::vector<double> properties_;
    std::optional<Vector3f> tintColor_;
};

}

#endif
