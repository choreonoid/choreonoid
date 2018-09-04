/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "GLSLSceneRenderer.h"
#include "ShaderPrograms.h"
#include <cnoid/SceneDrawables>
#include <cnoid/SceneCameras>
#include <cnoid/SceneLights>
#include <cnoid/SceneEffects>
#include <cnoid/EigenUtil>
#include <cnoid/NullOut>
#include <Eigen/StdVector>
#include <GL/glu.h>
#include <boost/optional.hpp>
#include <unordered_map>
#include <mutex>
#include <iostream>

using namespace std;
using namespace cnoid;

namespace {

/*
  If the following value is true, the isSolid flag of a mesh is
  reflected in the rendering.
  In the current implementation, this is achieved using glEnable(GL_CULL_FACE).
  However, this implementation does not render the scene correctly when the
  shadow is enabled.
*/
const bool ENABLE_IS_SOLID = false;

const bool USE_FBO_FOR_PICKING = true;
const bool SHOW_IMAGE_FOR_PICKING = false;

const float MinLineWidthForPicking = 5.0f;

typedef vector<Affine3, Eigen::aligned_allocator<Affine3>> Affine3Array;

std::mutex extensionMutex;
set<GLSLSceneRenderer*> renderers;
vector<std::function<void(GLSLSceneRenderer* renderer)>> extendFunctions;

const bool LOCK_VERTEX_ARRAY_API_TO_AVOID_CRASH_ON_NVIDIA_LINUX_OPENGL_DRIVER = true;

std::mutex vertexArrayMutex;

struct LockVertexArrayAPI
{
    LockVertexArrayAPI(){
        if(LOCK_VERTEX_ARRAY_API_TO_AVOID_CRASH_ON_NVIDIA_LINUX_OPENGL_DRIVER){
            vertexArrayMutex.lock();
        }
    }
    ~LockVertexArrayAPI(){
        if(LOCK_VERTEX_ARRAY_API_TO_AVOID_CRASH_ON_NVIDIA_LINUX_OPENGL_DRIVER){
            vertexArrayMutex.unlock();
        }
    }
};
        
class GLResource : public Referenced
{
public:
    virtual void discard() = 0;
};

typedef ref_ptr<GLResource> GLResourcePtr;

class VertexResource : public GLResource
{
public:
    static const int MAX_NUM_BUFFERS = 4;
    GLuint vao;
    GLuint vbos[MAX_NUM_BUFFERS];
    GLsizei numVertices;
    int numBuffers;
    SgObjectPtr sceneObject;
    ScopedConnection connection;
    SgLineSetPtr normalVisualization;

    VertexResource(const VertexResource&) = delete;
    VertexResource& operator=(const VertexResource&) = delete;

    VertexResource(GLSLSceneRendererImpl* renderer, SgObject* obj)
        : sceneObject(obj)
    {
        connection.reset(
            obj->sigUpdated().connect(
                [&](const SgUpdate&){ numVertices = 0; }));

        clearHandles();
        glGenVertexArrays(1, &vao);
    }

    void clearHandles(){
        vao = 0;
        for(int i=0; i < MAX_NUM_BUFFERS; ++i){
            vbos[i] = 0;
        }
        numBuffers = 0;
        numVertices = 0;
    }

    virtual void discard() override { clearHandles(); }

    bool isValid(){
        if(numVertices > 0){
            return true;
        } else if(numBuffers > 0){
            deleteBuffers();
        }
        return false;
    }

    GLuint newBuffer(){
        GLuint buffer;
        glGenBuffers(1, &buffer);
        vbos[numBuffers++] = buffer;
        return buffer;
    }

    void deleteBuffers(){
        if(numBuffers > 0){
            glDeleteBuffers(numBuffers, vbos);
            for(int i=0; i < numBuffers; ++i){
                vbos[i] = 0;
            }
            numBuffers = 0;
        }
    }

    GLuint vbo(int index) {
        return vbos[index];
    }

    ~VertexResource() {
        deleteBuffers();
        if(vao > 0){
            glDeleteVertexArrays(1, &vao);
        }
    }
};

typedef ref_ptr<VertexResource> VertexResourcePtr;

class TextureResource : public GLResource
{
public:
    bool isLoaded;
    bool isImageUpdateNeeded;
    GLuint textureId;
    GLuint samplerId;
    int width;
    int height;
    int numComponents;
        
    TextureResource(){
        isLoaded = false;
        isImageUpdateNeeded = false;
        textureId = 0;
        samplerId = 0;
        width = 0;
        height = 0;
        numComponents = 0;
    }

    ~TextureResource(){
        clear();
    }

    virtual void discard() override { isLoaded = false; }

    void clear() {
        if(isLoaded){
            if(textureId){
                glDeleteTextures(1, &textureId);
                textureId = 0;
            }
            if(samplerId){
                glDeleteSamplers(1, &samplerId);
                samplerId = 0;
            }
            isLoaded = false;
        }
    }
    
    bool isSameSizeAs(const Image& image){
        return (width == image.width() && height == image.height() && numComponents == image.numComponents());
    }
};

typedef ref_ptr<TextureResource> TextureResourcePtr;

struct SgObjectPtrHash {
    std::hash<SgObject*> hash;
    std::size_t operator()(const SgObjectPtr& p) const {
        return hash(p.get());
    }
};

typedef std::unordered_map<SgObjectPtr, GLResourcePtr, SgObjectPtrHash> GLResourceMap;

}

namespace cnoid {

class GLSLSceneRendererImpl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    GLSLSceneRenderer* self;

    PolymorphicFunctionSet<SgNode> renderingFunctions;

    GLuint defaultFBO;
    GLuint fboForPicking;
    GLuint colorBufferForPicking;
    GLuint depthBufferForPicking;
    int viewportWidth;
    int viewportHeight;
    bool needToChangeBufferSizeForPicking;

    ShaderProgram* currentProgram;
    LightingProgram* currentLightingProgram;
    MaterialProgram* materialProgram;
    NolightingProgram* currentNolightingProgram;
    
    SolidColorProgram solidColorProgram;
    PhongShadowProgram phongShadowProgram;

    struct ProgramInfo {
        ShaderProgram* program;
        LightingProgram* lightingProgram;
        NolightingProgram* nolightingProgram;
    };
    vector<ProgramInfo> programStack;

    bool isActuallyRendering;
    bool isPicking;
    bool isRenderingShadowMap;
    
    Affine3Array modelMatrixStack; // stack of the model matrices
    Affine3 viewMatrix;
    Matrix4 projectionMatrix;
    Matrix4 PV;

    vector<function<void()>> postRenderingFunctions;
    vector<function<void()>> transparentRenderingFunctions;

    std::set<int> shadowLightIndices;

    bool defaultLighting;
    Vector3f diffuseColor;
    Vector3f ambientColor;
    Vector3f specularColor;
    Vector3f emissionColor;
    float shininess;
    float alpha;

    SgMaterialPtr defaultMaterial;
    GLfloat defaultPointSize;
    GLfloat defaultLineWidth;
    GLResourceMap resourceMaps[2];
    bool doUnusedResourceCheck;
    bool isCheckingUnusedResources;
    bool hasValidNextResourceMap;
    bool isResourceClearRequested;
    int currentResourceMapIndex;
    GLResourceMap* currentResourceMap;
    GLResourceMap* nextResourceMap;

    vector<char> scaledImageBuf;
    Eigen::Affine2f textureTransform;
    bool hasValidTextureTransform;

    bool isCurrentFogUpdated;
    SgFogPtr prevFog;
    ScopedConnection currentFogConnection;

    bool defaultSmoothShading;
    bool isNormalVisualizationEnabled;
    float normalVisualizationLength;
    SgMaterialPtr normalVisualizationMaterial;

    // OpenGL states
    enum StateFlag {
        COLOR_MATERIAL,
        DIFFUSE_COLOR,
        AMBIENT_COLOR,
        EMISSION_COLOR,
        SPECULAR_COLOR,
        SHININESS,
        ALPHA,
        POINT_SIZE,
        LINE_WIDTH,
        NUM_STATE_FLAGS
    };

    vector<bool> stateFlag;

    boost::optional<bool> isCullFaceEnabled;

    float pointSize;
    float lineWidth;

    GLdouble pickX;
    GLdouble pickY;
    typedef std::shared_ptr<SgNodePath> SgNodePathPtr;
    SgNodePath currentNodePath;
    vector<SgNodePathPtr> pickingNodePathList;
    SgNodePath pickedNodePath;
    Vector3 pickedPoint;

    ostream* os_;
    ostream& os() { return *os_; }

    bool isUpsideDownEnabled;

    std::mutex newExtensionMutex;
    vector<std::function<void(GLSLSceneRenderer* renderer)>> newExtendFunctions;

    void renderChildNodes(SgGroup* group){
        for(auto p = group->cbegin(); p != group->cend(); ++p){
            renderingFunctions.dispatch(*p);
        }
    }
    
    GLSLSceneRendererImpl(GLSLSceneRenderer* self);
    ~GLSLSceneRendererImpl();
    void initialize();
    void onExtensionAdded(std::function<void(GLSLSceneRenderer* renderer)> func);
    bool initializeGL();
    void doRender();
    bool doPick(int x, int y);
    void renderScene();
    bool renderShadowMap(int lightIndex);
    void beginRendering();
    void renderCamera(SgCamera* camera, const Affine3& cameraPosition);
    void renderLights(LightingProgram* program);
    void renderFog(LightingProgram* program);
    void endRendering();
    void renderSceneGraphNodes();
    void pushProgram(ShaderProgram& program, bool isLightingProgram);
    void popProgram();
    inline void setPickColor(unsigned int id);
    inline unsigned int pushPickId(SgNode* node, bool doSetColor = true);
    void popPickId();
    void renderGroup(SgGroup* group);
    void renderTransform(SgTransform* transform);
    void renderSwitch(SgSwitch* node);
    void renderUnpickableGroup(SgUnpickableGroup* group);
    void renderShape(SgShape* shape);
    void renderShapeMain(SgShape* shape, VertexResource* resource, const Affine3& position, unsigned int pickId);
    void renderPointSet(SgPointSet* pointSet);        
    void renderLineSet(SgLineSet* lineSet);        
    void renderOverlay(SgOverlay* overlay);
    void renderOutlineGroup(SgOutlineGroup* outline);
    void renderOutlineGroupMain(SgOutlineGroup* outline, const Affine3& T);
    void flushNolightingTransformMatrices();
    VertexResource* getOrCreateVertexResource(SgObject* obj);
    void drawVertexResource(VertexResource* resource, GLenum primitiveMode, const Affine3& position);
    void renderTransparentObjects();
    void renderMaterial(const SgMaterial* material);
    bool renderTexture(SgTexture* texture);
    bool loadTextureImage(TextureResource* resource, const Image& image);
    void writeMeshVertices(SgMesh* mesh, VertexResource* resource);
    void writeMeshNormals(SgMesh* mesh, GLuint buffer, SgNormalArray& normals);
    void writeMeshTexCoords(SgMesh* mesh, GLuint buffer);
    void writeMeshColors(SgMesh* mesh, GLuint buffer);
    void renderPlot(SgPlot* plot, GLenum primitiveMode, std::function<SgVertexArrayPtr()> getVertices);
    void clearGLState();
    void setDiffuseColor(const Vector3f& color);
    void setAmbientColor(const Vector3f& color);
    void setEmissionColor(const Vector3f& color);
    void setSpecularColor(const Vector3f& color);
    void setShininess(float shininess);
    void setAlpha(float a);
    void setPointSize(float size);
    void setLineWidth(float width);
    void getCurrentCameraTransform(Affine3& T);
};

}


GLSLSceneRenderer::GLSLSceneRenderer()
{
    impl = new GLSLSceneRendererImpl(this);
    impl->initialize();
}


GLSLSceneRenderer::GLSLSceneRenderer(SgGroup* sceneRoot)
    : GLSceneRenderer(sceneRoot)
{
    impl = new GLSLSceneRendererImpl(this);
    impl->initialize();
}


GLSLSceneRendererImpl::GLSLSceneRendererImpl(GLSLSceneRenderer* self)
    : self(self)
{

}


void GLSLSceneRendererImpl::initialize()
{
    {
        std::lock_guard<std::mutex> guard(extensionMutex);
        renderers.insert(self);
    }
    
    defaultFBO = 0;
    fboForPicking = 0;
    colorBufferForPicking = 0;
    depthBufferForPicking = 0;
    viewportWidth = 1;
    viewportHeight = 1;
    needToChangeBufferSizeForPicking = true;

    currentProgram = 0;
    currentLightingProgram = 0;
    currentNolightingProgram = 0;
    materialProgram = &phongShadowProgram;

    isActuallyRendering = false;
    isPicking = false;
    isRenderingShadowMap = false;
    pickedPoint.setZero();

    doUnusedResourceCheck = true;
    currentResourceMapIndex = 0;
    hasValidNextResourceMap = false;
    isResourceClearRequested = false;
    currentResourceMap = &resourceMaps[0];
    nextResourceMap = &resourceMaps[1];

    modelMatrixStack.reserve(16);
    viewMatrix.setIdentity();
    projectionMatrix.setIdentity();

    defaultLighting = true;
    defaultSmoothShading = true;
    defaultMaterial = new SgMaterial;
    defaultMaterial->setDiffuseColor(Vector3f(0.8, 0.8, 0.8));
    defaultPointSize = 1.0f;
    defaultLineWidth = 1.0f;

    hasValidTextureTransform = false;

    prevFog = 0;

    isNormalVisualizationEnabled = false;
    normalVisualizationLength = 0.0f;
    normalVisualizationMaterial = new SgMaterial;
    normalVisualizationMaterial->setDiffuseColor(Vector3f(0.0f, 1.0f, 0.0f));

    isUpsideDownEnabled = false;

    stateFlag.resize(NUM_STATE_FLAGS, false);
    clearGLState();

    os_ = &nullout();

    renderingFunctions.setFunction<SgGroup>(
        [&](SgGroup* node){ renderGroup(node); });
    renderingFunctions.setFunction<SgTransform>(
        [&](SgTransform* node){ renderTransform(node); });
    renderingFunctions.setFunction<SgSwitch>(
        [&](SgSwitch* node){ renderSwitch(node); });
    renderingFunctions.setFunction<SgUnpickableGroup>(
        [&](SgUnpickableGroup* node){ renderUnpickableGroup(node); });
    renderingFunctions.setFunction<SgShape>(
        [&](SgShape* node){ renderShape(node); });
    renderingFunctions.setFunction<SgPointSet>(
        [&](SgPointSet* node){ renderPointSet(node); });
    renderingFunctions.setFunction<SgLineSet>(
        [&](SgLineSet* node){ renderLineSet(node); });
    renderingFunctions.setFunction<SgOverlay>(
        [&](SgOverlay* node){ renderOverlay(node); });
    renderingFunctions.setFunction<SgOutlineGroup>(
        [&](SgOutlineGroup* node){ renderOutlineGroup(node); });

    self->applyExtensions();
    renderingFunctions.updateDispatchTable();
}


GLSLSceneRenderer::~GLSLSceneRenderer()
{
    std::lock_guard<std::mutex> guard(extensionMutex);
    renderers.erase(this);
    
    delete impl;
}


GLSLSceneRendererImpl::~GLSLSceneRendererImpl()
{
    // clear handles to avoid the deletion of them without the corresponding GL context
    for(int i=0; i < 2; ++i){
        GLResourceMap& resourceMap = resourceMaps[i];
        for(GLResourceMap::iterator p = resourceMap.begin(); p != resourceMap.end(); ++p){
            GLResource* resource = p->second;
            resource->discard();
        }
    }

    if(fboForPicking){
        glDeleteRenderbuffers(1, &colorBufferForPicking);
        glDeleteRenderbuffers(1, &depthBufferForPicking);
        glDeleteFramebuffers(1, &fboForPicking);
    }
}


void GLSLSceneRenderer::addExtension(std::function<void(GLSLSceneRenderer* renderer)> func)
{
    {
        std::lock_guard<std::mutex> guard(extensionMutex);
        extendFunctions.push_back(func);
    }
    for(GLSLSceneRenderer* renderer : renderers){
        renderer->impl->onExtensionAdded(func);
    }
}


void GLSLSceneRenderer::applyExtensions()
{
    SceneRenderer::applyExtensions();
    
    std::lock_guard<std::mutex> guard(extensionMutex);
    for(size_t i=0; i < extendFunctions.size(); ++i){
        extendFunctions[i](this);
    }
}


void GLSLSceneRendererImpl::onExtensionAdded(std::function<void(GLSLSceneRenderer* renderer)> func)
{
    std::lock_guard<std::mutex> guard(newExtensionMutex);
    newExtendFunctions.push_back(func);
}


bool GLSLSceneRenderer::applyNewExtensions()
{
    bool applied = SceneRenderer::applyNewExtensions();
    
    std::lock_guard<std::mutex> guard(impl->newExtensionMutex);
    if(!impl->newExtendFunctions.empty()){
        for(size_t i=0; i < impl->newExtendFunctions.size(); ++i){
            impl->newExtendFunctions[i](this);
        }
        impl->newExtendFunctions.clear();
        applied = true;
    }

    return applied;
}


SceneRenderer::NodeFunctionSet* GLSLSceneRenderer::renderingFunctions()
{
    return &impl->renderingFunctions;
}


void GLSLSceneRenderer::setOutputStream(std::ostream& os)
{
    impl->os_ = &os;
}


bool GLSLSceneRenderer::initializeGL()
{
    GLSceneRenderer::initializeGL();
    return impl->initializeGL();
}


bool GLSLSceneRendererImpl::initializeGL()
{
    if(ogl_LoadFunctions() == ogl_LOAD_FAILED){
        return false;
    }

    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, reinterpret_cast<GLint*>(&defaultFBO));

    try {
        solidColorProgram.initialize();
        phongShadowProgram.initialize();
    }
    catch(GLSLProgram::Exception& ex){
        os() << ex.what() << endl;
        cout << ex.what() << endl;
        return false;
    }

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_DITHER);

    if(ENABLE_IS_SOLID){
        glEnable(GL_CULL_FACE);
        isCullFaceEnabled = true;
    } else {
        glDisable(GL_CULL_FACE);
    }

    glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

    isResourceClearRequested = true;

    isCurrentFogUpdated = false;

    return true;
}


void GLSLSceneRenderer::flush()
{
    glFlush();

    /**
       This is necessary when the rendering is done for an internal frame buffer object
       and the rendererd image data is retrieved from it because another frame buffer object
       may be bounded in the renderer.
    */
    glBindFramebuffer(GL_FRAMEBUFFER, impl->defaultFBO);
}


void GLSLSceneRenderer::setViewport(int x, int y, int width, int height)
{
    GLSceneRenderer::setViewport(x, y, width, height);
    impl->viewportWidth = width;
    impl->viewportHeight = height;
    impl->needToChangeBufferSizeForPicking = true;
}


void GLSLSceneRenderer::requestToClearResources()
{
    impl->isResourceClearRequested = true;
}


void GLSLSceneRenderer::doRender()
{
    impl->doRender();
}


void GLSLSceneRendererImpl::doRender()
{
    if(self->applyNewExtensions()){
        renderingFunctions.updateDispatchTable();
    }

    self->extractPreprocessedNodes();
    beginRendering();

    PhongShadowProgram& program = phongShadowProgram;
    
    if(shadowLightIndices.empty()){
        program.setNumShadows(0);
        
    } else {
        Array4i vp = self->viewport();
        self->setViewport(0, 0, program.shadowMapWidth(), program.shadowMapHeight());
        pushProgram(program.shadowMapProgram(), false);
        isRenderingShadowMap = true;
        isActuallyRendering = false;
        
        int shadowMapIndex = 0;
        set<int>::iterator iter = shadowLightIndices.begin();
        while(iter != shadowLightIndices.end() && shadowMapIndex < program.maxNumShadows()){
            program.activateShadowMapGenerationPass(shadowMapIndex);
            int shadowLightIndex = *iter;
            if(renderShadowMap(shadowLightIndex)){
                ++shadowMapIndex;
            }
            ++iter;
        }
        program.setNumShadows(shadowMapIndex);
        
        popProgram();
        isRenderingShadowMap = false;
        self->setViewport(vp[0], vp[1], vp[2], vp[3]);
    }
    
    program.activateMainRenderingPass();
    pushProgram(program, true);
    isActuallyRendering = true;
    const Vector3f& c = self->backgroundColor();
    glClearColor(c[0], c[1], c[2], 1.0f);

    switch(self->polygonMode()){
    case GLSceneRenderer::FILL_MODE:
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        break;
    case GLSceneRenderer::LINE_MODE:
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        break;
    case GLSceneRenderer::POINT_MODE:
        glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
        break;
    }
    
    renderScene();

    popProgram();
    endRendering();
}


bool GLSLSceneRenderer::doPick(int x, int y)
{
    return impl->doPick(x, y);
}


bool GLSLSceneRendererImpl::doPick(int x, int y)
{
    if(USE_FBO_FOR_PICKING){
        if(!fboForPicking){
            glGenFramebuffers(1, &fboForPicking);
            needToChangeBufferSizeForPicking = true;
        }
        glBindFramebuffer(GL_FRAMEBUFFER, fboForPicking);

        if(needToChangeBufferSizeForPicking){
            // color buffer
            if(colorBufferForPicking){
                glDeleteRenderbuffers(1, &colorBufferForPicking);
            }
            glGenRenderbuffers(1, &colorBufferForPicking);
            glBindRenderbuffer(GL_RENDERBUFFER, colorBufferForPicking);
            glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA, viewportWidth, viewportHeight);
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, colorBufferForPicking);
            
            // depth buffer
            if(depthBufferForPicking){
                glDeleteRenderbuffers(1, &depthBufferForPicking);
            }
            glGenRenderbuffers(1, &depthBufferForPicking);
            glBindRenderbuffer(GL_RENDERBUFFER, depthBufferForPicking);
            glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, viewportWidth, viewportHeight);
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthBufferForPicking);
            
            needToChangeBufferSizeForPicking = false;
        }
    }
    
    self->extractPreprocessedNodes();

    GLboolean isMultiSampleEnabled;
    if(!USE_FBO_FOR_PICKING){
        isMultiSampleEnabled = glIsEnabled(GL_MULTISAMPLE);
        if(isMultiSampleEnabled){
            glDisable(GL_MULTISAMPLE);
        }
    }
    
    if(!SHOW_IMAGE_FOR_PICKING){
        glScissor(x, y, 1, 1);
        glEnable(GL_SCISSOR_TEST);
    }

    isPicking = true;
    isActuallyRendering = false;
    beginRendering();
    pushProgram(solidColorProgram, false);
    currentNodePath.clear();
    pickingNodePathList.clear();

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    renderScene();
    
    popProgram();
    isPicking = false;

    glDisable(GL_SCISSOR_TEST);

    endRendering();

    if(!USE_FBO_FOR_PICKING){
        if(isMultiSampleEnabled){
            glEnable(GL_MULTISAMPLE);
        }
        glBindFramebuffer(GL_READ_FRAMEBUFFER, fboForPicking);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
    }
    
    GLfloat color[4];
    glReadPixels(x, y, 1, 1, GL_RGBA, GL_FLOAT, color);
    if(SHOW_IMAGE_FOR_PICKING){
        color[2] = 0.0f;
    }
    unsigned int id = (color[0] * 255) + ((int)(color[1] * 255) << 8) + ((int)(color[2] * 255) << 16) - 1;

    pickedNodePath.clear();

    if(0 < id && id < pickingNodePathList.size()){
        GLfloat depth;
        glReadPixels(x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
        Vector3 projected;
        if(self->unproject(x, y, depth, pickedPoint)){
            pickedNodePath = *pickingNodePathList[id];
        }
    }

    if(USE_FBO_FOR_PICKING){
        glBindFramebuffer(GL_FRAMEBUFFER, defaultFBO);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, defaultFBO);
    }

    return !pickedNodePath.empty();
}


void GLSLSceneRendererImpl::renderScene()
{
    SgCamera* camera = self->currentCamera();
    if(camera){
        renderCamera(camera, self->currentCameraPosition());

        postRenderingFunctions.clear();
        transparentRenderingFunctions.clear();

        renderSceneGraphNodes();

        for(auto&& func : postRenderingFunctions){
            func();
        }
        postRenderingFunctions.clear();
        
        if(!transparentRenderingFunctions.empty()){
            renderTransparentObjects();
        }
    }
}


bool GLSLSceneRendererImpl::renderShadowMap(int lightIndex)
{
    SgLight* light;
    Affine3 T;
    self->getLightInfo(lightIndex, light, T);
    if(light && light->on()){
        SgCamera* shadowMapCamera = phongShadowProgram.getShadowMapCamera(light, T);
        if(shadowMapCamera){
            renderCamera(shadowMapCamera, T);
            phongShadowProgram.setShadowMapViewProjection(PV);
            renderSceneGraphNodes();
            glFlush();
            glFinish();
            return true;
        }
    }
    return false;
}
    

void GLSLSceneRendererImpl::renderCamera(SgCamera* camera, const Affine3& cameraPosition)
{
    if(SgPerspectiveCamera* pers = dynamic_cast<SgPerspectiveCamera*>(camera)){
        double aspectRatio = self->aspectRatio();
        self->getPerspectiveProjectionMatrix(
            pers->fovy(aspectRatio), aspectRatio, pers->nearClipDistance(), pers->farClipDistance(),
            projectionMatrix);
        
    } else if(SgOrthographicCamera* ortho = dynamic_cast<SgOrthographicCamera*>(camera)){
        GLfloat left, right, bottom, top;
        self->getViewVolume(ortho, left, right, bottom, top);
        self->getOrthographicProjectionMatrix(
            left, right, bottom, top, ortho->nearClipDistance(), ortho->farClipDistance(),
            projectionMatrix);
        
    } else {
        self->getPerspectiveProjectionMatrix(
            radian(40.0), self->aspectRatio(), 0.01, 1.0e4,
            projectionMatrix);
    }

    if(isUpsideDownEnabled){
        Affine3 T = cameraPosition * AngleAxis(PI, Vector3(0.0, 0.0, 1.0));
        viewMatrix = T.inverse(Eigen::Isometry);
    } else {
        viewMatrix = cameraPosition.inverse(Eigen::Isometry);
    }
    PV = projectionMatrix * viewMatrix.matrix();

    modelMatrixStack.clear();
    modelMatrixStack.push_back(Affine3::Identity());
}


void GLSLSceneRendererImpl::beginRendering()
{
    isCheckingUnusedResources = isPicking ? false : doUnusedResourceCheck;

    if(isResourceClearRequested){
        resourceMaps[0].clear();
        resourceMaps[1].clear();
        hasValidNextResourceMap = false;
        isCheckingUnusedResources = false;
        isResourceClearRequested = false; 
    }
    if(hasValidNextResourceMap){
        currentResourceMapIndex = 1 - currentResourceMapIndex;
        currentResourceMap = &resourceMaps[currentResourceMapIndex];
        nextResourceMap = &resourceMaps[1 - currentResourceMapIndex];
        hasValidNextResourceMap = false;
    }
}


void GLSLSceneRendererImpl::endRendering()
{
    if(isCheckingUnusedResources){
        currentResourceMap->clear();
        hasValidNextResourceMap = true;
    }
}


void GLSLSceneRendererImpl::renderSceneGraphNodes()
{
    currentProgram->initializeFrameRendering();
    clearGLState();

    if(currentLightingProgram){
        renderLights(currentLightingProgram);
        renderFog(currentLightingProgram);
    }

    renderingFunctions.dispatch(self->sceneRoot());
}


void GLSLSceneRenderer::renderLights(LightingProgram* program)
{
    impl->renderLights(program);
}


void GLSLSceneRendererImpl::renderLights(LightingProgram* program)
{
    int lightIndex = 0;

    const int n = self->numLights();
    for(int i=0; i < n; ++i){
        if(lightIndex == program->maxNumLights()){
            break;
        }
        SgLight* light;
        Affine3 T;
        self->getLightInfo(i, light, T);
        if(light->on()){
            bool isCastingShadow = (shadowLightIndices.find(i) != shadowLightIndices.end());
            if(program->renderLight(lightIndex, light, T, viewMatrix, isCastingShadow)){
                ++lightIndex;
            }
        }
    }

    if(lightIndex < program->maxNumLights()){
        SgLight* headLight = self->headLight();
        if(headLight->on()){
            if(program->renderLight(
                   lightIndex, headLight, self->currentCameraPosition(), viewMatrix, false)){
                ++lightIndex;
            }
        }
    }

    program->setNumLights(lightIndex);
}


void GLSLSceneRenderer::renderFog(LightingProgram* program)
{
    impl->renderFog(program);
}


void GLSLSceneRendererImpl::renderFog(LightingProgram* program)
{
    SgFog* fog = 0;
    if(self->isFogEnabled()){
        int n = self->numFogs();
        if(n > 0){
            fog = self->fog(n - 1); // use the last fog
        }
    }
    if(fog != prevFog){
        isCurrentFogUpdated = true;
        if(!fog){
            currentFogConnection.disconnect();
        } else {
            currentFogConnection.reset(
                fog->sigUpdated().connect(
                    [&](const SgUpdate&){
                        if(!self->isFogEnabled()){
                            currentFogConnection.disconnect();
                        }
                        isCurrentFogUpdated = true;
                    }));
        }
    }

    if(isCurrentFogUpdated){
        if(!fog){
            currentLightingProgram->setFogEnabled(false);
        } else {
            currentLightingProgram->setFogEnabled(true);
            currentLightingProgram->setFogColor(fog->color());
            currentLightingProgram->setFogRange(0.0f, fog->visibilityRange());
        }
    }
    isCurrentFogUpdated = false;
    prevFog = fog;
}


const Affine3& GLSLSceneRenderer::currentModelTransform() const
{
    return impl->modelMatrixStack.back();
}


const Matrix4& GLSLSceneRenderer::projectionMatrix() const
{
    return impl->projectionMatrix;
}


const Matrix4& GLSLSceneRenderer::viewProjectionMatrix() const
{
    return impl->PV;
}


Matrix4 GLSLSceneRenderer::modelViewMatrix() const
{
    return impl->viewMatrix * impl->modelMatrixStack.back().matrix();
}


Matrix4 GLSLSceneRenderer::modelViewProjectionMatrix() const
{
    return impl->PV * impl->modelMatrixStack.back().matrix();
}


bool GLSLSceneRenderer::isPicking() const
{
    return impl->isPicking;
}


void GLSLSceneRendererImpl::pushProgram(ShaderProgram& program, bool isLightingProgram)
{
    ProgramInfo info;
    info.program = currentProgram;
    info.lightingProgram = currentLightingProgram;
    info.nolightingProgram = currentNolightingProgram;
    
    if(&program != currentProgram){
        if(currentProgram){
            currentProgram->deactivate();
        }
        currentProgram = &program;
        if(isLightingProgram){
            currentLightingProgram = static_cast<LightingProgram*>(currentProgram);
            currentNolightingProgram = 0;
        } else {
            currentLightingProgram = 0;
            currentNolightingProgram = static_cast<NolightingProgram*>(currentProgram);
        }
        program.activate();
    }
    programStack.push_back(info);
}


void GLSLSceneRenderer::pushShaderProgram(ShaderProgram& program, bool isLightingProgram)
{
    impl->pushProgram(program, isLightingProgram);
}


void GLSLSceneRendererImpl::popProgram()
{
    ProgramInfo& info = programStack.back();
    if(info.program != currentProgram){
        if(currentProgram){
            currentProgram->deactivate();
        }
        currentProgram = info.program;
        currentLightingProgram = info.lightingProgram;
        currentNolightingProgram = info.nolightingProgram;
        if(currentProgram){
            currentProgram->activate();
        }
    }
    programStack.pop_back();
}


void GLSLSceneRenderer::popShaderProgram()
{
    impl->popProgram();
}


const std::vector<SgNode*>& GLSLSceneRenderer::pickedNodePath() const
{
    return impl->pickedNodePath;
}


const Vector3& GLSLSceneRenderer::pickedPoint() const
{
    return impl->pickedPoint;
}


inline void GLSLSceneRendererImpl::setPickColor(unsigned int id)
{
    Vector3f color;
    color[0] = (id & 0xff) / 255.0;
    color[1] = ((id >> 8) & 0xff) / 255.0;
    color[2] = ((id >> 16) & 0xff) / 255.0;
    if(SHOW_IMAGE_FOR_PICKING){
        color[2] = 1.0f;
    }
    solidColorProgram.setColor(color);
}
        

/**
   @return id of the current object
*/
inline unsigned int GLSLSceneRendererImpl::pushPickId(SgNode* node, bool doSetColor)
{
    unsigned int id = 0;
    
    if(isPicking){
        id = pickingNodePathList.size() + 1;
        currentNodePath.push_back(node);
        pickingNodePathList.push_back(std::make_shared<SgNodePath>(currentNodePath));
        if(doSetColor){
            setPickColor(id);
        }
    }

    return id;
}


inline void GLSLSceneRendererImpl::popPickId()
{
    if(isPicking){
        currentNodePath.pop_back();
    }
}


void GLSLSceneRenderer::renderNode(SgNode* node)
{
    impl->renderingFunctions.dispatch(node);
}


void GLSLSceneRendererImpl::renderGroup(SgGroup* group)
{
    pushPickId(group);
    renderChildNodes(group);
    popPickId();
}


void GLSLSceneRenderer::renderCustomGroup(SgGroup* group, std::function<void()> traverseFunction)
{
    impl->pushPickId(group);
    traverseFunction();
    impl->popPickId();
}


void GLSLSceneRendererImpl::renderSwitch(SgSwitch* node)
{
    if(node->isTurnedOn()){
        renderGroup(node);
    }
}


void GLSLSceneRendererImpl::renderUnpickableGroup(SgUnpickableGroup* group)
{
    if(!isPicking){
        renderGroup(group);
    }
}


void GLSLSceneRendererImpl::renderTransform(SgTransform* transform)
{
    Affine3 T;
    transform->getTransform(T);
    modelMatrixStack.push_back(modelMatrixStack.back() * T);
    pushPickId(transform);

    renderChildNodes(transform);

    popPickId();
    modelMatrixStack.pop_back();
}


void GLSLSceneRenderer::renderCustomTransform(SgTransform* transform, std::function<void()> traverseFunction)
{
    Affine3 T;
    transform->getTransform(T);
    impl->modelMatrixStack.push_back(impl->modelMatrixStack.back() * T);
    impl->pushPickId(transform);

    traverseFunction();

    impl->popPickId();
    impl->modelMatrixStack.pop_back();
}    
    

VertexResource* GLSLSceneRendererImpl::getOrCreateVertexResource(SgObject* obj)
{
    VertexResource* resource;
    auto p = currentResourceMap->find(obj);
    if(p == currentResourceMap->end()){
        resource = new VertexResource(this, obj);
        p = currentResourceMap->insert(GLResourceMap::value_type(obj, resource)).first;
    } else {
        resource = static_cast<VertexResource*>(p->second.get());
    }

    if(isCheckingUnusedResources){
        nextResourceMap->insert(*p);
    }

    return resource;
}


void GLSLSceneRendererImpl::drawVertexResource(VertexResource* resource, GLenum primitiveMode, const Affine3& position)
{
    if(currentLightingProgram == &phongShadowProgram){
        phongShadowProgram.setTransformMatrices(viewMatrix, position, PV);
    } else if(currentNolightingProgram){
        const Matrix4f PVM = (PV * position.matrix()).cast<float>();
        currentNolightingProgram->setProjectionMatrix(PVM);
    }
    glBindVertexArray(resource->vao);
    glDrawArrays(primitiveMode, 0, resource->numVertices);
}


void GLSLSceneRendererImpl::renderShape(SgShape* shape)
{
    SgMesh* mesh = shape->mesh();
    if(mesh && mesh->hasVertices()){

        VertexResource* resource = getOrCreateVertexResource(mesh);
        if(!resource->isValid()){
            writeMeshVertices(mesh, resource);
        }
        
        SgMaterial* material = shape->material();
        if(material && material->transparency() > 0.0){
            if(!isRenderingShadowMap){
                const Affine3& position = modelMatrixStack.back();
                unsigned int pickId = pushPickId(shape, false);
                transparentRenderingFunctions.push_back(
                    [this, shape, resource, position, pickId](){
                        renderShapeMain(shape, resource, position, pickId); });
                popPickId();
            }
        } else {
            int pickId = pushPickId(shape, false);
            renderShapeMain(shape, resource, modelMatrixStack.back(), pickId);
            popPickId();
        }

        if(isNormalVisualizationEnabled && isActuallyRendering && resource->normalVisualization){
            renderLineSet(resource->normalVisualization);
        }
    }
}


void GLSLSceneRendererImpl::renderShapeMain
(SgShape* shape, VertexResource* resource, const Affine3& position, unsigned int pickId)
{
    SgMesh* mesh = shape->mesh();
    if(isPicking){
        setPickColor(pickId);
    } else {
        renderMaterial(shape->material());
        if(currentLightingProgram == &phongShadowProgram){
            bool hasTexture;
            if(shape->texture() && mesh->hasTexCoords()){
                hasTexture = renderTexture(shape->texture());
            } else {
                hasTexture = false;
            }
            phongShadowProgram.setTextureEnabled(hasTexture);
            phongShadowProgram.setVertexColorEnabled(mesh->hasColors());
        }
    }
    if(ENABLE_IS_SOLID){
        bool doCullFace = mesh->isSolid();
        if(!isCullFaceEnabled || *isCullFaceEnabled != doCullFace){
            if(doCullFace){
                glEnable(GL_CULL_FACE);
            } else {
                glDisable(GL_CULL_FACE);
            }
            isCullFaceEnabled = doCullFace;
        }
    }
    drawVertexResource(resource, GL_TRIANGLES, position);
}


void GLSLSceneRenderer::dispatchToTransparentPhase(std::function<void()> renderingFunction)
{
    impl->transparentRenderingFunctions.push_back(renderingFunction);
}


void GLSLSceneRendererImpl::renderTransparentObjects()
{
    if(!isPicking){
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
    }

    const int n = transparentRenderingFunctions.size();
    for(int i=0; i < n; ++i){
        transparentRenderingFunctions[i]();
    }

    if(!isPicking){
        glDisable(GL_BLEND);
        glDepthMask(GL_TRUE);
    }

    transparentRenderingFunctions.clear();
}


void GLSLSceneRendererImpl::renderMaterial(const SgMaterial* material)
{
    if(!material){
        material = defaultMaterial;
    }

    if(currentLightingProgram == materialProgram){
        setDiffuseColor(material->diffuseColor());
        setAmbientColor(material->ambientIntensity() * material->diffuseColor());
        setEmissionColor(material->emissiveColor());
        setSpecularColor(material->specularColor());
        setShininess((127.0f * material->shininess()) + 1.0f);
        setAlpha(1.0 - material->transparency());

    } else if(currentNolightingProgram){
        currentProgram->setColor(material->diffuseColor() + material->emissiveColor());
    }
}


bool GLSLSceneRendererImpl::renderTexture(SgTexture* texture)
{
    SgImage* sgImage = texture->image();
    if(!sgImage || sgImage->empty()){
        return false;
    }

    auto p = currentResourceMap->find(sgImage);
    TextureResource* resource;
    if(p != currentResourceMap->end()){
        resource = static_cast<TextureResource*>(p->second.get());
    } else {
        resource = new TextureResource;
        currentResourceMap->insert(GLResourceMap::value_type(sgImage, resource));
    }

    glActiveTexture(GL_TEXTURE0);
    if(resource->isLoaded){
        glBindTexture(GL_TEXTURE_2D, resource->textureId);
        glBindSampler(0, resource->samplerId);
        if(resource->isImageUpdateNeeded){
            loadTextureImage(resource, sgImage->constImage());
        }
    } else {
        GLuint samplerId;
        glGenTextures(1, &resource->textureId);
        glBindTexture(GL_TEXTURE_2D, resource->textureId);
        if(loadTextureImage(resource, sgImage->constImage())){
            glGenSamplers(1, &samplerId);
            glBindSampler(0, samplerId);
            glSamplerParameteri(samplerId, GL_TEXTURE_WRAP_S, texture->repeatS() ? GL_REPEAT : GL_CLAMP_TO_EDGE);
            glSamplerParameteri(samplerId, GL_TEXTURE_WRAP_T, texture->repeatT() ? GL_REPEAT : GL_CLAMP_TO_EDGE);
            glSamplerParameteri(samplerId, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glSamplerParameteri(samplerId, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            resource->samplerId = samplerId;
        }
    }
    
    if(isCheckingUnusedResources){
        nextResourceMap->insert(GLResourceMap::value_type(sgImage, resource)); 
    }

    auto tt = texture->textureTransform();
    if(!tt){
        hasValidTextureTransform = false;
    } else {
        Eigen::Rotation2Df R(tt->rotation());
        const auto& c = tt->center();
        Eigen::Translation<float, 2> C(c.x(), c.y());
        const auto& t = tt->translation();
        Eigen::Translation<float, 2> T(t.x(), t.y());
        const auto s = tt->scale().cast<float>();
        textureTransform = Eigen::Affine2f(C.inverse() * Eigen::Scaling(s.x(), s.y()) * R * C * T);
        hasValidTextureTransform = true;
    }

    return resource->isLoaded;
}


bool GLSLSceneRendererImpl::loadTextureImage(TextureResource* resource, const Image& image)
{
    GLenum format = GL_RGB;
    switch(image.numComponents()){
    case 1 : format = GL_RED; break;
    case 2 : format = GL_RG; break;
    case 3 : format = GL_RGB; break;
    case 4 : format = GL_RGBA; break;
    default:
        resource->clear();
        return false;
    }
    
    if(image.numComponents() == 3){
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    } else {
        glPixelStorei(GL_UNPACK_ALIGNMENT, image.numComponents());
    }
    resource->numComponents = image.numComponents();

    const int width = image.width();
    const int height = image.height();

    if(resource->isLoaded && resource->isSameSizeAs(image)){
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, format, GL_UNSIGNED_BYTE, image.pixels());

    } else {
        double w2 = log2(width);
        double h2 = log2(height);
        double pw = ceil(w2);
        double ph = ceil(h2);
        if((pw - w2 == 0.0) && (ph - h2 == 0.0)){
            glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, image.pixels());
        } else{
            GLsizei potWidth = pow(2.0, pw);
            GLsizei potHeight = pow(2.0, ph);
            scaledImageBuf.resize(potWidth * potHeight * image.numComponents());
            gluScaleImage(format, width, height, GL_UNSIGNED_BYTE, image.pixels(),
                          potWidth, potHeight, GL_UNSIGNED_BYTE, &scaledImageBuf.front());
            glTexImage2D(GL_TEXTURE_2D, 0, format, potWidth, potHeight, 0, format, GL_UNSIGNED_BYTE, &scaledImageBuf.front());
        }
        resource->isLoaded = true;
        resource->width = width;
        resource->height = height;
    }
    glGenerateMipmap(GL_TEXTURE_2D);

    resource->isImageUpdateNeeded = false;

    return true;
}


void GLSLSceneRenderer::onImageUpdated(SgImage* image)
{
    GLResourceMap* resourceMap = impl->hasValidNextResourceMap ? impl->nextResourceMap : impl->currentResourceMap;
    auto p = resourceMap->find(image);
    if(p != resourceMap->end()){
        TextureResource* resource = static_cast<TextureResource*>(p->second.get());
        resource->isImageUpdateNeeded = true;
    }
}


void GLSLSceneRendererImpl::writeMeshVertices(SgMesh* mesh, VertexResource* resource)
{
    auto& triangleVertices = mesh->triangleVertices();
    const int totalNumVertices = triangleVertices.size();
    
    const auto& orgVertices = *mesh->vertices();
    SgVertexArray vertices;
    vertices.reserve(totalNumVertices);
    resource->numVertices = totalNumVertices;

    const int numTriangles = mesh->numTriangles();
    int faceVertexIndex = 0;
    
    for(int i=0; i < numTriangles; ++i){
        for(int j=0; j < 3; ++j){
            const int orgVertexIndex = triangleVertices[faceVertexIndex++];
            vertices.push_back(orgVertices[orgVertexIndex]);
        }
    }

    {
        LockVertexArrayAPI lock;
        glBindVertexArray(resource->vao);
        glBindBuffer(GL_ARRAY_BUFFER, resource->newBuffer());
        glVertexAttribPointer((GLuint)0, 3, GL_FLOAT, GL_FALSE, 0, ((GLubyte*)NULL + (0)));
    }
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vector3f), vertices.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);

    SgNormalArray normals;
    writeMeshNormals(mesh, resource->newBuffer(), normals);
    if(isNormalVisualizationEnabled){
        auto lines = new SgLineSet;
        auto lineVertices = lines->getOrCreateVertices();
        for(size_t i=0; i < vertices.size(); ++i){
            const Vector3f& v = vertices[i];
            lineVertices->push_back(v);
            lineVertices->push_back(v + normals[i] * normalVisualizationLength);
            lines->addLine(i*2, i*2+1);
        }
        lines->setMaterial(normalVisualizationMaterial);
        resource->normalVisualization = lines;
    }

    if(mesh->hasTexCoords()){
        writeMeshTexCoords(mesh, resource->newBuffer());
    }
    
    if(mesh->hasColors()){
        writeMeshColors(mesh, resource->newBuffer());
    }
}


void GLSLSceneRendererImpl::writeMeshNormals(SgMesh* mesh, GLuint buffer, SgNormalArray& normals)
{
    auto& triangleVertices = mesh->triangleVertices();
    const int totalNumVertices = triangleVertices.size();
    normals.reserve(totalNumVertices);
    const int numTriangles = mesh->numTriangles();

    if(defaultSmoothShading && mesh->normals()){
        const auto& orgNormals = *mesh->normals();
        const auto& normalIndices = mesh->normalIndices();
        int faceVertexIndex = 0;
        if(normalIndices.empty()){
            for(int i=0; i < numTriangles; ++i){
                for(int j=0; j < 3; ++j){
                    const int orgVertexIndex = triangleVertices[faceVertexIndex++];
                    normals.push_back(orgNormals[orgVertexIndex]);
                }
            }
        } else {
            for(int i=0; i < numTriangles; ++i){
                for(int j=0; j < 3; ++j){
                    const int normalIndex = normalIndices[faceVertexIndex++];
                    normals.push_back(orgNormals[normalIndex]);
                }
            }
        }
    } else {
        // flat shading
        const auto& orgVertices = *mesh->vertices();
        for(int i=0; i < numTriangles; ++i){
            SgMesh::TriangleRef triangle = mesh->triangle(i);
            const Vector3f e1 = orgVertices[triangle[1]] - orgVertices[triangle[0]];
            const Vector3f e2 = orgVertices[triangle[2]] - orgVertices[triangle[0]];
            const Vector3f normal = e1.cross(e2).normalized();
            for(int j=0; j < 3; ++j){
                normals.push_back(normal);
            }
        }
    }

    {
        LockVertexArrayAPI lock;
        glBindBuffer(GL_ARRAY_BUFFER, buffer);
        glVertexAttribPointer((GLuint)1, 3, GL_FLOAT, GL_FALSE, 0, ((GLubyte*)NULL + (0)));
    }
    glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(Vector3f), normals.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(1);
}


void GLSLSceneRendererImpl::writeMeshTexCoords(SgMesh* mesh, GLuint buffer)
{
    auto& triangleVertices = mesh->triangleVertices();
    const int totalNumVertices = triangleVertices.size();
    SgTexCoordArrayPtr pOrgTexCoords;
    const auto& texCoordIndices = mesh->texCoordIndices();
    SgTexCoordArray texCoords;
    texCoords.reserve(totalNumVertices);
    if(!hasValidTextureTransform){
        pOrgTexCoords = mesh->texCoords();
    } else {
        const auto& orgTexCoords = *mesh->texCoords();
        const size_t n = orgTexCoords.size();
        pOrgTexCoords = new SgTexCoordArray(n);
        for(size_t i=0; i < n; ++i){
            (*pOrgTexCoords)[i] = textureTransform * orgTexCoords[i];
        }
    }

    const int numTriangles = mesh->numTriangles();
    int faceVertexIndex = 0;
    
    if(texCoordIndices.empty()){
        for(int i=0; i < numTriangles; ++i){
            for(int j=0; j < 3; ++j){
                const int orgVertexIndex = triangleVertices[faceVertexIndex++];
                texCoords.push_back((*pOrgTexCoords)[orgVertexIndex]);
            }
        }
    } else {
        for(int i=0; i < numTriangles; ++i){
            for(int j=0; j < 3; ++j){
                const int texCoordIndex = texCoordIndices[faceVertexIndex++];
                texCoords.push_back((*pOrgTexCoords)[texCoordIndex]);
            }
        }
    }

    {
        LockVertexArrayAPI lock;
        glBindBuffer(GL_ARRAY_BUFFER, buffer);
        glVertexAttribPointer((GLuint)2, 2, GL_FLOAT, GL_FALSE, 0, ((GLubyte*)NULL + (0)));
    }
    glBufferData(GL_ARRAY_BUFFER, texCoords.size() * sizeof(Vector2f), texCoords.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(2);
}


void GLSLSceneRendererImpl::writeMeshColors(SgMesh* mesh, GLuint buffer)
{
    auto& triangleVertices = mesh->triangleVertices();
    const int totalNumVertices = triangleVertices.size();
    const auto& orgColors = *mesh->colors();
    const auto& colorIndices = mesh->colorIndices();
    SgColorArray colors;
    colors.reserve(totalNumVertices);
    const int numTriangles = mesh->numTriangles();
    int faceVertexIndex = 0;

    if(colorIndices.empty()){
        for(int i=0; i < numTriangles; ++i){
            for(int j=0; j < 3; ++j){
                const int orgVertexIndex = triangleVertices[faceVertexIndex++];
                colors.push_back(orgColors[orgVertexIndex]);
            }
        }
    } else {
        for(int i=0; i < numTriangles; ++i){
            for(int j=0; j < 3; ++j){
                const int colorIndex = colorIndices[faceVertexIndex++];
                colors.push_back(orgColors[colorIndex]);
            }
        }
    }

    {
        LockVertexArrayAPI lock;
        glBindBuffer(GL_ARRAY_BUFFER, buffer);
        glVertexAttribPointer((GLuint)3, 3, GL_FLOAT, GL_FALSE, 0, ((GLubyte*)NULL + (0)));
    }
    glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(Vector3f), colors.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(3);
}
    

void GLSLSceneRendererImpl::renderPointSet(SgPointSet* pointSet)
{
    if(!pointSet->hasVertices()){
        return;
    }

    pushProgram(solidColorProgram, false);

    const double s = pointSet->pointSize();
    if(s > 0.0){
        setPointSize(s);
    } else {
        setPointSize(defaultPointSize);
    }
    
    renderPlot(pointSet, GL_POINTS,
               [pointSet]() -> SgVertexArrayPtr { return pointSet->vertices(); });

    popProgram();
}


void GLSLSceneRendererImpl::renderPlot
(SgPlot* plot, GLenum primitiveMode, std::function<SgVertexArrayPtr()> getVertices)
{
    pushPickId(plot);

    bool hasColors = plot->hasColors();
    
    if(isPicking){
        currentProgram->enableColorArray(false);
    } else {
        if(!hasColors){
            renderMaterial(plot->material());
        }
        currentProgram->enableColorArray(hasColors);
    }
    
    VertexResource* resource = getOrCreateVertexResource(plot);
    if(!resource->isValid()){
        glBindVertexArray(resource->vao);
        SgVertexArrayPtr vertices = getVertices();
        const size_t n = vertices->size();
        resource->numVertices = n;

        {
            LockVertexArrayAPI lock;
            glBindBuffer(GL_ARRAY_BUFFER, resource->newBuffer());
            glVertexAttribPointer((GLuint)0, 3, GL_FLOAT, GL_FALSE, 0, ((GLubyte *)NULL + (0)));
        }
        glBufferData(GL_ARRAY_BUFFER, vertices->size() * sizeof(Vector3f), vertices->data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);

        if(hasColors){
            SgColorArrayPtr colors;
            const SgColorArray& orgColors = *plot->colors();
            const SgIndexArray& colorIndices = plot->colorIndices();
            if(plot->colorIndices().empty()){
                if(orgColors.size() >= n){
                    colors = plot->colors();
                } else {
                    colors = new SgColorArray(n);
                    std::copy(orgColors.begin(), orgColors.end(), colors->begin());
                    std::fill(colors->begin() + orgColors.size(), colors->end(), orgColors.back());
                }
            } else {
                const int m = colorIndices.size();
                colors = new SgColorArray(m);
                for(int i=0; i < m; ++i){
                    (*colors)[i] = orgColors[colorIndices[i]];
                }
            }

            {
                LockVertexArrayAPI lock;
                glBindBuffer(GL_ARRAY_BUFFER, resource->newBuffer());
                glVertexAttribPointer((GLuint)1, 3, GL_FLOAT, GL_FALSE, 0, ((GLubyte*)NULL +(0)));
            }
            glBufferData(GL_ARRAY_BUFFER, n * sizeof(Vector3f), colors->data(), GL_STATIC_DRAW);
            glEnableVertexAttribArray(1);
        }
    }        

    drawVertexResource(resource, primitiveMode, modelMatrixStack.back());
    
    popPickId();
}


static SgVertexArrayPtr getLineSetVertices(SgLineSet* lineSet)
{
    const SgVertexArray& orgVertices = *lineSet->vertices();
    SgVertexArray* vertices = new SgVertexArray;
    const int n = lineSet->numLines();
    vertices->reserve(n * 2);
    for(int i=0; i < n; ++i){
        SgLineSet::LineRef line = lineSet->line(i);
        vertices->push_back(orgVertices[line[0]]);
        vertices->push_back(orgVertices[line[1]]);
    }
    return vertices;
}


void GLSLSceneRendererImpl::renderLineSet(SgLineSet* lineSet)
{
    if(isRenderingShadowMap){
        return;
    }
    
    if(!lineSet->hasVertices() || lineSet->numLines() <= 0){
        return;
    }

    pushProgram(solidColorProgram, false);
    
    const double w = lineSet->lineWidth();
    if(w > 0.0){
        setLineWidth(w);
    } else {
        setLineWidth(defaultLineWidth);
    }

    renderPlot(lineSet, GL_LINES,
               [lineSet](){ return getLineSetVertices(lineSet); });

    popProgram();
}


void GLSLSceneRendererImpl::renderOverlay(SgOverlay* overlay)
{
    if(!isActuallyRendering){
        return;
    }

    pushProgram(solidColorProgram, false);
    modelMatrixStack.push_back(Affine3::Identity());

    const Matrix4 PV0 = PV;
    SgOverlay::ViewVolume v;
    const Array4i vp = self->viewport();
    overlay->calcViewVolume(vp[2], vp[3], v);
    self->getOrthographicProjectionMatrix(v.left, v.right, v.bottom, v.top, v.zNear, v.zFar, PV);
            
    renderGroup(overlay);

    PV = PV0;
    modelMatrixStack.pop_back();
    popProgram();
}


void GLSLSceneRendererImpl::renderOutlineGroup(SgOutlineGroup* outline)
{
    if(isPicking){
        renderGroup(outline);
    } else {
        const Affine3& T = modelMatrixStack.back();
        postRenderingFunctions.push_back(
            [this, outline, T](){ renderOutlineGroupMain(outline, T); });
    }
}


void GLSLSceneRendererImpl::renderOutlineGroupMain(SgOutlineGroup* outline, const Affine3& T)
{
    modelMatrixStack.push_back(T);

    glClearStencil(0);
    glClear(GL_STENCIL_BUFFER_BIT);
    glEnable(GL_STENCIL_TEST);
    glStencilFunc(GL_ALWAYS, 1, -1);
    glStencilOp(GL_KEEP, GL_REPLACE, GL_REPLACE);

    renderChildNodes(outline);

    glStencilFunc(GL_NOTEQUAL, 1, -1);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

    float orgLineWidth = lineWidth;
    setLineWidth(outline->lineWidth()*2+1);
    GLint polygonMode;
    glGetIntegerv(GL_POLYGON_MODE, &polygonMode);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    pushProgram(solidColorProgram, false);
    solidColorProgram.setColor(outline->color());
    solidColorProgram.setColorChangable(false);
    glDisable(GL_DEPTH_TEST);

    renderChildNodes(outline);

    glEnable(GL_DEPTH_TEST);
    setLineWidth(orgLineWidth);
    glPolygonMode(GL_FRONT_AND_BACK, polygonMode);
    glDisable(GL_STENCIL_TEST);
    solidColorProgram.setColorChangable(true);
    popProgram();

    modelMatrixStack.pop_back();
}


void GLSLSceneRendererImpl::clearGLState()
{
    std::fill(stateFlag.begin(), stateFlag.end(), false);
    
    diffuseColor << 0.0f, 0.0f, 0.0f, 0.0f;
    ambientColor << 0.0f, 0.0f, 0.0f, 0.0f;
    emissionColor << 0.0f, 0.0f, 0.0f, 0.0f;
    specularColor << 0.0f, 0.0f, 0.0f, 0.0f;
    shininess = 0.0f;
    alpha = 1.0f;

    isCullFaceEnabled = boost::none;

    pointSize = defaultPointSize;    
    lineWidth = defaultLineWidth;
}


void GLSLSceneRenderer::setColor(const Vector3f& color)
{
    impl->currentProgram->setColor(color);
}


void GLSLSceneRendererImpl::setDiffuseColor(const Vector3f& color)
{
    if(!stateFlag[DIFFUSE_COLOR] || diffuseColor != color){
        materialProgram->setDiffuseColor(color);
        diffuseColor = color;
        stateFlag[DIFFUSE_COLOR] = true;
    }
}


void GLSLSceneRenderer::setDiffuseColor(const Vector3f& color)
{
    impl->setDiffuseColor(color);
}


void GLSLSceneRendererImpl::setAmbientColor(const Vector3f& color)
{
    if(!stateFlag[AMBIENT_COLOR] || ambientColor != color){
        materialProgram->setAmbientColor(color);
        ambientColor = color;
        stateFlag[AMBIENT_COLOR] = true;
    }
}


void GLSLSceneRenderer::setAmbientColor(const Vector3f& color)
{
    impl->setAmbientColor(color);
}


void GLSLSceneRendererImpl::setEmissionColor(const Vector3f& color)
{
    if(!stateFlag[EMISSION_COLOR] || emissionColor != color){
        materialProgram->setEmissionColor(color);
        emissionColor = color;
        stateFlag[EMISSION_COLOR] = true;
    }
}


void GLSLSceneRenderer::setEmissionColor(const Vector3f& color)
{
    impl->setEmissionColor(color);
}


void GLSLSceneRendererImpl::setSpecularColor(const Vector3f& color)
{
    if(!stateFlag[SPECULAR_COLOR] || specularColor != color){
        materialProgram->setSpecularColor(color);
        specularColor = color;
        stateFlag[SPECULAR_COLOR] = true;
    }
}


void GLSLSceneRenderer::setSpecularColor(const Vector3f& color)
{
    impl->setSpecularColor(color);
}


void GLSLSceneRendererImpl::setShininess(float s)
{
    if(!stateFlag[SHININESS] || shininess != s){
        materialProgram->setShininess(s);
        shininess = s;
        stateFlag[SHININESS] = true;
    }
}


void GLSLSceneRenderer::setShininess(float s)
{
    impl->setShininess(s);
}


void GLSLSceneRendererImpl::setAlpha(float a)
{
    if(!stateFlag[ALPHA] || alpha != a){
        materialProgram->setAlpha(a);
        alpha = a;
        stateFlag[ALPHA] = true;
    }
}


void GLSLSceneRenderer::setAlpha(float a)
{
    impl->setAlpha(a);
}


void GLSLSceneRenderer::clearShadows()
{
    impl->shadowLightIndices.clear();
}


void GLSLSceneRenderer::enableShadowOfLight(int index, bool on)
{
    if(on){
        impl->shadowLightIndices.insert(index);
    } else {
        impl->shadowLightIndices.erase(index);
    }
}


void GLSLSceneRenderer::enableShadowAntiAliasing(bool on)
{
    impl->phongShadowProgram.setShadowAntiAliasingEnabled(on);
}


void GLSLSceneRendererImpl::setPointSize(float size)
{
    if(!stateFlag[POINT_SIZE] || pointSize != size){
        float s = isPicking ? std::max(size, MinLineWidthForPicking) : size;
        solidColorProgram.setPointSize(s);
        pointSize = s;
        stateFlag[POINT_SIZE] = true;
    }
}


void GLSLSceneRenderer::setPointSize(float size)
{
    impl->setPointSize(size);
}


void GLSLSceneRendererImpl::setLineWidth(float width)
{
    if(!stateFlag[LINE_WIDTH] || lineWidth != width){
        if(isPicking){
            glLineWidth(std::max(width, MinLineWidthForPicking));
        } else {
            glLineWidth(width);
        }
        lineWidth = width;
        stateFlag[LINE_WIDTH] = true;
    }
}


void GLSLSceneRenderer::setLineWidth(float width)
{
    impl->setLineWidth(width);
}


void GLSLSceneRenderer::setDefaultLighting(bool on)
{
    if(on != impl->defaultLighting){
        impl->defaultLighting = on;
    }
}


void GLSLSceneRenderer::setDefaultSmoothShading(bool on)
{
    if(on != impl->defaultSmoothShading){
        impl->defaultSmoothShading = on;
        requestToClearResources();
    }
}


SgMaterial* GLSLSceneRenderer::defaultMaterial()
{
    return impl->defaultMaterial;
}


void GLSLSceneRenderer::enableTexture(bool on)
{
    /*
    if(on != impl->isTextureEnabled){
        impl->isTextureEnabled = on;
    }
    */
}


void GLSLSceneRenderer::setDefaultPointSize(double size)
{
    if(size != impl->defaultPointSize){
        impl->defaultPointSize = size;
    }
}


void GLSLSceneRenderer::setDefaultLineWidth(double width)
{
    if(width != impl->defaultLineWidth){
        impl->defaultLineWidth = width;
    }
}


void GLSLSceneRenderer::showNormalVectors(double length)
{
    bool isEnabled = (length > 0.0);
    if(isEnabled != impl->isNormalVisualizationEnabled || length != impl->normalVisualizationLength){
        impl->isNormalVisualizationEnabled = isEnabled;
        impl->normalVisualizationLength = length;
        requestToClearResources();
    }
}


void GLSLSceneRenderer::enableUnusedResourceCheck(bool on)
{
    if(!on){
        impl->nextResourceMap->clear();
    }
    impl->doUnusedResourceCheck = on;
}


void GLSLSceneRenderer::setUpsideDown(bool on)
{
    impl->isUpsideDownEnabled = on;
}
