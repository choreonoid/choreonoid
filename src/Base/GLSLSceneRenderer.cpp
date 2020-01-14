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
#include <fmt/format.h>
#include <GL/glu.h>
#include <unordered_map>
#include <deque>
#include <mutex>
#include <regex>
#include <iostream>
#include <stdexcept>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

const float MinLineWidthForPicking = 5.0f;
const bool USE_GL_FLOAT_FOR_NORMALS = false;

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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    static const int MAX_NUM_BUFFERS = 4;
    GLuint vao;
    GLuint vbos[MAX_NUM_BUFFERS];
    GLsizei numVertices;
    int numBuffers;
    SgObjectPtr sceneObject;
    ScopedConnection connection;
    Matrix4* pLocalTransform;
    Matrix4 localTransform;
    SgLineSetPtr boundingBoxLines;
    SgLineSetPtr normalVisualization;
    
    VertexResource(const VertexResource&) = delete;
    VertexResource& operator=(const VertexResource&) = delete;

    VertexResource(GLSLSceneRenderer::Impl* renderer, SgObject* obj)
        : sceneObject(obj)
    {
        connection.reset(
            obj->sigUpdated().connect(
                [&](const SgUpdate&){ numVertices = 0; }));

        clearHandles();
        glGenVertexArrays(1, &vao);
        pLocalTransform = nullptr;
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

class ScopedShaderProgramActivator
{
    GLSLSceneRenderer::Impl* renderer;
    ShaderProgram* prevProgram;
    NolightingProgram* prevNolightingProgram;
    LightingProgram* prevLightingProgram;
    MaterialLightingProgram* prevMaterialLightingProgram;
    bool changed;
    
public:
    ScopedShaderProgramActivator(ShaderProgram& program, GLSLSceneRenderer::Impl* renderer);
    ScopedShaderProgramActivator(ScopedShaderProgramActivator&&) noexcept;
    ScopedShaderProgramActivator(const ScopedShaderProgramActivator&) = delete;
    ScopedShaderProgramActivator& operator=(const ScopedShaderProgramActivator&) = delete;
    ~ScopedShaderProgramActivator();
};

}

namespace cnoid {

class GLSLSceneRenderer::Impl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    GLSLSceneRenderer* self;

    PolymorphicSceneNodeFunctionSet renderingFunctions;

    GLuint defaultFBO;
    GLuint fboForPicking;
    GLuint colorBufferForPicking;
    GLuint depthBufferForPicking;
    GLuint depthBufferForOverlay;
    int pickingImageWidth;
    int pickingImageHeight;

    ShaderProgram* currentProgram;
    NolightingProgram* currentNolightingProgram;
    LightingProgram* currentLightingProgram;
    MaterialLightingProgram* currentMaterialLightingProgram;

    NolightingProgram nolightingProgram;
    SolidColorProgram solidColorProgram;
    MinimumLightingProgram minimumLightingProgram;
    PhongLightingProgram phongLightingProgram;
    PhongShadowLightingProgram phongShadowLightingProgram;

    vector<ScopedShaderProgramActivator> programStack;

    /**
       This variable is incremented at every frame of rendering.
       The number is used to execute some operation onece at each rendering frame.
    */
    unsigned int renderingFrameId;
    
    bool isRenderingVisibleImage;
    bool isRenderingPickingImage;
    bool isPickingImageOutputEnabled;
    bool isShadowCastingEnabled;
    bool isRenderingShadowMap;
    bool isLightweightRenderingBeingProcessed;
    bool isLowMemoryConsumptionMode;
    bool isLowMemoryConsumptionRenderingBeingProcessed;
    bool isBoundingBoxRenderingMode;
    bool isBoundingBoxRenderingForLightweightRenderingGroupEnabled;
    
    Affine3Array modelMatrixStack; // stack of the model matrices
    Affine3Array modelMatrixBuffer; // Model matriices used later are stored in this buffer
    Affine3 viewTransform;
    Matrix4 projectionMatrix;
    Matrix4 PV;

    deque<function<void()>> transparentRenderingQueue;
    deque<function<void()>> overlayRenderingQueue;
    bool needToUpdateOverlayDepthBufferSize;
    
    std::set<int> shadowLightIndices;

    int lightingMode;
    SgMaterialPtr defaultMaterial;
    GLfloat defaultPointSize;
    GLfloat defaultLineWidth;
    
    GLResourceMap resourceMaps[2];
    GLResourceMap* currentResourceMap;
    GLResourceMap* nextResourceMap;
    int currentResourceMapIndex;
    bool doUnusedResourceCheck;
    bool isCheckingUnusedResources;
    bool hasValidNextResourceMap;
    bool isResourceClearRequested;

    vector<char> scaledImageBuf;

    bool isTextureEnabled;
    bool isTextureBeingRendered;
    bool isCurrentFogUpdated;
    SgFogPtr prevFog;
    ScopedConnection currentFogConnection;

    bool defaultSmoothShading;
    bool isNormalVisualizationEnabled;
    float normalVisualizationLength;
    SgMaterialPtr normalVisualizationMaterial;

    // OpenGL states
    enum StateFlag {
        CULL_FACE,
        POINT_SIZE,
        LINE_WIDTH,
        NUM_STATE_FLAGS
    };
    vector<bool> stateFlag;

    int backFaceCullingMode;
    bool isCullFaceEnabled;

    float pointSize;
    float lineWidth;

    GLdouble pickX;
    GLdouble pickY;
    typedef std::shared_ptr<SgNodePath> SgNodePathPtr;
    SgNodePath currentNodePath;
    SgNodePath emptyNodePath;

    vector<SgNodePathPtr> pickingNodePathList;
    SgNodePath pickedNodePath;
    Vector3 pickedPoint;
    int overlayPickIndex0;

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
    
    Impl(GLSLSceneRenderer* self);
    ~Impl();
    void initialize();
    void onExtensionAdded(std::function<void(GLSLSceneRenderer* renderer)> func);
    void updateDefaultFramebufferObject();
    bool initializeGL();
    void doRender();
    void setupFullLightingRendering();
    bool doPick(int x, int y);
    void renderScene();
    bool renderShadowMap(int lightIndex);
    void beginRendering();
    void renderCamera(SgCamera* camera, const Affine3& cameraPosition);
    void renderLights(LightingProgram* program);
    void renderFog(LightingProgram* program);
    void renderTransparentObjects();
    void renderOverlayObjects();    
    void endRendering();
    void renderSceneGraphNodes();
    void pushProgram(ShaderProgram& program);
    void popProgram();
    inline void setPickColor(int pickIndex);
    inline int pushPickNode(SgNode* node, bool doSetColor = true);
    void popPickNode();
    void renderGroup(SgGroup* group);
    void renderTransform(SgTransform* transform);
    void renderAutoScale(SgAutoScale* autoScale);
    void renderSwitchableGroup(SgSwitchableGroup* group);
    void renderUnpickableGroup(SgUnpickableGroup* group);
    VertexResource* getOrCreateVertexResource(SgObject* obj);
    void drawVertexResource(VertexResource* resource, GLenum primitiveMode, const Affine3& position);
    void drawBoundingBox(VertexResource* resource, const BoundingBox& bbox);
    void renderShape(SgShape* shape);
    void renderShapeMain(SgShape* shape, const Affine3& position, int pickIndex);
    void applyCullingMode(SgMesh* mesh);
    void renderPointSet(SgPointSet* pointSet);        
    void renderLineSet(SgLineSet* lineSet);        
    void renderOverlay(SgOverlay* overlay);
    void renderOverlayMain(SgOverlay* overlay, const Affine3& T, const SgNodePath& nodePath);
    void renderViewportOverlay(SgViewportOverlay* overlay);
    void renderViewportOverlayMain(SgViewportOverlay* overlay);
    void renderOutline(SgOutline* outline);
    void renderOutlineEdge(SgOutline* outline, const Affine3& T);
    void renderLightweightRenderingGroup(SgLightweightRenderingGroup* group);
    void flushNolightingTransformMatrices();
    void renderMaterial(const SgMaterial* material);
    bool renderTexture(SgTexture* texture);
    bool loadTextureImage(TextureResource* resource, const Image& image);
    void makeVertexBufferObjects(SgShape* shape, VertexResource* resource);
    void writeMeshVertices(SgMesh* mesh, VertexResource* resource, SgTexture* texResource);
    template<typename value_type, GLenum gltype, GLboolean normalized, class VertexArrayWrapper>
    void writeMeshVerticesSub(SgMesh* mesh, VertexResource* resource, VertexArrayWrapper& normals);
    void writeMeshVerticesFloat(SgMesh* mesh, VertexResource* resource);
    void writeMeshVerticesNormalizedShort(SgMesh* mesh, VertexResource* resource);
    template<typename value_type, GLenum gltype, GLint glsize, GLboolean normalized, class NormalArrayWrapper>
    bool writeMeshNormalsSub(SgMesh* mesh, VertexResource* resource, NormalArrayWrapper& normals);
    void writeMeshNormalsFloat(SgMesh* mesh, VertexResource* resource);
    void writeMeshNormalsShort(SgMesh* mesh, VertexResource* resource);
    void writeMeshNormalsByte(SgMesh* mesh, VertexResource* resource);
    void writeMeshNormalsPacked(SgMesh* mesh, VertexResource* resource);
    template<typename value_type, GLenum gltype, GLboolean normalized, class TexCoordArrayWrapper>
    void writeMeshTexCoordsSub(
        SgMesh* mesh, SgTexture* texture, VertexResource* resource, TexCoordArrayWrapper& texCoords);
    void writeMeshTexCoordsFloat(SgMesh* mesh, SgTexture* texture, VertexResource* resource);
    void writeMeshTexCoordsHalfFloat(SgMesh* mesh, SgTexture* texture, VertexResource* resource);
    void writeMeshTexCoordsUnsignedShort(SgMesh* mesh, SgTexture* texture, VertexResource* resource);
    void writeMeshColors(SgMesh* mesh, VertexResource* resource);
    void renderPlot(SgPlot* plot, GLenum primitiveMode, std::function<SgVertexArrayPtr()> getVertices);
    void clearGLState();
    void setPointSize(float size);
    void setLineWidth(float width);
    void getCurrentCameraTransform(Affine3& T);
};

}


GLSLSceneRenderer::GLSLSceneRenderer(SgGroup* sceneRoot)
    : GLSceneRenderer(sceneRoot)
{
    impl = new Impl(this);
    impl->initialize();
}


GLSLSceneRenderer::Impl::Impl(GLSLSceneRenderer* self)
    : self(self)
{

}


void GLSLSceneRenderer::Impl::initialize()
{
    {
        std::lock_guard<std::mutex> guard(extensionMutex);
        renderers.insert(self);
    }
    
    defaultFBO = 0;
    fboForPicking = 0;
    colorBufferForPicking = 0;
    depthBufferForPicking = 0;
    pickingImageWidth = 0;
    pickingImageHeight = 0;

    currentProgram = nullptr;
    currentNolightingProgram = nullptr;
    currentLightingProgram = nullptr;
    currentMaterialLightingProgram = nullptr;

    renderingFrameId = 1;
    isRenderingVisibleImage = false;
    isRenderingPickingImage = false;
    isPickingImageOutputEnabled = false;
    isShadowCastingEnabled = true;
    isRenderingShadowMap = false;
    isLowMemoryConsumptionMode = false;
    isBoundingBoxRenderingMode = false;
    isBoundingBoxRenderingForLightweightRenderingGroupEnabled = false;

    doUnusedResourceCheck = true;
    currentResourceMapIndex = 0;
    hasValidNextResourceMap = false;
    isResourceClearRequested = false;
    currentResourceMap = &resourceMaps[0];
    nextResourceMap = &resourceMaps[1];

    modelMatrixStack.reserve(16);
    viewTransform.setIdentity();
    projectionMatrix.setIdentity();

    depthBufferForOverlay = 0;
    needToUpdateOverlayDepthBufferSize = true;

    lightingMode = GLSceneRenderer::FULL_LIGHTING;
    defaultSmoothShading = true;
    defaultMaterial = new SgMaterial;
    defaultPointSize = 1.0f;
    defaultLineWidth = 1.0f;
    isTextureEnabled = true;

    isNormalVisualizationEnabled = false;
    normalVisualizationLength = 0.0f;
    normalVisualizationMaterial = new SgMaterial;
    normalVisualizationMaterial->setDiffuseColor(Vector3f(0.0f, 1.0f, 0.0f));

    isUpsideDownEnabled = false;

    stateFlag.resize(NUM_STATE_FLAGS, false);

    backFaceCullingMode = GLSceneRenderer::ENABLE_BACK_FACE_CULLING;

    pickedPoint.setZero();

    clearGLState();

    os_ = &nullout();

    renderingFunctions.setFunction<SgGroup>(
        [&](SgGroup* node){ renderGroup(node); });
    renderingFunctions.setFunction<SgTransform>(
        [&](SgTransform* node){ renderTransform(node); });
    renderingFunctions.setFunction<SgAutoScale>(
        [&](SgAutoScale* node){ renderAutoScale(node); });
    renderingFunctions.setFunction<SgSwitchableGroup>(
        [&](SgSwitchableGroup* node){ renderSwitchableGroup(node); });
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
    renderingFunctions.setFunction<SgViewportOverlay>(
        [&](SgViewportOverlay* node){ renderViewportOverlay(node); });
    renderingFunctions.setFunction<SgOutline>(
        [&](SgOutline* node){ renderOutline(node); });
    renderingFunctions.setFunction<SgLightweightRenderingGroup>(
        [&](SgLightweightRenderingGroup* node){ renderLightweightRenderingGroup(node); });

    self->applyExtensions();
    renderingFunctions.updateDispatchTable();
}


GLSLSceneRenderer::~GLSLSceneRenderer()
{
    std::lock_guard<std::mutex> guard(extensionMutex);
    renderers.erase(this);
    
    delete impl;
}


GLSLSceneRenderer::Impl::~Impl()
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


void GLSLSceneRenderer::Impl::onExtensionAdded(std::function<void(GLSLSceneRenderer* renderer)> func)
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
    GLSceneRenderer::setOutputStream(os);
    impl->os_ = &os;
}


void GLSLSceneRenderer::Impl::updateDefaultFramebufferObject()
{
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, reinterpret_cast<GLint*>(&defaultFBO));
    phongShadowLightingProgram.setDefaultFramebufferObject(defaultFBO);
}


bool GLSLSceneRenderer::initializeGL()
{
    return impl->initializeGL();
}


bool GLSLSceneRenderer::Impl::initializeGL()
{
    if(ogl_LoadFunctions() == ogl_LOAD_FAILED){
        return false;
    }

    GLint major, minor;
    glGetIntegerv(GL_MAJOR_VERSION, &major);
    glGetIntegerv(GL_MINOR_VERSION, &minor);
    const GLubyte* version = glGetString(GL_VERSION);
    const GLubyte* vendor = glGetString(GL_VENDOR);
    const GLubyte* renderer = glGetString(GL_RENDERER);
    const GLubyte* glsl = glGetString(GL_SHADING_LANGUAGE_VERSION);

    os() << fmt::format(_("OpenGL {0}.{1} ({2} {3}, GLSL {4}) is available for the \"{5}\" view.\n"),
                        major, minor, vendor, renderer, glsl, self->name());

    std::cmatch match;

    // Check the Intel GPU
    if(regex_match((const char*)renderer, match, regex("Mesa DRI Intel\\(R\\) (\\S+).*$"))){
        if(match.str(1) == "Sandybridge"){
            isShadowCastingEnabled = false;
        } else if(regex_match((const char*)version, match, regex(".*Mesa (\\d+)\\.(\\d+)\\.(\\d+).*$"))){
            int mesaMajor = stoi(match.str(1));
            if(mesaMajor >= 19){
                isShadowCastingEnabled = false;
            }
        }
    }
    // Check if the GPU is AMD's Radeon GPU
    else if(regex_match((const char*)renderer, regex("AMD Radeon.*"))){
        isShadowCastingEnabled = false;
    }
    // CHeck if the VMWare's virtual driver is used
    else if(regex_match((const char*)vendor, regex("VMware, Inc\\..*"))){
        isShadowCastingEnabled = false;
    }
    // Check if the GPU driver is Nouveau
    else if(regex_match((const char*)vendor, regex(".*nouveau.*"))){
        isShadowCastingEnabled = false;

    }

    if(!isShadowCastingEnabled){
        os() << fmt::format(_(" Shadow casting is disabled for this GPU due to some problems.\n"));
    }
    
    os().flush();
    
    updateDefaultFramebufferObject();

    try {
        nolightingProgram.initialize();
        solidColorProgram.initialize();
        minimumLightingProgram.initialize();
        phongLightingProgram.initialize();
        if(isShadowCastingEnabled){
            phongShadowLightingProgram.initialize();
        }
    }
    catch(std::runtime_error& error){
        os() << error.what() << endl;
        cout << error.what() << endl;
        return false;
    }

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_DITHER);
    glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

    isResourceClearRequested = true;
    isCurrentFogUpdated = false;

    return true;
}


void GLSLSceneRenderer::setViewport(int x, int y, int width, int height)
{
    glViewport(x, y, width, height);
    updateViewportInformation(x, y, width, height);
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


void GLSLSceneRenderer::updateViewportInformation(int x, int y, int width, int height)
{
    GLSceneRenderer::updateViewportInformation(x, y, width, height);
    impl->needToUpdateOverlayDepthBufferSize = true;
}


void GLSLSceneRenderer::requestToClearResources()
{
    impl->isResourceClearRequested = true;
}


void GLSLSceneRenderer::onSceneGraphUpdated(const SgUpdate& update)
{
    if(SgLightweightRenderingGroup* group = dynamic_cast<SgLightweightRenderingGroup*>(update.path().front())){
        requestToClearResources();
    }
    
    GLSceneRenderer::onSceneGraphUpdated(update);
}


ScopedShaderProgramActivator::ScopedShaderProgramActivator
(ShaderProgram& program, GLSLSceneRenderer::Impl* renderer)
    : renderer(renderer)
{
    if(&program == renderer->currentProgram){
        changed = false;
    } else {
        if(renderer->currentProgram){
            renderer->currentProgram->deactivate();
        }

        prevProgram = renderer->currentProgram;
        prevNolightingProgram = renderer->currentNolightingProgram;
        prevLightingProgram = renderer->currentLightingProgram;
        prevMaterialLightingProgram = renderer->currentMaterialLightingProgram;
    
        renderer->currentProgram = &program;
        renderer->currentNolightingProgram = dynamic_cast<NolightingProgram*>(&program);
        renderer->currentLightingProgram = dynamic_cast<LightingProgram*>(&program);
        renderer->currentMaterialLightingProgram = dynamic_cast<MaterialLightingProgram*>(&program);

        program.activate();
        renderer->clearGLState();
        changed = true;
    }
}


ScopedShaderProgramActivator::ScopedShaderProgramActivator(ScopedShaderProgramActivator&& r) noexcept
{
    renderer = r.renderer;
    prevProgram = r.prevProgram;
    prevNolightingProgram = r.prevNolightingProgram;
    prevLightingProgram = r.prevLightingProgram;
    prevMaterialLightingProgram = r.prevMaterialLightingProgram;
    changed = r.changed;
    r.changed = false;
}


ScopedShaderProgramActivator::~ScopedShaderProgramActivator()
{
    if(changed){
        renderer->currentProgram->deactivate();
        if(prevProgram){
            prevProgram->activate();
            renderer->clearGLState();
        }
        renderer->currentProgram = prevProgram;
        renderer->currentNolightingProgram = prevNolightingProgram;
        renderer->currentLightingProgram = prevLightingProgram;
        renderer->currentMaterialLightingProgram = prevMaterialLightingProgram;
    }
}


void GLSLSceneRenderer::Impl::pushProgram(ShaderProgram& program)
{
    programStack.emplace_back(program, this);
}

void GLSLSceneRenderer::pushShaderProgram(ShaderProgram& program)
{
    impl->pushProgram(program);
}


void GLSLSceneRenderer::Impl::popProgram()
{
    programStack.pop_back();
}


void GLSLSceneRenderer::popShaderProgram()
{
    impl->popProgram();
}


void GLSLSceneRenderer::doRender()
{
    impl->doRender();
}


void GLSLSceneRenderer::Impl::doRender()
{
    updateDefaultFramebufferObject();
    
    if(self->applyNewExtensions()){
        renderingFunctions.updateDispatchTable();
    }

    self->extractPreprocessedNodes();
    beginRendering();

    isLightweightRenderingBeingProcessed = false;
    isLowMemoryConsumptionRenderingBeingProcessed = isLowMemoryConsumptionMode;
    isTextureBeingRendered = false;

    switch(lightingMode){

    case GLSceneRenderer::NO_LIGHTING:
        pushProgram(nolightingProgram);
        break;

    case GLSceneRenderer::SOLID_COLOR_LIGHTING:
        pushProgram(solidColorProgram);
        break;

    case GLSceneRenderer::MINIMUM_LIGHTING:
        pushProgram(minimumLightingProgram);
        isLightweightRenderingBeingProcessed = true;
        isLowMemoryConsumptionRenderingBeingProcessed = true;
        break;

    case GLSceneRenderer::FULL_LIGHTING:
        setupFullLightingRendering();
        break;

    case GLSceneRenderer::NORMAL_LIGHTING:
    default:
        pushProgram(phongLightingProgram);
        isTextureBeingRendered = isTextureEnabled;
        break;
    }

    isRenderingVisibleImage = true;
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


void GLSLSceneRenderer::Impl::setupFullLightingRendering()
{
    isTextureBeingRendered = isTextureEnabled;

    if(shadowLightIndices.empty() || !isShadowCastingEnabled){
        // Same as NORMAL_LIGHTING
        pushProgram(phongLightingProgram);
            
    } else {
        isRenderingVisibleImage = false;
        isRenderingShadowMap = true;

        auto& program = phongShadowLightingProgram;

        int w, h;
        program.getShadowMapSize(w, h);
        Array4i vp = self->viewport();
        glViewport(0, 0, w, h);
        self->GLSceneRenderer::updateViewportInformation(0, 0, w, h);
        
        pushProgram(program.shadowMapProgram());
        
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

        glViewport(0, 0, vp[2], vp[3]);
        self->GLSceneRenderer::updateViewportInformation(0, 0, vp[2], vp[3]);
        
        program.activateMainRenderingPass();
        pushProgram(program);
    }
}


bool GLSLSceneRenderer::doPick(int x, int y)
{
    return impl->doPick(x, y);
}


bool GLSLSceneRenderer::Impl::doPick(int x, int y)
{
    int vx, vy, width, height;
    self->getViewport(vx, vy, width, height);

    if(!fboForPicking){
        glGenFramebuffers(1, &fboForPicking);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, fboForPicking);

    if(width != pickingImageWidth || height != pickingImageHeight){
        // color buffer
        if(!colorBufferForPicking){
            glGenRenderbuffers(1, &colorBufferForPicking);
        }
        glBindRenderbuffer(GL_RENDERBUFFER, colorBufferForPicking);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA, width, height);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, colorBufferForPicking);
            
        // depth buffer
        if(!depthBufferForPicking){
            glGenRenderbuffers(1, &depthBufferForPicking);
        }
        glBindRenderbuffer(GL_RENDERBUFFER, depthBufferForPicking);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_STENCIL, width, height);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, depthBufferForPicking);

        pickingImageWidth = width;
        pickingImageHeight = height;
    }

    self->extractPreprocessedNodes();

    if(!isPickingImageOutputEnabled){
        glScissor(x, y, 1, 1);
        glEnable(GL_SCISSOR_TEST);
    }

    isRenderingPickingImage = true;
    isRenderingVisibleImage = false;
    beginRendering();
    pushProgram(solidColorProgram);
    currentNodePath.clear();
    pickingNodePathList.clear();
    overlayPickIndex0 = std::numeric_limits<int>::max();

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    renderScene();
    
    popProgram();
    isRenderingPickingImage = false;

    if(!isPickingImageOutputEnabled){
        glDisable(GL_SCISSOR_TEST);
    }

    endRendering();

    glBindFramebuffer(GL_READ_FRAMEBUFFER, fboForPicking);
    glReadBuffer(GL_COLOR_ATTACHMENT0);

    GLfloat color[4];
    glReadPixels(x, y, 1, 1, GL_RGBA, GL_FLOAT, color);
    if(isPickingImageOutputEnabled){
        color[2] = 0.0f;
    }
    int pickIndex = (int)(color[0] * 255) + ((int)(color[1] * 255) << 8) + ((int)(color[2] * 255) << 16) - 1;

    pickedNodePath.clear();

    if(pickIndex >= 0 && pickIndex < static_cast<int>(pickingNodePathList.size())){
        GLfloat depth;
        if(pickIndex < overlayPickIndex0){
            glReadPixels(x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
        } else {
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, depthBufferForOverlay);
            glReadPixels(x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, depthBufferForPicking);
        }
        Vector3 projected;
        if(self->unproject(x, y, depth, pickedPoint)){
            pickedNodePath = *pickingNodePathList[pickIndex];
        }
    }

    glBindFramebuffer(GL_FRAMEBUFFER, defaultFBO);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, defaultFBO);

    return !pickedNodePath.empty();
}


void GLSLSceneRenderer::setPickingImageOutputEnabled(bool on)
{
    impl->isPickingImageOutputEnabled = on;
}


bool GLSLSceneRenderer::getPickingImage(Image& out_image)
{
    if(!impl->isPickingImageOutputEnabled){
        return false;
    }
    
    glBindFramebuffer(GL_FRAMEBUFFER, impl->fboForPicking);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, impl->fboForPicking);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    int w = impl->pickingImageWidth;
    int h = impl->pickingImageHeight;
    out_image.setSize(w, h, 4);
    glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, out_image.pixels());
    out_image.applyVerticalFlip();
    glBindFramebuffer(GL_FRAMEBUFFER, impl->defaultFBO);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, impl->defaultFBO);

    return true;
}


void GLSLSceneRenderer::Impl::renderScene()
{
    if(auto camera = self->currentCamera()){

        renderCamera(camera, self->currentCameraPosition());

        transparentRenderingQueue.clear();
        overlayRenderingQueue.clear();

        renderSceneGraphNodes();

        if(!transparentRenderingQueue.empty()){
            renderTransparentObjects();
        }
        if(!overlayRenderingQueue.empty()){
            renderOverlayObjects();
        }
    }
}


bool GLSLSceneRenderer::Impl::renderShadowMap(int lightIndex)
{
    SgLight* light;
    Affine3 T;
    self->getLightInfo(lightIndex, light, T);
    if(light && light->on()){
        SgCamera* shadowMapCamera = phongShadowLightingProgram.getShadowMapCamera(light, T);
        if(shadowMapCamera){
            renderCamera(shadowMapCamera, T);
            phongShadowLightingProgram.setShadowMapViewProjection(PV);
            renderSceneGraphNodes();
            glFlush();
            glFinish();
            return true;
        }
    }
    return false;
}
    

void GLSLSceneRenderer::Impl::renderCamera(SgCamera* camera, const Affine3& cameraPosition)
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
        viewTransform = T.inverse(Eigen::Isometry);
    } else {
        viewTransform = cameraPosition.inverse(Eigen::Isometry);
    }
    PV = projectionMatrix * viewTransform.matrix();

    modelMatrixStack.clear();
    modelMatrixStack.push_back(Affine3::Identity());
    modelMatrixBuffer.clear();
}


void GLSLSceneRenderer::Impl::beginRendering()
{
    ++renderingFrameId;
    if(renderingFrameId == 0){
        renderingFrameId = 1;
    }
    
    isCheckingUnusedResources = isRenderingPickingImage ? false : doUnusedResourceCheck;

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


void GLSLSceneRenderer::Impl::endRendering()
{
    if(isCheckingUnusedResources){
        currentResourceMap->clear();
        hasValidNextResourceMap = true;
    }
}


void GLSLSceneRenderer::Impl::renderSceneGraphNodes()
{
    currentProgram->initializeFrameRendering();

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


void GLSLSceneRenderer::Impl::renderLights(LightingProgram* program)
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
            bool isCastingShadow =
                isShadowCastingEnabled && (shadowLightIndices.find(i) != shadowLightIndices.end());
            if(program->setLight(lightIndex, light, T, viewTransform, isCastingShadow)){
                ++lightIndex;
            }
        }
    }

    if(lightIndex < program->maxNumLights()){
        SgLight* headLight = self->headLight();
        if(headLight->on()){
            if(program->setLight(
                   lightIndex, headLight, self->currentCameraPosition(), viewTransform, false)){
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


void GLSLSceneRenderer::Impl::renderFog(LightingProgram* program)
{
    SgFog* fog = nullptr;
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
        program->setFog(fog);
    }
    isCurrentFogUpdated = false;
    prevFog = fog;
}


void GLSLSceneRenderer::dispatchToTransparentPhase
(ReferencedPtr object, int id,
 const std::function<void(Referenced* object, const Affine3& position, int id)>& renderingFunction)
{
    int matrixIndex = impl->modelMatrixBuffer.size();
    impl->modelMatrixBuffer.push_back(impl->modelMatrixStack.back());

    impl->transparentRenderingQueue.emplace_back(
        [this, renderingFunction, object, matrixIndex, id](){
            renderingFunction(object, impl->modelMatrixBuffer[matrixIndex], id); });
}


void GLSLSceneRenderer::Impl::renderTransparentObjects()
{
    if(!isRenderingPickingImage){
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
    }

    for(auto& func : transparentRenderingQueue){
        func();
    }

    if(!isRenderingPickingImage){
        glDisable(GL_BLEND);
        glDepthMask(GL_TRUE);
    }

    transparentRenderingQueue.clear();
}


void GLSLSceneRenderer::Impl::renderOverlayObjects()
{
    if(!depthBufferForOverlay){
        glGenRenderbuffers(1, &depthBufferForOverlay);
    }
    if(needToUpdateOverlayDepthBufferSize){
        Array4i vp = self->viewport();
        glBindRenderbuffer(GL_RENDERBUFFER, depthBufferForOverlay);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_STENCIL, vp[2], vp[3]);
        needToUpdateOverlayDepthBufferSize = false;
    }
 
    GLuint defaultDepthBuffer;
    glGetFramebufferAttachmentParameteriv(
        GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME,
        (GLint*)&defaultDepthBuffer);

    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, depthBufferForOverlay);
    glClear(GL_DEPTH_BUFFER_BIT);

    if(isRenderingPickingImage){
        overlayPickIndex0 = pickingNodePathList.size();
    }

    for(auto& func : overlayRenderingQueue){
        func();
    }
    overlayRenderingQueue.clear();

    if(!transparentRenderingQueue.empty()){
        renderTransparentObjects();
    }

    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, defaultDepthBuffer);
}


const Affine3& GLSLSceneRenderer::currentModelTransform() const
{
    return impl->modelMatrixStack.back();
}


const Matrix4& GLSLSceneRenderer::projectionMatrix() const
{
    return impl->projectionMatrix;
}


const Affine3& GLSLSceneRenderer::viewTransform() const
{
    return impl->viewTransform;
}


const Matrix4& GLSLSceneRenderer::viewProjectionMatrix() const
{
    return impl->PV;
}


Matrix4 GLSLSceneRenderer::modelViewMatrix() const
{
    return (impl->viewTransform * impl->modelMatrixStack.back()).matrix();
}


Matrix4 GLSLSceneRenderer::modelViewProjectionMatrix() const
{
    return impl->PV * impl->modelMatrixStack.back().matrix();
}


double GLSLSceneRenderer::projectedPixelSizeRatio(const Vector3& position) const
{
    auto vp = viewport();
    Vector3 p2 = impl->viewTransform * position;
    Vector4 p3(1.0, 0.0, p2.z(), 1.0);
    Vector4 q = impl->projectionMatrix * p3;
    double r = (q.x() / q[3]) * vp[2] / 2.0;
    if(r < 0.0){
        r = 0.0;
    }
    return r;
}


bool GLSLSceneRenderer::isRenderingPickingImage() const
{
    return impl->isRenderingPickingImage;
}


const std::vector<SgNode*>& GLSLSceneRenderer::pickedNodePath() const
{
    return impl->pickedNodePath;
}


const Vector3& GLSLSceneRenderer::pickedPoint() const
{
    return impl->pickedPoint;
}


inline void GLSLSceneRenderer::Impl::setPickColor(int pickIndex)
{
    Vector3f color;
    int id = pickIndex + 1;
    color[0] = (id & 0xff) / 255.0;
    color[1] = ((id >> 8) & 0xff) / 255.0;
    color[2] = ((id >> 16) & 0xff) / 255.0;
    if(isPickingImageOutputEnabled){
        color[2] = 1.0f;
    }
    solidColorProgram.setColor(color);
}
        

/**
   @return the index of the current object in picking
*/
inline int GLSLSceneRenderer::Impl::pushPickNode(SgNode* node, bool doSetColor)
{
    int pickIndex = 0;
    
    if(isRenderingPickingImage){
        pickIndex = pickingNodePathList.size();
        currentNodePath.push_back(node);
        pickingNodePathList.push_back(std::make_shared<SgNodePath>(currentNodePath));
        if(doSetColor){
            setPickColor(pickIndex);
        }
    }

    return pickIndex;
}


inline void GLSLSceneRenderer::Impl::popPickNode()
{
    if(isRenderingPickingImage){
        currentNodePath.pop_back();
    }
}


void GLSLSceneRenderer::renderNode(SgNode* node)
{
    impl->renderingFunctions.dispatch(node);
}


void GLSLSceneRenderer::Impl::renderGroup(SgGroup* group)
{
    pushPickNode(group);
    renderChildNodes(group);
    popPickNode();
}


void GLSLSceneRenderer::renderCustomGroup(SgGroup* group, std::function<void()> traverseFunction)
{
    impl->pushPickNode(group);
    traverseFunction();
    impl->popPickNode();
}


void GLSLSceneRenderer::Impl::renderSwitchableGroup(SgSwitchableGroup* group)
{
    if(group->isTurnedOn()){
        renderGroup(group);
    }
}


void GLSLSceneRenderer::Impl::renderUnpickableGroup(SgUnpickableGroup* group)
{
    if(!isRenderingPickingImage){
        renderGroup(group);
    }
}


void GLSLSceneRenderer::Impl::renderTransform(SgTransform* transform)
{
    if(!transform->empty()){
        Affine3 T;
        transform->getTransform(T);
        modelMatrixStack.push_back(modelMatrixStack.back() * T);
        pushPickNode(transform);

        renderChildNodes(transform);

        popPickNode();
        modelMatrixStack.pop_back();
    }
}


void GLSLSceneRenderer::renderCustomTransform(SgTransform* transform, std::function<void()> traverseFunction)
{
    Affine3 T;
    transform->getTransform(T);
    impl->modelMatrixStack.push_back(impl->modelMatrixStack.back() * T);
    impl->pushPickNode(transform);

    traverseFunction();

    impl->popPickNode();
    impl->modelMatrixStack.pop_back();
}


void GLSLSceneRenderer::Impl::renderAutoScale(SgAutoScale* autoScale)
{
    double r = self->projectedPixelSizeRatio(modelMatrixStack.back().translation());

    if(r > 0.0){
        double s = autoScale->pixelSizeRatio() / r;
        Vector3 scale(s, s, s);
        Affine3 S(scale.asDiagonal());
    
        modelMatrixStack.push_back(modelMatrixStack.back() * S);
        renderGroup(autoScale);
        modelMatrixStack.pop_back();
    }
}
    

VertexResource* GLSLSceneRenderer::Impl::getOrCreateVertexResource(SgObject* obj)
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


void GLSLSceneRenderer::Impl::drawVertexResource(VertexResource* resource, GLenum primitiveMode, const Affine3& position)
{
    currentProgram->setTransform(PV, viewTransform, position, resource->pLocalTransform);
    glBindVertexArray(resource->vao);
    glDrawArrays(primitiveMode, 0, resource->numVertices);
}


void GLSLSceneRenderer::Impl::drawBoundingBox(VertexResource* resource, const BoundingBox& bbox)
{
    if(!resource->boundingBoxLines){
        auto lines = new SgLineSet;
        auto& v = *lines->getOrCreateVertices(8);
        const Vector3f min = bbox.min().cast<float>();
        const Vector3f max = bbox.max().cast<float>();
        v[0] << min.x(), min.y(), min.z();
        v[1] << min.x(), min.y(), max.z();
        v[2] << min.x(), max.y(), min.z();
        v[3] << min.x(), max.y(), max.z();
        v[4] << max.x(), min.y(), min.z();
        v[5] << max.x(), min.y(), max.z();
        v[6] << max.x(), max.y(), min.z();
        v[7] << max.x(), max.y(), max.z();
        lines->addLine(0, 1);
        lines->addLine(0, 2);
        lines->addLine(0, 4);
        lines->addLine(1, 3);
        lines->addLine(1, 5);
        lines->addLine(2, 3);
        lines->addLine(2, 6);
        lines->addLine(3, 7);
        lines->addLine(4, 5);
        lines->addLine(4, 6);
        lines->addLine(5, 7);
        lines->addLine(6, 7);
        lines->getOrCreateMaterial()->setDiffuseColor(Vector3f(1.0f, 1.0f, 1.0f));
        resource->boundingBoxLines = lines;
    }

    renderLineSet(resource->boundingBoxLines);
}
        
        
void GLSLSceneRenderer::Impl::renderShape(SgShape* shape)
{
    SgMesh* mesh = shape->mesh();
    if(mesh && mesh->hasVertices()){
        SgMaterial* material = shape->material();
        if(material && material->transparency() > 0.0 && !isRenderingPickingImage){
            if(!isRenderingShadowMap){
                SgShapePtr shapePtr = shape;
                int matrixIndex = modelMatrixBuffer.size();
                modelMatrixBuffer.push_back(modelMatrixStack.back());
                auto pickIndex = pushPickNode(shape, false);
                transparentRenderingQueue.emplace_back(
                    [this, shapePtr, matrixIndex, pickIndex](){
                        renderShapeMain(shapePtr, modelMatrixBuffer[matrixIndex], pickIndex); });
                popPickNode();
            }
        } else {
            auto pickIndex = pushPickNode(shape, false);
            renderShapeMain(shape, modelMatrixStack.back(), pickIndex);
            popPickNode();
        }
    }
}


void GLSLSceneRenderer::Impl::renderShapeMain(SgShape* shape, const Affine3& position, int pickIndex)
{
    auto mesh = shape->mesh();
    
    if(isRenderingPickingImage){
        setPickColor(pickIndex);
    } else {
        renderMaterial(shape->material());
        if(mesh->hasColors()){
            currentProgram->setVertexColorEnabled(true);
        }

        if(currentMaterialLightingProgram){
            bool isTextureValid = false;
            if(isTextureBeingRendered){
                if(auto texture = shape->texture()){
                    isTextureValid = renderTexture(texture);
                }
            }
            currentMaterialLightingProgram->setTextureEnabled(isTextureValid);
        }
    }

    VertexResource* resource = getOrCreateVertexResource(mesh);
    if(!resource->isValid()){
        makeVertexBufferObjects(shape, resource);
    }
    if(isBoundingBoxRenderingMode){
        drawBoundingBox(resource, mesh->boundingBox());
    } else {
        if(!isRenderingShadowMap){
            applyCullingMode(mesh);
        }
        drawVertexResource(resource, GL_TRIANGLES, position);

        if(isNormalVisualizationEnabled && isRenderingVisibleImage && resource->normalVisualization){
            renderLineSet(resource->normalVisualization);
        }
    }
}


void GLSLSceneRenderer::Impl::applyCullingMode(SgMesh* mesh)
{
    if(!stateFlag[CULL_FACE]){
        bool enableCullFace;
        switch(backFaceCullingMode){
        case GLSceneRenderer::ENABLE_BACK_FACE_CULLING:
            enableCullFace = mesh->isSolid();
            break;
        case GLSceneRenderer::DISABLE_BACK_FACE_CULLING:
            enableCullFace = false;
            break;
        case GLSceneRenderer::FORCE_BACK_FACE_CULLING:
        default:
            enableCullFace = true;
            break;
        }
        if(enableCullFace){
            glEnable(GL_CULL_FACE);
        } else {
            glDisable(GL_CULL_FACE);
        }
        isCullFaceEnabled = enableCullFace;
        stateFlag[CULL_FACE] = true;
        
    } else if(backFaceCullingMode == GLSceneRenderer::ENABLE_BACK_FACE_CULLING){
        if(mesh->isSolid()){
            if(!isCullFaceEnabled){
                glEnable(GL_CULL_FACE);
                isCullFaceEnabled = true;
            }
        } else {
            if(isCullFaceEnabled){
                glDisable(GL_CULL_FACE);
                isCullFaceEnabled = false;
            }
        }
    }
}


void GLSLSceneRenderer::Impl::renderMaterial(const SgMaterial* material)
{
    currentProgram->setMaterial(material ? material : defaultMaterial);
}


bool GLSLSceneRenderer::Impl::renderTexture(SgTexture* texture)
{
    SgImage* sgImage = texture->image();
    if(!sgImage || sgImage->empty()){
        return false;
    }

    auto p = currentResourceMap->find(sgImage);
    TextureResource* resource;
    if(p != currentResourceMap->end()){
        resource = static_cast<TextureResource*>(p->second.get());
        if(resource->isLoaded){
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, resource->textureId);
            glBindSampler(0, resource->samplerId);
            if(resource->isImageUpdateNeeded){
                loadTextureImage(resource, sgImage->constImage());
            }
        }
    } else {
        resource = new TextureResource;
        currentResourceMap->insert(GLResourceMap::value_type(sgImage, resource));

        GLuint samplerId;
        glActiveTexture(GL_TEXTURE0);
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

    return resource->isLoaded;
}


bool GLSLSceneRenderer::Impl::loadTextureImage(TextureResource* resource, const Image& image)
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


void GLSLSceneRenderer::Impl::makeVertexBufferObjects(SgShape* shape, VertexResource* resource)
{
    auto mesh = shape->mesh();

    if(isLowMemoryConsumptionRenderingBeingProcessed){
        writeMeshVerticesNormalizedShort(mesh, resource);
    } else {
        writeMeshVerticesFloat(mesh, resource);
    }

    if(isLowMemoryConsumptionRenderingBeingProcessed){
        writeMeshNormalsByte(mesh, resource);
        //writeMeshNormalsPacked(mesh, resource);
    } else if(USE_GL_FLOAT_FOR_NORMALS){
        writeMeshNormalsFloat(mesh, resource);
    } else {
        writeMeshNormalsShort(mesh, resource);
    } 

    auto texture = shape->texture();
    if(texture && mesh->hasTexCoords() && isTextureBeingRendered){
        if(isLowMemoryConsumptionRenderingBeingProcessed){
            writeMeshTexCoordsHalfFloat(mesh, texture, resource);
        } else {
            writeMeshTexCoordsFloat(mesh, texture, resource);
        }
    }
    
    if(mesh->hasColors()){
        writeMeshColors(mesh, resource);
    }
}


template<typename value_type, GLenum gltype, GLboolean normalized, class VertexArrayWrapper>
void GLSLSceneRenderer::Impl::writeMeshVerticesSub
(SgMesh* mesh, VertexResource* resource, VertexArrayWrapper& vertices)
{
    const auto& orgVertices = *mesh->vertices();
    auto& triangleVertices = mesh->triangleVertices();
    const int totalNumVertices = triangleVertices.size();
    const int numTriangles = mesh->numTriangles();
    resource->numVertices = totalNumVertices;

    int faceVertexIndex = 0;
    
    vertices.array.reserve(totalNumVertices);
    
    for(int i=0; i < numTriangles; ++i){
        for(int j=0; j < 3; ++j){
            const int orgVertexIndex = triangleVertices[faceVertexIndex++];
            vertices.append(orgVertices[orgVertexIndex]);
        }
    }

    {
        LockVertexArrayAPI lock;
        glBindVertexArray(resource->vao);
        glBindBuffer(GL_ARRAY_BUFFER, resource->newBuffer());
        glVertexAttribPointer((GLuint)0, 3, gltype, normalized, 0, ((GLubyte*)NULL + (0)));
    }
    auto size = vertices.array.size() * sizeof(value_type);
    glBufferData(GL_ARRAY_BUFFER, size, vertices.array.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
}


void GLSLSceneRenderer::Impl::writeMeshVerticesFloat(SgMesh* mesh, VertexResource* resource)
{
    struct VertexArrayWrapper {
        SgVertexArray array;
        void append(const Vector3f& v){ array.push_back(v); }
    } vertices;

    writeMeshVerticesSub<Vector3f, GL_FLOAT, GL_FALSE>(mesh, resource, vertices);
}


void GLSLSceneRenderer::Impl::writeMeshVerticesNormalizedShort(SgMesh* mesh, VertexResource* resource)
{
    /**
       GLshort type is used for storing vertex positions.
       Each value's range is [ -32768, 32767 ], which corresponds to normalized range [ -1, 1 ]
       that covers all the vertex positions.
       In the vertex shader, [ -1, 1 ] value is converted to the original position
    */
    typedef Eigen::Matrix<GLshort,3,1> Vector3s;

    struct VertexArrayWrapper {
        vector<Vector3s> array;
        Vector3f r;
        Vector3f c;
        VertexArrayWrapper(const Vector3f& ratio, const Vector3f& center)
            : r(ratio), c(center)
        {  }
        void append(const Vector3f& v){
            array.emplace_back(
                r.x() * (v.x() - c.x()),
                r.y() * (v.y() - c.y()),
                r.z() * (v.z() - c.z()));
        }
    };
            
    auto& bbox = mesh->boundingBox();
    if(!bbox){
        mesh->updateBoundingBox();
    }
    const Vector3 c = bbox.center();
    const Vector3 hs =  0.5 * bbox.size();

    resource->localTransform <<
        hs.x(), 0.0,    0.0,    c.x(),
        0.0,    hs.y(), 0.0,    c.y(),
        0.0,    0.0,    hs.z(), c.z(),
        0.0,    0.0,    0.0,    1.0;
    resource->pLocalTransform = &resource->localTransform;

    Vector3f ratio(32767.0 / hs.x(), 32767.0 / hs.y(), 32767.0 / hs.z());
    VertexArrayWrapper vertices(ratio, c.cast<float>());

    writeMeshVerticesSub<Vector3s, GL_SHORT, GL_TRUE>(mesh, resource, vertices);
}


template<typename value_type, GLenum gltype, GLint glsize, GLboolean normalized, class NormalArrayWrapper>
bool GLSLSceneRenderer::Impl::writeMeshNormalsSub
(SgMesh* mesh, VertexResource* resource, NormalArrayWrapper& normals)
{
    bool ready = false;
    
    auto& triangleVertices = mesh->triangleVertices();
    const int totalNumVertices = triangleVertices.size();
    const int numTriangles = mesh->numTriangles();
    
    normals.array.reserve(totalNumVertices);

    if(!defaultSmoothShading){
        // flat shading
        const auto& orgVertices = *mesh->vertices();
        for(int i=0; i < numTriangles; ++i){
            SgMesh::TriangleRef triangle = mesh->triangle(i);
            const Vector3f e1 = orgVertices[triangle[1]] - orgVertices[triangle[0]];
            const Vector3f e2 = orgVertices[triangle[2]] - orgVertices[triangle[0]];
            const Vector3f normal = e1.cross(e2).normalized();
            for(int j=0; j < 3; ++j){
                normals.append(normal);
            }
        }
        ready = true;

    } else if(mesh->normals()){
        const auto& orgNormals = *mesh->normals();
        const auto& normalIndices = mesh->normalIndices();
        int faceVertexIndex = 0;
        if(normalIndices.empty()){
            for(int i=0; i < numTriangles; ++i){
                for(int j=0; j < 3; ++j){
                    const int orgVertexIndex = triangleVertices[faceVertexIndex++];
                    normals.append(orgNormals[orgVertexIndex]);
                }
            }
        } else {
            for(int i=0; i < numTriangles; ++i){
                for(int j=0; j < 3; ++j){
                    const int normalIndex = normalIndices[faceVertexIndex++];
                    normals.append(orgNormals[normalIndex]);
                }
            }
        }
        ready = true;
    }

    if(ready){
        {
            LockVertexArrayAPI lock;
            glBindBuffer(GL_ARRAY_BUFFER, resource->newBuffer());
            glVertexAttribPointer((GLuint)1, glsize, gltype, normalized, 0, ((GLubyte*)NULL + (0)));
        }
        glBufferData(GL_ARRAY_BUFFER, normals.array.size() * sizeof(value_type), normals.array.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(1);
    }
    
    if(isNormalVisualizationEnabled){
        auto lines = new SgLineSet;
        auto lineVertices = lines->getOrCreateVertices();
        const auto& orgVertices = *mesh->vertices();
        int vertexIndex = 0;
        for(int i=0; i < numTriangles; ++i){
            for(int j=0; j < 3; ++j){
                const int orgVertexIndex = triangleVertices[vertexIndex];
                auto& v = orgVertices[orgVertexIndex];
                lineVertices->push_back(v);
                lineVertices->push_back(v + normals.get(vertexIndex) * normalVisualizationLength);
                lines->addLine(vertexIndex * 2, vertexIndex * 2 + 1);
                ++vertexIndex;
            }
        }
        lines->setMaterial(normalVisualizationMaterial);
        resource->normalVisualization = lines;
    }

    return ready;
}    
    

void GLSLSceneRenderer::Impl::writeMeshNormalsFloat(SgMesh* mesh, VertexResource* resource)
{
    struct NormalArrayWrapper {
        SgNormalArray array;
        void append(const Vector3f& n){ array.push_back(n); }
        Vector3f get(int index){ return array[index]; }
    } normals;
            
    writeMeshNormalsSub<Vector3f, GL_FLOAT, 3, GL_FALSE>(mesh, resource, normals);
}


void GLSLSceneRenderer::Impl::writeMeshNormalsShort(SgMesh* mesh, VertexResource* resource)
{
    typedef Eigen::Matrix<GLshort,3,1> Vector3s;

    struct NormalArrayWrapper {
        vector<Vector3s> array;
        void append(const Vector3f& n){
            array.push_back((32767.0f * n).cast<GLshort>());
        }
        Vector3f get(int index){
            return array[index].cast<float>() / 32767.0f;
        }
    } normals;
            
    writeMeshNormalsSub<Vector3s, GL_SHORT, 3, GL_TRUE>(mesh, resource, normals);
}


void GLSLSceneRenderer::Impl::writeMeshNormalsByte(SgMesh* mesh, VertexResource* resource)
{
    typedef Eigen::Matrix<GLbyte,3,1> Vector3b;

    struct NormalArrayWrapper {
        vector<Vector3b> array;
        void append(const Vector3f& n){
            array.push_back((127.0f * n).cast<GLbyte>());
        }
        Vector3f get(int index){
            return array[index].cast<float>() / 127.0f;
        }
    } normals;
            
    writeMeshNormalsSub<Vector3b, GL_BYTE, 3, GL_TRUE>(mesh, resource, normals);
}


/**
   The following data type implementation results in wrong normals when
   the code is compiled by VC++2017 with the AVX2 option.
   VC++2015 and GCC do not cause such a problem.
*/
void GLSLSceneRenderer::Impl::writeMeshNormalsPacked(SgMesh* mesh, VertexResource* resource)
{
    struct NormalArrayWrapper {
        vector<uint32_t> array;
        void append(const Vector3f& n){
            const uint32_t xs = n.x() < 0.0f;
            const uint32_t ys = n.y() < 0.0f;
            const uint32_t zs = n.z() < 0.0f;
            array.push_back(
                uint32_t(
                    zs << 29 | ((uint32_t)(n.z() * 511 + (zs << 9)) & 511) << 20 |
                    ys << 19 | ((uint32_t)(n.y() * 511 + (ys << 9)) & 511) << 10 |
                    xs << 9  | ((uint32_t)(n.x() * 511 + (xs << 9)) & 511)));
        }
        Vector3f get(int index){
            auto packed = array[index];
            Vector3f v;
            for(int i=0; i < 3; ++i){
                if(packed & 512){ // minus
                    v[i] = (static_cast<int>(packed & 511) - 512) / 511.0f;
                } else { // plus
                    v[i] = (packed & 511) / 511.0f;
                }
                packed >>= 10;
            }
            return v;
        }
    } normals;
            
    writeMeshNormalsSub<uint32_t, GL_INT_2_10_10_10_REV, 4, GL_TRUE>(mesh, resource, normals);
}


template<typename value_type, GLenum gltype, GLboolean normalized, class TexCoordArrayWrapper>
void GLSLSceneRenderer::Impl::writeMeshTexCoordsSub
(SgMesh* mesh, SgTexture* texture, VertexResource* resource, TexCoordArrayWrapper& texCoords)
{
    auto& triangleVertices = mesh->triangleVertices();
    const int totalNumVertices = triangleVertices.size();
    SgTexCoordArrayPtr pOrgTexCoords;
    const auto& texCoordIndices = mesh->texCoordIndices();

    auto tt = texture->textureTransform();
    if(!tt){
        pOrgTexCoords = mesh->texCoords();
    } else {
        Eigen::Rotation2Df R(tt->rotation());
        const auto& c = tt->center();
        Eigen::Translation<float, 2> C(c.x(), c.y());
        const auto& t = tt->translation();
        Eigen::Translation<float, 2> T(t.x(), t.y());
        const auto s = tt->scale().cast<float>();
        Eigen::Affine2f M = T * C * R * Eigen::Scaling(s.x(), s.y()) * C.inverse();

        const auto& orgTexCoords = *mesh->texCoords();
        const size_t n = orgTexCoords.size();
        pOrgTexCoords = new SgTexCoordArray(n);
        for(size_t i=0; i < n; ++i){
            (*pOrgTexCoords)[i] = M * orgTexCoords[i];
        }
    }

    texCoords.array.reserve(totalNumVertices);
    const int numTriangles = mesh->numTriangles();
    int faceVertexIndex = 0;
    
    if(texCoordIndices.empty()){
        for(int i=0; i < numTriangles; ++i){
            for(int j=0; j < 3; ++j){
                const int orgVertexIndex = triangleVertices[faceVertexIndex++];
                texCoords.append((*pOrgTexCoords)[orgVertexIndex]);
            }
        }
    } else {
        for(int i=0; i < numTriangles; ++i){
            for(int j=0; j < 3; ++j){
                const int texCoordIndex = texCoordIndices[faceVertexIndex++];
                texCoords.append((*pOrgTexCoords)[texCoordIndex]);
            }
        }
    }
    {
        LockVertexArrayAPI lock;
        glBindBuffer(GL_ARRAY_BUFFER, resource->newBuffer());
        glVertexAttribPointer((GLuint)2, 2, gltype, normalized, 0, 0);
    }
    auto size = texCoords.array.size() * sizeof(value_type);
    glBufferData(GL_ARRAY_BUFFER, size, texCoords.array.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(2);
}


void GLSLSceneRenderer::Impl::writeMeshTexCoordsFloat
(SgMesh* mesh, SgTexture* texture, VertexResource* resource)
{
    struct TexCoordArrayWrapper {
        SgTexCoordArray array;
        void append(const Vector2f& uv){
            array.push_back(uv);
        }
    } texCoords;

    writeMeshTexCoordsSub<Vector2f, GL_FLOAT, GL_FALSE>(mesh, texture, resource, texCoords);
}


void GLSLSceneRenderer::Impl::writeMeshTexCoordsHalfFloat
(SgMesh* mesh, SgTexture* texture, VertexResource* resource)
{
    typedef Eigen::Matrix<GLhalf,2,1> Vector2h;

    struct TexCoordArrayWrapper {
        vector<Vector2h> array;
        /**
           Qt provides 16-bit floating point support since version 5.11,
           and the qFloatToFloat16 function can be used by including the <QFloat16> header.
           It may be better to use it the corresponding Qt version is available.
        */
        GLhalf toHalf(const float& value){
            uint32_t x = *((uint32_t*)&value);
            uint32_t e = x & 0x7f800000;
            if(e == 0 || e < 0x38800000){
                return 0;
            } else if(e >0x47000000){
                return 0x7bff;
            }
            return ((x >> 16) & 0x8000) | ((x & 0x7fffffff) >> 13) - 0x1c000;
        }
        void append(const Vector2f& uv){
            array.emplace_back(toHalf(uv[0]), toHalf(uv[1]));
        }
    } texCoords;

    writeMeshTexCoordsSub<Vector2h, GL_HALF_FLOAT, GL_FALSE>(mesh, texture, resource, texCoords);
}


/**
   This is an experimental implementaion.
   When normalized unsigned short values are used for texture coordinates,
   the coordinate must be within [ 0, 1.0 ]. However, data with out-of-range
   values is common, and such data cannot be rendererd correctly with this implementation.
   As an alternative of lightweight implementation, writeMeshTexCoordsHalfFloat is available.
*/
void GLSLSceneRenderer::Impl::writeMeshTexCoordsUnsignedShort
(SgMesh* mesh, SgTexture* texture, VertexResource* resource)
{
    typedef Eigen::Matrix<GLushort,2,1> Vector2us;
    
    struct TexCoordArrayWrapper {
        vector<Vector2us> array;
        float clamp(float v){
            if(v > 1.0f){
                return 1.0f;
            } else if(v < 0.0f){
                return 0.0f;
            }
            return v;
        }
        void append(const Vector2f& uv){
            array.emplace_back(65535.0f * clamp(uv[0]), 65535.0f * clamp(uv[1]));
        }
    } texCoords;

    writeMeshTexCoordsSub<Vector2us, GL_UNSIGNED_SHORT, GL_TRUE>(mesh, texture, resource, texCoords);
}


void GLSLSceneRenderer::Impl::writeMeshColors(SgMesh* mesh, VertexResource* resource)
{
    auto& triangleVertices = mesh->triangleVertices();
    const int totalNumVertices = triangleVertices.size();
    const auto& orgColors = *mesh->colors();
    const auto& colorIndices = mesh->colorIndices();

    typedef Eigen::Array<GLubyte,3,1> Color;
    vector<Color> colors;
    colors.reserve(totalNumVertices);
    
    const int numTriangles = mesh->numTriangles();
    int faceVertexIndex = 0;

    if(colorIndices.empty()){
        for(int i=0; i < numTriangles; ++i){
            for(int j=0; j < 3; ++j){
                const int orgVertexIndex = triangleVertices[faceVertexIndex++];
                Vector3f c = 255.0f * orgColors[orgVertexIndex];
                colors.emplace_back(c[0], c[1], c[2]);
            }
        }
    } else {
        for(int i=0; i < numTriangles; ++i){
            for(int j=0; j < 3; ++j){
                const int colorIndex = colorIndices[faceVertexIndex++];
                Vector3f c = 255.0f * orgColors[colorIndex];
                colors.emplace_back(c[0], c[1], c[2]);
            }
        }
    }

    {
        LockVertexArrayAPI lock;
        glBindBuffer(GL_ARRAY_BUFFER, resource->newBuffer());
        glVertexAttribPointer((GLuint)3, 3, GL_UNSIGNED_BYTE, GL_TRUE, 0, ((GLubyte*)NULL + (0)));
    }
    glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(Color), colors.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(3);
}
    

void GLSLSceneRenderer::Impl::renderPointSet(SgPointSet* pointSet)
{
    if(!pointSet->hasVertices()){
        return;
    }

    ScopedShaderProgramActivator programActivator(solidColorProgram, this);

    const double s = pointSet->pointSize();
    if(s > 0.0){
        setPointSize(s);
    } else {
        setPointSize(defaultPointSize);
    }
    
    renderPlot(pointSet, GL_POINTS,
               [pointSet]() -> SgVertexArrayPtr { return pointSet->vertices(); });
}


void GLSLSceneRenderer::Impl::renderPlot
(SgPlot* plot, GLenum primitiveMode, std::function<SgVertexArrayPtr()> getVertices)
{
    pushPickNode(plot);

    bool hasColors = plot->hasColors();
    
    if(isRenderingPickingImage){
        solidColorProgram.setVertexColorEnabled(false);
    } else {
        if(!hasColors){
            renderMaterial(plot->material());
        } else {
            solidColorProgram.setVertexColorEnabled(true);
        }
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
            typedef Eigen::Array<GLubyte,3,1> Color;
            vector<Color> colors;
            colors.reserve(n);
            const SgColorArray& orgColors = *plot->colors();
            const SgIndexArray& colorIndices = plot->colorIndices();
            size_t i = 0;
            if(plot->colorIndices().empty()){
                const size_t m = std::min(n, orgColors.size());
                while(i < m){
                    Vector3f c = 255.0f * orgColors[i];
                    colors.emplace_back(c[0], c[1], c[2]);
                    ++i;
                }
            } else {
                const size_t m = std::min(n, colorIndices.size());
                size_t i = 0;
                while(i < m){
                    Vector3f c = 255.0f * orgColors[colorIndices[i]];
                    colors.emplace_back(c[0], c[1], c[2]);
                    ++i;
                }
            }
            if(i < n){
                const auto& c = colors.back();
                while(i < n){
                    colors.push_back(c);
                    ++i;
                }
            }
            {
                LockVertexArrayAPI lock;
                glBindBuffer(GL_ARRAY_BUFFER, resource->newBuffer());
                glVertexAttribPointer((GLuint)3, 3, GL_UNSIGNED_BYTE, GL_TRUE, 0, ((GLubyte*)NULL +(0)));
            }
            glBufferData(GL_ARRAY_BUFFER, n * sizeof(Color), colors.data(), GL_STATIC_DRAW);
            glEnableVertexAttribArray(3);
        }
    }        

    drawVertexResource(resource, primitiveMode, modelMatrixStack.back());
    
    popPickNode();
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


void GLSLSceneRenderer::Impl::renderLineSet(SgLineSet* lineSet)
{
    if(isRenderingShadowMap){
        return;
    }
    
    if(!lineSet->hasVertices() || lineSet->numLines() <= 0){
        return;
    }

    ScopedShaderProgramActivator programActivator(solidColorProgram, this);
    
    const double w = lineSet->lineWidth();
    if(w > 0.0){
        setLineWidth(w);
    } else {
        setLineWidth(defaultLineWidth);
    }

    renderPlot(lineSet, GL_LINES,
               [lineSet](){ return getLineSetVertices(lineSet); });
}


void GLSLSceneRenderer::Impl::renderOverlay(SgOverlay* overlay)
{
    if(isRenderingVisibleImage || isRenderingPickingImage){
        int matrixIndex = modelMatrixBuffer.size();
        modelMatrixBuffer.push_back(modelMatrixStack.back());
        const auto& nodePath = isRenderingPickingImage ? currentNodePath : emptyNodePath;
        overlayRenderingQueue.emplace_back(
            [this, overlay, matrixIndex, nodePath](){
                renderOverlayMain(overlay, modelMatrixBuffer[matrixIndex], nodePath); });
    }
}


void GLSLSceneRenderer::Impl::renderOverlayMain(SgOverlay* overlay, const Affine3& T, const SgNodePath& nodePath)
{
    if(isRenderingPickingImage){
        currentNodePath = nodePath;
    }
    ScopedShaderProgramActivator programActivator(solidColorProgram, this);
    modelMatrixStack.push_back(T);
    renderGroup(overlay);
    modelMatrixStack.pop_back();
}


void GLSLSceneRenderer::Impl::renderViewportOverlay(SgViewportOverlay* overlay)
{
    if(isRenderingVisibleImage){
        overlayRenderingQueue.emplace_back(
            [this, overlay](){ renderViewportOverlayMain(overlay); });
    }
}


void GLSLSceneRenderer::Impl::renderViewportOverlayMain(SgViewportOverlay* overlay)
{
    const Matrix4 PV0 = PV;
    SgViewportOverlay::ViewVolume v;
    int x, y, width, height;
    self->getViewport(x, y, width, height);
    overlay->calcViewVolume(width, height, v);
    self->getOrthographicProjectionMatrix(v.left, v.right, v.bottom, v.top, v.zNear, v.zFar, PV);

    pickedNodePath.clear();
    renderOverlayMain(overlay, Affine3::Identity(), emptyNodePath);

    PV = PV0;
    modelMatrixStack.pop_back();
}


void GLSLSceneRenderer::Impl::renderOutline(SgOutline* outline)
{
    renderGroup(outline);

    if(!isRenderingPickingImage && isRenderingVisibleImage){
        int matrixIndex = modelMatrixBuffer.size();
        modelMatrixBuffer.push_back(modelMatrixStack.back());
        overlayRenderingQueue.emplace_back(
            [this, outline, matrixIndex](){
                renderOutlineEdge(outline, modelMatrixBuffer[matrixIndex]); });
    }
}


void GLSLSceneRenderer::Impl::renderOutlineEdge(SgOutline* outline, const Affine3& T)
{
    modelMatrixStack.push_back(T);

    glClearStencil(0);
    glClear(GL_STENCIL_BUFFER_BIT);
    glEnable(GL_STENCIL_TEST);
    glDisable(GL_DEPTH_TEST);
    
    {
        ScopedShaderProgramActivator programActivator(nolightingProgram, this);

        glStencilFunc(GL_ALWAYS, 1, -1);
        glStencilOp(GL_KEEP, GL_REPLACE, GL_REPLACE);
        glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
        
        renderChildNodes(outline);
    }

    {
        ScopedShaderProgramActivator programActivator(solidColorProgram, this);
        solidColorProgram.setColor(outline->color());
        solidColorProgram.setColorChangable(false);

        glStencilFunc(GL_NOTEQUAL, 1, -1);
        glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

        float orgLineWidth = lineWidth;
        setLineWidth(outline->lineWidth()*2+1);

        GLint polygonMode;
        glGetIntegerv(GL_POLYGON_MODE, &polygonMode);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_FALSE);

        renderChildNodes(outline);

        setLineWidth(orgLineWidth);
        glPolygonMode(GL_FRONT_AND_BACK, polygonMode);
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

        solidColorProgram.setColorChangable(true);
    }

    glDisable(GL_STENCIL_TEST);
    glEnable(GL_DEPTH_TEST);

    modelMatrixStack.pop_back();
}


void GLSLSceneRenderer::Impl::renderLightweightRenderingGroup(SgLightweightRenderingGroup* group)
{
    if(isRenderingShadowMap){
        return;
    }

    bool wasLightweightRenderingBeingProcessed = isLightweightRenderingBeingProcessed;
    bool wasLowMemoryConsumptionRenderingBeingProcessed = isLowMemoryConsumptionRenderingBeingProcessed;
    bool wasTextureBeingRendered = isTextureBeingRendered;
    bool wasBoundingBoxRenderingMode = isBoundingBoxRenderingMode;

    bool pushed = false;
    if(currentLightingProgram){
        pushProgram(minimumLightingProgram);
        if(!isLightweightRenderingBeingProcessed){
            renderLights(&minimumLightingProgram);
        }
        pushed = true;
    }

    isLightweightRenderingBeingProcessed = true;
    isLowMemoryConsumptionRenderingBeingProcessed = true;
    isTextureBeingRendered = false;
    if(isBoundingBoxRenderingForLightweightRenderingGroupEnabled){
        isBoundingBoxRenderingMode = true;
    }

    renderChildNodes(group);

    if(pushed){
        popProgram();
    }

    isLightweightRenderingBeingProcessed = wasLightweightRenderingBeingProcessed;
    isLowMemoryConsumptionRenderingBeingProcessed = wasLowMemoryConsumptionRenderingBeingProcessed;
    isTextureBeingRendered = wasTextureBeingRendered;
    isBoundingBoxRenderingMode = wasBoundingBoxRenderingMode;
}


void GLSLSceneRenderer::Impl::clearGLState()
{
    std::fill(stateFlag.begin(), stateFlag.end(), false);
    pointSize = defaultPointSize;    
    lineWidth = defaultLineWidth;
}


void GLSLSceneRenderer::setColor(const Vector3f& color)
{
    impl->solidColorProgram.setColor(color);
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
    impl->phongShadowLightingProgram.setShadowAntiAliasingEnabled(on);
}


void GLSLSceneRenderer::Impl::setPointSize(float size)
{
    if(!stateFlag[POINT_SIZE] || pointSize != size){
        float s = isRenderingPickingImage ? std::max(size, MinLineWidthForPicking) : size;
        solidColorProgram.setPointSize(s);
        pointSize = s;
        stateFlag[POINT_SIZE] = true;
    }
}


void GLSLSceneRenderer::Impl::setLineWidth(float width)
{
    if(!stateFlag[LINE_WIDTH] || lineWidth != width){
        if(isRenderingPickingImage){
            glLineWidth(std::max(width, MinLineWidthForPicking));
        } else {
            glLineWidth(width);
        }
        lineWidth = width;
        stateFlag[LINE_WIDTH] = true;
    }
}


void GLSLSceneRenderer::setLightingMode(int mode)
{
    if(mode != impl->lightingMode){
        impl->lightingMode = mode;
        requestToClearResources();
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
    if(on != impl->isTextureEnabled){
        impl->isTextureEnabled = on;
        requestToClearResources();
    }
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


void GLSLSceneRenderer::setBackFaceCullingMode(int mode)
{
    impl->backFaceCullingMode = mode;
}


int GLSLSceneRenderer::backFaceCullingMode() const
{
    return impl->backFaceCullingMode;
}


void GLSLSceneRenderer::setBoundingBoxRenderingForLightweightRenderingGroupEnabled(bool on)
{
    impl->isBoundingBoxRenderingForLightweightRenderingGroupEnabled = on;
}


void GLSLSceneRenderer::setLowMemoryConsumptionMode(bool on)
{
    if(impl->isLowMemoryConsumptionMode != on){
        impl->isLowMemoryConsumptionMode = on;
        requestToClearResources();
    }
}


bool GLSLSceneRenderer::isShadowCastingAvailable() const
{
    return impl->isShadowCastingEnabled;
}
