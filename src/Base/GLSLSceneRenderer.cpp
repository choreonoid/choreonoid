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

    VertexResource(GLSLSceneRendererImpl* renderer, SgObject* obj)
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
    GLSLSceneRendererImpl* renderer;
    ShaderProgram* prevProgram;
    NolightingProgram* prevNolightingProgram;
    LightingProgram* prevLightingProgram;
    MaterialLightingProgram* prevMaterialLightingProgram;
    bool changed;
    
public:
    ScopedShaderProgramActivator(ShaderProgram& program, GLSLSceneRendererImpl* renderer);
    ScopedShaderProgramActivator(ScopedShaderProgramActivator&&) noexcept;
    ScopedShaderProgramActivator(const ScopedShaderProgramActivator&) = delete;
    ScopedShaderProgramActivator& operator=(const ScopedShaderProgramActivator&) = delete;
    ~ScopedShaderProgramActivator();
};

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
    int pickingBufferWidth;
    int pickingBufferHeight;

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

    bool isActuallyRendering;
    bool isPicking;
    bool isPickingBufferImageOutputEnabled;
    bool isShadowCastingEnabled;
    bool isRenderingShadowMap;
    bool isLightweightRenderingBeingProcessed;
    bool isLowMemoryConsumptionMode;
    bool isLowMemoryConsumptionRenderingBeingProcessed;
    bool isBoundingBoxRenderingMode;
    bool isBoundingBoxRenderingForLightweightRenderingGroupEnabled;
    
    Affine3Array modelMatrixStack; // stack of the model matrices
    Affine3 viewTransform;
    Matrix4 projectionMatrix;
    Matrix4 PV;

    vector<function<void()>> postRenderingFunctions;

    struct RenderingFunction {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Affine3 position; // corresponds to the model matrix
        ReferencedPtr object; // object to render
        int id; // some id related with the object
        typedef function<void(Referenced* object, Affine3& position, int id)> Func;
        Func func;

        RenderingFunction(Referenced* object, const Affine3& position, int id, const Func& func)
            : position(position), object(object), id(id), func(func) { }
        void operator()(){ func(object, position, id); }
    };
    vector<RenderingFunction, Eigen::aligned_allocator<RenderingFunction>> transparentRenderingFunctions;
    
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
    void endRendering();
    void renderSceneGraphNodes();
    void pushProgram(ShaderProgram& program);
    void popProgram();
    inline void setPickColor(int id);
    inline int pushPickId(SgNode* node, bool doSetColor = true);
    void popPickId();
    void renderGroup(SgGroup* group);
    void renderTransform(SgTransform* transform);
    void renderSwitch(SgSwitch* node);
    void renderUnpickableGroup(SgUnpickableGroup* group);
    VertexResource* getOrCreateVertexResource(SgObject* obj);
    void drawVertexResource(VertexResource* resource, GLenum primitiveMode, const Affine3& position);
    void drawBoundingBox(VertexResource* resource, const BoundingBox& bbox);
    void renderShape(SgShape* shape);
    void renderShapeMain(SgShape* shape, const Affine3& position, int pickId);
    void applyCullingMode(SgMesh* mesh);
    void renderPointSet(SgPointSet* pointSet);        
    void renderLineSet(SgLineSet* lineSet);        
    void renderOverlay(SgOverlay* overlay);
    void renderOutlineGroup(SgOutlineGroup* outline);
    void renderOutlineGroupMain(SgOutlineGroup* outline, const Affine3& T);
    void renderLightweightRenderingGroup(SgLightweightRenderingGroup* group);
    void flushNolightingTransformMatrices();
    void renderTransparentObjects();
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
    pickingBufferWidth = 0;
    pickingBufferHeight = 0;

    currentProgram = nullptr;
    currentNolightingProgram = nullptr;
    currentLightingProgram = nullptr;
    currentMaterialLightingProgram = nullptr;

    isActuallyRendering = false;
    isPicking = false;
    isPickingBufferImageOutputEnabled = false;
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

    backFaceCullingMode = GLSceneRenderer::ENABLE_BACK_FACE_CULLING;

    modelMatrixStack.reserve(16);
    viewTransform.setIdentity();
    projectionMatrix.setIdentity();

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

    pickedPoint.setZero();

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
    GLSceneRenderer::setOutputStream(os);
    impl->os_ = &os;
}


void GLSLSceneRendererImpl::updateDefaultFramebufferObject()
{
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, reinterpret_cast<GLint*>(&defaultFBO));
    phongShadowLightingProgram.setDefaultFramebufferObject(defaultFBO);
}


bool GLSLSceneRenderer::initializeGL()
{
    return impl->initializeGL();
}


bool GLSLSceneRendererImpl::initializeGL()
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
    glViewport(x, y, width, height);
    updateViewportInformation(x, y, width, height);
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
(ShaderProgram& program, GLSLSceneRendererImpl* renderer)
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


void GLSLSceneRendererImpl::pushProgram(ShaderProgram& program)
{
    programStack.emplace_back(program, this);
}

void GLSLSceneRenderer::pushShaderProgram(ShaderProgram& program)
{
    impl->pushProgram(program);
}


void GLSLSceneRendererImpl::popProgram()
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


void GLSLSceneRendererImpl::doRender()
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


void GLSLSceneRendererImpl::setupFullLightingRendering()
{
    isTextureBeingRendered = isTextureEnabled;

    if(shadowLightIndices.empty() || !isShadowCastingEnabled){
        // Same as NORMAL_LIGHTING
        pushProgram(phongLightingProgram);
            
    } else {
        auto& program = phongShadowLightingProgram;
        Array4i vp = self->viewport();
        int w, h;
        program.getShadowMapSize(w, h);
        self->setViewport(0, 0, w, h);
        pushProgram(program.shadowMapProgram());
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
    
        program.activateMainRenderingPass();
        pushProgram(program);
    }
}


bool GLSLSceneRenderer::doPick(int x, int y)
{
    return impl->doPick(x, y);
}


bool GLSLSceneRendererImpl::doPick(int x, int y)
{
    int vx, vy, width, height;
    self->getViewport(vx, vy, width, height);

    if(!fboForPicking){
        glGenFramebuffers(1, &fboForPicking);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, fboForPicking);

    if(width != pickingBufferWidth || height != pickingBufferHeight){
        // color buffer
        if(colorBufferForPicking){
            glDeleteRenderbuffers(1, &colorBufferForPicking);
        }
        glGenRenderbuffers(1, &colorBufferForPicking);
        glBindRenderbuffer(GL_RENDERBUFFER, colorBufferForPicking);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA, width, height);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, colorBufferForPicking);
            
        // depth buffer
        if(depthBufferForPicking){
            glDeleteRenderbuffers(1, &depthBufferForPicking);
        }
        glGenRenderbuffers(1, &depthBufferForPicking);
        glBindRenderbuffer(GL_RENDERBUFFER, depthBufferForPicking);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthBufferForPicking);

        pickingBufferWidth = width;
        pickingBufferHeight = height;
    }
    
    self->extractPreprocessedNodes();

    if(!isPickingBufferImageOutputEnabled){
        glScissor(x, y, 1, 1);
        glEnable(GL_SCISSOR_TEST);
    }

    isPicking = true;
    isActuallyRendering = false;
    beginRendering();
    pushProgram(solidColorProgram);
    currentNodePath.clear();
    pickingNodePathList.clear();

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    renderScene();
    
    popProgram();
    isPicking = false;

    if(!isPickingBufferImageOutputEnabled){
        glDisable(GL_SCISSOR_TEST);
    }

    endRendering();

    glBindFramebuffer(GL_READ_FRAMEBUFFER, fboForPicking);
    glReadBuffer(GL_COLOR_ATTACHMENT0);

    GLfloat color[4];
    glReadPixels(x, y, 1, 1, GL_RGBA, GL_FLOAT, color);
    if(isPickingBufferImageOutputEnabled){
        color[2] = 0.0f;
    }
    int id = (int)(color[0] * 255) + ((int)(color[1] * 255) << 8) + ((int)(color[2] * 255) << 16) - 1;

    pickedNodePath.clear();

    if(id > 0 && id < static_cast<int>(pickingNodePathList.size())){
        GLfloat depth;
        glReadPixels(x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
        Vector3 projected;
        if(self->unproject(x, y, depth, pickedPoint)){
            pickedNodePath = *pickingNodePathList[id];
        }
    }

    glBindFramebuffer(GL_FRAMEBUFFER, defaultFBO);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, defaultFBO);

    return !pickedNodePath.empty();
}


void GLSLSceneRenderer::setPickingBufferImageOutputEnabled(bool on)
{
    impl->isPickingBufferImageOutputEnabled = on;
}


bool GLSLSceneRenderer::getPickingBufferImage(Image& out_image)
{
    if(!impl->isPickingBufferImageOutputEnabled){
        return false;
    }
    
    glBindFramebuffer(GL_FRAMEBUFFER, impl->fboForPicking);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, impl->fboForPicking);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    int w = impl->pickingBufferWidth;
    int h = impl->pickingBufferHeight;
    out_image.setSize(w, h, 4);
    glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, out_image.pixels());
    out_image.applyVerticalFlip();
    glBindFramebuffer(GL_FRAMEBUFFER, impl->defaultFBO);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, impl->defaultFBO);

    return true;
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
        viewTransform = T.inverse(Eigen::Isometry);
    } else {
        viewTransform = cameraPosition.inverse(Eigen::Isometry);
    }
    PV = projectionMatrix * viewTransform.matrix();

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


void GLSLSceneRendererImpl::renderFog(LightingProgram* program)
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


bool GLSLSceneRenderer::isPicking() const
{
    return impl->isPicking;
}


const std::vector<SgNode*>& GLSLSceneRenderer::pickedNodePath() const
{
    return impl->pickedNodePath;
}


const Vector3& GLSLSceneRenderer::pickedPoint() const
{
    return impl->pickedPoint;
}


inline void GLSLSceneRendererImpl::setPickColor(int id)
{
    Vector3f color;
    color[0] = (id & 0xff) / 255.0;
    color[1] = ((id >> 8) & 0xff) / 255.0;
    color[2] = ((id >> 16) & 0xff) / 255.0;
    if(isPickingBufferImageOutputEnabled){
        color[2] = 1.0f;
    }
    solidColorProgram.setColor(color);
}
        

/**
   @return id of the current object
*/
inline int GLSLSceneRendererImpl::pushPickId(SgNode* node, bool doSetColor)
{
    int id = 0;
    
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
    currentProgram->setTransform(PV, viewTransform, position, resource->pLocalTransform);
    glBindVertexArray(resource->vao);
    glDrawArrays(primitiveMode, 0, resource->numVertices);
}


void GLSLSceneRendererImpl::drawBoundingBox(VertexResource* resource, const BoundingBox& bbox)
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
        
        
void GLSLSceneRendererImpl::renderShape(SgShape* shape)
{
    SgMesh* mesh = shape->mesh();
    if(mesh && mesh->hasVertices()){
        SgMaterial* material = shape->material();
        if(material && material->transparency() > 0.0){
            if(!isRenderingShadowMap){
                const Affine3& position = modelMatrixStack.back();
                auto pickId = pushPickId(shape, false);
                transparentRenderingFunctions.emplace_back(
                    shape, position, pickId,
                    [this](Referenced* object, Affine3& position, int id){
                        renderShapeMain(static_cast<SgShape*>(object), position, id); });
                popPickId();
            }
        } else {
            auto pickId = pushPickId(shape, false);
            renderShapeMain(shape, modelMatrixStack.back(), pickId);
            popPickId();
        }
    }
}


void GLSLSceneRendererImpl::renderShapeMain(SgShape* shape, const Affine3& position, int pickId)
{
    auto mesh = shape->mesh();
    
    if(isPicking){
        setPickColor(pickId);
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

        if(isNormalVisualizationEnabled && isActuallyRendering && resource->normalVisualization){
            renderLineSet(resource->normalVisualization);
        }
    }
}


void GLSLSceneRendererImpl::applyCullingMode(SgMesh* mesh)
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


void GLSLSceneRenderer::dispatchToTransparentPhase
(Referenced* object, int id,
 std::function<void(Referenced* object, const Affine3& position, int id)> renderingFunction)
{
    impl->transparentRenderingFunctions.emplace_back(
        object, impl->modelMatrixStack.back(), id, renderingFunction);
}


void GLSLSceneRendererImpl::renderTransparentObjects()
{
    if(!isPicking){
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
    }

    for(auto& func : transparentRenderingFunctions){
        func();
    }

    if(!isPicking){
        glDisable(GL_BLEND);
        glDepthMask(GL_TRUE);
    }

    transparentRenderingFunctions.clear();
}


void GLSLSceneRendererImpl::renderMaterial(const SgMaterial* material)
{
    currentProgram->setMaterial(material ? material : defaultMaterial);
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


void GLSLSceneRendererImpl::makeVertexBufferObjects(SgShape* shape, VertexResource* resource)
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
void GLSLSceneRendererImpl::writeMeshVerticesSub
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


void GLSLSceneRendererImpl::writeMeshVerticesFloat(SgMesh* mesh, VertexResource* resource)
{
    struct VertexArrayWrapper {
        SgVertexArray array;
        void append(const Vector3f& v){ array.push_back(v); }
    } vertices;

    writeMeshVerticesSub<Vector3f, GL_FLOAT, GL_FALSE>(mesh, resource, vertices);
}


void GLSLSceneRendererImpl::writeMeshVerticesNormalizedShort(SgMesh* mesh, VertexResource* resource)
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
bool GLSLSceneRendererImpl::writeMeshNormalsSub
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
    

void GLSLSceneRendererImpl::writeMeshNormalsFloat(SgMesh* mesh, VertexResource* resource)
{
    struct NormalArrayWrapper {
        SgNormalArray array;
        void append(const Vector3f& n){ array.push_back(n); }
        Vector3f get(int index){ return array[index]; }
    } normals;
            
    writeMeshNormalsSub<Vector3f, GL_FLOAT, 3, GL_FALSE>(mesh, resource, normals);
}


void GLSLSceneRendererImpl::writeMeshNormalsShort(SgMesh* mesh, VertexResource* resource)
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


void GLSLSceneRendererImpl::writeMeshNormalsByte(SgMesh* mesh, VertexResource* resource)
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
void GLSLSceneRendererImpl::writeMeshNormalsPacked(SgMesh* mesh, VertexResource* resource)
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
void GLSLSceneRendererImpl::writeMeshTexCoordsSub
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


void GLSLSceneRendererImpl::writeMeshTexCoordsFloat
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


void GLSLSceneRendererImpl::writeMeshTexCoordsHalfFloat
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
void GLSLSceneRendererImpl::writeMeshTexCoordsUnsignedShort
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


void GLSLSceneRendererImpl::writeMeshColors(SgMesh* mesh, VertexResource* resource)
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
    

void GLSLSceneRendererImpl::renderPointSet(SgPointSet* pointSet)
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


void GLSLSceneRendererImpl::renderPlot
(SgPlot* plot, GLenum primitiveMode, std::function<SgVertexArrayPtr()> getVertices)
{
    pushPickId(plot);

    bool hasColors = plot->hasColors();
    
    if(isPicking){
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


void GLSLSceneRendererImpl::renderOverlay(SgOverlay* overlay)
{
    if(!isActuallyRendering){
        return;
    }

    ScopedShaderProgramActivator programActivator(solidColorProgram, this);
    
    modelMatrixStack.push_back(Affine3::Identity());

    const Matrix4 PV0 = PV;
    SgOverlay::ViewVolume v;
    int x, y, width, height;
    self->getViewport(x, y, width, height);
    overlay->calcViewVolume(width, height, v);
    self->getOrthographicProjectionMatrix(v.left, v.right, v.bottom, v.top, v.zNear, v.zFar, PV);
            
    renderGroup(overlay);

    PV = PV0;
    modelMatrixStack.pop_back();
}


void GLSLSceneRendererImpl::renderOutlineGroup(SgOutlineGroup* outline)
{
    if(isPicking){
        renderGroup(outline);
    } else {
        const Affine3& T = modelMatrixStack.back();
        postRenderingFunctions.emplace_back(
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

    ScopedShaderProgramActivator programActivator(solidColorProgram, this);
    
    solidColorProgram.setColor(outline->color());
    solidColorProgram.setColorChangable(false);
    glDisable(GL_DEPTH_TEST);

    renderChildNodes(outline);

    glEnable(GL_DEPTH_TEST);
    setLineWidth(orgLineWidth);
    glPolygonMode(GL_FRONT_AND_BACK, polygonMode);
    glDisable(GL_STENCIL_TEST);
    solidColorProgram.setColorChangable(true);

    modelMatrixStack.pop_back();
}


void GLSLSceneRendererImpl::renderLightweightRenderingGroup(SgLightweightRenderingGroup* group)
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


void GLSLSceneRendererImpl::clearGLState()
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


void GLSLSceneRendererImpl::setPointSize(float size)
{
    if(!stateFlag[POINT_SIZE] || pointSize != size){
        float s = isPicking ? std::max(size, MinLineWidthForPicking) : size;
        solidColorProgram.setPointSize(s);
        pointSize = s;
        stateFlag[POINT_SIZE] = true;
    }
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
