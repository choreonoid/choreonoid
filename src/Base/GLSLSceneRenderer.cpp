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
#include <boost/dynamic_bitset.hpp>
#include <unordered_map>
#include <mutex>
#include <iostream>

using namespace std;
using namespace cnoid;

using namespace std::placeholders;

namespace {

const bool SHOW_IMAGE_FOR_PICKING = false;

const float MinLineWidthForPicking = 5.0f;

typedef vector<Affine3, Eigen::aligned_allocator<Affine3> > Affine3Array;

std::mutex extensionMutex;
set<GLSLSceneRenderer*> renderers;
vector<std::function<void(GLSLSceneRenderer* renderer)>> extendFunctions;

class ShapeHandleSet : public Referenced
{
public:
    GLuint vao;
    GLuint vbos[3];
    GLsizei numVertices;
    bool hasBuffers;
    ScopedConnection connection;

    ShapeHandleSet(GLSLSceneRendererImpl* renderer, SgObject* obj)
    {
        connection.reset(obj->sigUpdated().connect(std::bind(&ShapeHandleSet::onUpdated, this)));
        clear();
        glGenVertexArrays(1, &vao);
        hasBuffers = false;
    }

    void onUpdated(){
        numVertices = 0;
    }

    bool isValid(){
        if(numVertices > 0){
            return true;
        } else if(hasBuffers){
            deleteBuffers();
        }
        return false;
    }

    void clear(){
        vao = 0;
        for(int i=0; i < 3; ++i){
            vbos[i] = 0;
        }
        numVertices = 0;
    }

    void genBuffers(int n){
        glGenBuffers(n, vbos);
        hasBuffers = true;
    }

    void deleteBuffers(){
        glDeleteBuffers(3, vbos);
        for(int i=0; i < 3; ++i){
            vbos[i] = 0;
        }
        hasBuffers = false;
    }

    GLuint vbo(int index) {
        return vbos[index];
    }

    ~ShapeHandleSet() { 
        if(vao > 0){
            glDeleteVertexArrays(1, &vao);
        }
        if(hasBuffers){
            glDeleteBuffers(3, vbos);
        }
    }
};

typedef ref_ptr<ShapeHandleSet> ShapeHandleSetPtr;

struct SgObjectPtrHash {
    std::hash<SgObject*> hash;
    std::size_t operator()(const SgObjectPtr& p) const {
        return hash(p.get());
    }
};
typedef std::unordered_map<SgObjectPtr, ShapeHandleSetPtr, SgObjectPtrHash> ShapeHandleSetMap;

}

namespace cnoid {

class GLSLSceneRendererImpl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    GLSLSceneRenderer* self;

    PolymorphicFunctionSet<SgNode> renderingFunctions;

    GLint defaultFBO;

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
        
    bool isPicking;
    bool isRenderingShadowMap;
    
    Affine3Array modelMatrixStack; // stack of the model matrices
    Affine3 viewMatrix;
    Matrix4 projectionMatrix;
    Matrix4 PV;

    vector<function<void()>> transparentRenderingFunctions;

    std::set<int> shadowLightIndices;

    bool defaultLighting;
    Vector3f currentNolightingColor;
    Vector3f diffuseColor;
    Vector3f ambientColor;
    Vector3f specularColor;
    Vector3f emissionColor;
    float shininess;
    float alpha;

    SgMaterialPtr defaultMaterial;
    GLfloat defaultPointSize;
    GLfloat defaultLineWidth;
    ShapeHandleSetMap shapeHandleSetMaps[2];
    bool doUnusedShapeHandleSetCheck;
    bool isCheckingUnusedShapeHandleSets;
    bool hasValidNextShapeHandleSetMap;
    bool isShapeHandleSetClearRequested;
    int currentShapeHandleSetMapIndex;
    ShapeHandleSetMap* currentShapeHandleSetMap;
    ShapeHandleSetMap* nextShapeHandleSetMap;

    bool isCurrentFogUpdated;
    SgFogPtr prevFog;
    ScopedConnection currentFogConnection;

    GLdouble pickX;
    GLdouble pickY;
    typedef std::shared_ptr<SgNodePath> SgNodePathPtr;
    SgNodePath currentNodePath;
    vector<SgNodePathPtr> pickingNodePathList;
    SgNodePath pickedNodePath;
    Vector3 pickedPoint;

    ostream* os_;
    ostream& os() { return *os_; }

    // OpenGL states
    enum StateFlag {
        CURRENT_NOLIGHTING_COLOR,
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

    boost::dynamic_bitset<> stateFlag;

    float pointSize;
    float lineWidth;

    std::mutex newExtensionMutex;
    vector<std::function<void(GLSLSceneRenderer* renderer)>> newExtendFunctions;

    void renderChildNodes(SgGroup* group){
        for(SgGroup::const_iterator p = group->begin(); p != group->end(); ++p){
            renderingFunctions.dispatch(*p);
        }
    }
    
    GLSLSceneRendererImpl(GLSLSceneRenderer* self);
    ~GLSLSceneRendererImpl();
    void initialize();
    void onExtensionAdded(std::function<void(GLSLSceneRenderer* renderer)> func);
    bool initializeGL();
    void render();
    bool pick(int x, int y);
    void renderScene();
    bool renderShadowMap(int lightIndex);
    void beginRendering();
    void renderCamera(SgCamera* camera, const Affine3& cameraPosition);
    void renderLights(LightingProgram* program);
    void renderFog(LightingProgram* program);
    void onCurrentFogNodeUdpated();
    void endRendering();
    void renderSceneGraphNodes();
    void pushProgram(ShaderProgram& program, bool isLightingProgram);
    void popProgram();
    inline void setPickColor(unsigned int id);
    inline unsigned int pushPickId(SgNode* node, bool doSetColor = true);
    void popPickId();
    void renderGroup(SgGroup* group);
    void renderTransform(SgTransform* transform);
    void renderUnpickableGroup(SgUnpickableGroup* group);
    void renderShape(SgShape* shape);
    void renderTransparentShape(SgShape* shape, Affine3 modelMatrix, unsigned int pickId);
    void renderPointSet(SgPointSet* pointSet);        
    void renderLineSet(SgLineSet* lineSet);        
    void renderOverlay(SgOverlay* overlay);
    void renderOutlineGroup(SgOutlineGroup* outline);
    void flushNolightingTransformMatrices();
    ShapeHandleSet* getOrCreateShapeHandleSet(SgObject* obj, const Affine3& modelMatrix);
    void renderTransparentObjects();
    void renderMaterial(const SgMaterial* material);
    bool renderTexture(SgTexture* texture, bool withMaterial);
    void createMeshVertexArray(SgMesh* mesh, ShapeHandleSet* handleSet);
    void renderPlot(SgPlot* plot, GLenum primitiveMode, std::function<SgVertexArrayPtr()> getVertices);
    void clearGLState();
    void setNolightingColor(const Vector3f& color);
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

    currentProgram = 0;
    currentLightingProgram = 0;
    currentNolightingProgram = 0;
    materialProgram = &phongShadowProgram;

    isPicking = false;
    isRenderingShadowMap = false;
    pickedPoint.setZero();

    doUnusedShapeHandleSetCheck = true;
    currentShapeHandleSetMapIndex = 0;
    hasValidNextShapeHandleSetMap = false;
    isShapeHandleSetClearRequested = false;
    currentShapeHandleSetMap = &shapeHandleSetMaps[0];
    nextShapeHandleSetMap = &shapeHandleSetMaps[1];

    modelMatrixStack.reserve(16);
    viewMatrix.setIdentity();
    projectionMatrix.setIdentity();

    defaultLighting = true;
    defaultMaterial = new SgMaterial;
    defaultMaterial->setDiffuseColor(Vector3f(0.8, 0.8, 0.8));
    defaultPointSize = 1.0f;
    defaultLineWidth = 1.0f;

    prevFog = 0;

    stateFlag.resize(NUM_STATE_FLAGS, false);
    clearGLState();

    os_ = &nullout();

    renderingFunctions.setFunction<SgGroup>(
        [&](SgNode* node){ renderGroup(static_cast<SgGroup*>(node)); });
    renderingFunctions.setFunction<SgTransform>(
        [&](SgNode* node){ renderTransform(static_cast<SgTransform*>(node)); });
    renderingFunctions.setFunction<SgUnpickableGroup>(
        [&](SgNode* node){ renderUnpickableGroup(static_cast<SgUnpickableGroup*>(node)); });
    renderingFunctions.setFunction<SgShape>(
        [&](SgNode* node){ renderShape(static_cast<SgShape*>(node)); });
    renderingFunctions.setFunction<SgPointSet>(
        [&](SgNode* node){ renderPointSet(static_cast<SgPointSet*>(node)); });
    renderingFunctions.setFunction<SgLineSet>(
        [&](SgNode* node){ renderLineSet(static_cast<SgLineSet*>(node)); });
    renderingFunctions.setFunction<SgOverlay>(
        [&](SgNode* node){ renderOverlay(static_cast<SgOverlay*>(node)); });
    renderingFunctions.setFunction<SgOutlineGroup>(
        [&](SgNode* node){ renderOutlineGroup(static_cast<SgOutlineGroup*>(node)); });

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
        ShapeHandleSetMap& handleSetMap = shapeHandleSetMaps[i];
        for(ShapeHandleSetMap::iterator p = handleSetMap.begin(); p != handleSetMap.end(); ++p){
            ShapeHandleSet* handleSet = p->second;
            handleSet->clear();
        }
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
    for(int i=0; i < extendFunctions.size(); ++i){
        extendFunctions[i](this);
    }
}


void GLSLSceneRendererImpl::onExtensionAdded(std::function<void(GLSLSceneRenderer* renderer)> func)
{
    std::lock_guard<std::mutex> guard(newExtensionMutex);
    newExtendFunctions.push_back(func);
}


void GLSLSceneRenderer::applyNewExtensions()
{
    SceneRenderer::applyExtensions();
    
    std::lock_guard<std::mutex> guard(impl->newExtensionMutex);
    if(!impl->newExtendFunctions.empty()){
        for(int i=0; i < impl->newExtendFunctions.size(); ++i){
            impl->newExtendFunctions[i](this);
        }
        impl->newExtendFunctions.clear();
    }
}


SceneRenderer::NodeFunctionSet& GLSLSceneRenderer::renderingFunctions()
{
    return impl->renderingFunctions;
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

    defaultFBO = 0;
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &defaultFBO);

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
    glDisable(GL_CULL_FACE);

    glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

    isShapeHandleSetClearRequested = true;

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


void GLSLSceneRenderer::requestToClearCache()
{
    impl->isShapeHandleSetClearRequested = true;
}


void GLSLSceneRenderer::render()
{
    applyNewExtensions();
    impl->renderingFunctions.updateDispatchTable();
    impl->render();
}


void GLSLSceneRendererImpl::render()
{
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
    const Vector3f& c = self->backgroundColor();
    glClearColor(c[0], c[1], c[2], 1.0f);
    renderScene();
    popProgram();

    endRendering();
}


bool GLSLSceneRenderer::pick(int x, int y)
{
    return impl->pick(x, y);
}


bool GLSLSceneRendererImpl::pick(int x, int y)
{
    self->extractPreprocessedNodes();
    beginRendering();
    
    if(!SHOW_IMAGE_FOR_PICKING){
        glScissor(x, y, 1, 1);
        glEnable(GL_SCISSOR_TEST);
    }

    isPicking = true;
    pushProgram(solidColorProgram, false);
    currentNodePath.clear();
    pickingNodePathList.clear();
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    
    renderScene();
    
    popProgram();
    isPicking = false;

    glDisable(GL_SCISSOR_TEST);

    endRendering();
    
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

    return !pickedNodePath.empty();
}


void GLSLSceneRendererImpl::renderScene()
{
    SgCamera* camera = self->currentCamera();
    if(camera){
        renderCamera(camera, self->currentCameraPosition());

        transparentRenderingFunctions.clear();

        renderSceneGraphNodes();

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

    viewMatrix = cameraPosition.inverse(Eigen::Isometry);
    PV = projectionMatrix * viewMatrix.matrix();

    modelMatrixStack.clear();
    modelMatrixStack.push_back(Affine3::Identity());
}


void GLSLSceneRendererImpl::beginRendering()
{
    isCheckingUnusedShapeHandleSets = isPicking ? false : doUnusedShapeHandleSetCheck;

    if(isShapeHandleSetClearRequested){
        shapeHandleSetMaps[0].clear();
        shapeHandleSetMaps[1].clear();
        hasValidNextShapeHandleSetMap = false;
        isCheckingUnusedShapeHandleSets = false;
        isShapeHandleSetClearRequested = false; 
    }
    if(hasValidNextShapeHandleSetMap){
        currentShapeHandleSetMapIndex = 1 - currentShapeHandleSetMapIndex;
        currentShapeHandleSetMap = &shapeHandleSetMaps[currentShapeHandleSetMapIndex];
        nextShapeHandleSetMap = &shapeHandleSetMaps[1 - currentShapeHandleSetMapIndex];
        hasValidNextShapeHandleSetMap = false;
    }
}


void GLSLSceneRendererImpl::endRendering()
{
    if(isCheckingUnusedShapeHandleSets){
        currentShapeHandleSetMap->clear();
        hasValidNextShapeHandleSetMap = true;
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
                    std::bind(&GLSLSceneRendererImpl::onCurrentFogNodeUdpated, this)));
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


void GLSLSceneRendererImpl::onCurrentFogNodeUdpated()
{
    if(!self->isFogEnabled()){
        currentFogConnection.disconnect();
    }
    isCurrentFogUpdated = true;
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
    setNolightingColor(color);
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

    // renderGroup(transform);
    pushPickId(transform);
    renderChildNodes(transform);
    popPickId();
    
    modelMatrixStack.pop_back();
}


ShapeHandleSet* GLSLSceneRendererImpl::getOrCreateShapeHandleSet(SgObject* obj, const Affine3& modelMatrix)
{
    ShapeHandleSet* handleSet;
    ShapeHandleSetMap::iterator p = currentShapeHandleSetMap->find(obj);
    if(p == currentShapeHandleSetMap->end()){
        ShapeHandleSet* handleSet = new ShapeHandleSet(this, obj);
        p = currentShapeHandleSetMap->insert(ShapeHandleSetMap::value_type(obj, handleSet)).first;
    }
    handleSet = p->second;

    if(isCheckingUnusedShapeHandleSets){
        nextShapeHandleSetMap->insert(*p);
    }

    if(currentLightingProgram == &phongShadowProgram){
        phongShadowProgram.setTransformMatrices(viewMatrix, modelMatrix, PV);
    } else if(currentNolightingProgram){
        const Matrix4f PVM = (PV * modelMatrix.matrix()).cast<float>();
        currentNolightingProgram->setProjectionMatrix(PVM);
    }

    glBindVertexArray(handleSet->vao);

    return handleSet;
}


void GLSLSceneRendererImpl::renderShape(SgShape* shape)
{
    SgMesh* mesh = shape->mesh();
    if(mesh && mesh->hasVertices()){
        SgMaterial* material = shape->material();
        if(material && material->transparency() > 0.0){
            if(!isRenderingShadowMap){
                const Affine3& M = modelMatrixStack.back();
                unsigned int pickId = pushPickId(shape, false);
                transparentRenderingFunctions.push_back(
                    [this,shape,M,pickId](){
                        renderTransparentShape(shape, M, pickId); });
                popPickId();
            }
        } else {
            if(!isPicking){
                renderMaterial(shape->material());
            }
            ShapeHandleSet* handleSet = getOrCreateShapeHandleSet(mesh, modelMatrixStack.back());
            if(!handleSet->isValid()){
                createMeshVertexArray(mesh, handleSet);
            }
            pushPickId(shape);
            glDrawArrays(GL_TRIANGLES, 0, handleSet->numVertices);
            popPickId();
        }
    }
}


void GLSLSceneRendererImpl::renderTransparentShape(SgShape* shape, Affine3 modelMatrix, unsigned int pickId)
{
    if(isPicking){
        setPickColor(pickId);
    } else {
        renderMaterial(shape->material());
    }
    ShapeHandleSet* handleSet = getOrCreateShapeHandleSet(shape->mesh(), modelMatrix);
    if(!handleSet->isValid()){
        createMeshVertexArray(shape->mesh(), handleSet);
    }
    glDrawArrays(GL_TRIANGLES, 0, handleSet->numVertices);
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
        setNolightingColor(material->diffuseColor());
    }
}


bool GLSLSceneRendererImpl::renderTexture(SgTexture* texture, bool withMaterial)
{
    return false;
}


void GLSLSceneRenderer::onImageUpdated(SgImage* image)
{

}


void GLSLSceneRendererImpl::createMeshVertexArray(SgMesh* mesh, ShapeHandleSet* handleSet)
{
    SgIndexArray& triangleVertices = mesh->triangleVertices();
    const size_t totalNumVertices = triangleVertices.size();
    
    const SgVertexArray& orgVertices = *mesh->vertices();
    SgVertexArray vertices;
    vertices.reserve(totalNumVertices);
    handleSet->numVertices = totalNumVertices;

    const bool hasNormals = mesh->hasNormals();
    const SgNormalArray& orgNormals = *mesh->normals();
    const SgIndexArray& normalIndices = mesh->normalIndices();
    SgNormalArray normals;
    if(hasNormals){
        normals.reserve(totalNumVertices);
    }
        
    const int numTriangles = mesh->numTriangles();
    int faceVertexIndex = 0;
    int numFaceVertices = 0;
    
    for(size_t i=0; i < numTriangles; ++i){
        for(size_t j=0; j < 3; ++j){
            const int orgVertexIndex = triangleVertices[faceVertexIndex];
            vertices.push_back(orgVertices[orgVertexIndex]);
            if(hasNormals){
                if(normalIndices.empty()){
                    normals.push_back(orgNormals[orgVertexIndex]);
                } else {
                    const int normalIndex = normalIndices[faceVertexIndex];
                    normals.push_back(orgNormals[normalIndex]);
                }
            }
            ++faceVertexIndex;
        }
    }

    GLuint normalBufferHandle;
    if(hasNormals){
        handleSet->genBuffers(2);
        normalBufferHandle = handleSet->vbo(1);
    } else {
        handleSet->genBuffers(1);
        normalBufferHandle = 0;
    }
    GLuint positionBufferHandle = handleSet->vbo(0);
        
    glBindBuffer(GL_ARRAY_BUFFER, positionBufferHandle);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vector3f), vertices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer((GLuint)0, 3, GL_FLOAT, GL_FALSE, 0, ((GLubyte*)NULL + (0)));
    glEnableVertexAttribArray(0);
    
    if(hasNormals){
        glBindBuffer(GL_ARRAY_BUFFER, normalBufferHandle);
        glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(Vector3f), normals.data(), GL_STATIC_DRAW);
        glVertexAttribPointer((GLuint)1, 3, GL_FLOAT, GL_FALSE, 0, ((GLubyte*)NULL + (0)));
        glEnableVertexAttribArray(1);
    }
}


static SgVertexArrayPtr getPointSetVertices(SgPointSet* pointSet)
{
    return pointSet->vertices();
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
    
    renderPlot(pointSet, GL_POINTS, std::bind(getPointSetVertices, pointSet));

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
    
    ShapeHandleSet* handleSet = getOrCreateShapeHandleSet(plot, modelMatrixStack.back());
    if(!handleSet->isValid()){
        SgVertexArrayPtr vertices = getVertices();
        const int n = vertices->size();
        handleSet->numVertices = n;
        if(!hasColors){
            handleSet->genBuffers(1);
        } else {
            handleSet->genBuffers(2);
            glBindBuffer(GL_ARRAY_BUFFER, handleSet->vbo(1));
            SgColorArrayPtr colors;
            if(plot->colorIndices().empty()){
                const SgColorArray& orgColors = *plot->colors();
                if(orgColors.size() >= n){
                    colors = plot->colors();
                } else {
                    colors = new SgColorArray(n);
                    std::copy(orgColors.begin(), orgColors.end(), colors->begin());
                    std::fill(colors->begin() + orgColors.size(), colors->end(), orgColors.back());
                }
            }
            if(colors){
                glBufferData(GL_ARRAY_BUFFER, n * sizeof(Vector3f), colors->data(), GL_STATIC_DRAW);
                glVertexAttribPointer((GLuint)1, 3, GL_FLOAT, GL_FALSE, 0, ((GLubyte*)NULL +(0)));
                glEnableVertexAttribArray(1);
            }
        }
        glBindBuffer(GL_ARRAY_BUFFER, handleSet->vbo(0));
        glBufferData(GL_ARRAY_BUFFER, vertices->size() * sizeof(Vector3f), vertices->data(), GL_STATIC_DRAW);
        glVertexAttribPointer((GLuint)0, 3, GL_FLOAT, GL_FALSE, 0, ((GLubyte *)NULL + (0)));
        glEnableVertexAttribArray(0);
    }        

    glDrawArrays(primitiveMode, 0, handleSet->numVertices);
    
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

    renderPlot(lineSet, GL_LINES, std::bind(getLineSetVertices, lineSet));

    popProgram();
}


void GLSLSceneRendererImpl::renderOverlay(SgOverlay* overlay)
{
    if(isPicking || isRenderingShadowMap){
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
    renderGroup(outline);
}


void GLSLSceneRendererImpl::clearGLState()
{
    stateFlag.reset();
    
    diffuseColor << 0.0f, 0.0f, 0.0f, 0.0f;
    ambientColor << 0.0f, 0.0f, 0.0f, 0.0f;
    emissionColor << 0.0f, 0.0f, 0.0f, 0.0f;
    specularColor << 0.0f, 0.0f, 0.0f, 0.0f;
    shininess = 0.0f;
    alpha = 1.0f;

    pointSize = defaultPointSize;    
    lineWidth = defaultLineWidth;
}


void GLSLSceneRendererImpl::setNolightingColor(const Vector3f& color)
{
    if(!stateFlag[CURRENT_NOLIGHTING_COLOR] || color != currentNolightingColor){
        currentProgram->setColor(color);
        currentNolightingColor = color;
        stateFlag.set(CURRENT_NOLIGHTING_COLOR);
    }
}


void GLSLSceneRenderer::setColor(const Vector3f& color)
{
    impl->setNolightingColor(color);
}


void GLSLSceneRendererImpl::setDiffuseColor(const Vector3f& color)
{
    if(!stateFlag[DIFFUSE_COLOR] || diffuseColor != color){
        materialProgram->setDiffuseColor(color);
        diffuseColor = color;
        stateFlag.set(DIFFUSE_COLOR);
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
        stateFlag.set(AMBIENT_COLOR);
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
        stateFlag.set(EMISSION_COLOR);
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
        stateFlag.set(SPECULAR_COLOR);
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
        stateFlag.set(SHININESS);
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
        stateFlag.set(ALPHA);
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
        stateFlag.set(POINT_SIZE);
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
            //glLineWidth(std::max(width, MinLineWidthForPicking));
        } else {
            //glLineWidth(width);
        }
        lineWidth = width;
        stateFlag.set(LINE_WIDTH);
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
    //impl->defaultSmoothShading = on;
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
    /*
    bool doNormalVisualization = (length > 0.0);
    if(doNormalVisualization != impl->doNormalVisualization || length != impl->normalLength){
        impl->doNormalVisualization = doNormalVisualization;
        impl->normalLength = length;
    }
    */
}


void GLSLSceneRenderer::enableUnusedCacheCheck(bool on)
{
    if(!on){
        impl->nextShapeHandleSetMap->clear();
    }
    impl->doUnusedShapeHandleSetCheck = on;
}
