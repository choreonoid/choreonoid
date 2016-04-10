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
#include <boost/unordered_map.hpp>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <iostream>

using namespace std;
using namespace cnoid;

namespace {

const bool SHOW_IMAGE_FOR_PICKING = false;

const float MinLineWidthForPicking = 5.0f;

typedef vector<Affine3, Eigen::aligned_allocator<Affine3> > Affine3Array;

class ShapeHandleSet : public Referenced
{
public:
    GLuint vao;
    GLuint vbos[3];
    GLsizei numVertices;

    ShapeHandleSet() {
        clear();
        glGenVertexArrays(1, &vao);
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
    }

    void deleteBuffers(){
        glDeleteBuffers(3, vbos);
        for(int i=0; i < 3; ++i){
            vbos[i] = 0;
        }
    }

    GLuint vbo(int index) {
        return vbos[index];
    }

    ~ShapeHandleSet() { 
        if(vao > 0){
            glDeleteVertexArrays(1, &vao);
        }
        glDeleteBuffers(3, vbos);
    }
};
typedef ref_ptr<ShapeHandleSet> ShapeHandleSetPtr;

struct SgObjectPtrHash {
    std::size_t operator()(const SgObjectPtr& p) const {
        return boost::hash_value<SgObject*>(p.get());
    }
};
typedef boost::unordered_map<SgObjectPtr, ShapeHandleSetPtr, SgObjectPtrHash> ShapeHandleSetMap;

}


namespace cnoid {

class GLSLSceneRendererImpl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    GLSLSceneRenderer* self;

    ShaderProgram* currentProgram;
    LightingProgram* currentLightingProgram;
    NolightingProgram* currentNolightingProgram;
    
    SolidColorProgram solidColorProgram;
    PhongShadowProgram phongShadowProgram;

    bool isPicking;
    
    Affine3Array modelMatrixStack; // stack of the model matrices
    Affine3 viewMatrix;
    Matrix4 projectionMatrix;
    Matrix4 PV;

    Matrix4 shadowBias;
    Matrix4 BPV;

    bool defaultLighting;
    Vector4f currentNolightingColor;
    Vector4f diffuseColor;
    Vector4f ambientColor;
    Vector4f specularColor;
    Vector4f emissionColor;
    float shininess;
    float lastAlpha;

    SgMaterialPtr defaultMaterial;
    GLfloat defaultLineWidth;
    ShapeHandleSetMap shapeHandleSetMaps[2];
    bool doUnusedShapeHandleSetCheck;
    bool isCheckingUnusedShapeHandleSets;
    bool hasValidNextShapeHandleSetMap;
    bool isShapeHandleSetClearRequested;
    int currentShapeHandleSetMapIndex;
    ShapeHandleSetMap* currentShapeHandleSetMap;
    ShapeHandleSetMap* nextShapeHandleSetMap;
    ShapeHandleSet* currentShapeHandleSet;

    GLdouble pickX;
    GLdouble pickY;
    typedef boost::shared_ptr<SgNodePath> SgNodePathPtr;
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
        CULL_FACE,
        CCW,
        LIGHTING,
        LIGHT_MODEL_TWO_SIDE,
        BLEND,
        DEPTH_MASK,
        POINT_SIZE,
        LINE_WIDTH,
        NUM_STATE_FLAGS
    };

    boost::dynamic_bitset<> stateFlag;

    bool isColorMaterialEnabled;
    bool isCullFaceEnabled;
    bool isCCW;
    bool isLightingEnabled;
    bool isLightModelTwoSide;
    bool isBlendEnabled;
    bool isDepthMaskEnabled;
    float pointSize;
    float lineWidth;
    
    GLSLSceneRendererImpl(GLSLSceneRenderer* self);
    ~GLSLSceneRendererImpl();
    bool initializeGL();
    bool pick(int x, int y);
    void renderScene();
    void renderShadowMap();
    void beginRendering();
    void renderCamera(SgCamera* camera, const Affine3& cameraPosition);
    void renderLights();
    void endRendering();
    inline void setPickColor(unsigned int id);
    inline unsigned int pushPickID(SgNode* node, bool doSetColor = true);
    void popPickID();
    void visitInvariantGroup(SgInvariantGroup* group);
    void flushNolightingTransformMatrices();
    ShapeHandleSet* getOrCreateShapeHandleSet(SgObject* obj);
    void visitShape(SgShape* shape);
    void renderMaterial(const SgMaterial* material);
    bool renderTexture(SgTexture* texture, bool withMaterial);
    void createMeshVertexArray(SgMesh* mesh, ShapeHandleSet* handleSet);
    void visitPointSet(SgPointSet* pointSet);
    void visitLineSet(SgLineSet* lineSet);
    void visitOutlineGroup(SgOutlineGroup* outline);
    void clearGLState();
    void setNolightingColor(const Vector4f& color);
    void enableColorMaterial(bool on);
    void setDiffuseColor(const Vector4f& color);
    void setAmbientColor(const Vector4f& color);
    void setEmissionColor(const Vector4f& color);
    void setSpecularColor(const Vector4f& color);
    void setShininess(float shininess);
    void enableCullFace(bool on);
    void setFrontCCW(bool on);
    void enableLighting(bool on);
    void setLightModelTwoSide(bool on);
    void enableBlend(bool on);
    void enableDepthMask(bool on);
    void setPointSize(float size);
    void setLineWidth(float width);
    void getCurrentCameraTransform(Affine3& T);
};

}


GLSLSceneRenderer::GLSLSceneRenderer()
{
    impl = new GLSLSceneRendererImpl(this);
}


GLSLSceneRenderer::GLSLSceneRenderer(SgGroup* sceneRoot)
    : GLSceneRenderer(sceneRoot)
{
    impl = new GLSLSceneRendererImpl(this);
}


GLSLSceneRendererImpl::GLSLSceneRendererImpl(GLSLSceneRenderer* self)
    : self(self)
{
    currentProgram = 0;
    currentLightingProgram = 0;
    currentNolightingProgram = 0;

    shadowBias <<
        0.5, 0.0, 0.0, 0.5,
        0.0, 0.5, 0.0, 0.5,
        0.0, 0.0, 0.5, 0.5,
        0.0, 0.0, 0.0, 1.0;
    
    isPicking = false;
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
    defaultLineWidth = 1.0f;

    stateFlag.resize(NUM_STATE_FLAGS, false);
    clearGLState();

    os_ = &nullout();
}


GLSLSceneRenderer::~GLSLSceneRenderer()
{
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

    isShapeHandleSetClearRequested = true;

    return true;
}


void GLSLSceneRenderer::requestToClearCache()
{
    impl->isShapeHandleSetClearRequested = true;
}


void GLSLSceneRenderer::render()
{
    extractPreprocessedNodes();

    PhongShadowProgram& program = impl->phongShadowProgram;
    
    if(!program.isShadowEnabled()){
        impl->renderScene();

    } else {
        program.setRenderingPass(0);
        Array4i vp = viewport();
        setViewport(0, 0, program.shadowMapWidth(), program.shadowMapHeight());
        glEnable(GL_CULL_FACE);
        glCullFace(GL_FRONT);
        impl->renderShadowMap();

        program.setRenderingPass(1);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        setViewport(vp[0], vp[1], vp[2], vp[3]);
        glDisable(GL_CULL_FACE);
        impl->renderScene();
    }
}


bool GLSLSceneRenderer::pick(int x, int y)
{
    return impl->pick(x, y);
}


bool GLSLSceneRendererImpl::pick(int x, int y)
{
    if(!SHOW_IMAGE_FOR_PICKING){
        glScissor(x, y, 1, 1);
        glEnable(GL_SCISSOR_TEST);
    }

    isPicking = true;
    renderScene();
    isPicking = false;

    glDisable(GL_SCISSOR_TEST);
    
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
    }
    beginRendering();
    self->sceneRoot()->accept(*self);
    endRendering();
}


void GLSLSceneRendererImpl::renderShadowMap()
{
    SgLight* light;
    Affine3 T;
    self->getLightInfo(phongShadowProgram.shadowLightIndex(), light, T);
    if(light){
        SgCamera* shadowMapCamera = phongShadowProgram.getShadowMapCamera(light, T);
        if(shadowMapCamera){
            renderCamera(shadowMapCamera, T);
            BPV = shadowBias * PV;

            beginRendering();
            self->sceneRoot()->accept(*self);
            endRendering();

            glFlush();
            glFinish();
        }
    }
}
    

void GLSLSceneRendererImpl::renderCamera(SgCamera* camera, const Affine3& cameraPosition)
{
    if(SgPerspectiveCamera* pers = dynamic_cast<SgPerspectiveCamera*>(camera)){
        double aspectRatio = self->aspectRatio();
        self->getPerspectiveProjectionMatrix(
            pers->fovy(aspectRatio), aspectRatio, pers->nearDistance(), pers->farDistance(),
            projectionMatrix);
        
    } else if(SgOrthographicCamera* ortho = dynamic_cast<SgOrthographicCamera*>(camera)){
        GLfloat left, right, bottom, top;
        self->getViewVolume(ortho, left, right, bottom, top);
        self->getOrthographicProjectionMatrix(
            left, right, bottom, top, ortho->nearDistance(), ortho->farDistance(),
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
    currentShapeHandleSet = 0;
    
    currentLightingProgram = 0;
    currentNolightingProgram = 0;
    
    if(isPicking){
        currentProgram = &solidColorProgram;
        currentNolightingProgram = &solidColorProgram;
        currentNodePath.clear();
        pickingNodePathList.clear();
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    } else {
        currentProgram = &phongShadowProgram;
        if(phongShadowProgram.renderingPass() == 0){
            currentNolightingProgram = &phongShadowProgram.shadowMapProgram();
        } else {
            currentLightingProgram = &phongShadowProgram;
        }
        const Vector3f& c = self->backgroundColor();
        glClearColor(c[0], c[1], c[2], 1.0f);
    }
    currentProgram->use();
    currentProgram->bindGLObjects();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    clearGLState();

    if(currentLightingProgram){
        renderLights();
    }
}


void GLSLSceneRendererImpl::renderLights()
{
    int lightIndex = 0;

    const int n = self->numLights();
    for(int i=0; i < n; ++i){
        if(lightIndex == currentLightingProgram->maxNumLights()){
            break;
        }
        SgLight* light;
        Affine3 T;
        self->getLightInfo(i, light, T);
        if(light->on()){
            if(currentLightingProgram->renderLight(lightIndex, light, T, viewMatrix)){
                ++lightIndex;
            }
        }
    }

    if(lightIndex < currentLightingProgram->maxNumLights()){
        SgLight* headLight = self->headLight();
        if(headLight->on()){
            if(currentLightingProgram->renderLight(lightIndex, headLight, self->currentCameraPosition(), viewMatrix)){
                ++lightIndex;
            }
        }
    }

    currentLightingProgram->setNumLights(lightIndex);
}


void GLSLSceneRendererImpl::endRendering()
{
    if(isCheckingUnusedShapeHandleSets){
        currentShapeHandleSetMap->clear();
        hasValidNextShapeHandleSetMap = true;
    }
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
    Vector4f color;
    color[0] = (id & 0xff) / 255.0;
    color[1] = ((id >> 8) & 0xff) / 255.0;
    color[2] = ((id >> 16) & 0xff) / 255.0;
    color[3] = 1.0f;
    if(SHOW_IMAGE_FOR_PICKING){
        color[2] = 1.0f;
    }
    setNolightingColor(color);
}
        

/**
   @return id of the current object
*/
inline unsigned int GLSLSceneRendererImpl::pushPickID(SgNode* node, bool doSetColor)
{
    unsigned int id = 0;
    
    if(isPicking){
        id = pickingNodePathList.size() + 1;
        currentNodePath.push_back(node);
        pickingNodePathList.push_back(boost::make_shared<SgNodePath>(currentNodePath));
        if(doSetColor){
            setPickColor(id);
        }
    }

    return id;
}


inline void GLSLSceneRendererImpl::popPickID()
{
    if(isPicking){
        currentNodePath.pop_back();
    }
}


void GLSLSceneRenderer::visitGroup(SgGroup* group)
{
    impl->pushPickID(group);
    SceneVisitor::visitGroup(group);
    impl->popPickID();
}


void GLSLSceneRenderer::visitUnpickableGroup(SgUnpickableGroup* group)
{
    if(!impl->isPicking){
        visitGroup(group);
    }
}


void GLSLSceneRenderer::visitInvariantGroup(SgInvariantGroup* group)
{
    impl->visitInvariantGroup(group);
}


void GLSLSceneRendererImpl::visitInvariantGroup(SgInvariantGroup* group)
{
    self->visitGroup(group);
}


void GLSLSceneRenderer::visitTransform(SgTransform* transform)
{
    Affine3 T;
    transform->getTransform(T);

    Affine3Array& modelMatrixStack = impl->modelMatrixStack;
    modelMatrixStack.push_back(modelMatrixStack.back() * T);

    visitGroup(transform);
    
    modelMatrixStack.pop_back();
}


ShapeHandleSet* GLSLSceneRendererImpl::getOrCreateShapeHandleSet(SgObject* obj)
{
    ShapeHandleSet* handleSet;
    ShapeHandleSetMap::iterator p = currentShapeHandleSetMap->find(obj);
    if(p == currentShapeHandleSetMap->end()){
        p = currentShapeHandleSetMap->insert(ShapeHandleSetMap::value_type(obj, new ShapeHandleSet)).first;
    }
    handleSet = p->second;

    if(isCheckingUnusedShapeHandleSets){
        nextShapeHandleSetMap->insert(*p);
    }

    if(currentLightingProgram){
        currentLightingProgram->setTransformMatrices(viewMatrix, modelMatrixStack.back(), PV, BPV);
    } else if(currentNolightingProgram){
        const Matrix4f PVM = (PV * modelMatrixStack.back().matrix()).cast<float>();
        currentNolightingProgram->setProjectionMatrix(PVM);
    }

    glBindVertexArray(handleSet->vao);

    return handleSet;
}


void GLSLSceneRenderer::visitShape(SgShape* shape)
{
    impl->visitShape(shape);
}


void GLSLSceneRendererImpl::visitShape(SgShape* shape)
{
    SgMesh* mesh = shape->mesh();
    if(mesh && mesh->hasVertices()){
        if(!isPicking){
            renderMaterial(shape->material());
        }
        ShapeHandleSet* handleSet = getOrCreateShapeHandleSet(mesh);
        if(!handleSet->numVertices){
            createMeshVertexArray(mesh, handleSet);
        }
        pushPickID(shape);
        glDrawArrays(GL_TRIANGLES, 0, handleSet->numVertices);
        popPickID();
    }
}


void GLSLSceneRendererImpl::renderMaterial(const SgMaterial* material)
{
    if(!material){
        material = defaultMaterial;
    }
    
    float alpha = 1.0 - material->transparency();

    Vector4f color;
    color << material->diffuseColor(), alpha;

    if(currentNolightingProgram){
        setNolightingColor(color);

    } else if(currentLightingProgram){
        setDiffuseColor(color);
        
        color.head<3>() *= material->ambientIntensity();
        setAmbientColor(color);
        
        color << material->emissiveColor(), alpha;
        setEmissionColor(color);
        
        color << material->specularColor(), alpha;
        setSpecularColor(color);
        
        float shininess = (127.0f * material->shininess()) + 1.0f;
        setShininess(shininess);
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
    glVertexAttribPointer((GLuint)0, 3, GL_FLOAT, GL_FALSE, 0, ((GLubyte *)NULL + (0)));
    glEnableVertexAttribArray(0);
    
    if(hasNormals){
        glBindBuffer(GL_ARRAY_BUFFER, normalBufferHandle);
        glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(Vector3f), normals.data(), GL_STATIC_DRAW);
        glVertexAttribPointer((GLuint)1, 3, GL_FLOAT, GL_FALSE, 0, ((GLubyte *)NULL + (0)));
        glEnableVertexAttribArray(1);
    }
}


void GLSLSceneRenderer::visitPointSet(SgPointSet* pointSet)
{
    impl->visitPointSet(pointSet);
}


void GLSLSceneRendererImpl::visitPointSet(SgPointSet* pointSet)
{
    if(!pointSet->hasVertices()){
        return;
    }
    const double s = pointSet->pointSize();
    if(s > 0.0){
        setPointSize(s);
    }
    //renderPlot(pointSet, *pointSet->vertices(), (GLenum)GL_POINTS);
    if(s > 0.0){
        setPointSize(s);
    }
}


void GLSLSceneRenderer::visitLineSet(SgLineSet* lineSet)
{
    impl->visitLineSet(lineSet);
}


void GLSLSceneRendererImpl::visitLineSet(SgLineSet* lineSet)
{
    const int n = lineSet->numLines();

    if(lineSet->hasVertices() && n > 0){

        ShaderProgram* orgProgram = currentProgram;

        if(isPicking){
            pushPickID(lineSet);
        } else {
            solidColorProgram.use();
            currentProgram = &solidColorProgram;
            renderMaterial(lineSet->material());
        }

        ShapeHandleSet* handleSet = getOrCreateShapeHandleSet(lineSet);
        if(!handleSet->numVertices){
            const SgVertexArray& orgVertices = *lineSet->vertices();
            SgVertexArray vertices;
            handleSet->numVertices = n * 2;
            vertices.reserve(handleSet->numVertices);
            for(int i=0; i < n; ++i){
                SgLineSet::LineRef line = lineSet->line(i);
                vertices.push_back(orgVertices[line[0]]);
                vertices.push_back(orgVertices[line[1]]);
            }
            handleSet->genBuffers(1);
            glBindBuffer(GL_ARRAY_BUFFER, handleSet->vbo(0));
            glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vector3f), vertices.data(), GL_STATIC_DRAW);
            glVertexAttribPointer((GLuint)0, 3, GL_FLOAT, GL_FALSE, 0, ((GLubyte *)NULL + (0)));
            glEnableVertexAttribArray(0);
        }
        
        const double w = lineSet->lineWidth();
        if(w > 0.0){
            setLineWidth(w);
        } else {
            setLineWidth(defaultLineWidth);
        }
        
        glDrawArrays(GL_LINES, 0, handleSet->numVertices);

        if(isPicking){
            popPickID();
        } else {
            currentProgram = orgProgram;
            currentProgram->use();
        }
    }
}


void GLSLSceneRenderer::visitPreprocessed(SgPreprocessed* preprocessed)
{

}


void GLSLSceneRenderer::visitLight(SgLight* light)
{

}


void GLSLSceneRenderer::visitOverlay(SgOverlay* overlay)
{
    if(isPicking()){
        return;
    }
    
}


bool GLSLSceneRenderer::isPicking()
{
    return impl->isPicking;
}


void GLSLSceneRendererImpl::clearGLState()
{
    stateFlag.reset();
    
    diffuseColor << 0.0f, 0.0f, 0.0f, 0.0f;
    ambientColor << 0.0f, 0.0f, 0.0f, 0.0f;
    emissionColor << 0.0f, 0.0f, 0.0f, 0.0f;
    specularColor << 0.0f, 0.0f, 0.0f, 0.0f;
    shininess = 0.0f;

    lineWidth = defaultLineWidth;
}


void GLSLSceneRendererImpl::setNolightingColor(const Vector4f& color)
{
    if(!stateFlag[CURRENT_NOLIGHTING_COLOR] || color != currentNolightingColor){
        currentProgram->setColor(color);
        currentNolightingColor = color;
        stateFlag.set(CURRENT_NOLIGHTING_COLOR);
    }
}


void GLSLSceneRenderer::setColor(const Vector4f& color)
{
    impl->setNolightingColor(color);
}


void GLSLSceneRendererImpl::enableColorMaterial(bool on)
{
    if(!isPicking){
        if(!stateFlag[COLOR_MATERIAL] || isColorMaterialEnabled != on){
            if(on){
                //glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
                //glEnable(GL_COLOR_MATERIAL);
            } else {
                //glDisable(GL_COLOR_MATERIAL);
            }
            isColorMaterialEnabled = on;
            stateFlag.set(COLOR_MATERIAL);
        }
    }
}


void GLSLSceneRenderer::enableColorMaterial(bool on)
{
    impl->enableColorMaterial(on);
}


void GLSLSceneRendererImpl::setDiffuseColor(const Vector4f& color)
{
    if(!stateFlag[DIFFUSE_COLOR] || diffuseColor != color){
        currentLightingProgram->setDiffuseColor(color);
        diffuseColor = color;
        stateFlag.set(DIFFUSE_COLOR);
    }
}


void GLSLSceneRenderer::setDiffuseColor(const Vector4f& color)
{
    impl->setDiffuseColor(color);
}


void GLSLSceneRendererImpl::setAmbientColor(const Vector4f& color)
{
    if(!stateFlag[AMBIENT_COLOR] || ambientColor != color){
        currentLightingProgram->setAmbientColor(color);
        ambientColor = color;
        stateFlag.set(AMBIENT_COLOR);
    }
}


void GLSLSceneRenderer::setAmbientColor(const Vector4f& color)
{
    impl->setAmbientColor(color);
}


void GLSLSceneRendererImpl::setEmissionColor(const Vector4f& color)
{
    if(!stateFlag[EMISSION_COLOR] || emissionColor != color){
        currentLightingProgram->setEmissionColor(color);
        emissionColor = color;
        stateFlag.set(EMISSION_COLOR);
    }
}


void GLSLSceneRenderer::setEmissionColor(const Vector4f& color)
{
    impl->setEmissionColor(color);
}


void GLSLSceneRendererImpl::setSpecularColor(const Vector4f& color)
{
    if(!stateFlag[SPECULAR_COLOR] || specularColor != color){
        currentLightingProgram->setSpecularColor(color);
        specularColor = color;
        stateFlag.set(SPECULAR_COLOR);
    }
}


void GLSLSceneRenderer::setSpecularColor(const Vector4f& color)
{
    impl->setSpecularColor(color);
}


void GLSLSceneRendererImpl::setShininess(float s)
{
    if(!stateFlag[SHININESS] || shininess != s){
        currentLightingProgram->setShininess(s);
        shininess = s;
        stateFlag.set(SHININESS);
    }
}


void GLSLSceneRenderer::setShininess(float s)
{
    impl->setShininess(s);
}


void GLSLSceneRendererImpl::enableCullFace(bool on)
{
    if(!stateFlag[CULL_FACE] || isCullFaceEnabled != on){
        if(on){
            //glEnable(GL_CULL_FACE);
        } else {
            //glDisable(GL_CULL_FACE);
        }
        isCullFaceEnabled = on;
        stateFlag.set(CULL_FACE);
    }
}


void GLSLSceneRenderer::enableCullFace(bool on)
{
    impl->enableCullFace(on);
}


void GLSLSceneRendererImpl::setFrontCCW(bool on)
{
    if(!stateFlag[CCW] || isCCW != on){
        if(on){
            //glFrontFace(GL_CCW);
        } else {
            //glFrontFace(GL_CW);
        }
        isCCW = on;
        stateFlag.set(CCW);
    }
}


void GLSLSceneRenderer::setFrontCCW(bool on)
{
    impl->setFrontCCW(on);
}


/**
   Lighting should not be enabled in rendering code
   which may be rendered with displaylists.
*/
void GLSLSceneRendererImpl::enableLighting(bool on)
{
    if(isPicking || !defaultLighting){
        return;
    }
    if(!stateFlag[LIGHTING] || isLightingEnabled != on){
        if(on){
            //glEnable(GL_LIGHTING);
        } else {
            //glDisable(GL_LIGHTING);
        }
        isLightingEnabled = on;
        stateFlag.set(LIGHTING);
    }
}


void GLSLSceneRenderer::enableLighting(bool on)
{
    impl->enableLighting(on);
}


void GLSLSceneRenderer::enableShadowOfLight(int index, bool on)
{
    impl->phongShadowProgram.setShadowEnabled(on);
    impl->phongShadowProgram.setShadowLight(index);
}


void GLSLSceneRendererImpl::enableBlend(bool on)
{
    if(isPicking){
        return;
    }
    if(!stateFlag[BLEND] || isBlendEnabled != on){
        if(on){
            //glEnable(GL_BLEND);
            //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            //enableDepthMask(false);
        } else {
            //glDisable(GL_BLEND);
            //enableDepthMask(true);
        }
        isBlendEnabled = on;
        stateFlag.set(BLEND);
    }
}


void GLSLSceneRenderer::enableBlend(bool on)
{
    impl->enableBlend(on);
}


void GLSLSceneRendererImpl::enableDepthMask(bool on)
{
    if(!stateFlag[DEPTH_MASK] || isDepthMaskEnabled != on){
        //glDepthMask(on);
        isDepthMaskEnabled = on;
        stateFlag.set(DEPTH_MASK);
    }
}


void GLSLSceneRenderer::enableDepthMask(bool on)
{
    impl->enableDepthMask(on);
}


void GLSLSceneRendererImpl::setPointSize(float size)
{
    if(!stateFlag[POINT_SIZE] || pointSize != size){
        if(isPicking){
            //glPointSize(std::max(size, MinLineWidthForPicking));
        } else {
            //glPointSize(size);
        }
        pointSize = size;
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
    /*
    if(size != impl->defaultPointSize){
        impl->defaultPointSize = size;
    }
    */
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


const Affine3& GLSLSceneRenderer::currentModelTransform() const
{
    return impl->modelMatrixStack.back();
}


const Matrix4& GLSLSceneRenderer::projectionMatrix() const
{
    return impl->projectionMatrix;
}


void GLSLSceneRenderer::visitOutlineGroup(SgOutlineGroup* outline)
{
    impl->visitOutlineGroup(outline);
}


void GLSLSceneRendererImpl::visitOutlineGroup(SgOutlineGroup* outlineGroup)
{
    for(SgGroup::const_iterator p = outlineGroup->begin(); p != outlineGroup->end(); ++p){
        (*p)->accept(*self);
    }
}
