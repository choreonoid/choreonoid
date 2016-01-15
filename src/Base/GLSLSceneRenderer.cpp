/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "GLSLSceneRenderer.h"
#include "GLSLProgram.h"
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
#include <iostream>

using namespace std;
using namespace cnoid;

namespace {

const bool SHOW_IMAGE_FOR_PICKING = false;

const float MinLineWidthForPicking = 5.0f;

typedef vector<Affine3, Eigen::aligned_allocator<Affine3> > Affine3Array;

class HandleSet : public Referenced
{
public:
    GLuint vao;
    GLuint vbos[3];
    GLsizei numVertices;

    HandleSet() {
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

    ~HandleSet() { 
        if(vao > 0){
            glDeleteVertexArrays(1, &vao);
        }
        glDeleteBuffers(3, vbos);
    }
};
typedef ref_ptr<HandleSet> HandleSetPtr;

struct SgObjectPtrHash {
    std::size_t operator()(const SgObjectPtr& p) const {
        return boost::hash_value<SgObject*>(p.get());
    }
};
typedef boost::unordered_map<SgObjectPtr, HandleSetPtr, SgObjectPtrHash> HandleSetMap;

}

namespace cnoid {

class GLSLSceneRendererImpl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    GLSLSceneRenderer* self;

    GLSLProgram phongProgram;

    bool useUniformBlockToPassTransformationMatrices;
    
    GLSLUniformBlockBuffer transformBlockBuffer;
    GLint modelViewMatrixIndex;
    GLint normalMatrixIndex;
    GLint MVPIndex;

    GLint ModelViewMatrixLocation;
    GLint NormalMatrixLocation;
    GLint MVPLocation;

    GLint LightIntensityLocation;
    GLint LightPositionLocation;

    GLSLProgram pickingProgram;

    GLint PickingMVPLocation;
    GLint PickingIDLocation;
    
    bool isPicking;

    HandleSetMap handleSetMaps[2];
    bool doUnusedHandleSetCheck;
    bool isCheckingUnusedHandleSets;
    bool hasValidNextHandleSetMap;
    bool isHandleSetClearRequested;
    int currentHandleSetMapIndex;
    HandleSetMap* currentHandleSetMap;
    HandleSetMap* nextHandleSetMap;
    HandleSet* currentHandleSet;

    Affine3Array modelViewStack; // stack of the model/view matrices

    Affine3 viewMatrix;
    Matrix4 projectionMatrix;
    Affine3 currentModelTransform;


    bool defaultLighting;
    Vector4f currentColor;
    Vector4f diffuseColor;
    Vector4f ambientColor;
    Vector4f specularColor;
    Vector4f emissionColor;
    float shininess;
    float lastAlpha;

    GLint DiffuseColorLocation;
    GLint AmbientColorLocation;
    GLint SpecularColorLocation;
    GLint ShininessLocation;

    SgMaterialPtr defaultMaterial;

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
        CURRENT_COLOR,
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
    void beginRendering(bool doRenderingCommands);
    void beginActualRendering(SgCamera* camera);
    void renderCamera(SgCamera* camera, const Affine3& cameraPosition);
    void renderLights(const Affine3& cameraPosition);
    void renderLight(const SgLight* light, int index, const Affine3& T);
    void endRendering();
    void render();
    bool pick(int x, int y);
    inline void setPickColor(unsigned int id);
    inline unsigned int pushPickID(SgNode* node, bool doSetColor = true);
    void popPickID();
    void visitInvariantGroup(SgInvariantGroup* group);
    void setRenderingUniformMatrixVariables();
    void setPickingUniformMatrixVariables();
    void visitShape(SgShape* shape);
    void visitPointSet(SgPointSet* pointSet);
    void renderPlot(SgPlot* plot, SgVertexArray& expandedVertices, GLenum primitiveMode);
    void visitLineSet(SgLineSet* lineSet);
    void renderMaterial(const SgMaterial* material);
    bool renderTexture(SgTexture* texture, bool withMaterial);
    void renderMesh(SgMesh* mesh, bool hasTexture);
    void createMeshVertexArray(SgMesh* mesh, HandleSet* handleSet);
    void visitOutlineGroup(SgOutlineGroup* outline);

    void clearGLState();
    void setColor(const Vector4f& color);
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
    isPicking = false;
    pickedPoint.setZero();

    doUnusedHandleSetCheck = true;
    currentHandleSetMapIndex = 0;
    hasValidNextHandleSetMap = false;
    isHandleSetClearRequested = false;
    currentHandleSetMap = &handleSetMaps[0];
    nextHandleSetMap = &handleSetMaps[1];

    modelViewStack.reserve(16);
    
    viewMatrix.setIdentity();
    projectionMatrix.setIdentity();

    defaultLighting = true;
    defaultMaterial = new SgMaterial;

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
        HandleSetMap& handleSetMap = handleSetMaps[i];
        for(HandleSetMap::iterator p = handleSetMap.begin(); p != handleSetMap.end(); ++p){
            HandleSet* handleSet = p->second;
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
        phongProgram.loadVertexShader(":/Base/shader/phong.vert");
        phongProgram.loadFragmentShader(":/Base/shader/phong.frag");
        phongProgram.link();

        pickingProgram.loadVertexShader(":/Base/shader/picking.vert");
        pickingProgram.loadFragmentShader(":/Base/shader/picking.frag");
        pickingProgram.link();
    }
    catch(GLSLProgram::Exception& ex){
        os() << ex.what() << endl;
        cout << ex.what() << endl;
        return false;
    }

    useUniformBlockToPassTransformationMatrices = 
        transformBlockBuffer.initialize(phongProgram, "TransformBlock");

    if(useUniformBlockToPassTransformationMatrices){
        modelViewMatrixIndex = transformBlockBuffer.checkUniformMatrix("modelViewMatrix");
        normalMatrixIndex = transformBlockBuffer.checkUniformMatrix("normalMatrix");
        MVPIndex = transformBlockBuffer.checkUniformMatrix("MVP");
        transformBlockBuffer.bind(phongProgram, 1);
        transformBlockBuffer.bindBufferBase(1);
    } else {
        ModelViewMatrixLocation = phongProgram.getUniformLocation("modelViewMatrix");
        NormalMatrixLocation = phongProgram.getUniformLocation("normalMatrix");
        MVPLocation = phongProgram.getUniformLocation("MVP");
    }

    LightIntensityLocation = phongProgram.getUniformLocation("lightIntensity");
    LightPositionLocation = phongProgram.getUniformLocation("lightPosition");
    
    DiffuseColorLocation = phongProgram.getUniformLocation("diffuseColor");
    AmbientColorLocation = phongProgram.getUniformLocation("ambientColor");
    SpecularColorLocation = phongProgram.getUniformLocation("specularColor");
    ShininessLocation = phongProgram.getUniformLocation("shininess");

    PickingMVPLocation = pickingProgram.getUniformLocation("MVP");
    PickingIDLocation = pickingProgram.getUniformLocation("pickingID");

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_DITHER);

    isHandleSetClearRequested = true;

    return true;
}


void GLSLSceneRenderer::requestToClearCache()
{
    impl->isHandleSetClearRequested = true;
}


void GLSLSceneRenderer::initializeRendering()
{
    impl->beginRendering(false);
}


void GLSLSceneRenderer::beginRendering()
{
    impl->beginRendering(true);
}


void GLSLSceneRendererImpl::beginRendering(bool doRenderingCommands)
{
    isCheckingUnusedHandleSets = isPicking ? false : doUnusedHandleSetCheck;

    if(isHandleSetClearRequested){
        handleSetMaps[0].clear();
        handleSetMaps[1].clear();
        hasValidNextHandleSetMap = false;
        isCheckingUnusedHandleSets = false;
        isHandleSetClearRequested = false;
    }
    if(hasValidNextHandleSetMap){
        currentHandleSetMapIndex = 1 - currentHandleSetMapIndex;
        currentHandleSetMap = &handleSetMaps[currentHandleSetMapIndex];
        nextHandleSetMap = &handleSetMaps[1 - currentHandleSetMapIndex];
        hasValidNextHandleSetMap = false;
    }
    currentHandleSet = 0;
    
    if(doRenderingCommands){
        if(isPicking){
            pickingProgram.use();
            currentNodePath.clear();
            pickingNodePathList.clear();
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        } else {
            phongProgram.use();
            const Vector3f& c = self->backgroundColor();
            glClearColor(c[0], c[1], c[2], 1.0f);
        }
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    self->extractPreproNodes();

    if(doRenderingCommands){
        SgCamera* camera = self->currentCamera();
        if(camera){
            beginActualRendering(camera);
        }
    }
}


void GLSLSceneRendererImpl::beginActualRendering(SgCamera* camera)
{
    const Affine3& cameraPosition = self->currentCameraPosition();
    
    renderCamera(camera, cameraPosition);

    if(!isPicking){
        renderLights(cameraPosition);
    }

    clearGLState();
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
    modelViewStack.clear();
    modelViewStack.push_back(viewMatrix);
}


void GLSLSceneRendererImpl::renderLights(const Affine3& cameraPosition)
{
    SgLight* headLight = self->headLight();
    if(headLight->on()){
        renderLight(headLight, 0, cameraPosition);
    }

    //const int numLights = std::min(self->numLights(), (int)(maxLights - numSystemLights));
    const int numLights = 0;

    for(int i=0; i < numLights; ++i){
        SgLight* light;
        Affine3 T;
        self->getLightInfo(i, light, T);
        renderLight(light, i + 1, T);
    }
}


void GLSLSceneRendererImpl::renderLight(const SgLight* light, int index, const Affine3& T)
{
    if(index != 0){
        return;
    }
        
    bool isValid = false;

    if(light->on()){
        
        if(const SgDirectionalLight* dirLight = dynamic_cast<const SgDirectionalLight*>(light)){
            isValid = true;
            Vector3 d = viewMatrix.linear() * T.linear() * -dirLight->direction();
            Vector4f pos(d.x(), d.y(), d.z(), 0.0f);
            glUniform4fv(LightPositionLocation, 1, pos.data());
            
        } else if(const SgPointLight* pointLight = dynamic_cast<const SgPointLight*>(light)){
            if(const SgSpotLight* spotLight = dynamic_cast<const SgSpotLight*>(pointLight)){

            } else {
            
            }
        }
    }
        
    if(isValid){
        Vector3f intensity(light->intensity() * light->color());
        glUniform3fv(LightIntensityLocation, 1, intensity.data());
    }
}


void GLSLSceneRenderer::endRendering()
{
    impl->endRendering();
}


void GLSLSceneRendererImpl::endRendering()
{
    if(isCheckingUnusedHandleSets){
        currentHandleSetMap->clear();
        hasValidNextHandleSetMap = true;
    }
}


void GLSLSceneRenderer::render()
{
    impl->render();
}


void GLSLSceneRendererImpl::render()
{
    beginRendering(true);

    self->sceneRoot()->accept(*self);

    endRendering();
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
    render();
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
    float r = (id & 0xff) / 255.0;
    float g = ((id >> 8) & 0xff) / 255.0;
    float b = ((id >> 16) & 0xff) / 255.0;
    if(SHOW_IMAGE_FOR_PICKING){
        b = 1.0f;
    }
    glUniform3f(PickingIDLocation, r, g, b);
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

    Affine3Array& modelViewStack = impl->modelViewStack;
    modelViewStack.push_back(modelViewStack.back() * T);

    visitGroup(transform);
    
    modelViewStack.pop_back();
}


void GLSLSceneRendererImpl::setRenderingUniformMatrixVariables()
{
    const Affine3f MV = modelViewStack.back().cast<float>();
    const Matrix3f N = MV.linear();
    const Matrix4f MVP = (projectionMatrix * modelViewStack.back().matrix()).cast<float>();

    if(useUniformBlockToPassTransformationMatrices){
        transformBlockBuffer.write(modelViewMatrixIndex, MV);
        transformBlockBuffer.write(normalMatrixIndex, N);
        transformBlockBuffer.write(MVPIndex, MVP);
        transformBlockBuffer.flush();
    } else {
        glUniformMatrix4fv(ModelViewMatrixLocation, 1, GL_FALSE, MV.data());
        glUniformMatrix3fv(NormalMatrixLocation, 1, GL_FALSE, N.data());
        glUniformMatrix4fv(MVPLocation, 1, GL_FALSE, MVP.data());
    }
}


void GLSLSceneRendererImpl::setPickingUniformMatrixVariables()
{
    const Matrix4f MVP = (projectionMatrix * modelViewStack.back().matrix()).cast<float>();
    glUniformMatrix4fv(PickingMVPLocation, 1, GL_FALSE, MVP.data());
}


void GLSLSceneRendererImpl::visitShape(SgShape* shape)
{
    SgMesh* mesh = shape->mesh();
    if(mesh){
        if(mesh->hasVertices()){
            SgMaterial* material = shape->material();
            pushPickID(shape);
            if(!isPicking){
                renderMaterial(material);
            }
            renderMesh(mesh, false);
            popPickID();
        }
    }
}


void GLSLSceneRenderer::visitUnpickableGroup(SgUnpickableGroup* group)
{
    if(!impl->isPicking){
        visitGroup(group);
    }
}


void GLSLSceneRenderer::visitShape(SgShape* shape)
{
    impl->visitShape(shape);
}


void GLSLSceneRendererImpl::renderMaterial(const SgMaterial* material)
{
    if(!material){
        material = defaultMaterial;
    }
    
    float alpha = 1.0 - material->transparency();

    Vector4f color;
    color << material->diffuseColor(), alpha;
    setDiffuseColor(color);
        
    color.head<3>() *= material->ambientIntensity();
    setAmbientColor(color);

    color << material->emissiveColor(), alpha;
    setEmissionColor(color);
    
    color << material->specularColor(), alpha;
    setSpecularColor(color);
    
    float shininess = (127.0 * material->shininess()) + 1.0;
    setShininess(shininess);
}


bool GLSLSceneRendererImpl::renderTexture(SgTexture* texture, bool withMaterial)
{
    return false;
}


void GLSLSceneRenderer::onImageUpdated(SgImage* image)
{

}


void GLSLSceneRendererImpl::renderMesh(SgMesh* mesh, bool hasTexture)
{
    bool isNewMesh = false;
    HandleSet* handleSet;
    HandleSetMap::iterator p = currentHandleSetMap->find(mesh);
    if(p == currentHandleSetMap->end()){
        p = currentHandleSetMap->insert(HandleSetMap::value_type(mesh, new HandleSet)).first;
        isNewMesh = true;
    }
    handleSet = p->second;

    if(isCheckingUnusedHandleSets){
        nextHandleSetMap->insert(*p);
    }

    if(!isPicking){
        setRenderingUniformMatrixVariables();
    } else {
        setPickingUniformMatrixVariables();
    }

    glBindVertexArray(handleSet->vao);

    if(isNewMesh){
        createMeshVertexArray(mesh, handleSet);
    }

    glDrawArrays(GL_TRIANGLES, 0, handleSet->numVertices);
}


void GLSLSceneRendererImpl::createMeshVertexArray(SgMesh* mesh, HandleSet* handleSet)
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
    renderPlot(pointSet, *pointSet->vertices(), (GLenum)GL_POINTS);
    if(s > 0.0){
        setPointSize(s);
    }
}


void GLSLSceneRendererImpl::renderPlot(SgPlot* plot, SgVertexArray& expandedVertices, GLenum primitiveMode)
{
    
}


void GLSLSceneRenderer::visitLineSet(SgLineSet* lineSet)
{
    impl->visitLineSet(lineSet);
}


void GLSLSceneRendererImpl::visitLineSet(SgLineSet* lineSet)
{
    const int n = lineSet->numLines();
    if(!lineSet->hasVertices() || (n <= 0)){
        return;
    }

    /*
    const SgVertexArray& orgVertices = *lineSet->vertices();
    SgVertexArray& vertices = buf->vertices;
    vertices.clear();
    vertices.reserve(n * 2);
    for(int i=0; i < n; ++i){
        SgLineSet::LineRef line = lineSet->line(i);
        vertices.push_back(orgVertices[line[0]]);
        vertices.push_back(orgVertices[line[1]]);
    }

    const double w = lineSet->lineWidth();
    if(w > 0.0){
        setLineWidth(w);
    }
    renderPlot(lineSet, vertices, GL_LINES);
    if(w > 0.0){
        setLineWidth(defaultLineWidth);
    }
    */
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
}


void GLSLSceneRendererImpl::setColor(const Vector4f& color)
{
    if(!isPicking){
        if(!stateFlag[CURRENT_COLOR] || color != currentColor){
            //glColor4f(color[0], color[1], color[2], color[3]);
            currentColor = color;
            stateFlag.set(CURRENT_COLOR);
        }
    }
}


void GLSLSceneRenderer::setColor(const Vector4f& color)
{
    impl->setColor(color);
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
        glUniform3fv(DiffuseColorLocation, 1, color.data());
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
        glUniform3fv(AmbientColorLocation, 1, color.data());
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
        //glUniform3fv(EmissionLocation, 1, color.data());
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
        glUniform3fv(SpecularColorLocation, 1, color.data());
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
        glUniform1f(ShininessLocation, s);
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
    /*
    if(width != impl->defaultLineWidth){
        impl->defaultLineWidth = width;
    }
    */
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
        impl->nextHandleSetMap->clear();
    }
    impl->doUnusedHandleSetCheck = on;
}


const Affine3& GLSLSceneRenderer::currentModelTransform() const
{
    impl->currentModelTransform = impl->viewMatrix.inverse() * impl->modelViewStack.back();
    return impl->currentModelTransform;
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
