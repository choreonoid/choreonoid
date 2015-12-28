/**
   @author Shin'ichiro Nakaoka
*/

#include "GLSLTestView.h"
#include "MessageView.h"
#include "ViewManager.h"
#include "gl_core_3_3.h"
#include <cnoid/EigenTypes>
#include <QGLWidget>
#include <QBoxLayout>
#include <QFile>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

const int SampleIndex = 0;

}

namespace cnoid {

class GLSLTestViewImpl : public QGLWidget
{
public:
    GLSLTestView* self;
    ostream& os;
    GLuint vertexShader;
    GLuint fragmentShader;
    GLuint vaoHandle;
    GLuint rotationMatrixLocation;
    float angle;

    GLSLTestViewImpl(QGLFormat& format, GLSLTestView* self);
    ~GLSLTestViewImpl();
    GLint createShader(GLenum shaderType, const string& filename);
    virtual void initializeGL();
    virtual void resizeGL(int width, int height);
    virtual void paintGL();
};

}

void GLSLTestView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<GLSLTestView>(
        "GLSLTestView", N_("GLSLTest"), ViewManager::SINGLE_DEFAULT);
}


GLSLTestView::GLSLTestView()
{
    QGLFormat format;
    format.setVersion(3, 3);
    format.setProfile(QGLFormat::CoreProfile);
    
    impl = new GLSLTestViewImpl(format, this);

    QVBoxLayout* vbox = new QVBoxLayout();
    vbox->setContentsMargins(0, 0, 0, 0);
    vbox->addWidget(impl);
    setLayout(vbox);
}


GLSLTestViewImpl::GLSLTestViewImpl(QGLFormat& format, GLSLTestView* self)
    : QGLWidget(format, self),
      self(self),
      os(MessageView::mainInstance()->cout())
{
    setAutoBufferSwap(true);
    setAutoFillBackground(false);
    setFocusPolicy(Qt::WheelFocus);
    setMouseTracking(true);

    vertexShader = 0;
    fragmentShader = 0;
    vaoHandle = 0;
    rotationMatrixLocation = -1;
    angle = 0.0;
}


GLSLTestView::~GLSLTestView()
{
    delete impl;
}


GLSLTestViewImpl::~GLSLTestViewImpl()
{
        
}


GLint GLSLTestViewImpl::createShader(GLenum shaderType, const string& filename)
{
    GLint shader = glCreateShader(shaderType);
    if(!shader){
        os << "Error creating shader." << endl;
    } else {
        QFile file(filename.c_str());
        file.open(QIODevice::ReadOnly);
        const QByteArray data = file.readAll();
        const GLchar* codes[] = { data.data() };
        const GLint codeSizes[] = { data.size() };
        glShaderSource(shader, 1, codes, codeSizes);
        glCompileShader(shader);

        GLint result;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &result);
        if(result == GL_FALSE){
            os << "Shader compilation of \"" << filename << "\" failed!\n" << endl;
            GLint logLen;
            glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logLen);
            if(logLen > 0){
                char* log = new char[logLen];
                GLsizei written;
                glGetShaderInfoLog(shader, logLen, &written, log);
                os << "Shader log:\n" << log << endl;
                delete [] log;
            }
            glDeleteShader(shader);
            shader = 0;
        }
    }
    return shader;
}
    

void GLSLTestViewImpl::initializeGL()
{
    if(ogl_LoadFunctions() == ogl_LOAD_FAILED){
        hide();
        return;
    }

    GLint major, minor;
    glGetIntegerv(GL_MAJOR_VERSION, &major);
    glGetIntegerv(GL_MINOR_VERSION, &minor);
    os << "OpenGL version is " << major << "." << minor << "." << endl;

    if(major >= 2){
        os << "GLSL version is " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;
    }

    string vertexShaderFilename;
    string fragmentShaderFilename;

    if(SampleIndex == 0){
        vertexShaderFilename = ":/Base/shader/basic.vert";
        fragmentShaderFilename = ":/Base/shader/basic.frag";
    } else if(SampleIndex == 1){
        vertexShaderFilename = ":/Base/shader/basic_uniformblock.vert";
        fragmentShaderFilename = ":/Base/shader/basic_uniformblock.frag";
    }

    vertexShader = createShader(GL_VERTEX_SHADER, vertexShaderFilename);
    if(!vertexShader){
        hide();
        return;
    }

    fragmentShader = createShader(GL_FRAGMENT_SHADER, fragmentShaderFilename);
    if(!fragmentShader){
        hide();
        return;
    }

    GLuint programHandle = glCreateProgram();
    if(!programHandle){
        os << "Error creating program object." << endl;
        hide();
        return;
    }

    glAttachShader(programHandle, vertexShader);
    glAttachShader(programHandle, fragmentShader);
    glLinkProgram(programHandle);
    GLint status;
    glGetProgramiv(programHandle, GL_LINK_STATUS, &status);
    if(status == GL_FALSE){
        os << "Failed to link shader program!" << endl;
        GLint logLen;
        glGetProgramiv(programHandle, GL_INFO_LOG_LENGTH, &logLen);
        if(logLen > 0){
            char* log = new char[logLen];
            GLsizei written;
            glGetProgramInfoLog(programHandle, logLen, &written, log);
            os << "Program log:\n" << log << endl;
            delete [] log;
        }
        hide();
        return;
    }

    glUseProgram(programHandle);

    rotationMatrixLocation = glGetUniformLocation(programHandle, "RotationMatrix");

    float positionData[] = {
        -0.8f, -0.8f, 0.0f,
        0.8f, -0.8f, 0.0f,
        0.0f,  0.8f, 0.0f };
    float colorData[] = {
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f };

    // Create and populate the buffer objects
    GLuint vboHandles[2];
    glGenBuffers(2, vboHandles);
    GLuint positionBufferHandle = vboHandles[0];
    GLuint colorBufferHandle = vboHandles[1];

    glBindBuffer(GL_ARRAY_BUFFER, positionBufferHandle);
    glBufferData(GL_ARRAY_BUFFER, 9 * sizeof(float), positionData, GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, colorBufferHandle);
    glBufferData(GL_ARRAY_BUFFER, 9 * sizeof(float), colorData, GL_STATIC_DRAW);

    // Create and set-up the vertex array object
    glGenVertexArrays(1, &vaoHandle);
    glBindVertexArray(vaoHandle);

    glEnableVertexAttribArray(0);  // Vertex position
    glEnableVertexAttribArray(1);  // Vertex color

    glBindBuffer(GL_ARRAY_BUFFER, positionBufferHandle);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (GLubyte*)NULL);

    glBindBuffer(GL_ARRAY_BUFFER, colorBufferHandle);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (GLubyte*)NULL);
    
    
    /*
    glClearColor( 0.5f, 0.5f, 0.5f, 1.0f);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    glFrontFace(GL_CCW);
    glDisable(GL_FOG);
    */
}


void GLSLTestViewImpl::resizeGL(int width, int height)
{
    glViewport(0, 0, width, height);
}


void GLSLTestViewImpl::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT);

    if(rotationMatrixLocation >= 0){
        Eigen::Affine3f T(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()));
        glUniformMatrix4fv(rotationMatrixLocation, 1, GL_FALSE, T.matrix().data());
        angle += 0.1;
    }

    glBindVertexArray(vaoHandle);
    glDrawArrays(GL_TRIANGLES, 0, 3);

    /*
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(degree(camera->fovy(aspectRatio)), aspectRatio, camera->nearDistance(), camera->farDistance());

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glEnable(GL_LIGHTING);

    glGetIntegerv(GL_MAX_LIGHTS, &maxLights);

    pos << T.translation().cast<float>(), 1.0f;
    glLightfv(id, GL_POSITION, pos.data());
            
    glLightf(id, GL_CONSTANT_ATTENUATION, pointLight->constantAttenuation());
    glLightf(id, GL_LINEAR_ATTENUATION, pointLight->linearAttenuation());
    glLightf(id, GL_QUADRATIC_ATTENUATION, pointLight->quadraticAttenuation());

    glDrawArrays(GL_TRIANGLES, 0, vertices.size());
    */
}
