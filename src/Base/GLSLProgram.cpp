/**
   @author Shin'ichiro Nakaoka
*/

#include "GLSLProgram.h"
#include <QFile>
#include <fmt/format.h>
#include <stdexcept>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;


GLSLProgram::GLSLProgram()
{
    programHandle = 0;
    isLinked_ = false;
}


void GLSLProgram::release()
{
    if(programHandle){

        GLint numShaders = 0;
        glGetProgramiv(programHandle, GL_ATTACHED_SHADERS, &numShaders);

        if(numShaders > 0){
            vector<GLuint> shaderNames(numShaders);
            glGetAttachedShaders(programHandle, numShaders, NULL, &shaderNames.front());

            for(GLint i = 0; i < numShaders; i++){
                glDeleteShader(shaderNames[i]);
            }
        }
        
        glDeleteProgram(programHandle);
        programHandle = 0;
        isLinked_ = false;
    }
}


void GLSLProgram::loadVertexShader(const char* filename)
{
    loadShader(filename, GL_VERTEX_SHADER);
}


void GLSLProgram::loadFragmentShader(const char* filename)
{
    loadShader(filename, GL_FRAGMENT_SHADER);
}


void GLSLProgram::loadShader(const char* filename, int shaderType)
{
    QFile file(filename);

    if(!file.exists()){
        throw std::runtime_error(format(_("Shader \"{}\" is not found."), filename));
    }
    
    file.open(QIODevice::ReadOnly);
    const QByteArray data = file.readAll();
    const GLchar* codes[] = { data.data() };
    const GLint codeSizes[] = { data.size() };

    GLuint shaderHandle = glCreateShader(shaderType);
    
    glShaderSource(shaderHandle, 1, codes, codeSizes);
    glCompileShader(shaderHandle);

    GLint result;
    glGetShaderiv(shaderHandle, GL_COMPILE_STATUS, &result);
    if(result == GL_FALSE){
        string msg;
        GLint length;
        glGetShaderiv(shaderHandle, GL_INFO_LOG_LENGTH, &length);
        if(length > 0){
            vector<char> log(length);
            GLsizei written;
            glGetShaderInfoLog(shaderHandle, length, &written, &log[0]);
            msg = format(_("Shader compilation of \"{0}\" failed.\n{1}"), filename, &log[0]);
        } else {
            msg = format(_("Shader compilation of \"{}\" failed."), filename);
        }
        glDeleteShader(shaderHandle);
        throw std::runtime_error(msg);

    } else {
        if(!programHandle){
            programHandle = glCreateProgram();
            if(!programHandle){
                throw std::runtime_error(_("Unable to create shader program."));
            }
        }
        glAttachShader(programHandle, shaderHandle);
    }
}


void GLSLProgram::link()
{
    if(isLinked_){
        return;
    }
    
    if(!programHandle){
        throw std::runtime_error(_("Program has not been compiled."));
    }

    glLinkProgram(programHandle);

    GLint status;
    glGetProgramiv(programHandle, GL_LINK_STATUS, &status);
    if(status == GL_FALSE){
        string msg;
        GLint length;
        glGetProgramiv(programHandle, GL_INFO_LOG_LENGTH, &length);
        if(length > 0){
            vector<char> log(length);
            GLsizei written;
            glGetProgramInfoLog(programHandle, length, &written, &log[0]);
            msg = format("Program link failed:\n{}", &log[0]);
        } else {
            msg = _("Program link failed.");
        }
        throw std::runtime_error(msg);
    }

    // uniformLocations.clear();

    isLinked_ = true;
}


void GLSLProgram::validate()
{
    if(!programHandle || !isLinked_){
        throw std::runtime_error(_("Program is not linked"));
    }

    GLint status;
    glValidateProgram(programHandle);
    glGetProgramiv(programHandle, GL_VALIDATE_STATUS, &status);
    if(status == GL_FALSE){
        string msg;
        int length = 0;
        glGetProgramiv(programHandle, GL_INFO_LOG_LENGTH, &length);
        if(length > 0){
            vector<char> log(length);
            GLsizei written;
            glGetProgramInfoLog(programHandle, length, &written, &log[0]);
            msg = format("Program failed to validate\n{}", &log[0]);
        } else {
            msg = _("Program failed to validate");
        }
        throw std::runtime_error(msg);
    }
}


void GLSLProgram::use()
{
    if(!programHandle || !isLinked_){
        throw std::runtime_error(_("Shader has not been linked."));
    }
    glUseProgram(programHandle);
}


GLSLUniformBlockBuffer::GLSLUniformBlockBuffer()
{
    uboHandle = 0;
    lastProgramHandle = 0;
}


GLSLUniformBlockBuffer::~GLSLUniformBlockBuffer()
{

}


bool GLSLUniformBlockBuffer::initialize(GLSLProgram& program, const std::string& blockName)
{
    this->blockName = blockName;
    
    lastProgramHandle = program.handle();
    
    GLuint blockIndex = glGetUniformBlockIndex(program.handle(), blockName.c_str());
    if(blockIndex == GL_INVALID_INDEX){
        return false;
    }
    
    GLint blockSize;
    glGetActiveUniformBlockiv(program.handle(), blockIndex, GL_UNIFORM_BLOCK_DATA_SIZE, &blockSize);
    localBuffer.resize(blockSize);

    glGenBuffers(1, &uboHandle);
    glBindBuffer(GL_UNIFORM_BUFFER, uboHandle);
    glBufferData(GL_UNIFORM_BUFFER, localBuffer.size(), NULL, GL_DYNAMIC_DRAW);

    return true;
}


GLuint GLSLUniformBlockBuffer::checkUniform(const char* name)
{
    GLuint index;
    glGetUniformIndices(lastProgramHandle, 1, &name, &index);

    if(index >= infos.size()){
        infos.resize(index + 1);
    }
    
    glGetActiveUniformsiv(lastProgramHandle, 1, &index, GL_UNIFORM_OFFSET, &(infos[index].offset));

    return index;
}


GLuint GLSLUniformBlockBuffer::checkUniformMatrix(const char* name)
{
    GLuint index = checkUniform(name);
    glGetActiveUniformsiv(lastProgramHandle, 1, &index, GL_UNIFORM_MATRIX_STRIDE, &(infos[index].matrixStrides));

    return index;
}
