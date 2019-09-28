/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_GLSL_PROGRAM_H
#define CNOID_BASE_GLSL_PROGRAM_H

#include "glcore.h"
#include <cnoid/EigenTypes>
#include <vector>
#include <string>
#include <cstring>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT GLSLProgram
{
public:
    GLSLProgram();
    GLSLProgram(const GLSLProgram&) = delete;
    ~GLSLProgram() = default;
    GLSLProgram& operator=(const GLSLProgram&) = delete;

    void release();
    void loadVertexShader(const char* filename);
    void loadFragmentShader(const char* filename);
    void loadShader(const char* filename, int shaderType);
    void link();
    void validate();
    void use();
    
    GLint handle() const { return programHandle; }
    bool isLinked() const { return isLinked_; }

    GLuint getUniformLocation(const char* name) const {
        return glGetUniformLocation(programHandle, name);
    }
    GLuint getUniformLocation(const std::string& name) const {
        return glGetUniformLocation(programHandle, name.c_str());
    }
    GLuint getSubroutineIndex(GLenum shaderType, const GLchar* name) const {
#ifdef CNOID_GL_CORE_4_4
        return glGetSubroutineIndex(programHandle, shaderType, name);
#else
        return -1;
#endif
    }
    void setUniformSubroutines(GLenum shaderType, GLsizei count, const GLuint *indices){
#ifdef CNOID_GL_CORE_4_4
        return glUniformSubroutinesuiv(shaderType, count, indices);
#endif
    }

private:
    GLuint programHandle;
    bool isLinked_;
};


class CNOID_EXPORT GLSLUniformBlockBuffer
{
public:
    GLSLUniformBlockBuffer();
    ~GLSLUniformBlockBuffer();
    
    bool initialize(GLSLProgram& program, const std::string& blockName);

    /**
       @return uniform index
    */
    GLuint checkUniform(const char* name);    

    GLuint checkUniformMatrix(const char* name);    
    
    void bind(GLSLProgram& program, GLuint bindingPoint) {
        GLuint blockIndex = glGetUniformBlockIndex(program.handle(), blockName.c_str());
        glUniformBlockBinding(program.handle(), blockIndex, bindingPoint);
    }

    void bindBufferBase(GLuint bindingPoint) {
        glBindBufferBase(GL_UNIFORM_BUFFER, bindingPoint, uboHandle);
    }

    void write(GLuint index, float v){
        std::memcpy(&localBuffer[infos[index].offset], &v, sizeof(v));
    }

    void write(GLuint index, const Vector3f& v){
        std::memcpy(&localBuffer[infos[index].offset], v.data(), sizeof(v));
    }

    void write(GLuint index, const Vector4f& v){
        std::memcpy(&localBuffer[infos[index].offset], v.data(), sizeof(v));
    }

    void write(GLuint index, const Matrix3f& M){
        const UniformInfo& info = infos[index];
        const int size = 3 * sizeof(Matrix3f::Scalar);
        std::memcpy(&localBuffer[info.offset], &M(0, 0), size);
        std::memcpy(&localBuffer[info.offset + info.matrixStrides], &M(0, 1), size);
        std::memcpy(&localBuffer[info.offset + info.matrixStrides * 2], &M(0, 2), size);
    }

    void write(GLuint index, const Matrix4f& M){
        std::memcpy(&localBuffer[infos[index].offset], M.data(), sizeof(M));
    }
    
    void write(GLuint index, const Affine3f& T){
        std::memcpy(&localBuffer[infos[index].offset], T.matrix().data(), sizeof(T));
    }

    void flush(){
        glBindBuffer(GL_UNIFORM_BUFFER, uboHandle);
        glBufferSubData(GL_UNIFORM_BUFFER, 0, localBuffer.size(), &localBuffer[0]);
    }

private:
    GLuint uboHandle;
    std::vector<GLubyte> localBuffer;
    struct UniformInfo {
        GLint offset;
        GLint matrixStrides;
    };
    std::vector<UniformInfo> infos;
    GLuint lastProgramHandle;
    std::string blockName;
};

}

#endif
