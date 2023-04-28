#ifndef CNOID_BASE_GL_FREE_TYPE_H
#define CNOID_BASE_GL_FREE_TYPE_H

#include "glcore.h"
#include <ft2build.h>
#include FT_FREETYPE_H
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>

namespace cnoid {

class MessageOut;

class GLFreeType
{
public:
    GLFreeType();
    ~GLFreeType();
    void clearGL(bool isGLContextActive);
    void setMessageOut(MessageOut* mo);
    bool initializeGL(const char* faceName, int resolution, GLuint glTextureUnit);
    bool setText(const std::string& text, float textHeight);

    struct Vertex {
        GLfloat x;
        GLfloat y;
        GLfloat s;
        GLfloat t;
        Vertex(GLfloat x, GLfloat y, GLfloat s, GLfloat t)
            : x(x), y(y), s(s), t(t) { }
    };

    GLuint textureId() const { return textureId_; }
    GLuint samplerId() const { return samplerId_; }

    const std::vector<Vertex> vertices() const { return vertices_; }
    int vertexDataSize() const { return vertices_.size() * sizeof(Vertex); }

private:
    FT_Library ft;
    FT_Face face;
    int resolution;
    GLuint textureUnit;
    GLuint textureId_;
    GLuint textureWidth;
    GLuint textureHeight;
    GLuint nextCharacterX;
    GLuint nextCharacterY;
    GLuint maxCharacterHeight;
    GLuint samplerId_;

    struct CharaInfo {
        float ax;
        float ay;
        float width;
        float height;
        float left;
        float top;
        float offsetX;
        float offsetY;
    };

    std::vector<std::unique_ptr<CharaInfo>> asciiCharaInfos;
    std::unordered_map<int, std::unique_ptr<CharaInfo>> charaInfoMap;

    std::vector<Vertex> vertices_;

    MessageOut* messageOut;

    CharaInfo* createCharacter(int character, bool& isTextureActivated);
};

}

#endif
