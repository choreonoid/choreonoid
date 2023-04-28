#include "GLFreeType.h"
#include <cnoid/MessageOut>
#include <fmt/format.h>
#include FT_ERRORS_H
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;


static const char* getFreeTypeErrorString(FT_Error error_code)
{
#if FREETYPE_MAJOR >= 2 && FREETYPE_MINOR >= 10
    return FT_Error_String(error_code);
#else
    return nullptr;
#endif
}


GLFreeType::GLFreeType()
{
    ft = nullptr;
    face = nullptr;
    textureUnit = 0;
    textureId_ = 0;
    textureWidth = 1024;
    textureHeight = 1024;
    nextCharacterX = 0;
    nextCharacterY = 0;
    maxCharacterHeight = 0;
    samplerId_ = 0;
    messageOut = MessageOut::master();
}


GLFreeType::~GLFreeType()
{
    if(face){
        FT_Done_Face(face);
    }
    if(ft){
        FT_Done_FreeType(ft);
    }
}


void GLFreeType::clearGL(bool isGLContextActive)
{
    if(!isGLContextActive){
        textureId_ = 0;
        samplerId_ = 0;
    } else {
        if(textureId_){
            glDeleteTextures(1, &textureId_);
            textureId_ = 0;
        }
        if(samplerId_){
            glDeleteSamplers(1, &samplerId_);
            samplerId_ = 0;
        }
    }
    asciiCharaInfos.clear();
    charaInfoMap.clear();
}


void GLFreeType::setMessageOut(MessageOut* mo)
{
    messageOut = mo;
}


bool GLFreeType::initializeGL(const char* faceName, int resolution, GLuint textureUnit)
{
    if(!ft){
        if(auto error = FT_Init_FreeType(&ft)){
            if(auto message = getFreeTypeErrorString(error)){
                messageOut->putErrorln(message);
            } else {
                messageOut->putErrorln(format(_("FT_Init_FreeType failed with error code {0}."), error));
            }
            ft = nullptr;
        }
    }
    if(!ft){
        return false;
    }
    
    clearGL(true);

    this->resolution = resolution;
    
    if(face){
        FT_Done_Face(face);
        face = nullptr;
    }
    
    if(auto error = FT_New_Face(ft, faceName, 0, &face)){
        if(auto message = getFreeTypeErrorString(error)){
            messageOut->putErrorln(message);
        } else {
            messageOut->putErrorln(format(_("FT_New_Face for \"{0}\" failed with error code {1}."), faceName, error));
        }
        face = nullptr;

    } else {
        FT_Set_Pixel_Sizes(face, 0, resolution);

        this->textureUnit = textureUnit;
        glActiveTexture(GL_TEXTURE0 + textureUnit);
        glGenTextures(1, &textureId_);
        glBindTexture(GL_TEXTURE_2D, textureId_);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, textureWidth, textureHeight, 0, GL_RED, GL_UNSIGNED_BYTE, nullptr);

        glGenSamplers(1, &samplerId_);
        glBindSampler(textureUnit, samplerId_);
        glSamplerParameteri(samplerId_, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glSamplerParameteri(samplerId_, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glSamplerParameteri(samplerId_, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glSamplerParameteri(samplerId_, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    }
    
    return face != nullptr;
}


bool GLFreeType::setText(const std::string& text, float textHeight)
{
    vertices_.clear();

    if(!face){
        return false;
    }
        
    vertices_.reserve(6 * text.size());

    if(asciiCharaInfos.empty()){
        asciiCharaInfos.resize(128);
    }

    double x = 0.0;
    double y = 0.0;
    double sy = textHeight / resolution;
    double sx = sy;
    bool isTextureActivated = false;

    for(size_t i=0; i < text.size(); ++i){
        CharaInfo* info = nullptr;
        // TODO: Get a unicode character
        int character = text[i];
        if(character < 128){
            info = asciiCharaInfos[character].get();
            if(!info && character >= 32){
                info = createCharacter(character, isTextureActivated);
                asciiCharaInfos[character].reset(info);
            }
        }
        if(info){
            float x2 = x + info->left * sx;
            float y2 = -y - info->top * sy;
            float w = info->width * sx;
            float h = info->height * sy;

            x += info->ax * sx;
            y += info->ay * sy;

            if(!w || !h){
                continue;
            }

            double s1 = info->offsetX;
            double s2 = info->offsetX + info->width / textureWidth;
            double t1 = info->offsetY;
            double t2 = info->offsetY + info->height / textureHeight;

            vertices_.emplace_back(x2,     -y2,     s1, t1);
            vertices_.emplace_back(x2 + w, -y2,     s2, t1);
            vertices_.emplace_back(x2,     -y2 - h, s1, t2);
            vertices_.emplace_back(x2 + w, -y2,     s2, t1);
            vertices_.emplace_back(x2,     -y2 - h, s1, t2);
            vertices_.emplace_back(x2 + w, -y2 - h, s2, t2);
        }
    }

    return true;
}


GLFreeType::CharaInfo* GLFreeType::createCharacter(int character, bool& isTextureActivated)
{
    CharaInfo* info = nullptr;
    
    if(FT_Load_Char(face, character, FT_LOAD_RENDER)){
        return info;
    }

    if(!isTextureActivated){
        glActiveTexture(GL_TEXTURE0 + textureUnit);
        glBindTexture(GL_TEXTURE_2D, textureId_);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        isTextureActivated = true;
    }

    FT_GlyphSlot g = face->glyph;
    FT_Bitmap bitmap = g->bitmap;
    glTexSubImage2D(
        GL_TEXTURE_2D, 0, nextCharacterX, nextCharacterY,
        bitmap.width, bitmap.rows, GL_RED, GL_UNSIGNED_BYTE, bitmap.buffer);

    info = new CharaInfo;
    info->ax = g->advance.x >> 6;
    info->ay = g->advance.y >> 6;
    info->width = bitmap.width;
    info->height = bitmap.rows;
    info->left = g->bitmap_left;
    info->top = g->bitmap_top;
    info->offsetX = static_cast<float>(nextCharacterX) / textureWidth;
    info->offsetY = static_cast<float>(nextCharacterY) / textureHeight;

    // One pixel size must be added as a margin between characters to avoid
    // rendering noizy lines at character boundaries.
    nextCharacterX += g->bitmap.width + 1;
    
    maxCharacterHeight = std::max(maxCharacterHeight, bitmap.rows);

    return info;
}
