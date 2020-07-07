/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifdef _WIN32

#include "UTF8.h"
#include <vector>
#include <windows.h>
#include <mbctype.h>

namespace {

int codepage = -1;

bool isUtf8Codepage()
{
    if(codepage < 0){
        codepage = _getmbcp();
    }
    return codepage == CP_UTF8;
}

}

namespace cnoid {

const std::string fromUTF8(const std::string& text)
{
    if(isUtf8Codepage()){
        return text;
    }
    int size = ::MultiByteToWideChar(CP_UTF8, 0, text.c_str(), text.size(), NULL, 0);
    if(size >= 0){
        std::vector<wchar_t> wtext(size + 1);
        ::MultiByteToWideChar(CP_UTF8, 0, text.c_str(), text.size(), &wtext[0], size + 1);
        size = ::WideCharToMultiByte(codepage, 0, &wtext[0], size, NULL, 0, NULL, NULL);
        if(size >= 0){
            std::vector<char> converted(size + 1);
            ::WideCharToMultiByte(codepage, 0, &wtext[0], size, &converted[0], size + 1, NULL, NULL);
            return std::string(&converted[0], size);
        }
    }
    return text;
}

const std::string fromUTF8(const char* text)
{
    if(isUtf8Codepage()){
        return text;
    }
    int size = ::MultiByteToWideChar(CP_UTF8, 0, text, -1, NULL, 0);
    if(size >= 0){
        std::vector<wchar_t> wtext(size + 1);
        ::MultiByteToWideChar(CP_UTF8, 0, text, -1, &wtext[0], size + 1);
        size = ::WideCharToMultiByte(codepage, 0, &wtext[0], size, NULL, 0, NULL, NULL);
        if(size >= 0){
            std::vector<char> converted(size + 1);
            ::WideCharToMultiByte(codepage, 0, &wtext[0], size, &converted[0], size + 1, NULL, NULL);
            return std::string(&converted[0], size);
        }
    }
    return text;
}
    
const std::string toUTF8(const std::string& text)
{
    if(isUtf8Codepage()){
        return text;
    }
    int size = ::MultiByteToWideChar(codepage, 0, text.c_str(), text.size(), NULL, 0);
    if(size >= 0){
        std::vector<wchar_t> wtext(size + 1);
        ::MultiByteToWideChar(codepage, 0, text.c_str(), text.size(), &wtext[0], size + 1);
        size = ::WideCharToMultiByte(CP_UTF8, 0, &wtext[0], size, NULL, 0, NULL, NULL);
        if(size >= 0){
            std::vector<char> converted(size + 1);
            ::WideCharToMultiByte(CP_UTF8, 0, &wtext[0], size, &converted[0], size + 1, NULL, NULL);
            return std::string(&converted[0], size);
        }
    }
    return text;
}

const std::string toUTF8(const char* text)
{
    if(isUtf8Codepage()){
        return text;
    }
    int size = ::MultiByteToWideChar(codepage, 0, text, -1, NULL, 0);
    if(size >= 0){
        std::vector<wchar_t> wtext(size + 1);
        ::MultiByteToWideChar(codepage, 0, text, -1, &wtext[0], size + 1);
        size = ::WideCharToMultiByte(CP_UTF8, 0, &wtext[0], size, NULL, 0, NULL, NULL);
        if(size >= 0){
            std::vector<char> converted(size + 1);
            ::WideCharToMultiByte(CP_UTF8, 0, &wtext[0], size, &converted[0], size + 1, NULL, NULL);
            return std::string(&converted[0], size);
        }
    }
    return text;
}

}

#endif
