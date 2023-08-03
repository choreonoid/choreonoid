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
    int size1 = ::MultiByteToWideChar(CP_UTF8, 0, text.c_str(), text.size(), NULL, 0);
    if(size1 >= 0){
        std::vector<wchar_t> wtext(size1 + 1);
        ::MultiByteToWideChar(CP_UTF8, 0, text.c_str(), text.size(), &wtext[0], size1 + 1);
        int size2 = ::WideCharToMultiByte(codepage, 0, &wtext[0], size1, NULL, 0, NULL, NULL);
        if(size2 >= 0){
            std::vector<char> converted(size2 + 1);
            ::WideCharToMultiByte(codepage, 0, &wtext[0], size1, &converted[0], size2 + 1, NULL, NULL);
            return std::string(&converted[0], size2);
        }
    }
    return text;
}

const std::string fromUTF8(const char* text)
{
    if(isUtf8Codepage()){
        return text;
    }
    int size1 = ::MultiByteToWideChar(CP_UTF8, 0, text, -1, NULL, 0);
    if(size1 >= 0){
        std::vector<wchar_t> wtext(size1 + 1);
        ::MultiByteToWideChar(CP_UTF8, 0, text, -1, &wtext[0], size1 + 1);
        int size2 = ::WideCharToMultiByte(codepage, 0, &wtext[0], size1, NULL, 0, NULL, NULL);
        if(size2 >= 0){
            std::vector<char> converted(size2 + 1);
            ::WideCharToMultiByte(codepage, 0, &wtext[0], size1, &converted[0], size2 + 1, NULL, NULL);
            return std::string(&converted[0], size2 - 1 /* Cut the last null character */);
        }
    }
    return text;
}
    
const std::string toUTF8(const std::string& text)
{
    if(isUtf8Codepage()){
        return text;
    }
    int size1 = ::MultiByteToWideChar(codepage, 0, text.c_str(), text.size(), NULL, 0);
    if(size1 >= 0){
        std::vector<wchar_t> wtext(size1 + 1);
        ::MultiByteToWideChar(codepage, 0, text.c_str(), text.size(), &wtext[0], size1 + 1);
        int size2 = ::WideCharToMultiByte(CP_UTF8, 0, &wtext[0], size1, NULL, 0, NULL, NULL);
        if(size2 >= 0){
            std::vector<char> converted(size2 + 1);
            ::WideCharToMultiByte(CP_UTF8, 0, &wtext[0], size1, &converted[0], size2 + 1, NULL, NULL);
            return std::string(&converted[0], size2);
        }
    }
    return text;
}

const std::string toUTF8(const char* text)
{
    if(isUtf8Codepage()){
        return text;
    }
    int size1 = ::MultiByteToWideChar(codepage, 0, text, -1, NULL, 0);
    if(size1 >= 0){
        std::vector<wchar_t> wtext(size1 + 1);
        ::MultiByteToWideChar(codepage, 0, text, -1, &wtext[0], size1 + 1);
        int size2 = ::WideCharToMultiByte(CP_UTF8, 0, &wtext[0], size1, NULL, 0, NULL, NULL);
        if(size2 >= 0){
            std::vector<char> converted(size2 + 1);
            ::WideCharToMultiByte(CP_UTF8, 0, &wtext[0], size1, &converted[0], size2 + 1, NULL, NULL);
            return std::string(&converted[0], size2 - 1 /* Cut the last null character */);
        }
    }
    return text;
}

}

#endif
