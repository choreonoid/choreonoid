/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "UTF8.h"
#include <vector>

#ifdef _WIN32
#include <windows.h>
#include <mbctype.h>
#endif


namespace cnoid {

#ifdef _MSC_VER
const std::string fromUTF8(const std::string& text)
{
    int size = ::MultiByteToWideChar(CP_UTF8, 0, text.c_str(), text.size(), NULL, 0);
    if(size >= 0){
        std::vector<wchar_t> wtext(size + 1);
        ::MultiByteToWideChar(CP_UTF8, 0, text.c_str(), text.size(), &wtext[0], size + 1);

        int codepage = _getmbcp();
        size = ::WideCharToMultiByte(codepage, 0,  &wtext[0], size, NULL, 0, NULL, NULL);
        if(size >= 0){
            std::vector<char> converted(size + 1);
            ::WideCharToMultiByte(codepage, 0,  &wtext[0], size, &converted[0], size + 1, NULL, NULL);
            return std::string(&converted[0], size);
        }
    }
    return text;
}

const std::string fromUTF8(const char* text)
{
    int size = ::MultiByteToWideChar(CP_UTF8, 0, text, -1, NULL, 0);
    if(size >= 0){
        std::vector<wchar_t> wtext(size + 1);
        ::MultiByteToWideChar(CP_UTF8, 0, text, -1, &wtext[0], size + 1);

        int codepage = _getmbcp();
        size = ::WideCharToMultiByte(codepage, 0,  &wtext[0], size, NULL, 0, NULL, NULL);
        if(size >= 0){
            std::vector<char> converted(size + 1);
            ::WideCharToMultiByte(codepage, 0,  &wtext[0], size, &converted[0], size + 1, NULL, NULL);
            return std::string(&converted[0], size);
        }
    }
    return text;
}
    
const std::string toUTF8(const std::string& text)
{
    int codepage = _getmbcp();
    int size = ::MultiByteToWideChar(codepage, 0, text.c_str(), text.size(), NULL, 0);
    if(size >= 0){
        std::vector<wchar_t> wtext(size + 1);
        ::MultiByteToWideChar(codepage, 0, text.c_str(), text.size(), &wtext[0], size + 1);

        size = ::WideCharToMultiByte(CP_UTF8, 0,  &wtext[0], size, NULL, 0, NULL, NULL);
        if(size >= 0){
            std::vector<char> converted(size + 1);
            ::WideCharToMultiByte(CP_UTF8, 0,  &wtext[0], size, &converted[0], size + 1, NULL, NULL);
            return std::string(&converted[0], size);
        }
    }
    return text;
}


const std::string toUTF8(const char* text)
{
    int codepage = _getmbcp();
    int size = ::MultiByteToWideChar(codepage, 0, text, -1, NULL, 0);
    if(size >= 0){
        std::vector<wchar_t> wtext(size + 1);
        ::MultiByteToWideChar(codepage, 0, text, -1, &wtext[0], size + 1);

        size = ::WideCharToMultiByte(CP_UTF8, 0,  &wtext[0], size, NULL, 0, NULL, NULL);
        if(size >= 0){
            std::vector<char> converted(size + 1);
            ::WideCharToMultiByte(CP_UTF8, 0,  &wtext[0], size, &converted[0], size + 1, NULL, NULL);
            return std::string(&converted[0], size);
        }
    }
    return text;
}
#endif
    
}
