#ifndef CNOID_UTIL_SIMPLE_SCANNER_H
#define CNOID_UTIL_SIMPLE_SCANNER_H

#include "UTF8.h"
#include <cnoid/stdx/filesystem>
#include <fast_float/fast_float.h>
#include <fmt/format.h>
#include <fstream>
#include <stdexcept>
#include <cstdlib>

namespace cnoid {

class SimpleScanner
{
public:
    typedef std::ifstream::pos_type pos_type;

    std::ifstream ifs;
    char* buf;
    size_t bufsize;
    const char* pos;
    const char* dummy; // This makes the parse 10% faster in some architectures
    const char* bufEndPos;
    size_t lineNumber;
    std::string filename;
    std::string tmpString;

    static constexpr size_t buf1size = 256;
    char buf1[buf1size];
    std::vector<char> buf2;

    SimpleScanner()
    {
        clear(false);
    }

    void clear(bool doRelease = true)
    {
        buf = buf1;
        bufsize = buf1size;
        bufEndPos = buf + bufsize;
        buf[0] = '\0';
        pos = buf;
        lineNumber = 0;

        if(doRelease){
            buf2.clear();
            buf2.shrink_to_fit();
        }
    }        

    bool open(const std::string& filename)
    {
        clear();
        
        // The binary mode is faster on Windows
        ifs.open(fromUTF8(filename), std::ios::in | std::ios::binary);
        if(ifs.is_open()){
            this->filename = filename;
            return true;
        }
        return false;
    }

    void close()
    {
        if(ifs.is_open()){
            ifs.close();
        }
    }

    bool getLine()
    {
        pos = buf;
        bool result = false;

#ifndef _WIN32
        if(ifs.getline(buf, bufsize)){
            result = true;
        }
#else
        // The following code is faster on Windows
        if(ifs.get(buf, bufsize, '\n')){
            ifs.ignore();
            result = true;
        } else if(!ifs.eof() && ifs.fail()){
            ifs.clear();
            if(ifs.peek() == '\n'){
                ifs.ignore();
                result = true;
            }
        }
#endif
        if(result){
            ++lineNumber;
            return true;
        }

        if(!ifs.eof()){
            int gcount = ifs.gcount();
            if(gcount <= 0){
                throwEx("I/O error");
            } else {
                bufsize = bufsize * 2;
                buf2.resize(bufsize);
                buf = &buf2[0];
                bufEndPos = buf + bufsize;
                ifs.clear();
                ifs.seekg(-gcount, std::ios::cur);
                return getLine();
            }
        }
        buf[0] = '\0';
        return !ifs.eof();
    }

    const std::string& currentLine()
    {
        char* end = buf;
        while(true){
            if(*end == '\n' || *end == '\0'){
                break;
            }
            ++end;
        }
        tmpString.assign(buf, end - buf);
        return tmpString;
    }

    void moveForward()
    {
        ++pos;
    }

    void skipSpaces()
    {
        while(*pos == ' '){
            ++pos;
        }
    }

    void skipSpacesAndTabs()
    {
        while(*pos == ' ' || *pos == '\t'){
            ++pos;
        }
    }

    int peekChar()
    {
        return *pos;
    }

    int readChar()
    {
        int c = *pos;
        if(c != '\0'){
            ++pos;
        }
        return c;
    }

    bool checkCharAtCurrentPosition(int chara)
    {
        if(*pos == chara){
            ++pos;
            return true;
        }
        return false;
    }

    bool checkChar(int chara)
    {
        skipSpaces();
        return checkCharAtCurrentPosition(chara);
    }

    bool checkLF()
    {
        skipSpaces();
        if(*pos == '\0'){
            return true;
        } else if(*pos == '\r'){
            ++pos;
            return true;
        }
        return false;
    }

    void checkLFEx()
    {
        if(!checkLF()){
            throwEx("Invalid value");
        }
    }

    bool checkEOF()
    {
        if(checkLF()){
            return ifs.eof();
        }
        return false;
    }

    bool checkStringAtCurrentPosition(const char* str)
    {
        const char* pos0 = pos;
        while(*str != '\0'){
            if(*str++ != *pos++){
                pos = pos0;
                return false;
            }
        }
        return true;
    }

    bool checkString(const char* str)
    {
        skipSpaces();
        return checkStringAtCurrentPosition(str);
    }

    void checkStringEx(const char* str)
    {
        const char* org = str;
        if(!checkString(str)){
            throwEx(fmt::format("\"{}\" is expected", org));
        }
    }

    bool seekToString(pos_type initialSeekPos, const char* str, size_t maxLength, pos_type& out_seekPos)
    {
        ifs.seekg(initialSeekPos);
        getLine();
        
        pos_type currentLinePos = initialSeekPos;
        pos_type nextLinePos = ifs.tellg();
        const char* str0 = str;
        char* found = nullptr;
        while(true){
            while(*pos != '\0'){
                ++pos;
            }
            if(!getLine()){
                break;
            }
            currentLinePos = nextLinePos;
            nextLinePos = ifs.tellg();
            if(currentLinePos - initialSeekPos > maxLength){
                break;
            }
            skipSpaces();

            if(*pos == *str){
                ++pos;
                ++str;
                while(true){
                    if(*str == '\0'){
                        out_seekPos = currentLinePos;
                        ifs.seekg(out_seekPos);
                        clear();
                        return true; // found
                    }
                    if(*pos++ != *str++){
                        str = str0;
                        break;
                    }
                }
            }
        }
        out_seekPos = initialSeekPos;
        return false;
    }
            
    bool readStringAtCurrentPosition(std::string& out_string)
    {
        const char* pos0 = pos;
        // Todo: use isspace, iscontl, etc.
        while((*pos != ' ') && (*pos != '\t') && (*pos != '\r') && (*pos != '\0')){
            ++pos;
        }
        out_string.assign(pos0, pos - pos0);
        return !out_string.empty();
    }

    bool readString(std::string& out_string)
    {
        skipSpaces();
        return readStringAtCurrentPosition(out_string);
    }

    bool readStringToEOL(std::string& out_string)
    {
        skipSpaces();
        const char* pos0 = pos;
        while((*pos != '\r') && (*pos != '\n') && (*pos != '\0')){
            ++pos;
        }
        out_string.assign(pos0, pos - pos0);
        return !out_string.empty();
    }
    
    float readFloatEx()
    {
        float value;
        auto result = fast_float::from_chars(pos, bufEndPos, value);
        if(result.ec == std::errc()){
            pos = result.ptr;
        } else {
            throwEx("Invalid value");
        }
        return value;
    }

    bool readInt(int& out_value)
    {
        char* tail;
        out_value = std::strtol(pos, &tail, 0);
        if(tail != pos){
            pos = tail;
            return true;
        }
        return false;
    }

    int readIntEx()
    {
        char* tail;
        int value = std::strtol(pos, &tail, 0);
        if(tail != pos){
            pos = tail;
        } else {
            throwEx("Invald value");
        }
        return value;
    }
    
    void throwEx(const std::string& error)
    {
        stdx::filesystem::path path(fromUTF8(filename));
        throw std::runtime_error(
            fmt::format("{0} at line {1} of \"{2}\".",
                        error, lineNumber, toUTF8(path.filename().string())));
    }
};

}

#endif
