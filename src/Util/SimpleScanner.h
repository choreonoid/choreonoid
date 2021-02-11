#ifndef CNOID_UTIL_SIMPLE_SCANNER_H
#define CNOID_UTIL_SIMPLE_SCANNER_H

#include "strtofloat.h"
#include "UTF8.h"
#include <cnoid/stdx/filesystem>
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
    static const size_t bufsize = 256;
    char buf[bufsize];
    char* pos;
    size_t lineNumber;
    std::string filename;
    
    void open(const std::string& filename)
    {
        // The binary mode is faster on Windows
        ifs.open(fromUTF8(filename), std::ios::in | std::ios::binary);
        this->filename = filename;
        clear();
    }

    void clear()
    {
        lineNumber = 0;
        buf[0] = '\0';
        pos = buf;
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
            if(ifs.gcount() > 0){
                throwEx("Too long line");
            } else {
                throwEx("I/O error");
            }
        }
        buf[0] = '\0';
        return !ifs.eof();
    }

    void skipSpaces()
    {
        while(*pos == ' '){
            ++pos;
        }
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

    bool checkString(const char* str)
    {
        skipSpaces();
        char* pos0 = pos;
        while(*str != '\0'){
            if(*str++ != *pos++){
                pos = pos0;
                return false;
            }
        }
        return true;
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
            
    bool readString(std::string& out_string)
    {
        skipSpaces();
        char* pos0 = pos;
        while(*pos != '\r' && *pos != '\0'){
            ++pos;
        }
        out_string.assign(pos0, pos - pos0);
        return !out_string.empty();
    }
    
    void readFloatEx(float& out_value)
    {
        skipSpaces();
        char* tail;
        out_value = cnoid::strtof(pos, &tail);
        if(tail != pos){
            pos = tail;
        } else {
            throwEx("Invalid value");
        }
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
