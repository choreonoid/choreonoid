/*! @file
  @brief Implementation of text scanner class
  @author Shin'ichiro Nakaoka
*/

#include "EasyScanner.h"
#include "strtofloat.h"
#include <cstdio>
#include <cctype>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <iostream>
#include <fmt/format.h>
#include <errno.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;


std::string EasyScanner::Exception::getFullMessage() const
{
    string m(message);
    
    if(lineNumber > 0){
        m += format(" at line {}", lineNumber);
    }
	
    if(!filename.empty()){
        m += format(" of {}", filename);
    }
    
    return m;
}


EasyScanner::EasyScanner()
{
    init();
}


/**
   @param filename file to read.
*/
EasyScanner::EasyScanner(string filename)
{
    init();
    loadFile(filename);
}


void EasyScanner::init()
{
    textBuf = 0;
    size = 0;
    textBufEnd = 0;
    lineNumber = 0;
    lineNumberOffset = 1;
    
    commentChar = '#';
    quoteChar = 0xffff;
    isLineOriented = true;
    defaultErrorMessage = "unknown error of the lexical scanner";

    whiteSpaceChars.push_back(' ');
    whiteSpaceChars.push_back('\t');

    symbols.reset(new SymbolMap());
}


/*!
  Copy Constructor. New object inherits another's propety and symbols.
  @param scanner original object
  @param copy_text If true, new object has same text as original
*/
EasyScanner::EasyScanner(const EasyScanner& org, bool copyText) :
    whiteSpaceChars(org.whiteSpaceChars)
{
    commentChar = org.commentChar;
    quoteChar = org.quoteChar;
    isLineOriented = org.isLineOriented;
    filename = org.filename;
    defaultErrorMessage = org.defaultErrorMessage;
    lineNumber = org.lineNumber;
    lineNumberOffset = org.lineNumberOffset;

    symbols = org.symbols;

    if(copyText && org.textBuf){
        size = org.size;
        textBuf = new char[size+1];
        memcpy(textBuf, org.textBuf, size + 1);
        text = textBuf;
        textBufEnd = textBuf + size;
    } else {
        textBuf = 0;
        size = 0;
        textBufEnd = 0;
    }
}


/*! This function directly sets a text in the main memory */
void EasyScanner::setText(const char* text, size_t len)
{
    if(textBuf) delete[] textBuf;

    size = len;
    textBuf = new char[size + 1];
    memcpy(textBuf, text, len);
    textBuf[size] = 0;
    this->text = textBuf;
    textBufEnd = textBuf + size;
    lineNumber = lineNumberOffset;
    filename = "";
}


EasyScanner::~EasyScanner()
{
    if(textBuf) delete[] textBuf;
}


void EasyScanner::setLineNumberOffset(int offset)
{
    lineNumberOffset = offset;
}


void EasyScanner::moveToHead()
{
    text = textBuf;
    lineNumber = lineNumberOffset;
}


void EasyScanner::putSymbols()
{
    SymbolMap::iterator p = symbols->begin();
    while(p != symbols->end()){
        cout << p->first << " = " << p->second << std::endl;
        p++;
    }
}


void EasyScanner::throwException(const char* message)
{
    Exception ex;
    ex.message = message ? message : defaultErrorMessage;
    ex.filename = filename;
    ex.lineNumber = lineNumber;
    throw ex;
}


void EasyScanner::throwException(const std::string& message)
{
    throwException(message.c_str());
}


/*!
  This function sets the identifier character of comment beginning.
  @param cc Identifier character. Default is '#'. If you want no comment, set 0.
*/
void EasyScanner::setCommentChar(char cc)
{
    commentChar = cc ? cc : 0xffff;
}


void EasyScanner::setLineOriented(bool on)
{
    isLineOriented = on;
}


/*!  If there is a character to ignore, you can set it by this function */
void EasyScanner::setWhiteSpaceChar(char ws)
{
    whiteSpaceChars.push_back(ws);
}


/*!
  If you want to read quoted string, set quote character by this function.
  In default, this is unset.
*/
void EasyScanner::setQuoteChar(char qs)
{
    quoteChar = qs;
}


/**
   This function loads a text from a given file.
   The function thorws EasyScanner::Exception when the file cannot be loaded.
*/
void EasyScanner::loadFile(const string& filename)
{
    lineNumber = 0;
    this->filename.clear();
    
    FILE* file = fopen(filename.c_str(), "rb");

    if(!file){
        string message;
        switch(errno){
        case ENOENT:
            message = format(_("\"{}\" is not found."), filename);
            break;
        default:
            message = format(_("I/O error in accessing \"{}\"."), filename);
            break;
        }
        throwException(message.c_str());
    }

    this->filename = filename;

    fseek(file, 0, SEEK_END);
    size = ftell(file);
    rewind(file);
    if(textBuf) delete[] textBuf;
    textBuf = new char[size + 1];
    size = fread(textBuf, sizeof(char), size, file);
    textBuf[size] = 0;
    fclose(file);
    text = textBuf;
    textBufEnd = textBuf + size;
    lineNumber = lineNumberOffset;
}


/**
   move the current position to just before the end (LF or EOF) of a line
*/
void EasyScanner::skipToLineEnd()
{
    while(*text != '\r' && *text != '\n' && *text != '\0') text++;
}


void EasyScanner::skipSpace()
{
    int n = whiteSpaceChars.size();
    while(true){
        int i=0;
        while(i < n){
            if(*text == whiteSpaceChars[i]){
                text++;
                i = 0;
            } else {
                i++;
            }
        }
        if(*text == commentChar){
            text++;
            skipToLineEnd();
        }
        
        if(isLineOriented){
            break;
        }
        if(*text == '\n'){
            text++;
        } else if(*text == '\r'){
            text++;
            if(*text == '\n'){
                text++;
            }
        } else {
            break;
        }
        lineNumber++;
    }
}


/**
   This function does not call 'skipSpace()'.
   On the other hand, 'readLF()' calls 'skipSpace()' before calling 'readLF0()'
*/
bool EasyScanner::readLF0()
{
    if(*text == '\n'){
        text++;
        lineNumber++;
        return true;
    } else if(*text == '\r'){
        text++;
        if(*text == '\n'){
            text++;
        }
        lineNumber++;
        return true;
    }
    return false;
}


bool EasyScanner::checkLF()
{
    skipSpace();    
    if(*text == '\n' || *text == '\r'){
        return true;
    }
    return false;
}


int EasyScanner::readToken()
{
    skipSpace();

    if(isdigit((unsigned char)*text) || *text == '+' || *text == '-'){
        char* tail;
        intValue = strtol(text, &tail, 0);
        if(tail != text){
            text = tail;
            return T_INTEGER;
        }
        doubleValue = cnoid::strtod(text, &tail);
        if(tail != text){
            text = tail;
            return T_DOUBLE;
        }
        charValue = *text;
        text++;
        return T_SIGLUM;

    } else if(isalpha((unsigned char)*text)){
        char* org = text;
        text++;
        while(isalnum((unsigned char)*text) || *text == '_') text++;
        stringValue.assign(org, text - org);
        if(stringValue.size() == 1){
            charValue = *org;
            return T_ALPHABET;
        } else {
            return T_WORD;
        }

    } else if(*text == quoteChar) {
        return extractQuotedString() ? T_STRING : T_SIGLUM;

    } else if(ispunct((unsigned char)*text)){
        charValue = *text;
        text++;
        return T_SIGLUM;

    } else if(readLF0()){
        return T_LF;

    } else if(*text == '\0'){
        return T_EOF;
    }

    return T_NONE;
}





/*!
  This function makes all the characters in stringValue lower case
*/
void EasyScanner::toLower()
{
    for(size_t i=0; i < stringValue.size(); ++i){
        stringValue[i] = tolower(stringValue[i]);
    }
}


bool EasyScanner::extractQuotedString()
{
    text++;
    char* org = text;

    if(isLineOriented){
        while(true){
            if(*text == '\r' || *text == '\n' || *text == '\0'){
                text = org;
                return false;
            }
            if(*text == quoteChar) break;
            text++;
        }
    } else {
        while(true){
            if(*text == '\0'){
                text = org;
                return false;
            }
            readLF0();
            if(*text == quoteChar) break;
            text++;
        }
    }

    stringValue.assign(org, text - org);
    text++;
    return true;
}


bool EasyScanner::readFloat()
{
    char* tail;

    if(checkLF()) return false;

    floatValue = cnoid::strtof(text, &tail);

    if(tail != text){
        text = tail;
        return true;
    }

    return false;
}


bool EasyScanner::readDouble()
{
    char* tail;

    if(checkLF()) return false;

    doubleValue = cnoid::strtod(text, &tail);

    if(tail != text){
        text = tail;
        return true;
    }

    return false;
}


bool EasyScanner::readInt()
{
    char* tail;

    if(checkLF()) return false;

    intValue = strtol(text, &tail, 0);
    if(tail != text){
        text = tail;
        return true;
    }

    return false;
}


bool EasyScanner::readChar()
{
    skipSpace();

    if(isgraph((unsigned char)*text)){
        charValue = *text;
        text++;
        return true;
    }

    return false;
}


bool EasyScanner::readChar(int chara)
{
    skipSpace();

    if(*text == chara){
        text++;
        return true;
    }

    return false;
}

int EasyScanner::peekChar()
{
    skipSpace();

    return *text;
}


bool EasyScanner::readWord0()
{
    char* org = text;

    while(true){
        int c = (unsigned char)*text;
        if(!isalnum(c) && isascii(c) && c != '_'){
            break;
        }
        text++;
    }

    if(text - org > 0){
        stringValue.assign(org, text - org);
        return true;
    }

    return false;
}


bool EasyScanner::readString0(const int delimiterChar)
{
    char* org = text;

    while(true){
        int c = (unsigned char)*text;
        if(isspace(c) || iscntrl(c) || c == delimiterChar){
            break;
        }
        text++;
    }

    if(text - org > 0){
        stringValue.assign(org, text - org);
        return true;
    }

    return false;
}


bool EasyScanner::readString(const char* str)
{
    skipSpace();

    char* org = text;
    while(*str != '\0'){
        if(*str++ != *text++){
            text = org;
            return false;
        }
    }

    return true;
}


/**
   read a quoted string. If 'allowNoQuotedWord' is true,
   the function read a word without quotations.
*/
bool EasyScanner::readQuotedString(bool allowNoQuotedWord)
{
    skipSpace();

    if(*text == quoteChar){
        return extractQuotedString();

    } else if(allowNoQuotedWord){
        return readString0(' ');
    }

    return false;
}


bool EasyScanner::readUnquotedTextBlock()
{
    skipSpace();

    char* org = text;
    while(true){
        if(*text == '\r' || *text == '\n' || *text == commentChar || *text == '\0'){
            break;
        }
        text++;
    }

    if(text != org){
        stringValue.assign(org, text - org);
        return true;
    }
    return false;
}



bool EasyScanner::readSymbol()
{
    if(readWord()){
        symbolValue = getSymbolID(stringValue);
        if(symbolValue){
            return true;
        }
    }

    return false;
}


bool EasyScanner::readSymbol(int id)
{
    char* org = text;
    int orglineNumber = lineNumber;

    if(readWord()){
        symbolValue = getSymbolID(stringValue);
        if(symbolValue == id){
            return true;
        } else {
            text = org;
            lineNumber = orglineNumber;
        }
    }

    return false;
}


bool EasyScanner::skipLine()
{
    while(true){
        if(readLF0()){
            return true;
        }
        if(*text == '\0'){
            return false;
        }
        text++;
    }
}


bool EasyScanner::readLine()
{
    char* org = text;

    if(skipLine()){
        // eliminate newline code
        char* end = text - 1;
        if(*end == '\n'){
            end--;
            if(*end == '\r'){
                end--;
            }
        }
        end++;

        stringValue.assign(org, end - org);
        return true;
    }

    return false;
}


bool EasyScanner::skipBlankLines()
{
    do {
        if(*text == '\0'){
            return false;
        }
    } while(readLF());

    return true;
}


// operators

EasyScanner& cnoid::operator>>(EasyScanner& scanner, double& value)
{
    if(!scanner.readDouble()){
        scanner.throwException("scan error: can't read double value");
    }
    value = scanner.doubleValue;
    return scanner;
}


EasyScanner& cnoid::operator>>(EasyScanner& scanner, int& value)
{
    if(!scanner.readInt()){
        scanner.throwException("scan error: can't read int value");
        throw scanner;
    }
    value = scanner.intValue;
    return scanner;
}


EasyScanner& cnoid::operator>>(EasyScanner& scanner, const char* matchString)
{
    scanner.skipSpace();
    while(*matchString != '\0'){
        if(*scanner.text++ != *matchString++){
            scanner.throwException("scan error: unmatched string");
        }
    }
    return scanner;
}


EasyScanner& cnoid::operator>>(EasyScanner& scanner, char matchChar)
{
    scanner.skipSpace();
    if(*scanner.text++ != matchChar){
        scanner.throwException("scan error: unmatched cahracter");
    }
    return scanner;
}


EasyScanner& cnoid::operator>>(EasyScanner& scanner, string& str)
{
    scanner.skipSpace();
    if(!scanner.readQuotedString(true)){
        scanner.throwException("scan error: can't read string");
    }
    str = scanner.stringValue;
    return scanner;
}


EasyScanner& cnoid::operator>>(EasyScanner& scanner, EasyScanner::Endl)
{
    if(!scanner.readLF()){
        scanner.throwException("scan error: end of line unmatched");
    }
    return scanner;
}
