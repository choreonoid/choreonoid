
/*! @file
  @brief The header file of a text scanner class
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_EASYSCANNER_H_INCLUDED
#define CNOID_UTIL_EASYSCANNER_H_INCLUDED

#include <unordered_map>
#include <string>
#include <vector>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

/**
   @todo introduce a pimpl to hide the use of unordered_map, vector
*/
class CNOID_EXPORT  EasyScanner {

public:

    class Endl {
        //int dummy;
    };

    class CNOID_EXPORT Exception {
public:
        std::string message;
        std::string filename;
        int lineNumber;
        std::string getFullMessage() const;
    };

    enum TokenType {
        T_NONE = 0, T_SPACE, T_ALPHABET, T_INTEGER, T_DOUBLE, T_WORD,
        T_STRING, T_SIGLUM, T_LF, T_EOF
    };

    typedef std::unordered_map<std::string, int> SymbolMap;
    typedef std::pair<std::string, int> SymbolPair;

    Endl endl;

    EasyScanner();
    EasyScanner(std::string filename);
    EasyScanner(const EasyScanner& org, bool copyText = false);
    virtual ~EasyScanner();

    void putSymbols();

    inline void registerSymbol(int id, const std::string& symbol) {
        symbols->insert(SymbolPair(symbol, id));
    }

    void setSymbols(std::shared_ptr<SymbolMap> symbols){
        this->symbols = symbols;
    }

    inline int  getSymbolID(const std::string& symbol) {
        SymbolMap::iterator p = symbols->find(symbol);
        return (p != symbols->end()) ? p->second : 0;
    }

    /// if 0, comment is disabled
    void setCommentChar(char cc);

    void setLineOriented(bool on);
    void setQuoteChar(char qc);
    void setWhiteSpaceChar(char ws);

    void loadFile(const std::string& filename);

    void setText(const char* text, size_t len);

    void setLineNumberOffset(int offset);

    void setDefaultErrorMessage(const std::string& message){
        defaultErrorMessage = message;
    }

    void moveToHead();

    int  readToken();

    void toLower();

    bool readFloat();
    bool readDouble();
    bool readInt();
    bool readChar();
    bool readChar(int chara);
    int  peekChar();

    /**
       In contrast to readString(),
       this function does not recognize siglums except '_' as a part of a word.
    */
    inline bool readWord() {
        skipSpace();
        return readWord0();
    }

    /**
       In contrast to readWord(),
       this function allows a string to include siglums such as !,",#,$,%,&,...
    */
    inline bool readString(const int delimiterChar = ',') {
        skipSpace();
        return readString0(delimiterChar);
    }

    bool readString(const char* str);

    inline bool readString(const std::string& str) {
        return readString(str.c_str());
    }

    bool readQuotedString(bool allowNoQuotedWord = false);

    bool readUnquotedTextBlock();

    bool readSymbol();
    bool readSymbol(int id);

    inline bool isEOF(){
        skipSpace();
        return (*text == '\0');
    }

    /// reading a line feed
    inline bool readLF() {
        skipSpace();
        return readLF0();
    }

    inline bool readLFEOF() {
        skipSpace();
        return readLF0() ? true : (*text == '\0');
    }

    bool checkLF();

    bool readLine();
    bool skipLine();
    bool skipBlankLines();
    void skipToLineEnd();

    void skipSpace();

    void throwException(const char* message);
    void throwException(const std::string& message);

    /**
       The exception version of readInt().
       \return Scanned int value.
    */
    inline int readIntEx(const char* message = 0) {
        if(!readInt()) throwException(message);
        return intValue;
    }
    /**
       The exception version of readDouble().
       \return Scanned double value.
    */
    inline double readDoubleEx(const char* message = 0) {
        if(!readDouble()) throwException(message);
        return doubleValue;
    }
    /**
       The exception version of readDouble().
       \return Scanned double value.
    */
    inline float readFloatEx(const char* message = 0) {
        if(!readFloat()) throwException(message);
        return floatValue;
    }
    /**
       The exception version of readChar().
       \return Scanned char value.
    */
    inline int readCharEx(const char* message = 0) {
        if(!readChar())
            throwException(message);
        return charValue;
    }
    /**
       The exception version of readChar().
    */
    inline void readCharEx(int chara, const char* message = 0) {
        if(!readChar(chara)) throwException(message);
    }
    /**
       The exception version of readWord().
       \return Scanned word string.
    */
    inline const std::string& readWordEx(const char* message = 0) {
        if(!readWord()) throwException(message);
        return stringValue;
    }

    inline void checkStringEx(const char* str, const char* message = 0){
        if(!readString(str)) throwException(message);
    }

    /**
       The exception version of readString().
       \return Scanned word string.
    */
    inline const std::string& readStringEx(const char* message = 0) {
        if(!readString()) throwException(message);
        return stringValue;
    }

    inline const std::string& readQuotedStringEx(const char* message = 0) {
        if(!readQuotedString()) throwException(message);
        return stringValue;
    }
    /**
       The exception version of readSymbol().
       \return ID of the scanned symbol.
    */
    inline int readSymbolEx(const char* message = 0) {
        if(!readSymbol()) throwException(message);
        return symbolValue;
    }
    /**
       The exception version of readLF().
    */
    inline void readLFex(const char* message = 0) {
        if(!readLF()) throwException(message);
    }

    inline void readLFEOFex(const char* message = 0) {
        if(!readLFEOF()) throwException(message);
    }

    int intValue;
    double doubleValue;
    float floatValue;
    std::string stringValue;
    char charValue;
    int symbolValue;

    std::string defaultErrorMessage;
    int lineNumber;

    char* text;

    std::string filename;

private:
    void init();
    bool extractQuotedString();

    bool readLF0();
    bool readWord0();
    bool readString0(const int delimiterChar);
    
    char* textBuf;
    size_t size;
    char* textBufEnd;
    int lineNumberOffset;
    int commentChar;
    int quoteChar;
    bool isLineOriented;

    std::vector<int> whiteSpaceChars;

    std::shared_ptr<SymbolMap> symbols;

    friend CNOID_EXPORT EasyScanner& operator>>(EasyScanner& scanner, double& value);
    friend CNOID_EXPORT EasyScanner& operator>>(EasyScanner& scanner, int& value);
    friend CNOID_EXPORT EasyScanner& operator>>(EasyScanner& scanner, const char* matchString);
    friend CNOID_EXPORT EasyScanner& operator>>(EasyScanner& scanner, char matchChar);
    friend CNOID_EXPORT EasyScanner& operator>>(EasyScanner& scanner, std::string& str);
    friend CNOID_EXPORT EasyScanner& operator>>(EasyScanner& scanner, EasyScanner::Endl endl);

};


CNOID_EXPORT EasyScanner& operator>>(EasyScanner& scanner, double& value);
CNOID_EXPORT EasyScanner& operator>>(EasyScanner& scanner, int& value);
CNOID_EXPORT EasyScanner& operator>>(EasyScanner& scanner, const char* matchString);
CNOID_EXPORT EasyScanner& operator>>(EasyScanner& scanner, char matchChar);
CNOID_EXPORT EasyScanner& operator>>(EasyScanner& scanner, std::string& str);
CNOID_EXPORT EasyScanner& operator>>(EasyScanner& scanner, EasyScanner::Endl endl);
}

#endif
