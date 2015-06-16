/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_YAML_WRITER_H_INCLUDED
#define CNOID_UTIL_YAML_WRITER_H_INCLUDED

#include "ValueTree.h"
#include <stack>
#include <string>
#include <fstream>
#include <boost/lexical_cast.hpp>
#include <boost/intrusive_ptr.hpp>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT YAMLWriter
{
public:
    YAMLWriter(const std::string filename);
    YAMLWriter(std::ostream& os);
    ~YAMLWriter();

    void putNode(const ValueNode* node);

    void setIndentWidth(int n);
    void setKeyOrderPreservationMode(bool on);

    void startDocument();

    void putComment(const std::string& comment, bool doNewLine = true);

    void putString(const std::string& value);
    void putSingleQuotedString(const std::string& value);
    void putDoubleQuotedString(const std::string& value);
    void putBlockStyleString(const std::string& value, bool isLiteral);
    inline void putLiteralString(const std::string& value) { putBlockStyleString(value, true); }
    inline void putFoldedString(const std::string& value) { putBlockStyleString(value, false); }

    template <class DataType> inline void putScalar(const DataType& value){
        putString(boost::lexical_cast<std::string>(value));
    }

    void putScalar(const double& value);
    void setDoubleFormat(const char* format);

    void startMapping();
    void startFlowStyleMapping();
        
    void putKey(const std::string& key, StringStyle style = PLAIN_STRING);
        
    template <class DataType> inline void putKeyValue(const std::string& key, const DataType& value){
        putKey(key);
        putScalar(value);
    }

    inline void putKeyValue(const std::string& key, const std::string& value){
        putKey(key);
        putDoubleQuotedString(value);
    }
            
    void endMapping();

    void startListing();
    void startFlowStyleListing();
    void endListing();

#ifdef CNOID_BACKWARD_COMPATIBILITY
    void startSequence() { startListing(); }
    void startFlowStyleSequence() { startFlowStyleListing(); }
    void endSequence() { endListing(); }
#endif        

private:

    std::ofstream ofs;
    std::ostream& os;

    int indentWidth;
    bool isCurrentNewLine;
    int numDocuments;
    bool isKeyOrderPreservationMode;
    bool doInsertLineFeed;

    const char* doubleFormat;

    enum { TOP, MAPPING, LISTING };

    struct State {
        int type;
        bool isFlowStyle;
        bool isKeyPut;
        bool hasValuesBeenPut;
        std::string indentString;
    };
        
    std::stack<State> states;

    State* current;

    bool isTopLevel();
    State& pushState(int type, bool isFlowStyle);
    void popState();
    void indent();
    void newLine();
    bool makeValuePutReady();
    bool startValuePut();
    void endValuePut();
    void putString_(const std::string& value);
    void putSingleQuotedString_(const std::string& value);
    void putDoubleQuotedString_(const std::string& value);
    void putKey_(const std::string& key, StringStyle style);
    void startMappingSub(bool isFlowStyle);
    void startListingSub(bool isFlowStyle);
    void putNodeMain(const ValueNode* node, bool doCheckLF);
    void putScalarNode(const ScalarNode* scalar);
    void putMappingNode(const Mapping* mapping);
    void putListingNode(const Listing* listing);
};

#ifdef CNOID_BACKWARD_COMPATIBILITY
typedef YAMLWriter YamlWriter;
#endif
    
}


#endif
