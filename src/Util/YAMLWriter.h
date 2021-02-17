/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_YAML_WRITER_H
#define CNOID_UTIL_YAML_WRITER_H

#include "ValueTree.h"
#include <string>
#include "exportdecl.h"

namespace cnoid {

class YAMLWriterImpl;

class CNOID_EXPORT YAMLWriter
{
public:
    YAMLWriter();
    YAMLWriter(const std::string filename);
    YAMLWriter(std::ostream& os);
    ~YAMLWriter();

    void setOutput(std::ostream& os);
    void flush();
    bool openFile(const std::string& filename);
    bool isFileOpen();
    void closeFile();

    void setMessageSink(std::ostream& os);
    void putMessage(const std::string& message);
    std::ostream& messageSink();

    void putNode(const ValueNode* node);

    void setIndentWidth(int n);
    int indentWidth() const;
    void setKeyOrderPreservationMode(bool on);

    void startDocument();

    void putComment(const std::string& comment, bool doNewLine = true);

    void putString(const char* value);
    void putString(const std::string& value);
    void putSingleQuotedString(const char* value);
    void putSingleQuotedString(const std::string& value);
    void putDoubleQuotedString(const char* value);
    void putDoubleQuotedString(const std::string& value);
    void putBlockStyleString(const char* value, bool isLiteral);
    void putBlockStyleString(const std::string& value, bool isLiteral);
    void putLiteralString(const char* value) { putBlockStyleString(value, true); }
    void putLiteralString(const std::string& value) { putBlockStyleString(value, true); }
    void putFoldedString(const char* value) { putBlockStyleString(value, false); }
    void putFoldedString(const std::string& value) { putBlockStyleString(value, false); }
    void putScalar(bool value);
    void putScalar(int value);
    void putScalar(double value);
    void putScalar(const char* value) { putString(value); }
    void putScalar(const std::string& value){ putString(value); }
    void setDoubleFormat(const char* format);

    void startMapping();
    void startFlowStyleMapping();
    void putKey(const char* key, StringStyle style = PLAIN_STRING);
    void putKey(const std::string& key, StringStyle style = PLAIN_STRING);

    template <class DataType> void putKeyValue(const char* key, const DataType& value){
        putKey(key);
        putScalar(value);
    }

    template <class DataType> void putKeyValue(const std::string& key, const DataType& value){
        putKey(key);
        putScalar(value);
    }

    void endMapping();

    void startListing();
    void startFlowStyleListing();
    void endListing();

    const Mapping* info() const;
    Mapping* info();
    
    template<typename T> T info(const std::string& key) const;
    template<typename T> T info(const std::string& key, const T& defaultValue) const;
    template<typename T> T getOrCreateInfo(const std::string& key, const T& defaultValue);
    template<typename T> void setInfo(const std::string& key, const T& value);

    void resetInfo(Mapping* info);

#ifdef CNOID_BACKWARD_COMPATIBILITY
    void startSequence() { startListing(); }
    void startFlowStyleSequence() { startFlowStyleListing(); }
    void endSequence() { endListing(); }
#endif        

private:
    YAMLWriterImpl* impl;
};

template<> CNOID_EXPORT double YAMLWriter::info(const std::string& key) const;
template<> CNOID_EXPORT double YAMLWriter::info(const std::string& key, const double& defaultValue) const;
template<> CNOID_EXPORT bool YAMLWriter::info(const std::string& key, const bool& defaultValue) const;
template<> CNOID_EXPORT double YAMLWriter::getOrCreateInfo(const std::string& key, const double& defaultValue);
template<> CNOID_EXPORT bool YAMLWriter::getOrCreateInfo(const std::string& key, const bool& defaultValue);
template<> CNOID_EXPORT void YAMLWriter::setInfo(const std::string& key, const double& value);
template<> CNOID_EXPORT void YAMLWriter::setInfo(const std::string& key, const bool& value);

#ifdef CNOID_BACKWARD_COMPATIBILITY
typedef YAMLWriter YamlWriter;
#endif
    
}

#endif
