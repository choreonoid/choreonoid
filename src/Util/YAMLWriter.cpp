/**
   @author Shin'ichiro Nakaoka
*/

#include "YAMLWriter.h"
#include "NullOut.h"
#include <iostream>
#include <algorithm>
#include <stack>
#include <fstream>

using namespace std;
using namespace cnoid;

namespace cnoid {

enum { TOP, MAPPING, LISTING };

struct State {
    int type;
    bool isFlowStyle;
    bool isKeyPut;
    bool hasValuesBeenPut;
    string indentString;
};

class YAMLWriterImpl
{
public:
    std::ofstream ofs;
    std::ostream& os;
    std::ostream* messageSink_;

    int indentWidth;
    int numDocuments;
    bool isCurrentNewLine;
    bool isKeyOrderPreservationMode;
    bool doInsertLineFeed;

    const char* doubleFormat;

    std::stack<State> states;

    State* current;

    MappingPtr info;

    string linebuf;

    YAMLWriterImpl(const std::string filename);
    YAMLWriterImpl(std::ostream& os);
    ~YAMLWriterImpl();

    bool isTopLevel();
    State& pushState(int type, bool isFlowStyle);
    void popState();
    void newLine();
    void indent();
    bool makeValuePutReady();
    bool startValuePut();
    void endValuePut();
    void putString(const std::string& value);
    void putString(const char* value);
    template<class StringType> void putSingleQuotedString(const StringType& value);
    template<class StringType> void putDoubleQuotedString(const StringType& value);
    void putBlockStyleString(const std::string& value, bool isLiteral);
    void startMappingSub(bool isFlowStyle);
    template<class StringType> void putKey(const StringType& key, StringStyle style);
    void endMapping();
    void startListingSub(bool isFlowStyle);
    void endListing();
    void putNodeMain(const ValueNode* node, bool doCheckLF);
    void putScalarNode(const ScalarNode* scalar);
    void putMappingNode(const Mapping* mapping);
    void putListingNode(const Listing* listing);
};

}


YAMLWriter::YAMLWriter(const std::string filename)
{
    impl = new YAMLWriterImpl(filename);
}


YAMLWriterImpl::YAMLWriterImpl(const std::string filename)
    : YAMLWriterImpl(ofs)
{
    ofs.open(filename.c_str());
}


YAMLWriter::YAMLWriter(std::ostream& os)
{
    impl = new YAMLWriterImpl(os);
}


YAMLWriterImpl::YAMLWriterImpl(std::ostream& os)
    : os(os)
{
    indentWidth = 2;
    isCurrentNewLine = true;
    current = 0;
    numDocuments = 0;
    isKeyOrderPreservationMode = false;
    messageSink_ = &nullout();

    doubleFormat = "%.7g";

    pushState(TOP, false);

    info = new Mapping;
}    


YAMLWriter::~YAMLWriter()
{
    delete impl;
}


YAMLWriterImpl::~YAMLWriterImpl()
{
    os.flush();
    ofs.close();
}


bool YAMLWriter::isOpen()
{
    return impl->ofs.is_open();
}


void YAMLWriter::setMessageSink(std::ostream& os)
{
    impl->messageSink_ = &os;
}


void YAMLWriter::putMessage(const std::string& message)
{
    (*impl->messageSink_) << message;
}


std::ostream& YAMLWriter::messageSink()
{
    return *impl->messageSink_;
}


void YAMLWriter::setIndentWidth(int n)
{
    if(impl->isTopLevel()){
        impl->indentWidth = n;
    }
}


void YAMLWriter::setKeyOrderPreservationMode(bool on)
{
    impl->isKeyOrderPreservationMode = on;
}


bool YAMLWriterImpl::isTopLevel()
{
    return (states.size() <= 1);
}


State& YAMLWriterImpl::pushState(int type, bool isFlowStyle)
{
    bool parentFlowStyle = current ? current->isFlowStyle : isFlowStyle;
    const int level = std::max(static_cast<int>(states.size() - 1), 0);
    states.push(State());
    State& state = states.top();
    state.type = type;
    state.isFlowStyle = parentFlowStyle ? true : isFlowStyle;
    state.isKeyPut = false;
    state.hasValuesBeenPut = false;
    state.indentString = string(level * indentWidth, ' ');
    current = &state;
    return state;
}


void YAMLWriterImpl::popState()
{
    states.pop();
    current = &states.top();
}


void YAMLWriterImpl::newLine()
{
    if(!isCurrentNewLine){
        os << "\n";
        isCurrentNewLine = true;
    }
}


void YAMLWriterImpl::indent()
{
    if(!isCurrentNewLine){
        newLine();
    }
    os << current->indentString;
}


void YAMLWriter::startDocument()
{
    impl->newLine();
    if(impl->numDocuments > 0){
        impl->os << "\n";
    }
     impl->os << "---\n";
    
    ++ impl->numDocuments;
}


void YAMLWriter::putComment(const std::string& comment, bool doNewLine)
{
    if(doNewLine){
        impl->indent();
    }
    impl->os << "# " << comment;
    impl->isCurrentNewLine = false;
    impl->newLine();
}


bool YAMLWriterImpl::makeValuePutReady()
{
    switch(current->type){
    case MAPPING:
        return current->isKeyPut;
    case LISTING:
        if(!current->isFlowStyle){
            indent();
            os << "- ";
        }
        isCurrentNewLine = false;
        return true;
    default:
        return true;
    }
}


bool YAMLWriterImpl::startValuePut()
{
    if(makeValuePutReady()){
        if(current->type == LISTING && current->isFlowStyle){
            if(current->hasValuesBeenPut){
                os << ", ";
            }
            if(doInsertLineFeed){
                newLine();
                indent();
                doInsertLineFeed = false;
                isCurrentNewLine = false;
            }
        }
        return true;
    }
    return false;
}


void YAMLWriterImpl::endValuePut()
{
    current->hasValuesBeenPut = true;
    if(current->type == MAPPING){
        current->isKeyPut = false;
    }
    if(!current->isFlowStyle){
        newLine();
    }
}


void YAMLWriterImpl::putString(const std::string& value)
{
    if(startValuePut()){
        if(value.empty()){
            os << "\"\"";
        } else {
            os << value;
        }
        endValuePut();
    }
}


void YAMLWriterImpl::putString(const char* value)
{
    if(startValuePut()){
        if(value[0] == '\0'){
            os << "\"\"";
        } else {
            os << value;
        }
        endValuePut();
    }
}


void YAMLWriter::putString(const char* value)
{
    impl->putString(value);
}


void YAMLWriter::putString(const std::string& value)
{
    impl->putString(value);
}


template<class StringType> void YAMLWriterImpl::putSingleQuotedString(const StringType& value)
{
    if(startValuePut()){
        os << "'" << value << "'";
        endValuePut();
    }
}


void YAMLWriter::putSingleQuotedString(const char* value)
{
    impl->putSingleQuotedString(value);
}


void YAMLWriter::putSingleQuotedString(const std::string& value)
{
    impl->putSingleQuotedString(value);
}


template<class StringType> void YAMLWriterImpl::putDoubleQuotedString(const StringType& value)
{
    if(startValuePut()){
        os << "\"" << value << "\"";
        endValuePut();
    }
}


void YAMLWriter::putDoubleQuotedString(const char* value)
{
    impl->putDoubleQuotedString(value);
}


void YAMLWriter::putDoubleQuotedString(const std::string& value)
{
    impl->putDoubleQuotedString(value);
}


void YAMLWriterImpl::putBlockStyleString(const std::string& value, bool isLiteral)
{
    if(current->isFlowStyle){
        ValueNode::SyntaxException ex;
        ex.setMessage("A block-style string cannot be inserted into a flow-style container");
        throw ex;
    }
    
    if(startValuePut()){

        if(isLiteral){
            os << "|\n";
        } else {
            os << ">\n";
        }
        const int level = std::max(static_cast<int>(states.size() - 1), 0);
        string indentString(level * indentWidth, ' ');
        os << indentString;

        const auto size = value.size();
        string::size_type pos = 0;
        int lineNumber = 0;
        while(true){
            auto found = value.find_first_of("\r\n", pos);
            if(found != string::npos){
                linebuf.assign(value, pos, (found - pos));
                pos = found + 1;
                if(pos < size && value[pos] == '\n'){
                    ++pos;
                }
            } else {
                linebuf.assign(value, pos, (size - pos));
                pos = size;
            }
            
            if(lineNumber > 0){
                os << "\n";
                os << indentString;
            }
            os << linebuf;

            if(pos == size){
                break;
            }
            ++lineNumber;
        }

        endValuePut();
    }
}


void YAMLWriter::putBlockStyleString(const char* value, bool isLiteral)
{
    impl->putBlockStyleString(value, isLiteral);
}


void YAMLWriter::putBlockStyleString(const std::string& value, bool isLiteral)
{
    impl->putBlockStyleString(value, isLiteral);
}


void YAMLWriter::putScalar(bool value)
{
    impl->putString(value ? "true" : "false");
}


void YAMLWriter::putScalar(int value)
{
    char buf[20];
#ifdef _WIN32
    _snprintf(buf, 20, "%d", value);
#else
    snprintf(buf, 20, "%d", value);
#endif
    impl->putString(buf);
}


void YAMLWriter::putScalar(double value)
{
    char buf[32];
#ifdef _WIN32
    _snprintf(buf, 32, impl->doubleFormat, value);
#else
    snprintf(buf, 32, impl->doubleFormat, value);
#endif
    impl->putString(buf);
}


void YAMLWriter::setDoubleFormat(const char* format)
{
    impl->doubleFormat = format;
}


void YAMLWriter::startMapping()
{
    impl->startMappingSub(false);
}


void YAMLWriter::startFlowStyleMapping()
{
    impl->startMappingSub(true);
}


void YAMLWriterImpl::startMappingSub(bool isFlowStyle)
{
    if(startValuePut()){
        int parentType = current->type;
        State& state = pushState(MAPPING, isFlowStyle);
        if(!state.isFlowStyle){
            if(parentType == MAPPING){
                newLine();
            }
        } else {
            os << "{ ";
            isCurrentNewLine = false;
        }
    }
}


template<class KeyStringType> void YAMLWriterImpl::putKey(const KeyStringType& key, StringStyle style)
{
    if(current->type == MAPPING && !current->isKeyPut){
        if(current->isFlowStyle){
            if(current->hasValuesBeenPut){
                os << ", ";
            }
        } else {
            indent();
        }

        switch(style){
        case SINGLE_QUOTED:
            os << "'" << key << "': ";
            break;
        case DOUBLE_QUOTED:
            os << "\"" << key << "\": ";
            break;
        default:
            os << key << ": ";
            break;
        }

        current->isKeyPut = true;
        isCurrentNewLine = false;
    }
}


void YAMLWriter::putKey(const char* key, StringStyle style)
{
    impl->putKey(key, style);
}


void YAMLWriter::putKey(const std::string& key, StringStyle style)
{
    impl->putKey(key, style);
}


void YAMLWriterImpl::endMapping()
{
    if(current->type == MAPPING){
        if(current->isFlowStyle){
            os << " }";
        }
        popState();
        endValuePut();
    }
}


void YAMLWriter::endMapping()
{
    impl->endMapping();
}


void YAMLWriter::startListing()
{
    impl->startListingSub(false);
}


void YAMLWriter::startFlowStyleListing()
{
    impl->startListingSub(true);
}


void YAMLWriterImpl::startListingSub(bool isFlowStyle)
{
    if(startValuePut()){
        State& state = pushState(LISTING, isFlowStyle);
        if(!state.isFlowStyle){
            if(!isTopLevel()){
                newLine();
            }
        } else {
            os << "[ ";
            isCurrentNewLine = false;
            doInsertLineFeed = false;
        }
    }
}


void YAMLWriterImpl::endListing()
{
    if(current->type == LISTING){
        if(current->isFlowStyle){
            os << " ]";
        }
        popState();
        endValuePut();
    }
}


void YAMLWriter::endListing()
{
    impl->endListing();
}


void YAMLWriter::putNode(const ValueNode* node)
{
    impl->putNodeMain(node, false);
}


void YAMLWriterImpl::putNodeMain(const ValueNode* node, bool doCheckLF)
{
    switch(node->nodeType()){

    case ValueNode::SCALAR:
    {
        const ScalarNode* scalar = static_cast<const ScalarNode*>(node);
        if(!doCheckLF){
            putScalarNode(scalar);
        } else {
            ValueNode::TypeBit LFType = scalar->LFType();
            if(!LFType){
                putScalarNode(scalar);
            } else if(LFType & ValueNode::INSERT_LF){
                doInsertLineFeed = true;
                putScalarNode(scalar);
            } else if(LFType & ValueNode::APPEND_LF){
                putScalarNode(scalar);
                doInsertLineFeed = true;
            }
        }
        break;
    }

    case ValueNode::MAPPING:
        putMappingNode(static_cast<const Mapping*>(node));
        break;

    case ValueNode::LISTING:
        putListingNode(static_cast<const Listing*>(node));
        break;

    default:
        ValueNode::UnknownNodeTypeException ex;
        ex.setMessage("Unknown node type");
        throw ex;
    }
}


void YAMLWriterImpl::putScalarNode(const ScalarNode* scalar)
{
    switch(scalar->stringStyle()){
    case PLAIN_STRING:
        putString(scalar->stringValue());
        break;

    case SINGLE_QUOTED:
        putSingleQuotedString(scalar->stringValue());
        break;

    case DOUBLE_QUOTED:
        putDoubleQuotedString(scalar->stringValue());
        break;

    case LITERAL_STRING:
        putBlockStyleString(scalar->stringValue(), true);

    case FOLDED_STRING:
        putBlockStyleString(scalar->stringValue(), false);
        break;

    default:
        putDoubleQuotedString(scalar->stringValue());
        break;
    }
}
    

void YAMLWriterImpl::putMappingNode(const Mapping* mapping)
{
    startMappingSub(mapping->isFlowStyle());

    if(isKeyOrderPreservationMode){
        const int n(mapping->size());
        vector<Mapping::const_iterator> iters(n);
        int index = 0;
        for(Mapping::const_iterator it = mapping->begin(); it != mapping->end(); ++it){
            iters[index++] = it;
        }

        struct KeyOrderCmpFunc {
            bool operator()(const Mapping::const_iterator& it1, const Mapping::const_iterator& it2) const {
                return (it1->second->indexInMapping() < it2->second->indexInMapping());
            }
        };

        std::sort(iters.begin(), iters.end(), KeyOrderCmpFunc());

        for(int i=0; i < n; ++i){
            Mapping::const_iterator& it = iters[i];
            const string& key = it->first;
            if(!key.empty()){
                putKey(key, mapping->keyStringStyle());
                const ValueNodePtr& node = it->second;
                putNodeMain(node, false);
            }
        }        
    } else {
        for(Mapping::const_iterator it = mapping->begin(); it != mapping->end(); ++it){
            const string& key = it->first;
            if(!key.empty()){
                putKey(key, mapping->keyStringStyle());
                const ValueNodePtr& node = it->second;
                putNodeMain(node, false);
            }
        }
    }

    endMapping();
}


void YAMLWriterImpl::putListingNode(const Listing* listing)
{
    bool doCheckLF;
    if(listing->isFlowStyle()){
        startListingSub(true);
        doCheckLF = true;
    } else {
        startListingSub(false);
        doCheckLF = false;
    }

    const int n = listing->size();
    for(int i=0; i < n; ++i){
        putNodeMain(listing->at(i), doCheckLF);
    }

    endListing();
}


const Mapping* YAMLWriter::info() const
{
    return impl->info;
}


Mapping* YAMLWriter::info()
{
    return impl->info;
}


template<> double YAMLWriter::info(const std::string& key) const
{
    return impl->info->get(key).toDouble();
}


template<> double YAMLWriter::info(const std::string& key, const double& defaultValue) const
{
    double value;
    if(impl->info->read(key, value)){
        return value;
    }
    return defaultValue;
}


template<> bool YAMLWriter::info(const std::string& key, const bool& defaultValue) const
{
    bool value;
    if(impl->info->read(key, value)){
        return value;
    }
    return defaultValue;
}


template<> double YAMLWriter::getOrCreateInfo(const std::string& key, const double& defaultValue)
{
    double value;
    if(!impl->info->read(key, value)){
        impl->info->write(key, defaultValue);
        value = defaultValue;
    }
    return value;
}


template<> bool YAMLWriter::getOrCreateInfo(const std::string& key, const bool& defaultValue)
{
    bool value;
    if(!impl->info->read(key, value)){
        impl->info->write(key, defaultValue);
        value = defaultValue;
    }
    return value;
}


template<> void YAMLWriter::setInfo(const std::string& key, const double& value)
{
    impl->info->write(key, value);
}


template<> void YAMLWriter::setInfo(const std::string& key, const bool& value)
{
    impl->info->write(key, value);
}


void YAMLWriter::resetInfo(Mapping* info)
{
    impl->info = info;
}
