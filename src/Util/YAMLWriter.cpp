#include "YAMLWriter.h"
#include "NullOut.h"
#include "UTF8.h"
#include <fmt/format.h>
#include <iostream>
#include <algorithm>
#include <stack>
#include <fstream>
#include <unordered_set>
#include <unordered_map>

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

enum { TOP, MAPPING, LISTING };

struct State {
    int type;
    bool isFlowStyle;
    bool isKeyPut;
    bool hasValuesBeenPut;
    string indentString;
};

class YAMLWriter::Impl
{
public:
    std::ofstream ofs;
    std::ostream* os_;
    std::ostream* messageSink_;

    int indentWidth;
    int numDocuments;
    bool isCurrentNewLine;
    bool isKeyOrderPreservationMode;
    bool doInsertLineFeed;
    const char* doubleFormat;
    std::stack<State> states;
    State* current;
    string linebuf;
    unordered_set<ref_ptr<const ValueNode>> nodeSet;
    typedef unordered_map<ref_ptr<const ValueNode>, int> AnchorMap;
    AnchorMap anchorMap;
    int anchorIndex;
    string anchor;

    Impl(YAMLWriter* self, const std::string& filename);
    Impl(YAMLWriter* self, std::ostream& os);
    ~Impl();

    ostream& os() { return *os_; }
    bool isTopLevel();
    State& pushState(int type, bool isFlowStyle);
    void popState();
    void newLine();
    void indent();
    bool makeValuePutReady();
    bool startValuePut(bool doPutValueInSameLine);
    void endValuePut();
    void putString(const std::string& value);
    void putString(const char* value);
    template<class StringType> void putSingleQuotedString(const StringType& value);
    void putDoubleQuotedString(const char* value);
    void putDoubleQuotedString(const string& value);
    void putBlockStyleString(const std::string& value, bool isLiteral);
    void startMappingSub(bool isFlowStyle);
    template<class StringType> void putKey(const StringType& key, StringStyle style);
    void endMapping();
    void startListingSub(bool isFlowStyle);
    void endListing();
    void scanSharedNodes(const ValueNode* node);
    void scanSharedNodesIter(const ValueNode* node);
    void putNodeMain(const ValueNode* node, bool doCheckLF);
    bool setAnchorOrPutAliasForSharedNode(const ValueNode* node);
    void putScalarNode(const ScalarNode* scalar);
    void putMappingNode(const Mapping* mapping);
    void putListingNode(const Listing* listing);
};

}


YAMLWriter::YAMLWriter()
{
    impl = new Impl(this, nullout());
}


YAMLWriter::YAMLWriter(const std::string& filename)
{
    impl = new Impl(this, filename);
    openFile(filename);
}


YAMLWriter::Impl::Impl(YAMLWriter* self, const std::string&)
    : Impl(self, ofs)
{

}


YAMLWriter::YAMLWriter(std::ostream& os)
{
    impl = new Impl(this, os);
}


YAMLWriter::Impl::Impl(YAMLWriter* self, std::ostream& os)
    : os_(&os)
{
    indentWidth = 2;
    isCurrentNewLine = true;
    current = nullptr;
    numDocuments = 0;
    isKeyOrderPreservationMode = false;
    messageSink_ = &nullout();

    doubleFormat = "%g";

    pushState(TOP, false);

    self->info_ = new Mapping;
}    


YAMLWriter::~YAMLWriter()
{
    closeFile();
    delete impl;
}


YAMLWriter::Impl::~Impl()
{

}


void YAMLWriter::setOutput(std::ostream& os)
{
    impl->os_ = &os;
}


void YAMLWriter::flush()
{
    impl->os().flush();
}


bool YAMLWriter::openFile(const std::string& filename)
{
    impl->ofs.open(fromUTF8(filename).c_str(), ios_base::out | ios_base::binary);
    if(impl->ofs.is_open()){
        setOutput(impl->ofs);
        return true;
    }
    return false;
}


bool YAMLWriter::isFileOpen()
{
    return impl->ofs.is_open();
}


void YAMLWriter::closeFile()
{
    impl->os().flush();
    impl->ofs.close();
    setOutput(nullout());
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


int YAMLWriter::indentWidth() const
{
    return impl->indentWidth;
}


void YAMLWriter::setKeyOrderPreservationMode(bool on)
{
    impl->isKeyOrderPreservationMode = on;
}


bool YAMLWriter::Impl::isTopLevel()
{
    return (states.size() <= 1);
}


State& YAMLWriter::Impl::pushState(int type, bool isFlowStyle)
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


void YAMLWriter::Impl::popState()
{
    states.pop();
    current = &states.top();
}


void YAMLWriter::Impl::newLine()
{
    if(!isCurrentNewLine){
        os() << "\n";
        isCurrentNewLine = true;
    }
}


void YAMLWriter::Impl::indent()
{
    if(!isCurrentNewLine){
        newLine();
    }
    os() << current->indentString;
}


void YAMLWriter::startDocument()
{
    impl->newLine();
    if(impl->numDocuments > 0){
        impl->os() << "\n";
    }
     impl->os() << "---\n";
    
    ++ impl->numDocuments;
}


void YAMLWriter::putComment(const std::string& comment, bool doNewLine)
{
    if(doNewLine){
        impl->indent();
    }
    impl->os() << "# " << comment;
    impl->isCurrentNewLine = false;
    impl->newLine();
}


bool YAMLWriter::Impl::makeValuePutReady()
{
    switch(current->type){
    case MAPPING:
        return current->isKeyPut;
    case LISTING:
        if(!current->isFlowStyle){
            if(!current->hasValuesBeenPut){
                newLine();
            }
            indent();
            os() << "- ";
        }
        isCurrentNewLine = false;
        return true;
    default:
        return true;
    }
}


bool YAMLWriter::Impl::startValuePut(bool doPutValueInSameLine)
{
    if(!makeValuePutReady()){
        return false;
    }
    
    if(current->type == LISTING && current->isFlowStyle){
        if(current->hasValuesBeenPut){
            os() << ", ";
        }
        if(doInsertLineFeed){
            newLine();
            indent();
            doInsertLineFeed = false;
            isCurrentNewLine = false;
        }
    }

    // Put an anchor
    if(!anchor.empty()){
        os() << anchor;
        if(doPutValueInSameLine || current->isFlowStyle){
            os() << " ";
        }
        anchor.clear();
    }

    return true;
}


void YAMLWriter::Impl::endValuePut()
{
    current->hasValuesBeenPut = true;
    if(current->type == MAPPING){
        current->isKeyPut = false;
    }
    if(!current->isFlowStyle){
        newLine();
    }
}


void YAMLWriter::Impl::putString(const std::string& value)
{
    if(startValuePut(true)){
        if(value.empty()){
            os() << "\"\"";
        } else {
            os() << value;
        }
        endValuePut();
    }
}


void YAMLWriter::Impl::putString(const char* value)
{
    if(startValuePut(true)){
        if(value[0] == '\0'){
            os() << "\"\"";
        } else {
            os() << value;
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


template<class StringType> void YAMLWriter::Impl::putSingleQuotedString(const StringType& value)
{
    if(startValuePut(true)){
        os() << "'" << value << "'";
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


void YAMLWriter::Impl::putDoubleQuotedString(const char* value)
{
    if(startValuePut(true)){
        os() << "\"";
        while(true){
            switch(*value){
            case '\0': goto end; break;
            case '\\': os() << "\\\\"; break;
            case '\"': os() << "\\\""; break;
            default: os() << *value; break;
            }
            ++value;
        }
end:
        os() << "\"";
        endValuePut();
    }
}


void YAMLWriter::Impl::putDoubleQuotedString(const string& value)
{
    if(startValuePut(true)){
        os() << "\"";
        for(size_t i=0; i < value.size(); ++i){
            switch(value[i]){
            case '\\': os() << "\\\\"; break;
            case '\"': os() << "\\\""; break;
            default: os() << value[i]; break;
            }
        }
        os() << "\"";
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


void YAMLWriter::Impl::putBlockStyleString(const std::string& value, bool isLiteral)
{
    if(current->isFlowStyle){
        ValueNode::SyntaxException ex;
        ex.setMessage("A block-style string cannot be inserted into a flow-style container");
        throw ex;
    }
    
    if(startValuePut(true)){

        if(isLiteral){
            os() << "|\n";
        } else {
            os() << ">\n";
        }
        const int level = std::max(static_cast<int>(states.size() - 1), 0);
        string indentString(level * indentWidth, ' ');
        os() << indentString;

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
                os() << "\n";
                os() << indentString;
            }
            os() << linebuf;

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


void YAMLWriter::Impl::startMappingSub(bool isFlowStyle)
{
    if(startValuePut(isFlowStyle)){
        int parentType = current->type;
        State& state = pushState(MAPPING, isFlowStyle);
        if(state.isFlowStyle){
            os() << "{ ";
            isCurrentNewLine = false;
        }
    }
}


template<class KeyStringType> void YAMLWriter::Impl::putKey(const KeyStringType& key, StringStyle style)
{
    if(current->type == MAPPING && !current->isKeyPut){
        if(current->isFlowStyle){
            if(current->hasValuesBeenPut){
                os() << ", ";
            }
        } else {
            if(!current->hasValuesBeenPut){
                newLine();
            }
            indent();
        }

        switch(style){
        case SINGLE_QUOTED:
            os() << "'" << key << "': ";
            break;
        case DOUBLE_QUOTED:
            os() << "\"" << key << "\": ";
            break;
        default:
            os() << key << ": ";
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


void YAMLWriter::Impl::endMapping()
{
    if(current->type == MAPPING){
        if(current->isFlowStyle){
            os() << " }";
        } else {
            if(!current->hasValuesBeenPut){
                os() << "{ }"; // Put an empty mapping
            }
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


void YAMLWriter::Impl::startListingSub(bool isFlowStyle)
{
    if(startValuePut(isFlowStyle)){
        State& state = pushState(LISTING, isFlowStyle);
        if(state.isFlowStyle){
            os() << "[ ";
            isCurrentNewLine = false;
            doInsertLineFeed = false;
        }
    }
}


void YAMLWriter::Impl::endListing()
{
    if(current->type == LISTING){
        if(current->isFlowStyle){
            os() << " ]";
        } else {
            if(!current->hasValuesBeenPut){
                os() << "[ ]"; // Put an empty listing
            }
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
    impl->scanSharedNodes(node);
    impl->putNodeMain(node, false);
    impl->nodeSet.clear();
    impl->anchorMap.clear();
}


void YAMLWriter::Impl::scanSharedNodes(const ValueNode* node)
{
    nodeSet.clear();
    anchorMap.clear();
    anchorIndex = 0;
    scanSharedNodesIter(node);
    nodeSet.clear();
}


void YAMLWriter::Impl::scanSharedNodesIter(const ValueNode* node)
{
    bool anchored = false;
    
    auto inserted = nodeSet.insert(node);
    if(!inserted.second){
        auto inserted = anchorMap.insert(AnchorMap::value_type(node, anchorIndex));
        if(inserted.second){
            ++anchorIndex;
        }
        return;
    }

    if(node->isMapping()){
        auto mapping = node->toMapping();
        if(!isKeyOrderPreservationMode){
            for(auto& kv : *mapping){
                scanSharedNodesIter(kv.second);
            }
        } else {
            const int n(mapping->size());
            vector<Mapping::const_iterator> iters(n);
            int index = 0;
            for(auto it = mapping->begin(); it != mapping->end(); ++it){
                iters[index++] = it;
            }
            struct KeyOrderCmpFunc {
                bool operator()(const Mapping::const_iterator& it1, const Mapping::const_iterator& it2) const {
                    return (it1->second->indexInMapping() < it2->second->indexInMapping());
                }
            };
            std::sort(iters.begin(), iters.end(), KeyOrderCmpFunc());
            for(auto& it : iters){
                scanSharedNodesIter(it->second);
            }
        }
    } else if(node->isListing()){
        for(auto& element : *node->toListing()){
            scanSharedNodesIter(element);
        }
    }
}


void YAMLWriter::Impl::putNodeMain(const ValueNode* node, bool doCheckLF)
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


bool YAMLWriter::Impl::setAnchorOrPutAliasForSharedNode(const ValueNode* node)
{
    auto it = anchorMap.find(node);
    if(it != anchorMap.end()){
        auto inserted = nodeSet.insert(node);
        if(!inserted.second){
            startValuePut(false);
            os() << format("*A{}", it->second);
            endValuePut();
            return true;
        } else {
            anchor = format("&A{}", it->second);
        }
    }
    return false;
}


void YAMLWriter::Impl::putScalarNode(const ScalarNode* scalar)
{
    if(setAnchorOrPutAliasForSharedNode(scalar)){
        return;
    }
    
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
    

void YAMLWriter::Impl::putMappingNode(const Mapping* mapping)
{
    if(setAnchorOrPutAliasForSharedNode(mapping)){
        return;
    }
    
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


void YAMLWriter::Impl::putListingNode(const Listing* listing)
{
    if(setAnchorOrPutAliasForSharedNode(listing)){
        return;
    }

    bool isFlowStyle = listing->isFlowStyle();
    startListingSub(isFlowStyle);

    for(auto& node : *listing){
        putNodeMain(node, isFlowStyle);
    }

    endListing();
}
