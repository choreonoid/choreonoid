/**
   @author Shin'ichiro Nakaoka
*/

#include "YAMLWriter.h"
#include <iostream>
#include <algorithm>
#include <boost/tokenizer.hpp>

using namespace std;
using namespace boost;
using namespace cnoid;


YAMLWriter::YAMLWriter(const std::string filename)
    : os(ofs)
{
    indentWidth = 2;
    isCurrentNewLine = true;
    current = 0;
    numDocuments = 0;
    isKeyOrderPreservationMode = false;

    doubleFormat = "%.7g";

    ofs.open(filename.c_str());

    pushState(TOP, false);
}


YAMLWriter::YAMLWriter(std::ostream& os)
    : os(os)
{
    indentWidth = 2;
    isCurrentNewLine = true;
    current = 0;
    numDocuments = 0;
    isKeyOrderPreservationMode = false;

    doubleFormat = "%.7g";

    pushState(TOP, false);
}


YAMLWriter::~YAMLWriter()
{
    os.flush();
    ofs.close();
}


void YAMLWriter::setIndentWidth(int n)
{
    if(isTopLevel()){
        indentWidth = n;
    }
}


void YAMLWriter::setKeyOrderPreservationMode(bool on)
{
    isKeyOrderPreservationMode = true;
}


bool YAMLWriter::isTopLevel()
{
    return (states.size() <= 1);
}


YAMLWriter::State& YAMLWriter::pushState(int type, bool isFlowStyle)
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


void YAMLWriter::popState()
{
    states.pop();
    current = &states.top();
}


void YAMLWriter::newLine()
{
    if(!isCurrentNewLine){
        os << "\n";
        isCurrentNewLine = true;
    }
}


void YAMLWriter::indent()
{
    if(!isCurrentNewLine){
        newLine();
    }
    os << current->indentString;
}


void YAMLWriter::startDocument()
{
    newLine();
    if(numDocuments > 0){
        os << "\n";
    }
    os << "---\n";
    
    ++numDocuments;
}


void YAMLWriter::putComment(const std::string& comment, bool doNewLine)
{
    if(doNewLine){
        indent();
    }
    os << "# " << comment;
    isCurrentNewLine = false;
    newLine();
}


bool YAMLWriter::makeValuePutReady()
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


bool YAMLWriter::startValuePut()
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


void YAMLWriter::endValuePut()
{
    current->hasValuesBeenPut = true;
    if(current->type == MAPPING){
        current->isKeyPut = false;
    }
    if(!current->isFlowStyle){
        newLine();
    }
}


void YAMLWriter::putString_(const std::string& value)
{
    if(startValuePut()){
        os << value;
        endValuePut();
    }
}


void YAMLWriter::putString(const std::string& value)
{
    putString_(value);
}


void YAMLWriter::putSingleQuotedString_(const std::string& value)
{
    if(startValuePut()){
        os << "'" << value << "'";
        endValuePut();
    }
}


void YAMLWriter::putSingleQuotedString(const std::string& value)
{
    putSingleQuotedString_(value);
}


void YAMLWriter::putDoubleQuotedString_(const std::string& value)
{
    if(startValuePut()){
        os << "\"" << value << "\"";
        endValuePut();
    }
}


void YAMLWriter::putDoubleQuotedString(const std::string& value)
{
    putDoubleQuotedString_(value);
}


void YAMLWriter::putBlockStyleString(const std::string& value, bool isLiteral)
{
    if(current->isFlowStyle){
        ValueNode::SyntaxException ex;
        ex.setMessage("A block-style string cannot be inserted into a flow-style container");
        throw ex;
    }
    
    if(startValuePut()){
        static char_separator<char> sep("\r", "\n");
        typedef tokenizer<char_separator<char> > Tokenizer;
        Tokenizer tokens(value, sep);

        if(isLiteral){
            os << "|\n";
        } else {
            os << ">\n";
        }
        const int level = std::max(static_cast<int>(states.size() - 1), 0);
        string indentString(level * indentWidth, ' ');
        os << indentString;
        bool afterLF = false;
        Tokenizer::iterator it = tokens.begin();
        while(it != tokens.end()){
            if(afterLF){
                os << indentString;
                afterLF = false;
            }
            if(*it == "\n"){
                if(++it == tokens.end()){
                    break;
                }
                os << "\n";
                afterLF = true;
            } else {
                os << *it++;
            }
        }
        
        endValuePut();
    }
}


void YAMLWriter::putScalar(const double& value)
{
    char buf[20];
#ifdef _WIN32
    _snprintf(buf, 20, doubleFormat, value);
#else
    snprintf(buf, 20, doubleFormat, value);
#endif
    putString_(buf);
}


void YAMLWriter::setDoubleFormat(const char* format)
{
    doubleFormat = format;
}


void YAMLWriter::startMapping()
{
    startMappingSub(false);
}


void YAMLWriter::startFlowStyleMapping()
{
    startMappingSub(true);
}


void YAMLWriter::startMappingSub(bool isFlowStyle)
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


void YAMLWriter::putKey_(const std::string& key, StringStyle style)
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


void YAMLWriter::putKey(const std::string& key, StringStyle style)
{
    putKey_(key, style);
}


void YAMLWriter::endMapping()
{
    if(current->type == MAPPING){
        if(current->isFlowStyle){
            os << " }";
        }
        popState();
        endValuePut();
    }
}


void YAMLWriter::startListing()
{
    startListingSub(false);
}


void YAMLWriter::startFlowStyleListing()
{
    startListingSub(true);
}


void YAMLWriter::startListingSub(bool isFlowStyle)
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


void YAMLWriter::endListing()
{
    if(current->type == LISTING){
        if(current->isFlowStyle){
            os << " ]";
        }
        popState();
        endValuePut();
    }
}


void YAMLWriter::putNode(const ValueNode* node)
{
    putNodeMain(node, false);
}


void YAMLWriter::putNodeMain(const ValueNode* node, bool doCheckLF)
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


void YAMLWriter::putScalarNode(const ScalarNode* scalar)
{
    if(scalar->stringStyle == PLAIN_STRING){
        putString_(scalar->stringValue);
    } else if(scalar->stringStyle == SINGLE_QUOTED){
        putSingleQuotedString_(scalar->stringValue);
    } else if(scalar->stringStyle == DOUBLE_QUOTED){
        putDoubleQuotedString_(scalar->stringValue);
    } else if(scalar->stringStyle == LITERAL_STRING){
        putLiteralString(scalar->stringValue);
    } else if(scalar->stringStyle == FOLDED_STRING){
        putFoldedString(scalar->stringValue);
    } else {
        putDoubleQuotedString_(scalar->stringValue);
    }
}
    

void YAMLWriter::putMappingNode(const Mapping* mapping)
{
    if(mapping->isFlowStyle()){
        startFlowStyleMapping();
    } else {
        startMapping();
    }

    if(isKeyOrderPreservationMode){
        const int n(mapping->size());
        vector<Mapping::const_iterator> iters(n);
        int index = 0;
        for(Mapping::const_iterator it = mapping->begin(); it != mapping->end(); ++it){
            iters[index++] = it;
        }

        struct KeyOrderCmpFunc {
            bool operator()(const Mapping::const_iterator& it1, const Mapping::const_iterator& it2) const {
                return (it1->second->indexInMapping < it2->second->indexInMapping);
            }
        };

        std::sort(iters.begin(), iters.end(), &Mapping::compareIters);

        for(int i=0; i < n; ++i){
            Mapping::const_iterator& it = iters[i];
            const string& key = it->first;
            if(!key.empty()){
                putKey_(key, mapping->keyQuoteStyle);
                const ValueNodePtr& node = it->second;
                putNodeMain(node, false);
            }
        }        
    } else {
        for(Mapping::const_iterator it = mapping->begin(); it != mapping->end(); ++it){
            const string& key = it->first;
            if(!key.empty()){
                putKey_(key, mapping->keyQuoteStyle);
                const ValueNodePtr& node = it->second;
                putNodeMain(node, false);
            }
        }
    }

    endMapping();
}


void YAMLWriter::putListingNode(const Listing* listing)
{
    bool doCheckLF;
    if(listing->isFlowStyle()){
        startFlowStyleListing();
        doCheckLF = true;
    } else {
        startListing();
        doCheckLF = false;
    }

    const int n = listing->size();
    for(int i=0; i < n; ++i){
        putNodeMain(listing->values[i], doCheckLF);
    }

    endListing();
}
