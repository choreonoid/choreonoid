/**
   @author Shin'ichiro Nakaoka
*/

#include "YAMLWriter.h"
#include "UTF8.h"
#include <iostream>
#include <algorithm>
#include <boost/tokenizer.hpp>

using namespace std;
using namespace boost;
using namespace cnoid;


YAMLWriter::YAMLWriter(const std::string filename)
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


YAMLWriter::~YAMLWriter()
{
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
        ofs << "\n";
        isCurrentNewLine = true;
    }
}


void YAMLWriter::indent()
{
    if(!isCurrentNewLine){
        newLine();
    }
    ofs << current->indentString;
}


void YAMLWriter::startDocument()
{
    newLine();
    if(numDocuments > 0){
        ofs << "\n";
    }
    ofs << "---\n";
    
    ++numDocuments;
}


void YAMLWriter::putComment(const std::string& comment, bool doNewLine)
{
    if(doNewLine){
        indent();
    }
    ofs << "# " << toUTF8(comment);
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
            ofs << "- ";
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
                ofs << ", ";
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
        ofs << value;
        endValuePut();
    }
}


void YAMLWriter::putString(const std::string& value)
{
    putString_(toUTF8(value));
}


void YAMLWriter::putSingleQuotedString_(const std::string& value)
{
    if(startValuePut()){
        ofs << "'" << value << "'";
        endValuePut();
    }
}


void YAMLWriter::putSingleQuotedString(const std::string& value)
{
    putSingleQuotedString_(toUTF8(value));
}


void YAMLWriter::putDoubleQuotedString_(const std::string& value)
{
    if(startValuePut()){
        ofs << "\"" << value << "\"";
        endValuePut();
    }
}


void YAMLWriter::putDoubleQuotedString(const std::string& value)
{
    putDoubleQuotedString_(toUTF8(value));
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
            ofs << "|\n";
        } else {
            ofs << ">\n";
        }
        const int level = std::max(static_cast<int>(states.size() - 1), 0);
        string indentString(level * indentWidth, ' ');
        ofs << indentString;
        bool afterLF = false;
        Tokenizer::iterator it = tokens.begin();
        while(it != tokens.end()){
            if(afterLF){
                ofs << indentString;
                afterLF = false;
            }
            if(*it == "\n"){
                if(++it == tokens.end()){
                    break;
                }
                ofs << "\n";
                afterLF = true;
            } else {
                ofs << toUTF8(*it++);
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
            ofs << "{ ";
            isCurrentNewLine = false;
        }
    }
}


void YAMLWriter::putKey_(const std::string& key, StringStyle style)
{
    if(current->type == MAPPING && !current->isKeyPut){
        if(current->isFlowStyle){
            if(current->hasValuesBeenPut){
                ofs << ", ";
            }
        } else {
            indent();
        }

        switch(style){
        case SINGLE_QUOTED:
            ofs << "'" << key << "': ";
            break;
        case DOUBLE_QUOTED:
            ofs << "\"" << key << "\": ";
            break;
        default:
            ofs << key << ": ";
            break;
        }

        current->isKeyPut = true;
        isCurrentNewLine = false;
    }
}


void YAMLWriter::putKey(const std::string& key, StringStyle style)
{
    putKey_(toUTF8(key), style);
}


void YAMLWriter::endMapping()
{
    if(current->type == MAPPING){
        if(current->isFlowStyle){
            ofs << " }";
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
            ofs << "[ ";
            isCurrentNewLine = false;
            doInsertLineFeed = false;
        }
    }
}


void YAMLWriter::endListing()
{
    if(current->type == LISTING){
        if(current->isFlowStyle){
            ofs << " ]";
        }
        popState();
        endValuePut();
    }
}


void YAMLWriter::putNode(ValueNode& node)
{
    ValueNodePtr pNode = &node;
    putNode(pNode);
}


void YAMLWriter::putNode(const ValueNodePtr& node)
{
    switch(node->type()){

    case ValueNode::SCALAR: {
        const ScalarNode* scalar = static_cast<const ScalarNode*>(node.get());
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
        break;

    case ValueNode::MAPPING:
        putMappingNode(static_cast<const Mapping*>(node.get()));
        break;

    case ValueNode::LISTING:
        putListingNode(static_cast<const Listing*>(node.get()));
        break;

    case ValueNode::LF_NODE:
        if(current->isFlowStyle){
            doInsertLineFeed = true;
        }
        break;

    default:
        cout << "hogehoge" << endl;
        throw "hoge";
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
                putNode(node);
            }
        }        
    } else {
        for(Mapping::const_iterator it = mapping->begin(); it != mapping->end(); ++it){
            const string& key = it->first;
            if(!key.empty()){
                putKey_(key, mapping->keyQuoteStyle);
                const ValueNodePtr& node = it->second;
                putNode(node);
            }
        }
    }

    endMapping();
}


void YAMLWriter::putListingNode(const Listing* listing)
{
    if(listing->isFlowStyle()){
        startFlowStyleListing();
    } else {
        startListing();
    }

    const int n = listing->size();
    for(int i=0; i < n; ++i){
        putNode(listing->values[i]);
    }

    endListing();
}

        

