/**
   @author Shin'ichiro Nakaoka
*/

#include "YAMLReader.h"
#include <cerrno>
#include <stack>
#include <iostream>
#include <yaml.h>
#include <boost/format.hpp>

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {
const bool debugTrace = false;
}

namespace cnoid {

class YAMLReaderImpl
{
public:
    YAMLReaderImpl();
    ~YAMLReaderImpl();

    void setMappingFactory(YAMLReader::MappingFactoryBase* factory);
    void clearDocuments();
    bool load(const std::string& filename);
    bool parse(const std::string& yamlstring);
    bool parse();
    void popNode();
    void addNode(ValueNode* node);
    void onDocumentStart(yaml_event_t& event);
    void onDocumentEnd(yaml_event_t& event);
    void onMappingStart(yaml_event_t& event);
    void onMappingEnd(yaml_event_t& event);
    void onListingStart(yaml_event_t& event);
    void onListingEnd(yaml_event_t& event);
    void onScalar(yaml_event_t& event);
    void onAlias(yaml_event_t& event);

    static ScalarNode* createScalar(const yaml_event_t& event);
        
    yaml_parser_t parser;
    FILE* file;

    YAMLReader::MappingFactoryBase* mappingFactory;

    vector<ValueNodePtr> documents;
    int currentDocumentIndex;

    enum State { NONE, MAPPING_KEY, MAPPING_VALUE, LISTING };

    struct NodeInfo {
        ValueNodePtr node;
        string key;
    };

    stack<NodeInfo> nodeStack;

    bool isRegularMultiListingExpected;
    vector<int> expectedListingSizes;

    string errorMessage;
};
}


YAMLReader::YAMLReader()
{
    impl = new YAMLReaderImpl();
}


YAMLReaderImpl::YAMLReaderImpl()
{
    yaml_parser_initialize(&parser);
    file = 0;
    mappingFactory = new YAMLReader::MappingFactory<Mapping>();
    currentDocumentIndex = 0;
    isRegularMultiListingExpected = false;
}


YAMLReader::~YAMLReader()
{
    delete impl;
}


YAMLReaderImpl::~YAMLReaderImpl()
{
    yaml_parser_delete(&parser);
    
    if(file){
        fclose(file);
        file = 0;
    }

    delete mappingFactory;
}


void YAMLReader::setMappingFactory(MappingFactoryBase* factory)
{
    impl->setMappingFactory(factory);
}


void YAMLReaderImpl::setMappingFactory(YAMLReader::MappingFactoryBase* factory)
{
    delete mappingFactory;
    mappingFactory = factory;
}


void YAMLReader::expectRegularMultiListing()
{
    impl->isRegularMultiListingExpected = true;
}


void YAMLReader::clearDocuments()
{
    impl->clearDocuments();
}


void YAMLReaderImpl::clearDocuments()
{
    while(!nodeStack.empty()){
        nodeStack.pop();
    }
    documents.clear();
}


bool YAMLReader::load(const std::string& filename)
{
    return impl->load(filename);
}


bool YAMLReaderImpl::load(const std::string& filename)
{
    clearDocuments();

    if(isRegularMultiListingExpected){
        expectedListingSizes.clear();
    }
    
    currentDocumentIndex = 0;

    bool result = false;

    FILE* file = fopen(filename.c_str(), "rb");

    if(file==NULL){
        errorMessage = strerror(errno);
    } else {
        yaml_parser_set_input_file(&parser, file);
        try {
            result = parse();
        }
        catch(const ValueNode::Exception& ex){
            errorMessage = str(format("%1% at line %2%, column %3%")
                               % ex.message() % ex.line() % ex.column());
        }
        fclose(file);
    }

    return result;
}


bool YAMLReader::parse(const std::string& yamlstring)
{
    return impl->parse(yamlstring);
}

bool YAMLReaderImpl::parse(const std::string& yamlstring)
{
    clearDocuments();

    if(isRegularMultiListingExpected){
        expectedListingSizes.clear();
    }
    
    currentDocumentIndex = 0;

    bool result = false;
    
    yaml_parser_set_input_string(&parser, (const unsigned char *)(yamlstring.c_str()), yamlstring.length());
    try {
        result = parse();
    }
    catch(const ValueNode::Exception& ex){
        errorMessage = str(format("%1% at line %2%, column %3%")
                           % ex.message() % ex.line() % ex.column());
    }

    return result;
}


bool YAMLReaderImpl::parse()
{
    yaml_event_t event;
    
    bool done = false;
    
    while (!done) {

        if(!yaml_parser_parse(&parser, &event)){
            goto error;
        }

        switch(event.type){
            
        case YAML_STREAM_START_EVENT:
            break;
            
        case YAML_STREAM_END_EVENT:
            done = true;
            break;
            
        case YAML_DOCUMENT_START_EVENT:
            onDocumentStart(event);
            break;
            
        case YAML_DOCUMENT_END_EVENT:
            onDocumentEnd(event);
            break;
            
        case YAML_MAPPING_START_EVENT:
            onMappingStart(event);
            break;
            
        case YAML_MAPPING_END_EVENT:
            onMappingEnd(event);
            break;
            
        case YAML_SEQUENCE_START_EVENT:
            onListingStart(event);
            break;
            
        case YAML_SEQUENCE_END_EVENT:
            onListingEnd(event);
            break;
            
        case YAML_SCALAR_EVENT:
            onScalar(event);
            break;
            
        case YAML_ALIAS_EVENT:
            onAlias(event);
            break;
            
        default:
            break;
        }

        yaml_event_delete(&event);
    }

    return !documents.empty();

error:
    if(debugTrace){
        cout << "error" << endl;
    }
    if(parser.error != YAML_NO_ERROR && parser.problem != NULL){
        ValueNode::Exception ex;
        ex.setPosition(parser.problem_mark.line, parser.problem_mark.column);
        ex.setMessage(parser.problem);
        throw ex;
    }
    return false;
}


void YAMLReaderImpl::popNode()
{
    ValueNodePtr current = nodeStack.top().node;
    nodeStack.pop();
    if(nodeStack.empty()){
        documents.push_back(current);
    } else {
        addNode(current.get());
    }
}


void YAMLReaderImpl::addNode(ValueNode* node)
{
    NodeInfo& info = nodeStack.top();
    ValueNode* parent = info.node.get();
    if(parent->isMapping()){
        Mapping* mapping = static_cast<Mapping*>(parent);
        mapping->insert(info.key, node);
        info.key.clear();
    } else if(parent->isListing()){
        Listing* listing = static_cast<Listing*>(parent);
        listing->append(node);
    }
}


void YAMLReaderImpl::onDocumentStart(yaml_event_t& event)
{
    if(debugTrace){
        cout << "YAMLReaderImpl::onDocumentStart()" << endl;
    }
}


void YAMLReaderImpl::onDocumentEnd(yaml_event_t& event)
{
    if(debugTrace){
        cout << "YAMLReaderImpl::onDocumentEnd()" << endl;
    }
}


void YAMLReaderImpl::onMappingStart(yaml_event_t& event)
{
    if(debugTrace){
        cout << "YAMLReaderImpl::onMappingStart()" << endl;
    }

    NodeInfo info;
    Mapping* mapping = mappingFactory->create(event.start_mark.line, event.start_mark.column);
    mapping->setFlowStyle(event.data.mapping_start.style == YAML_FLOW_MAPPING_STYLE);
    info.node = mapping;
    
    nodeStack.push(info);
}


void YAMLReaderImpl::onMappingEnd(yaml_event_t& event)
{
    if(debugTrace){
        cout << "YAMLReaderImpl::onMappingEnd()" << endl;
    }

    popNode();
}


void YAMLReaderImpl::onListingStart(yaml_event_t& event)
{
    if(debugTrace){
        cout << "YAMLReaderImpl::onListingStart()" << endl;
    }

    NodeInfo info;
    Listing* listing;

    if(!isRegularMultiListingExpected){
        listing = new Listing(event.start_mark.line, event.start_mark.column);
    } else {
        size_t level = nodeStack.size();
        if(expectedListingSizes.size() <= level){
            expectedListingSizes.resize(level + 1, 0);
        }
        const int prevSize = expectedListingSizes[level];
        listing = new Listing(event.start_mark.line, event.start_mark.column, prevSize);
    }
    
    listing->setFlowStyle(event.data.sequence_start.style == YAML_FLOW_SEQUENCE_STYLE);
    info.node = listing;
    nodeStack.push(info);
}


void YAMLReaderImpl::onListingEnd(yaml_event_t& event)
{
    if(debugTrace){
        cout << "YAMLReaderImpl::onListingEnd()" << endl;
    }

    if(isRegularMultiListingExpected){
        Listing* listing = static_cast<Listing*>(nodeStack.top().node.get());
        const int level = nodeStack.size() - 1;
        expectedListingSizes[level] = listing->size();
    }
    
    popNode();
}


void YAMLReaderImpl::onScalar(yaml_event_t& event)
{
    if(debugTrace){
        cout << "YAMLReaderImpl::onScalar()" << endl;
    }

    yaml_char_t* value = event.data.scalar.value;
    size_t length = event.data.scalar.length;

    if(nodeStack.empty()){
        ValueNode::SyntaxException ex;
        ex.setMessage("Scalar value cannot be put on the top-level text position");
        const yaml_mark_t& start_mark = event.start_mark;
        ex.setPosition(start_mark.line, start_mark.column);
        throw ex;
    }

    NodeInfo& info = nodeStack.top();
    ValueNodePtr& parent = info.node;
     
    if(parent->isMapping()){
        if(info.key.empty()){
            info.key = string((char*)value, length);
            if(info.key.empty()){
                ValueNode::SyntaxException ex;
                ex.setMessage("empty key");
                const yaml_mark_t& start_mark = event.start_mark;
                ex.setPosition(start_mark.line, start_mark.column);
                throw ex;
            }
        } else {
            ScalarNode* scalar = createScalar(event);
            addNode(scalar);
        }
    } else if(parent->isListing()){
        ScalarNode* scalar = createScalar(event);
        addNode(scalar);
    }
}


ScalarNode* YAMLReaderImpl::createScalar(const yaml_event_t& event)
{
    ScalarNode* scalar = new ScalarNode((char*)event.data.scalar.value, event.data.scalar.length);

    const yaml_mark_t& start_mark = event.start_mark;
    scalar->line_ = start_mark.line;
    scalar->column_ = start_mark.column;

    switch(event.data.scalar.style){
    case YAML_PLAIN_SCALAR_STYLE:
        scalar->stringStyle = PLAIN_STRING;
        break;
    case YAML_SINGLE_QUOTED_SCALAR_STYLE:
        scalar->stringStyle = SINGLE_QUOTED;
        break;
    case YAML_DOUBLE_QUOTED_SCALAR_STYLE:
        scalar->stringStyle = DOUBLE_QUOTED;
        break;
    case YAML_LITERAL_SCALAR_STYLE:
        scalar->stringStyle = LITERAL_STRING;
        break;
    case YAML_FOLDED_SCALAR_STYLE:
        scalar->stringStyle = FOLDED_STRING;
        break;
    default:
        scalar->stringStyle = DOUBLE_QUOTED;
    }

    return scalar;
}


void YAMLReaderImpl::onAlias(yaml_event_t& event)
{
    if(debugTrace){
        cout << "YAMLReaderImpl::onAlias()" << endl;
    }
}


int YAMLReader::numDocuments()
{
    return impl->documents.size();
}


ValueNode* YAMLReader::document(int index)
{
    if(index >= static_cast<int>(impl->documents.size())){
        ValueNode::DocumentNotFoundException ex;
        if(index == 0){
            ex.setMessage("The yaml file does not contains any documents.");
        } else {
            ex.setMessage(str(format("The yaml file does not contains %1%-th document.") % index));
        }
        ex.setPosition(-1, -1);
        throw ex;
    }
    
    return impl->documents[index].get();
}


ValueNode* YAMLReader::loadDocument(const std::string& filename)
{
    if(!load(filename)){
        ValueNode::FileException ex;
        ex.setMessage(errorMessage());
        ex.setPosition(-1, -1);
        throw ex;
    }
    return document();
}

        
const std::string& YAMLReader::errorMessage()
{
    return impl->errorMessage;
}
