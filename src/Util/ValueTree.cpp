/**
   @author Shin'ichiro Nakaoka
*/

#include "ValueTree.h"
#include <stack>
#include <iostream>
#include <yaml.h>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#ifdef _WIN32
#define snprintf _snprintf_s
#endif

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {

const bool debugTrace = false;

const char* getTypeName(int typeBits){
    if(typeBits & ValueNode::SCALAR){
        return "scalar";
    } else if(typeBits & ValueNode::MAPPING){
        return "mapping";
    } else if(typeBits & ValueNode::LISTING){
        return "listing";
    } else {
        return "unknown type node";
    }
}

map<string, bool> booleanSymbols;

const char* defaultDoubleFormat = "%.6g";

ValueNodePtr invalidNode;
MappingPtr invalidMapping;
ListingPtr invalidListing;

}

ValueNode::Initializer ValueNode::initializer;

ValueNode::Initializer::Initializer()
{
    invalidNode = new ValueNode(INVALID_NODE);
    invalidMapping = new Mapping();
    invalidMapping->typeBits = INVALID_NODE;
    invalidListing = new Listing();
    invalidListing->typeBits = INVALID_NODE;
    
    booleanSymbols["true"] = true;
    booleanSymbols["yes"] = true;
    booleanSymbols["on"] = true;
    booleanSymbols["false"] = false;
    booleanSymbols["no"] = false;
    booleanSymbols["off"] = false;
}


ValueNode::Exception::~Exception()
{

}


// disabled
ValueNode::ValueNode(const ValueNode&)
{
    // throw an exception here ?
}


// disabled
ValueNode& ValueNode::operator=(const ValueNode&)
{
    // throw an exception here ?
    return *this;
}

/*
  void ValueNode::initialize()
  {
  static bool initialized = false;

  if(!initialized){

  invalidNode = new ValueNode(INVALID_NODE);

  invalidMapping = new Mapping();
  invalidMapping->typeBits = INVALID_NODE;

  invalidListing = new Listing();
  invalidListing->typeBits = INVALID_NODE;
        
  booleanSymbols["true"] = true;
  booleanSymbols["yes"] = true;
  booleanSymbols["on"] = true;
  booleanSymbols["false"] = false;
  booleanSymbols["no"] = false;
  booleanSymbols["off"] = false;

  initialized = true;
  }
  }
*/


bool ValueNode::read(int &out_value) const
{
    if(isScalar()){
        const char* nptr = &(static_cast<const ScalarNode* const>(this)->stringValue[0]);
        char* endptr;
        out_value = strtol(nptr, &endptr, 10);
        if(endptr > nptr){
            return true;
        }
    }
    return false;
}


int ValueNode::toInt() const
{
    if(!isScalar()){
        throwNotScalrException();
    }

    const ScalarNode* const scalar = static_cast<const ScalarNode* const>(this);
    
    const char* nptr = &(scalar->stringValue[0]);
    char* endptr;
    const int value = strtol(nptr, &endptr, 10);

    if(endptr == nptr){
        ScalarTypeMismatchException ex;
        ex.setMessage(str(format("\"%1%\" at line %2%, column %3% should be an integer value.")
                          % scalar->stringValue % line() % column()));
        ex.setPosition(line(), column());
        throw ex;
    }

    return value;
}


bool ValueNode::read(double& out_value) const
{
    if(isScalar()){
        const char* nptr = &(static_cast<const ScalarNode* const>(this)->stringValue[0]);
        char* endptr;
        out_value = strtod(nptr, &endptr);
        if(endptr > nptr){
            return true;
        }
    }
    return false;
}


double ValueNode::toDouble() const
{
    if(!isScalar()){
        throwNotScalrException();
    }

    const ScalarNode* const scalar = static_cast<const ScalarNode* const>(this);

    const char* nptr = &(scalar->stringValue[0]);
    char* endptr;
    const double value = strtod(nptr, &endptr);

    if(endptr == nptr){
        ScalarTypeMismatchException ex;
        ex.setMessage(str(format("\"%1%\" at line %2%, column %3% should be a double value.")
                          % scalar->stringValue % line() % column()));
        ex.setPosition(line(), column());
        throw ex;
    }

    return value;
}


bool ValueNode::read(bool& out_value) const
{
    if(isScalar()){
        const ScalarNode* const scalar = static_cast<const ScalarNode* const>(this);
        map<string, bool>::iterator p = booleanSymbols.find(scalar->stringValue);
        if(p != booleanSymbols.end()){
            out_value = p->second;
            return true;
        }
    }
    return false;
}


bool ValueNode::toBool() const
{
    if(!isScalar()){
        throwNotScalrException();
    }

    const ScalarNode* const scalar = static_cast<const ScalarNode* const>(this);
    map<string, bool>::iterator p = booleanSymbols.find(scalar->stringValue);
    if(p != booleanSymbols.end()){
        return p->second;
    }
    
    ScalarTypeMismatchException ex;
    ex.setMessage(str(format("\"%1%\" at line %2%, column %3% should be a bool value.")
                      % scalar->stringValue % line() % column()));
    ex.setPosition(line(), column());
    throw ex;
}


#ifdef _WIN32

bool ValueNode::read(std::string& out_value) const
{
    if(isScalar()){
        out_value = fromUTF8(static_cast<const ScalarNode* const>(this)->stringValue);
        return !out_value.empty();
    }
    return false;
}


const std::string ValueNode::toString() const
{
    if(!isScalar()){
        throwNotScalrException();
    }
    return fromUTF8(static_cast<const ScalarNode* const>(this)->stringValue);
}


const std::string ValueNode::toUTF8String() const
{
    if(!isScalar()){
        throwNotScalrException();
    }
    return static_cast<const ScalarNode* const>(this)->stringValue;
}    

#else

bool ValueNode::read(std::string& out_value) const
{
    if(isScalar()){
        out_value = static_cast<const ScalarNode* const>(this)->stringValue;
        return !out_value.empty();
    }
    return false;
}


const std::string& ValueNode::toString() const
{
    if(!isScalar()){
        throwNotScalrException();
    }
    return static_cast<const ScalarNode* const>(this)->stringValue;
}


const std::string& ValueNode::toUTF8String() const
{
    return toString();
}
#endif


bool ValueNode::readUTF8String(std::string& out_value) const
{
    if(isScalar()){
        out_value = static_cast<const ScalarNode* const>(this)->stringValue;
        return !out_value.empty();
    }
    return false;
}


ScalarNode::ScalarNode(const std::string& value, StringStyle stringStyle)
    : stringValue(value),
      stringStyle(stringStyle)
{
    typeBits = SCALAR;
    line_ = -1;
    column_ = -1;
}


ScalarNode::ScalarNode(const char* text, size_t length)
    : stringValue(text, length)
{
    typeBits = SCALAR;
    stringStyle = PLAIN_STRING;
}


ScalarNode::ScalarNode(const char* text, size_t length, StringStyle stringStyle)
    : stringValue(text, length),
      stringStyle(stringStyle)
{
    typeBits = SCALAR;
    line_ = -1;
    column_ = -1;
}


ScalarNode::ScalarNode(int value)
    : stringValue(lexical_cast<string>(value))
{
    typeBits = SCALAR;
    line_ = -1;
    column_ = -1;
    stringStyle = PLAIN_STRING;
}


const Mapping* ValueNode::toMapping() const
{
    if(!isMapping()){
        throwNotMappingException();
    }
    return static_cast<const Mapping*>(this);
}


Mapping* ValueNode::toMapping()
{
    if(!isMapping()){
        throwNotMappingException();
    }
    return static_cast<Mapping*>(this);
}


const Listing* ValueNode::toListing() const
{
    if(!isListing()){
        throwNotListingException();
    }
    return static_cast<const Listing*>(this);
}


Listing* ValueNode::toListing()
{
    if(!isListing()){
        throwNotListingException();
    }
    return static_cast<Listing*>(this);
}


void ValueNode::throwException(const std::string& message) const
{
    Exception ex;
    
    if(hasLineInfo()){
        ex.setMessage(str(format("%1% (line %2%, column %3%).")
                          % message % line() % column()));
    } else {
        ex.setMessage(message);
    }
    ex.setPosition(line(), column());
    throw ex;
}


void ValueNode::throwNotScalrException() const
{
    NotScalarException ex;
    if(hasLineInfo()){
        ex.setMessage(str(format("The %1% at line %2%, column %3% should be a scalar value.")
                          % getTypeName(typeBits) % line() % column()));
    } else {
        ex.setMessage("Scalar value cannot be obtained from a non-scalar type yaml node.");
    }
    ex.setPosition(line(), column());
    throw ex;
}


void ValueNode::throwNotMappingException() const
{
    NotMappingException ex;
    ex.setPosition(line(), column());
    throw ex;
}


void ValueNode::throwNotListingException() const
{
    NotListingException ex;
    ex.setPosition(line(), column());
    throw ex;
}


Mapping::Mapping()
{
    typeBits = MAPPING;
    line_ = -1;
    column_ = -1;
    mode = READ_MODE;
    indexCounter = 0;
    keyQuoteStyle = PLAIN_STRING;
    isFlowStyle_ = false;
    doubleFormat_ = defaultDoubleFormat;
}


Mapping::Mapping(int line, int column)
{
    typeBits = MAPPING;
    line_ = line;
    column_ = column;
    mode = READ_MODE;
    indexCounter = 0;
    isFlowStyle_ = false;
    doubleFormat_ = defaultDoubleFormat;
}


Mapping::~Mapping()
{
    clear();
}


void Mapping::clear()
{
    values.clear();
    indexCounter = 0;
}


void Mapping::setDoubleFormat(const char* format)
{
    doubleFormat_ = format;
}


void Mapping::setKeyQuoteStyle(StringStyle style)
{
    keyQuoteStyle = style;
}


ValueNode* Mapping::find(const std::string& key) const
{
    if(!isValid()){
        throwNotMappingException();
    }
    const_iterator p = values.find(toUTF8(key));
    if(p != values.end()){
        return p->second.get();
    } else {
        return invalidNode.get();
    }
}


Mapping* Mapping::findMapping(const std::string& key) const
{
    if(!isValid()){
        throwNotMappingException();
    }
    const_iterator p = values.find(toUTF8(key));
    if(p != values.end()){
        ValueNode* node = p->second.get();
        if(node->isMapping()){
            return static_cast<Mapping*>(node);
        }
    }
    return invalidMapping.get();
}


Listing* Mapping::findListing(const std::string& key) const
{
    if(!isValid()){
        throwNotMappingException();
    }
    const_iterator p = values.find(toUTF8(key));
    if(p != values.end()){
        ValueNode* node = p->second.get();
        if(node->isListing()){
            return static_cast<Listing*>(node);
        }
    }
    return invalidListing.get();
}


ValueNode& Mapping::get(const std::string& key) const
{
    if(!isValid()){
        throwNotMappingException();
    }
    const_iterator p = values.find(toUTF8(key));
    if(p == values.end()){
        throwKeyNotFoundException(key);
    }
    return *p->second;
}


void Mapping::throwKeyNotFoundException(const std::string& key) const
{
    KeyNotFoundException ex;
    ex.setMessage(str(format("Key \"%1%\" is not found in the mapping that begins at line %2%, column %3%.")
                      % key % line() % column()));
    ex.setPosition(line(), column());
    ex.setKey(key);
    throw ex;
}


inline void Mapping::insertSub(const std::string& key, ValueNode* node)
{
    if(key.empty()){
        EmptyKeyException ex;
        throw ex;
    }
    //values.insert(make_pair(key, node));
    values[key] = node;
    node->indexInMapping = indexCounter++;
}


void Mapping::insert(const std::string& key, ValueNode* node)
{
    if(!isValid()){
        throwNotMappingException();
    }
    const string uKey(toUTF8(key));
    insertSub(uKey, node);
}


Mapping* Mapping::openMapping(const std::string& key, bool doOverwrite)
{
    if(!isValid()){
        throwNotMappingException();
    }

    Mapping* mapping = 0;
    const string uKey(toUTF8(key));
    iterator p = values.find(uKey);
    if(p != values.end()){
        ValueNode* node = p->second.get();
        if(!node->isMapping()){
            values.erase(p);
        } else {
            mapping = static_cast<Mapping*>(node);
            if(doOverwrite){
                mapping->clear();
            }
            mapping->indexInMapping = indexCounter++;
        }
    }

    if(!mapping){
        mapping = new Mapping();
        mapping->doubleFormat_ = doubleFormat_;
        insertSub(uKey, mapping);
    }

    return mapping;
}


Mapping* Mapping::openFlowStyleMapping(const std::string& key, bool doOverwrite)
{
    Mapping* m = openMapping(key, doOverwrite);
    m->setFlowStyle(true);
    return m;
}


Listing* Mapping::openListing(const std::string& key, bool doOverwrite)
{
    if(!isValid()){
        throwNotMappingException();
    }

    Listing* sequence = 0;
    const string uKey(toUTF8(key));
    iterator p = values.find(uKey);
    if(p != values.end()){
        ValueNode* node = p->second.get();
        if(!node->isListing()){
            values.erase(p);
        } else {
            sequence = static_cast<Listing*>(node);
            if(doOverwrite){
                sequence->clear();
            }
            sequence->indexInMapping = indexCounter++;
        }
    }

    if(!sequence){
        sequence = new Listing();
        sequence->doubleFormat_ = doubleFormat_;
        insertSub(uKey, sequence);
    }

    return sequence;
}


Listing* Mapping::openFlowStyleListing(const std::string& key, bool doOverwrite)
{
    Listing* s = openListing(key, doOverwrite);
    s->setFlowStyle(true);
    return s;
}


bool Mapping::remove(const std::string& key)
{
    return (values.erase(key) > 0);
}


bool Mapping::read(const std::string &key, std::string &out_value) const
{
    ValueNode* node = find(toUTF8(key));
    if(node->isValid()){
        return node->read(out_value);
    }
    return false;
}


bool Mapping::readUTF8(const std::string &key, std::string &out_value) const
{
    ValueNode* node = find(toUTF8(key));
    if(node->isValid()){
        return node->readUTF8String(out_value);
    }
    return false;
}


bool Mapping::read(const std::string &key, bool &out_value) const
{
    ValueNode* node = find(toUTF8(key));
    if(node->isValid()){
        return node->read(out_value);
    }
    return false;
}


bool Mapping::read(const std::string &key, int &out_value) const
{
    ValueNode* node = find(toUTF8(key));
    if(node->isValid()){
        return node->read(out_value);
    }
    return false;
}


bool Mapping::read(const std::string &key, double &out_value) const
{
    ValueNode* node = find(toUTF8(key));
    if(node->isValid()){
        return node->read(out_value);
    }
    return false;
}


void Mapping::writeUTF8(const std::string &key, const std::string& value, StringStyle stringStyle)
{
    string uKey(toUTF8(key));
    iterator p = values.find(uKey);
    if(p == values.end()){
        insertSub(uKey, new ScalarNode(value, stringStyle));
    } else {
        ValueNode* node = p->second.get();
        if(node->isScalar()){
            ScalarNode* scalar = static_cast<ScalarNode*>(node);
            scalar->stringValue = value;
            scalar->stringStyle = stringStyle;
            scalar->indexInMapping = indexCounter++;
        } else {
            throwNotScalrException();
        }
    }
}


/**
   This is for internal use. Text are not converted to UTF-8.
*/
void Mapping::writeSub(const std::string &key, const char* text, size_t length, StringStyle stringStyle)
{
    const string uKey(toUTF8(key));
    iterator p = values.find(uKey);
    if(p == values.end()){
        insertSub(uKey, new ScalarNode(text, length, stringStyle));
    } else {
        ValueNode* node = p->second.get();
        if(node->isScalar()){
            ScalarNode* scalar = static_cast<ScalarNode*>(node);
            scalar->stringValue = string(text, length);
            scalar->stringStyle = stringStyle;
            scalar->indexInMapping = indexCounter++;
        } else {
            throwNotScalrException();
        }
    }
}


void Mapping::write(const std::string &key, bool value)
{
    if(value){
        writeSub(key, "true", 4, PLAIN_STRING);
    } else {
        writeSub(key, "false", 5, PLAIN_STRING);
    }
}


void Mapping::write(const std::string &key, int value)
{
    char buf[32];
    int n = snprintf(buf, 32, "%d", value);
    writeSub(key, buf, n, PLAIN_STRING);
}


void Mapping::write(const std::string &key, double value)
{
    char buf[32];
    int n = snprintf(buf, 32, doubleFormat_, value);
    writeSub(key, buf, n, PLAIN_STRING);
}


void Mapping::writePath(const std::string &key, const std::string& value)
{
    write(key, filesystem::path(value).string(), DOUBLE_QUOTED);
}


bool Mapping::compareIters(const Mapping::const_iterator& it1, const Mapping::const_iterator& it2)
{
    return (it1->second->indexInMapping < it2->second->indexInMapping);
}


Listing::Listing()
{
    typeBits = LISTING;
    line_ = -1;
    column_ = -1;
    doubleFormat_ = defaultDoubleFormat;
    isFlowStyle_ = false;
    doInsertLFBeforeNextElement = false;
}


Listing::Listing(int size)
    : values(size)
{
    typeBits = LISTING;
    line_ = -1;
    column_ = -1;
    doubleFormat_ = defaultDoubleFormat;
    isFlowStyle_ = false;
    doInsertLFBeforeNextElement = false;
}


Listing::Listing(int line, int column)
{
    typeBits = LISTING;
    line_ = line;
    column_ = column;
    doubleFormat_ = defaultDoubleFormat;
    isFlowStyle_ = false;
    doInsertLFBeforeNextElement = false;
}


Listing::Listing(int line, int column, int reservedSize)
    : values(reservedSize)
{
    typeBits = LISTING;
    line_ = line;
    column_ = column;
    values.resize(0);
    doubleFormat_ = defaultDoubleFormat;
    isFlowStyle_ = false;
    doInsertLFBeforeNextElement = false;
}


Listing::~Listing()
{
    clear();
}


void Listing::clear()
{
    values.clear();
}


void Listing::reserve(int size)
{
    values.reserve(size);
}


void Listing::setDoubleFormat(const char* format)
{
    doubleFormat_ = format;
}


void Listing::insertLF(int maxColumns, int numValues)
{
    if(values.empty()){
        if(numValues > 0 && numValues > maxColumns){
            doInsertLFBeforeNextElement = true;
        }
    } else if((values.size() % maxColumns) == 0){
        values.back()->typeBits |= (TypeBit)APPEND_LF;
    }
}


void Listing::appendLF()
{
    if(values.empty()){
        doInsertLFBeforeNextElement = true;
    } else {
        values.back()->typeBits |= APPEND_LF;
    }
}


Mapping* Listing::newMapping()
{
    Mapping* mapping = new Mapping();
    mapping->doubleFormat_ = doubleFormat_;
    append(mapping);
    return mapping;
}


void Listing::append(int value)
{
    char buf[32];
    int n = snprintf(buf, 32, "%d", value);
    ScalarNode* node = new ScalarNode(buf, n, PLAIN_STRING);
    if(doInsertLFBeforeNextElement){
        node->typeBits |= INSERT_LF;
        doInsertLFBeforeNextElement = false;
    }
    values.push_back(node);
}


void Listing::write(int i, int value)
{
    char buf[32];
    int n = snprintf(buf, 32, "%d", value);
    values[i] = new ScalarNode(buf, n, PLAIN_STRING);
}


/*
  void Listing::append(size_t value)
  {
  char buf[32];
  int n = snprintf(buf, 32, "%zd", value);
  values.push_back(new ScalarNode(buf, n, PLAIN_STRING));
  }
*/


void Listing::append(double value)
{
    char buf[32];
    int n = snprintf(buf, 32, doubleFormat_, value);
    ScalarNode* node = new ScalarNode(buf, n, PLAIN_STRING);
    if(doInsertLFBeforeNextElement){
        node->typeBits |= INSERT_LF;
        doInsertLFBeforeNextElement = false;
    }
    values.push_back(node);
}


void Listing::append(const std::string& value, StringStyle stringStyle)
{
    ScalarNode* node = new ScalarNode(toUTF8(value), stringStyle);
    if(doInsertLFBeforeNextElement){
        node->typeBits |= INSERT_LF;
        doInsertLFBeforeNextElement = false;
    }
    values.push_back(node);
}


void Listing::write(int i, const std::string& value, StringStyle stringStyle)
{
    values[i] = new ScalarNode(toUTF8(value), stringStyle);
}
