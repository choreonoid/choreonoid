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
#include "gettext.h"

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


ValueNode::Exception::Exception()
{
    line_ = -1;
    column_ = -1;
}


ValueNode::Exception::~Exception()
{

}


std::string ValueNode::Exception::message() const
{
    if(!message_.empty()){
        if(line_ >= 0){
            return str(format(_("%1% at line %2%, column %3%.")) % message_ % line_ % column_);
        } else {
            return str(format(_("%1%.")) % message_);
        }
    } else {
        if(line_ >= 0){
            return str(format(_("Error at line %2%, column %3%.")) % line_ % column_);
        } else {
            return string();
        }
    }
}


ValueNode::ValueNode(const ValueNode& org)
    : typeBits(org.typeBits),
      line_(org.line_),
      column_(org.column_),
      indexInMapping(org.indexInMapping)
{

}


ValueNode* ValueNode::clone() const
{
    return new ValueNode(*this);
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
        ex.setPosition(line(), column());
        ex.setMessage(str(format(_("The value \"%1%\" must be an integer value")) % scalar->stringValue));
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
        ex.setPosition(line(), column());
        ex.setMessage(str(format(_("The value \"%1%\" must be a double value")) % scalar->stringValue));
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
    ex.setPosition(line(), column());
    ex.setMessage(str(format(_("The value \"%1%\" must be a boolean value")) % scalar->stringValue));
    throw ex;
}


#ifdef _WIN32

bool ValueNode::read(std::string& out_value) const
{
    if(isScalar()){
        out_value = static_cast<const ScalarNode* const>(this)->stringValue;
        return !out_value.empty();
    }
    return false;
}


const std::string ValueNode::toString() const
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

#endif


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


ScalarNode::ScalarNode(const ScalarNode& org)
    : ValueNode(org),
      stringValue(org.stringValue),
      stringStyle(org.stringStyle)
{

}


ValueNode* ScalarNode::clone() const
{
    return new ScalarNode(*this);
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
    ex.setPosition(line(), column());
    ex.setMessage(message);
    throw ex;
}


void ValueNode::throwNotScalrException() const
{
    NotScalarException ex;
    ex.setPosition(line(), column());
    ex.setMessage(str(format(_("A %1% value must be a scalar value")) % getTypeName(typeBits)));
    throw ex;
}


void ValueNode::throwNotMappingException() const
{
    NotMappingException ex;
    ex.setPosition(line(), column());
    ex.setMessage(_("The value is not a mapping"));
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


Mapping::Mapping(const Mapping& org)
    : ValueNode(org),
      values(org.values),
      mode(org.mode),
      doubleFormat_(org.doubleFormat_),
      isFlowStyle_(org.isFlowStyle_),
      keyQuoteStyle(org.keyQuoteStyle)
{
    
}


ValueNode* Mapping::clone() const
{
    return new Mapping(*this);
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
    const_iterator p = values.find(key);
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
    const_iterator p = values.find(key);
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
    const_iterator p = values.find(key);
    if(p != values.end()){
        ValueNode* node = p->second.get();
        if(node->isListing()){
            return static_cast<Listing*>(node);
        }
    }
    return invalidListing.get();
}


ValueNodePtr Mapping::extract(const std::string& key)
{
    if(!isValid()){
        throwNotMappingException();
    }
    iterator p = values.find(key);
    if(p != values.end()){
        ValueNodePtr value = p->second;
        values.erase(p);
        return value;
    }
    return 0;
}



ValueNode& Mapping::get(const std::string& key) const
{
    if(!isValid()){
        throwNotMappingException();
    }
    const_iterator p = values.find(key);
    if(p == values.end()){
        throwKeyNotFoundException(key);
    }
    return *p->second;
}


void Mapping::throwKeyNotFoundException(const std::string& key) const
{
    KeyNotFoundException ex;
    ex.setPosition(line(), column());
    ex.setKey(key);
    ex.setMessage(str(format(_("Key \"%1%\" is not found in the mapping")) % key));
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
    const string uKey(key);
    insertSub(uKey, node);
}


void Mapping::insert(const Mapping* other)
{
    if(!isValid()){
        throwNotMappingException();
    }
    values.insert(other->values.begin(), other->values.end());
}


Mapping* Mapping::openMapping(const std::string& key, bool doOverwrite)
{
    if(!isValid()){
        throwNotMappingException();
    }

    Mapping* mapping = 0;
    const string uKey(key);
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
    const string uKey(key);
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
    ValueNode* node = find(key);
    if(node->isValid()){
        return node->read(out_value);
    }
    return false;
}


bool Mapping::read(const std::string &key, bool &out_value) const
{
    ValueNode* node = find(key);
    if(node->isValid()){
        return node->read(out_value);
    }
    return false;
}


bool Mapping::read(const std::string &key, int &out_value) const
{
    ValueNode* node = find(key);
    if(node->isValid()){
        return node->read(out_value);
    }
    return false;
}


bool Mapping::read(const std::string &key, double &out_value) const
{
    ValueNode* node = find(key);
    if(node->isValid()){
        return node->read(out_value);
    }
    return false;
}

void Mapping::write(const std::string &key, const std::string& value, StringStyle stringStyle)
{
    iterator p = values.find(key);
    if(p == values.end()){
        insertSub(key, new ScalarNode(value, stringStyle));
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


void Mapping::writeSub(const std::string &key, const char* text, size_t length, StringStyle stringStyle)
{
    iterator p = values.find(key);
    if(p == values.end()){
        insertSub(key, new ScalarNode(text, length, stringStyle));
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


Listing::Listing(const Listing& org)
    : ValueNode(org),
      values(org.values),
      doubleFormat_(org.doubleFormat_),
      isFlowStyle_(org.isFlowStyle_),
      doInsertLFBeforeNextElement(org.doInsertLFBeforeNextElement)
{

}


ValueNode* Listing::clone() const
{
    return new Listing(*this);
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
    ScalarNode* node = new ScalarNode(value, stringStyle);
    if(doInsertLFBeforeNextElement){
        node->typeBits |= INSERT_LF;
        doInsertLFBeforeNextElement = false;
    }
    values.push_back(node);
}


void Listing::insert(int index, ValueNode* node)
{
    if(index >= 0){
        if(index > values.size()){
            index = values.size();
        }
        values.insert(values.begin() + index, node);
    }
}


void Listing::write(int i, const std::string& value, StringStyle stringStyle)
{
    values[i] = new ScalarNode(value, stringStyle);
}
