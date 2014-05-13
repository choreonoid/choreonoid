/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_VALUE_TREE_H_INCLUDED
#define CNOID_UTIL_VALUE_TREE_H_INCLUDED

#include "UTF8.h"
#include <map>
#include <vector>
#include <cstring>
#include <iosfwd>
#include <boost/intrusive_ptr.hpp>
#include <boost/function.hpp>
#include "exportdecl.h"

namespace cnoid {

class YAMLReaderImpl;
class YAMLWriter;

class ValueNode;

void intrusive_ptr_add_ref(cnoid::ValueNode* obj);
void intrusive_ptr_release(cnoid::ValueNode* obj);

class ScalarNode;
class Mapping;
class Listing;

#ifndef CNOID_BACKWARD_COMPATIBILITY
enum StringStyle { PLAIN_STRING, SINGLE_QUOTED, DOUBLE_QUOTED, LITERAL_STRING, FOLDED_STRING };
#else
enum StringStyle { PLAIN_STRING, YAML_PLAIN_STRING = PLAIN_STRING,
                   SINGLE_QUOTED, YAML_SINGLE_QUOTED = SINGLE_QUOTED,
                   DOUBLE_QUOTED, YAML_DOUBLE_QUOTED = DOUBLE_QUOTED,
                   LITERAL_STRING, YAML_LITERAL = LITERAL_STRING,
                   FOLDED_STRING, YAML_FOLDED = FOLDED_STRING
};
#endif

class CNOID_EXPORT ValueNode
{
    struct Initializer {
        Initializer();
    };
    static Initializer initializer;
        
public:

#ifndef CNOID_BACKWARD_COMPATIBILITY
    enum Type { INVALID_NODE = 0, SCALAR = 1, MAPPING = 2, LISTING = 3, LF_NODE = 4 };
#else 
    enum Type { INVALID_NODE = 0, SCALAR = 1, MAPPING = 2, LISTING = 3, SEQUENCE = 3, LF_NODE = 4};
#endif

    bool isValid() const { return type_ != INVALID_NODE; }

    Type type() const { return type_; }

    int toInt() const;
    double toDouble() const;
    bool toBool() const;

    bool isScalar() const { return type_ == SCALAR; }
    bool isString() const { return type_ == SCALAR; }

#ifdef _WIN32
    const std::string toString() const;
    const std::string toUTF8String() const;

    operator std::string () const {
        return toString();
    }
#else
    const std::string& toString() const;
    const std::string& toUTF8String() const;

    operator const std::string& () const {
        return toString();
    }
#endif

    template<typename T> T to() const { return ""; }

    bool isMapping() const { return type_ == MAPPING; }
    const Mapping* toMapping() const;
    Mapping* toMapping();

    bool isListing() const { return type_ == LISTING; }
    const Listing* toListing() const;
    Listing* toListing();
        
#ifdef CNOID_BACKWARD_COMPATIBILITY
    bool isSequence() const { return type_ == LISTING; }
    const Listing* toSequence() const { return toListing(); }
    Listing* toSequence() { return toListing(); }
#endif

    bool read(int &out_value) const;
    bool read(double &out_value) const;
    bool read(bool &out_value) const;
    bool read(std::string &out_value) const;
    bool readUTF8String(std::string& out_value) const;

    bool hasLineInfo() const { return (line_ >= 0); }
    int line() const { return line_ + 1; }
    int column() const { return column_ + 1; }

    void throwException(const std::string& message) const;

    /**
       \todo integrate the exception classes with the common ones defined in Exception.h
    */
    class CNOID_EXPORT Exception {
public:
        virtual ~Exception();
        int line() const { return line_; }
        int column() const { return column_; }
        const std::string& message() const { return message_; }
        void setPosition(int line, int column) {
            line_ = line;
            column_ = column;
        }
        void setMessage(const std::string& m){
            message_ = m;
        }
private:
        int line_;
        int column_;
        std::string message_;
    };
        
    class KeyNotFoundException : public Exception {
    public:
        const std::string& key() { return key_; }
        void setKey(const std::string& key) { key_ = key; }
    private:
        std::string key_;
    };

    class EmptyKeyException : public Exception {
    };
        
    class NotScalarException : public Exception {
    };
        
    class ScalarTypeMismatchException : public Exception {
    };

    class NotMappingException : public Exception {
    };

    class NotListingException : public Exception {
    };

    class SyntaxException : public Exception {
    };

    class DocumentNotFoundException : public Exception {
    };

    class FileException : public Exception {
    };
        
private:
        
    int refCounter;
        
protected:

    ValueNode() : refCounter(0) { }
    ValueNode(Type type) : refCounter(0), type_(type), line_(-1), column_(-1) { }

    virtual ~ValueNode() { }

    void throwNotScalrException() const;
    void throwNotMappingException() const;
    void throwNotListingException() const;

    Type type_;

private:

    // disabled copy operations
    ValueNode(const ValueNode&);
    ValueNode& operator=(const ValueNode&);

    int line_;
    int column_;
    int indexInMapping; // used for YAMLWriter

    friend class YAMLReaderImpl;
    friend class YAMLWriter;
    friend class ScalarNode;
    friend class Mapping;
    friend class Listing;

    friend void cnoid::intrusive_ptr_add_ref(ValueNode* obj);
    friend void cnoid::intrusive_ptr_release(ValueNode* obj);
};

template<> inline double ValueNode::to<double>() const { return toDouble(); }
template<> inline int ValueNode::to<int>() const { return toInt(); }
template<> inline std::string ValueNode::to<std::string>() const { return toString(); }
    
typedef boost::intrusive_ptr<ValueNode> ValueNodePtr;

    
class CNOID_EXPORT ScalarNode : public ValueNode
{
public:
    ScalarNode(const std::string& value, StringStyle stringStyle = PLAIN_STRING);
    ScalarNode(int value);
    
private:
    ScalarNode(const char* text, size_t length);
    ScalarNode(const char* text, size_t length, StringStyle stringStyle);

    std::string stringValue;
    StringStyle stringStyle;

    friend class YAMLReaderImpl;
    friend class YAMLWriter;
    friend class ValueNode;
    friend class Mapping;
    friend class Listing;
};


class CNOID_EXPORT Mapping : public ValueNode
{
    typedef std::map<std::string, ValueNodePtr> Container;
        
public:

    typedef Container::iterator iterator;
    typedef Container::const_iterator const_iterator;

    Mapping();
    Mapping(int line, int column);
    virtual ~Mapping();
    bool empty() const { return values.empty(); }
    size_t size() const { return values.size(); }
    void clear();

    void setFlowStyle(bool isFlowStyle = true) { isFlowStyle_ = isFlowStyle; }
    bool isFlowStyle() const { return isFlowStyle_; }

    void setDoubleFormat(const char* format);
    const char* doubleFormat() { return doubleFormat_; }
        
    void setKeyQuoteStyle(StringStyle style);

    ValueNode* find(const std::string& key) const;
    Mapping* findMapping(const std::string& key) const;
    Listing* findListing(const std::string& key) const;

    ValueNode& get(const std::string& key) const;

    ValueNode& operator[](const std::string& key) const {
        return get(key);
    }

    void insert(const std::string& key, ValueNodePtr node);

    Mapping* openMapping(const std::string& key) {
        return openMapping(key, false);
    }
        
    Mapping* openFlowStyleMapping(const std::string& key) {
        return openFlowStyleMapping(key, false);
    }

    Mapping* createMapping(const std::string& key) {
        return openMapping(key, true);
    }
        
    Mapping* createFlowStyleMapping(const std::string& key) {
        return openFlowStyleMapping(key, true);
    }

    Listing* openListing(const std::string& key) {
        return openListing(key, false);
    }
        
    Listing* openFlowStyleListing(const std::string& key){
        return openFlowStyleListing(key, false);
    }

    Listing* createListing(const std::string& key){
        return openListing(key, true);
    }
        
    Listing* createFlowStyleListing(const std::string& key){
        return openFlowStyleListing(key, true);
    }

    bool remove(const std::string& key);

    bool read(const std::string &key, std::string &out_value) const;
    bool readUTF8(const std::string &key, std::string &out_value) const;
    bool read(const std::string &key, bool &out_value) const;
    bool read(const std::string &key, int &out_value) const;
    bool read(const std::string &key, double &out_value) const;

    bool read(const std::string &key, boost::function<void(double)> setterFunc) const;

        
    template <class T>
        T read(const std::string& key) const {
        T value;
        if(read(key, value)){
            return value;
        } else {
            throwKeyNotFoundException(key);
        }
    }

    template <class T>
        T get(const std::string& key, const T& defaultValue) const {
        T value;
        if(read(key, value)){
            return value;
        } else {
            return defaultValue;
        }
    }

    std::string get(const std::string& key, const char* defaultValue) const {
        std::string value;
        if(read(key, value)){
            return value;
        } else {
            return defaultValue;
        }
    }

    void writeUTF8(const std::string &key, const std::string& value, StringStyle stringStyle = PLAIN_STRING);

    void write(const std::string &key, const std::string& value, StringStyle stringStyle = PLAIN_STRING) {
        writeUTF8(key, toUTF8(value), stringStyle);
    }

    void writeUTF8(const std::string &key, const char* value, StringStyle stringStyle = PLAIN_STRING){
        writeUTF8(key, std::string(value), stringStyle);
    }
        
    void write(const std::string &key, const char* value, StringStyle stringStyle = PLAIN_STRING){
        write(key, std::string(value), stringStyle);
    }

    void write(const std::string &key, bool value);
    void write(const std::string &key, int value);
    void write(const std::string &key, double value);
    void writePath(const std::string &key, const std::string& value);

    typedef enum { READ_MODE, WRITE_MODE } AssignMode;

    void setAssignMode(AssignMode mode) { this->mode = mode; }

    template <class T>
        void assign(const std::string& key, T& io_value, const T& defaultValue){
        switch(mode){
        case READ_MODE:
            if(!read(key, io_value)){
                io_value = defaultValue;
            }
            break;
        case WRITE_MODE:
            write(key, io_value);
            break;
        }
    }
        
    iterator begin() { return values.begin(); }
    iterator end() { return values.end(); }
    const_iterator begin() const { return values.begin(); }
    const_iterator end() const { return values.end(); }

    void throwKeyNotFoundException(const std::string& key) const;

#ifdef CNOID_BACKWARD_COMPATIBILITY
    Listing* findSequence(const std::string& key) const { return findListing(key); }
    Listing* openSequence(const std::string& key) { return openListing(key); }
    Listing* openFlowStyleSequence(const std::string& key){ return openFlowStyleListing(key); }
    Listing* createSequence(const std::string& key){ return createListing(key); }
    Listing* createFlowStyleSequence(const std::string& key){ return createFlowStyleListing(key); }
#endif
        
private:

    Mapping(const Mapping&);
    Mapping& operator=(const Mapping&);

    Mapping* openMapping(const std::string& key, bool doOverwrite);
    Mapping* openFlowStyleMapping(const std::string& key, bool doOverwrite);
    Listing* openListing(const std::string& key, bool doOverwrite);
    Listing* openFlowStyleListing(const std::string& key, bool doOverwrite);

    inline void insertSub(const std::string& key, ValueNode* node);

    void writeSub(const std::string &key, const char* text, size_t length, StringStyle stringStyle);

    static bool compareIters(const Mapping::const_iterator& it1, const Mapping::const_iterator& it2);

    Container values;
    AssignMode mode;
    int indexCounter;
    const char* doubleFormat_;
    bool isFlowStyle_;
    StringStyle keyQuoteStyle;

    friend class Listing;
    friend class YAMLReaderImpl;
    friend class YAMLWriter;
};

typedef boost::intrusive_ptr<Mapping> MappingPtr;


/**
   @todo add 'openMapping' and 'openListing' methods
   @note The name "Sequence" should not be used for this class
   because it confilcts with the name defined in the boost's concept check library.
*/
class CNOID_EXPORT Listing : public ValueNode
{
    typedef std::vector<ValueNodePtr> Container;

public:

    Listing();
    Listing(int size);
    ~Listing();
        
    typedef Container::iterator iterator;
    typedef Container::const_iterator const_iterator;

    bool empty() const { return values.empty(); }
    int size() const { return values.size(); }
    void clear();
    void reserve(int size);

    void setFlowStyle(bool isFlowStyle = true) { isFlowStyle_ = isFlowStyle; }
    bool isFlowStyle() const { return isFlowStyle_; }

    void setDoubleFormat(const char* format);
    const char* doubleFormat() { return doubleFormat_; }

    ValueNode* front() const {
        return values.front().get();
    }

    ValueNode* back() const {
        return values.back().get();
    }

    ValueNode* at(int i) const {
        return values[i].get();
    }

    /**
       deprecated
    */
    ValueNode& get(int i) const {
        return *values[i];
    }

    void write(int i, int value);
    void write(int i, const std::string& value, StringStyle stringStyle = PLAIN_STRING);

    bool read(int i, bool &out_value) const;
    bool read(int i, int &out_value) const;
    bool read(int i, double &out_value) const;

    /**
       \todo This operator should return ValueNode*.
    */
    ValueNode& operator[](int i) const {
        return *values[i];
    }

    /// \todo implement the following funcion (ticket #35)
    //MappingPtr extractMapping(const std::string& key) const;

    Mapping* newMapping();

    void append(ValueNodePtr node) {
        values.push_back(node);
    }
        
    void append(int value);

    /**
       @param maxColumns LF is automatically inserted when the column pos is over maxColumsn
       @param numValues If numValues is not greater than maxColumns, the initial LF is skipped.
       This feature is disabled if numValues = 0.
    */
    void append(int value, int maxColumns, int numValues = 0) {
        insertLF(maxColumns, numValues);
        append(value);
    }

    void append(size_t value);

    /**
       @param maxColumns LF is automatically inserted when the column pos is over maxColumsn
       @param numValues If numValues is not greater than maxColumns, the initial LF is skipped.
       This feature is disabled if numValues = 0.
    */
    /*
      void append(size_t value, int maxColumns, int numValues = 0){
      insertLF(maxColumns, numValues);
      append(value);
      }
    */
        
    void append(double value);

    /**
       @param maxColumns LF is automatically inserted when the column pos is over maxColumsn
       @param numValues If numValues is not greater than maxColumns, the initial LF is skipped.
       This feature is disabled if numValues = 0.
    */
    void append(double value, int maxColumns, int numValues = 0) {
        insertLF(maxColumns, numValues);
        append(value);
    }

    void append(const std::string& value, StringStyle stringStyle = PLAIN_STRING);

    /**
       @param maxColumns LF is automatically inserted when the column pos is over maxColumsn
       @param numValues If numValues is not greater than maxColumns, the initial LF is skipped.
       This feature is disabled if numValues = 0.
    */
    void append(const std::string& value, int maxColumns, int numValues = 0, StringStyle stringStyle = PLAIN_STRING){
        insertLF(maxColumns, numValues);
        append(value, stringStyle);
    }

    void appendLF();
        
    iterator begin() { return values.begin(); }
    iterator end() { return values.end(); }
    const_iterator begin() const { return values.begin(); }
    const_iterator end() const { return values.end(); };

private:

    Listing(int line, int column);
    Listing(int line, int column, int reservedSize);
        
    Listing(const Listing&);
    Listing& operator=(const Listing&);

    void insertLF(int maxColumns, int numValues);
        
    Container values;
    const char* doubleFormat_;
    bool isFlowStyle_;

    friend class Mapping;
    friend class YAMLReaderImpl;
    friend class YAMLWriter;
};

typedef boost::intrusive_ptr<Listing> ListingPtr;

inline void intrusive_ptr_add_ref(cnoid::ValueNode* obj){
    obj->refCounter++;
}

inline void intrusive_ptr_release(cnoid::ValueNode* obj){
    obj->refCounter--;
    if(obj->refCounter == 0){
        delete obj;
    }
}

#ifdef CNOID_BACKWARD_COMPATIBILITY
typedef ValueNode YamlNode;
typedef ValueNodePtr YamlNodePtr;
typedef ScalarNode YamlScalar;
typedef Mapping YamlMapping;
typedef MappingPtr YamlMappingPtr;
typedef Listing YamlSequence;
typedef ListingPtr YamlSequencePtr;
typedef Listing Sequence;
typedef ListingPtr SequencePtr;
#endif
}

#endif
