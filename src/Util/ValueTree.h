/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_VALUE_TREE_H
#define CNOID_UTIL_VALUE_TREE_H

#include "Referenced.h"
#include <map>
#include <vector>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class YAMLReaderImpl;
class ValueNode;
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

class CNOID_EXPORT ValueNode : public Referenced
{
    struct Initializer {
        Initializer();
    };
    static Initializer initializer;
        
public:
    virtual ValueNode* clone() const;

    enum TypeBit { INVALID_NODE = 0, SCALAR = 1, MAPPING = 2, LISTING = 4, INSERT_LF = 8, APPEND_LF = 16, ANGLE_DEGREE = 32 };

    bool isValid() const { return typeBits; }
    TypeBit LFType() const { return (TypeBit)(typeBits & (INSERT_LF | APPEND_LF)); }
    TypeBit nodeType() const { return (TypeBit)(typeBits & 7); }

    int toInt() const;
    double toDouble() const;
    double toAngle() const;
    bool toBool() const;

    bool isScalar() const { return typeBits & SCALAR; }
    bool isString() const { return typeBits & SCALAR; }

#ifdef _WIN32
    const std::string toString() const;

    operator const std::string () const {
        return toString();
    }
#else
    const std::string& toString() const;

    operator const std::string& () const {
        return toString();
    }
#endif

    bool isDegreeMode() const { return typeBits & ANGLE_DEGREE; }
    void setDegreeMode() { typeBits |= ANGLE_DEGREE; }

    //template<typename T> T to() const { return ""; }
    template<typename T> T to() const;

    bool isMapping() const { return typeBits & MAPPING; }
    const Mapping* toMapping() const;
    Mapping* toMapping();

    bool isListing() const { return typeBits & LISTING; }
    const Listing* toListing() const;
    Listing* toListing();
        
#ifdef CNOID_BACKWARD_COMPATIBILITY
    bool isSequence() const { return typeBits & LISTING; }
    const Listing* toSequence() const { return toListing(); }
    Listing* toSequence() { return toListing(); }
#endif

    bool read(int &out_value) const;
    bool read(double &out_value) const;
    bool read(float &out_value) const;
    bool read(bool &out_value) const;
    bool read(std::string &out_value) const;

    bool hasLineInfo() const { return (line_ >= 0); }
    int line() const { return line_ + 1; }
    int column() const { return column_ + 1; }

    void throwException(const std::string& message) const;

    /**
       \todo integrate the exception classes with the common ones defined in Exception.h
    */
    class CNOID_EXPORT Exception {
public:
        Exception();
        virtual ~Exception();
        int line() const { return line_; }
        int column() const { return column_; }
        std::string message() const;
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

    class UnknownNodeTypeException : public Exception {
    };

    // This function is only used by YAMLWriter
    int indexInMapping() const { return indexInMapping_; }

protected:

    ValueNode() { }
    ValueNode(TypeBit type) : typeBits(type), line_(-1), column_(-1) { }

    virtual ~ValueNode() { }

    void throwNotScalrException() const;
    void throwNotMappingException() const;
    void throwNotListingException() const;

    int typeBits;

private:
    ValueNode(const ValueNode& org);
    ValueNode& operator=(const ValueNode&);

    int line_;
    int column_;

    //! \todo Move this information to a value of the map defined in Mapping
    int indexInMapping_;

    friend class YAMLReaderImpl;
    friend class ScalarNode;
    friend class Mapping;
    friend class Listing;
};

template<> inline double ValueNode::to<double>() const { return toDouble(); }
template<> inline float ValueNode::to<float>() const { return static_cast<float>(toDouble()); }
template<> inline int ValueNode::to<int>() const { return toInt(); }
template<> inline std::string ValueNode::to<std::string>() const { return toString(); }
    
typedef ref_ptr<ValueNode> ValueNodePtr;

    
class CNOID_EXPORT ScalarNode : public ValueNode
{
public:
    ScalarNode(const std::string& value, StringStyle stringStyle = PLAIN_STRING);
    ScalarNode(int value);
    
    virtual ValueNode* clone() const;

    const std::string& stringValue() const { return stringValue_; }
    StringStyle stringStyle() const { return stringStyle_; }
    
private:
    ScalarNode(const char* text, size_t length);
    ScalarNode(const char* text, size_t length, StringStyle stringStyle);
    ScalarNode(const ScalarNode& org);

    std::string stringValue_;
    StringStyle stringStyle_;

    friend class YAMLReaderImpl;
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

    virtual ValueNode* clone() const;
    virtual Mapping* cloneMapping() const;
    
    bool empty() const { return values.empty(); }
    int size() const { return static_cast<int>(values.size()); }
    void clear();

    void setFlowStyle(bool isFlowStyle = true) { isFlowStyle_ = isFlowStyle; }
    bool isFlowStyle() const { return isFlowStyle_; }

    void setDoubleFormat(const char* format);
    const char* doubleFormat() { return doubleFormat_; }
        
    void setKeyQuoteStyle(StringStyle style);

    ValueNode* find(const std::string& key) const;
    Mapping* findMapping(const std::string& key) const;
    Listing* findListing(const std::string& key) const;

    ValueNodePtr extract(const std::string& key);

    bool extract(const std::string& key, double& out_value);
    bool extract(const std::string& key, std::string& out_value);

    ValueNode& get(const std::string& key) const;

    ValueNode& operator[](const std::string& key) const {
        return get(key);
    }

    void insert(const std::string& key, ValueNode* node);

    void insert(const Mapping* other);

    Mapping* openMapping(const std::string& key) {
        return openMapping_(key, false);
    }
        
    Mapping* openFlowStyleMapping(const std::string& key) {
        return openFlowStyleMapping_(key, false);
    }

    Mapping* createMapping(const std::string& key) {
        return openMapping_(key, true);
    }
        
    Mapping* createFlowStyleMapping(const std::string& key) {
        return openFlowStyleMapping_(key, true);
    }

    Listing* openListing(const std::string& key) {
        return openListing_(key, false);
    }
        
    Listing* openFlowStyleListing(const std::string& key){
        return openFlowStyleListing_(key, false);
    }

    Listing* createListing(const std::string& key){
        return openListing_(key, true);
    }
        
    Listing* createFlowStyleListing(const std::string& key){
        return openFlowStyleListing_(key, true);
    }

    bool remove(const std::string& key);

    bool read(const std::string& key, std::string& out_value) const;
    bool read(const std::string& key, bool& out_value) const;
    bool read(const std::string& key, int& out_value) const;
    bool read(const std::string& key, double& out_value) const;
    bool read(const std::string& key, float& out_value) const;

    template <class T> T get(const std::string& key) const {
        T value;
        if(read(key, value)){
            return value;
        } else {
            throwKeyNotFoundException(key);
            return value;
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

    void write(const std::string &key, const std::string& value, StringStyle stringStyle = PLAIN_STRING);
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

    StringStyle keyStringStyle() const { return keyStringStyle_; }

    //! \deprecated
    template <class T> T read(const std::string& key) const { return get<T>(key); }

#ifdef CNOID_BACKWARD_COMPATIBILITY
    Listing* findSequence(const std::string& key) const { return findListing(key); }
    Listing* openSequence(const std::string& key) { return openListing(key); }
    Listing* openFlowStyleSequence(const std::string& key){ return openFlowStyleListing(key); }
    Listing* createSequence(const std::string& key){ return createListing(key); }
    Listing* createFlowStyleSequence(const std::string& key){ return createFlowStyleListing(key); }
#endif
        
private:

    Mapping(const Mapping& org);
    Mapping& operator=(const Mapping&);

    Mapping* openMapping_(const std::string& key, bool doOverwrite);
    Mapping* openFlowStyleMapping_(const std::string& key, bool doOverwrite);
    Listing* openListing_(const std::string& key, bool doOverwrite);
    Listing* openFlowStyleListing_(const std::string& key, bool doOverwrite);

    inline void insertSub(const std::string& key, ValueNode* node);

    void writeSub(const std::string &key, const char* text, size_t length, StringStyle stringStyle);

    Container values;
    AssignMode mode;
    int indexCounter;
    const char* doubleFormat_;
    bool isFlowStyle_;
    StringStyle keyStringStyle_;

    friend class Listing;
    friend class YAMLReaderImpl;
};

typedef ref_ptr<Mapping> MappingPtr;


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
        
    virtual ValueNode* clone() const;

    typedef Container::iterator iterator;
    typedef Container::const_iterator const_iterator;

    bool empty() const { return values.empty(); }
    int size() const { return static_cast<int>(values.size()); }
    void clear();
    void reserve(int size);

    void setFlowStyle(bool isFlowStyle = true) { isFlowStyle_ = isFlowStyle; }
    bool isFlowStyle() const { return isFlowStyle_; }

    void setDoubleFormat(const char* format);
    const char* doubleFormat() { return doubleFormat_; }

    ValueNode* front() const {
        return values.front();
    }

    ValueNode* back() const {
        return values.back();
    }

    ValueNode* at(int i) const {
        return values[i];
    }

    /**
       deprecated
    */
    ValueNode& get(int i) const {
        return *values[i];
    }

    void write(int i, int value);
    void write(int i, const std::string& value, StringStyle stringStyle = PLAIN_STRING);

    /**
       \todo This operator should return ValueNode*.
    */
    ValueNode& operator[](int i) const {
        return *values[i];
    }

    /// \todo implement the following funcion (ticket #35)
    //MappingPtr extractMapping(const std::string& key) const;

    Mapping* newMapping();

    void append(ValueNode* node) {
        values.push_back(node);
    }

    void insert(int index, ValueNode* node);
        
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
        
    Listing(const Listing& org);
    Listing& operator=(const Listing&);

    void insertLF(int maxColumns, int numValues);
        
    Container values;
    const char* doubleFormat_;
    bool isFlowStyle_;
    bool doInsertLFBeforeNextElement;

    friend class Mapping;
    friend class YAMLReaderImpl;
};

typedef ref_ptr<Listing> ListingPtr;

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
