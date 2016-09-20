/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_ITEM_PATH_H
#define CNOID_BASE_ITEM_PATH_H

#include <string>
#include <boost/tokenizer.hpp>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ItemPath
{
    typedef boost::escaped_list_separator<char> Separator;
    typedef boost::tokenizer<Separator> Tokenizer;
        
public:
    typedef Tokenizer::iterator iterator;
    typedef iterator Iterator;
        
    ItemPath(const std::string& path);
        
    inline bool isAbsolute() { return isAbsolute_; }
    inline bool isRelative() { return !isAbsolute_; }
        
    inline iterator begin() { return pathBegin; }
    inline iterator end() { return pathEnd; }
        
    std::string folder();
    inline std::string leaf() { return *pathLeaf; }
        
private:
    std::string path;
        
    Tokenizer::iterator pathBegin;
    Tokenizer::iterator pathLeaf;
    Tokenizer::iterator pathEnd;
        
    bool isAbsolute_;
};

}

#endif
