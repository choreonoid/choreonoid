/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_ITEM_PATH_H
#define CNOID_BASE_ITEM_PATH_H

#include <string>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ItemPath
{
public:
    typedef std::vector<std::string>::iterator iterator;

    ItemPath(const std::string& path);
        
    bool isAbsolute() { return isAbsolute_; }
    bool isRelative() { return !isAbsolute_; }
        
    iterator begin() { return path.begin(); }
    iterator end() { return path.end(); }
        
private:
    std::vector<std::string> path;
    bool isAbsolute_;
};

}

#endif
