/**
   \file
   \brief Implementations of the LinkPath class
   \author Shin'ichiro Nakaoka
*/
  
#include "LinkPath.h"
#include "Link.h"
#include <algorithm>

using namespace std;
using namespace cnoid;


LinkPath::LinkPath()
{

}


LinkPath::LinkPath(Link* base, Link* end)
{
    setPath(base, end);
}


/// path from the root link
LinkPath::LinkPath(Link* base)
{
    setPath(base);
}


/// This method is disabled.
void LinkPath::find(Link* /* root */, bool /* doUpward */, bool /* doDownward */)
{
    throw "The find method for LinkTraverse cannot be used in LinkPath";
}


bool LinkPath::setPath(Link* base, Link* end)
{
    links_.clear();
    numUpwardConnections = 0;
    bool found = findPathSub(base, nullptr, end, false);
    if(!found){
        links_.clear();
    }
    return found;
}


bool LinkPath::findPathSub(Link* link, Link* prev, Link* end, bool isUpward)
{
    links_.push_back(link);
    if(isUpward){
        ++numUpwardConnections;
    }
    
    if(link == end){
        return true;
    }

    for(Link* child = link->child(); child; child = child->sibling()){
        if(child != prev){
            if(findPathSub(child, link, end, false)){
                return true;
            }
        }
    }

    Link* parent = link->parent();
    if(parent && parent != prev){
        if(findPathSub(parent, link, end, true)){
            return true;
        }
    }

    links_.pop_back();
    if(isUpward){
        --numUpwardConnections;
    }

    return false;
}


/// path from the root link
void LinkPath::setPath(Link* end)
{
    links_.clear();
    numUpwardConnections = 0;
    findPathFromRootSub(end);
    std::reverse(links_.begin(), links_.end());
}


void LinkPath::findPathFromRootSub(Link* link)
{
    links_.push_back(link);
    if(link->parent()){
        findPathFromRootSub(link->parent());
    }
}
