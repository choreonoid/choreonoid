/**
   @author Shin'ichiro Nakaoka
*/

#include "ItemPath.h"
#include "Item.h"
#include "RootItem.h"

using namespace std;
using namespace cnoid;


ItemPath::ItemPath(const std::string& path) :
    path(path)
{
    Tokenizer pathElements(path, Separator("\\", "/", ""));

    pathBegin = pathElements.begin();
    pathEnd = pathElements.end();

    if(pathBegin != pathEnd && (*pathBegin).empty()){
        pathBegin++;
        isAbsolute_ = true;
    } else {
        isAbsolute_ = false;
    }
  
    pathLeaf = pathBegin;

    for(iterator it = pathBegin; it != pathEnd; ++it){
        pathLeaf = it;
    }
}


std::string ItemPath::folder()
{
    std::string path;
    iterator it = pathBegin;
    while(true){
        path.append(*it++);
        if(it == pathLeaf){
            break;
        }
        path.append("/");
    }
    return path;
}
