/**
   @author Shin'ichiro Nakaoka
*/

#include "ItemPath.h"

using namespace std;
using namespace cnoid;


ItemPath::ItemPath(const std::string& pathstr)
{
    isAbsolute_ = false;

    if(pathstr.empty()){
        return;
    }
    
    auto size = pathstr.size();
    string::size_type pos = 0;
    if(pathstr[pos] == '/'){
        isAbsolute_ = true;
        ++pos;
    }
    while(pos < size){
        auto found = pathstr.find('/', pos);
        if(found != string::npos){
            path.emplace_back(pathstr, pos, found - pos);
            pos = found + 1;
        } else {
            path.emplace_back(pathstr, pos, size - pos);
            pos = size;
        }
    }
}
