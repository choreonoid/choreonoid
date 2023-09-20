#include "ItemPath.h"
#include <cnoid/Tokenizer>

using namespace std;
using namespace cnoid;

ItemPath::ItemPath(const std::string& pathstr)
{
    if(pathstr.empty()){
        path.push_back("");
        isAbsolute_ = false;
        return;
    }
    
    Tokenizer<EscapedListSeparator<char>> tokens(pathstr, EscapedListSeparator<char>("\\", "/", ""));
    auto iter = tokens.begin();
    if(iter != tokens.end() && iter->empty()){
        ++iter;
        isAbsolute_ = true;
    } else {
        isAbsolute_ = false;
    }
    while(iter != tokens.end()){
        path.push_back(*iter);
        ++iter;
    }
}
