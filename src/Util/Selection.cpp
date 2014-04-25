/**
   @author Shin'ichiro Nakaoka
*/

#include "Selection.h"
#include "gettext.h"

using namespace std;
using namespace cnoid;

const char* Selection::label(int index) const
{
    return domainname_ ? dgettext(domainname_, symbols_[index].c_str()) : symbols_[index].c_str();
}


const char* Selection::selectedLabel() const
{
    return domainname_ ? dgettext(domainname_, symbols_[selectedIndex_].c_str()) : symbols_[selectedIndex_].c_str();
}


bool Selection::select(int index)
{
    if(index >= 0 && index < symbols_.size()){
        selectedIndex_ = index;
        return true;
    }
    return false;
}


bool Selection::select(const std::string& symbol)
{
    for(int i=0; i < symbols_.size(); ++i){
        if(symbols_[i] == symbol){
            selectedIndex_ = i;
            return true;
        }
    }
    return false;
}


int Selection::index(const std::string& symbol) const
{
    for(int i=0; i < symbols_.size(); ++i){
        if(symbols_[i] == symbol){
            return i;
        }
    }
    return -1;
}

    
