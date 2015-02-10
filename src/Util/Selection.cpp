/**
   @author Shin'ichiro Nakaoka
*/

#include "Selection.h"
#include "gettext.h"

using namespace std;
using namespace cnoid;


Selection::Selection(const char* domainname)
        : domainname_(domainname)
{
    selectedIndex_ = -1;
}


Selection::Selection(size_t size, const char* domainname)
    : symbols_(size),
      domainname_(domainname)
{
    selectedIndex_ = -1;
}


void Selection::resize(int s)
{
    symbols_.resize(s);
    if(selectedIndex_ >= s){
        selectedIndex_ = s - 1;
    }
}


void Selection::clear()
{
    symbols_.clear();
    selectedIndex_ = -1;
}


void Selection::setSymbol(int index, const std::string& symbol)
{
    symbols_[index] = symbol;
}



Selection& Selection::operator<<(const std::string& symbol)
{
    symbols_.push_back(symbol);
    return *this;
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

    
const char* Selection::label(int index) const
{
    return domainname_ ? dgettext(domainname_, symbols_[index].c_str()) : symbols_[index].c_str();
}


bool Selection::select(int index)
{
    if(index >= 0 && index < symbols_.size()){
        selectedIndex_ = index;
        return true;
    }
    return false;
}    


bool Selection::selectIndex(int index)
{
    return select(index);
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


const char* Selection::selectedLabel() const
{
    if(selectedIndex_ >= 0){
        if(domainname_){
            return dgettext(domainname_, symbols_[selectedIndex_].c_str());
        } else {
            return symbols_[selectedIndex_].c_str();
        }
    }
    return 0;
}
