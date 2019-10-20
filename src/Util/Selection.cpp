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
    : domainname_(domainname)
{
    symbols_.reserve(size);
    selectedIndex_ = -1;
}


Selection::Selection(const Selection& org)
    : symbols_(org.symbols_),
      selectedIndex_(org.selectedIndex_),
      domainname_(org.domainname_)
{

}


void Selection::resize(int s)
{
    symbols_.resize(s);
    if(selectedIndex_ >= s){
        if(s >= 0){
            selectedIndex_ = 0;
        } else {
            selectedIndex_ = -1;
        }
    }
}


void Selection::clear()
{
    symbols_.clear();
    selectedIndex_ = -1;
}


void Selection::setSymbol(int index, const std::string& symbol)
{
    if(index >= static_cast<int>(symbols_.size())){
        symbols_.resize(index + 1);
    }
    symbols_[index] = symbol;

    if(selectedIndex_ < 0 && index == 0){
        selectedIndex_ = 0;
    }
}


Selection& Selection::operator<<(const std::string& symbol)
{
    if(symbols_.empty()){
        selectedIndex_ = 0;
    }
    symbols_.push_back(symbol);
    return *this;
}


int Selection::index(const std::string& symbol) const
{
    for(size_t i=0; i < symbols_.size(); ++i){
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
    if(index >= 0 && index < static_cast<int>(symbols_.size())){
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
    for(size_t i=0; i < symbols_.size(); ++i){
        if(symbols_[i] == symbol){
            selectedIndex_ = i;
            return true;
        }
    }
    return false;
}


const char* Selection::selectedSymbol() const
{
    if(selectedIndex_ >= 0){
        return symbols_[selectedIndex_].c_str();
    }
    return 0;
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
