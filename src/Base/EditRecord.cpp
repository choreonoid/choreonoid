#include "EditRecord.h"

using namespace cnoid;


EditRecord::EditRecord(bool isManualOperation)
    : isManualOperation_(isManualOperation),
      isReverse_(false)
{

}


EditRecord::EditRecord(const EditRecord& org)
    : isManualOperation_(org.isManualOperation_),
      isReverse_(org.isReverse_)
{

}


EditRecord* EditRecord::getFlipped() const
{
    auto flipped = clone();
    flipped->flip();
    return flipped;
}


EditRecordGroup::EditRecordGroup(const std::string& label, bool isManualOperation)
    : EditRecord(isManualOperation),
      label_(label)
{

}


EditRecordGroup::EditRecordGroup(const EditRecordGroup& org)
    : EditRecord(org),
      label_(org.label_)
{
    group_.reserve(org.group_.size());
    for(auto& record : org.group_){
        group_.push_back(record->clone());
    }
}


EditRecord* EditRecordGroup::clone() const
{
    return new EditRecordGroup(*this);
}


std::string EditRecordGroup::label() const
{
    return label_;
}


bool EditRecordGroup::undo()
{
    bool done = false;
    if(!group_.empty()){
        done = true;
        for(size_t i = group_.size() - 1; i >= 0; --i){
            if(!group_[i]->applyUndo()){
                for(size_t j = i + 1; j < group_.size(); ++j){
                    group_[j]->applyRedo();
                }
                done = false;
                break;
            }
        }
    }
    return done;
}


bool EditRecordGroup::redo()
{
    bool done = false;
    if(!group_.empty()){
        done = true;
        for(size_t i = 0; i < group_.size(); ++i){
            if(!group_[i]->applyRedo()){
                for(size_t j = i - 1; j >= 0; --j){
                    group_[j]->applyUndo();
                }
                done = false;
                break;
            }
        }
    }
    return done;
}
