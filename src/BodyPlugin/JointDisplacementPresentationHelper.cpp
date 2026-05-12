#include "JointDisplacementPresentationHelper.h"
#include "BodyItem.h"
#include <cnoid/Body>

using namespace cnoid;


JointDisplacementPresentationHelper::JointDisplacementPresentationHelper()
    : bodyItem_(nullptr), body_(nullptr), handler_(nullptr)
{

}


JointDisplacementPresentationHelper::JointDisplacementPresentationHelper(BodyItem* bodyItem)
    : bodyItem_(nullptr), body_(nullptr), handler_(nullptr)
{
    setBodyItem(bodyItem);
}


JointDisplacementPresentationHelper::JointDisplacementPresentationHelper(Body* body)
    : bodyItem_(nullptr), body_(nullptr), handler_(nullptr)
{
    setBody(body);
}


JointDisplacementPresentationHelper::~JointDisplacementPresentationHelper()
{

}


void JointDisplacementPresentationHelper::setBodyItem(BodyItem* bodyItem)
{
    if(bodyItem == bodyItem_){
        return;
    }
    bodyItem_ = bodyItem;
    body_ = bodyItem ? bodyItem->body() : nullptr;
    modelUpdateConnection_.disconnect();
    if(bodyItem_){
        modelUpdateConnection_ =
            bodyItem_->sigModelUpdated().connect(
                [this](int flags){
                    if(flags & BodyItem::HandlerSetUpdate){
                        refreshHandler();
                        sigHandlerSetChanged_();
                    }
                });
    }
    refreshHandler();
}


void JointDisplacementPresentationHelper::setBody(Body* body)
{
    modelUpdateConnection_.disconnect();
    bodyItem_ = nullptr;
    body_ = body;
    refreshHandler();
}


void JointDisplacementPresentationHelper::refreshHandler()
{
    handler_ = body_
        ? body_->findHandler<JointDisplacementPresentationHandler>()
        : nullptr;
}
