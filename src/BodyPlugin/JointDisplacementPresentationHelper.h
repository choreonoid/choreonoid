#ifndef CNOID_BODY_PLUGIN_JOINT_DISPLACEMENT_PRESENTATION_HELPER_H
#define CNOID_BODY_PLUGIN_JOINT_DISPLACEMENT_PRESENTATION_HELPER_H

#include <cnoid/JointDisplacementPresentationHandler>
#include <cnoid/Link>
#include <cnoid/Signal>
#include <cnoid/ConnectionSet>
#include "exportdecl.h"

namespace cnoid {

class BodyItem;

/**
   Thin wrapper around JointDisplacementPresentationHandler. Lets call sites
   convert between the internal q value and the presentation value uniformly,
   regardless of whether a handler is attached to the target body and whether
   that handler is responsible for the joint in question. When no relevant
   handler is present, the conversion functions are zero-overhead pass-throughs.

   The helper also watches the target BodyItem for handler-set changes (via
   BodyItem::sigModelUpdated with the HandlerSetUpdate flag) so that GUI clients
   can simply connect to sigHandlerSetChanged() to be notified when the set of
   active handlers changes, e.g. when a gun-axis joint is set up or torn down.
*/
class CNOID_EXPORT JointDisplacementPresentationHelper
{
public:
    JointDisplacementPresentationHelper();
    JointDisplacementPresentationHelper(BodyItem* bodyItem);
    JointDisplacementPresentationHelper(Body* body);
    ~JointDisplacementPresentationHelper();

    JointDisplacementPresentationHelper(const JointDisplacementPresentationHelper&) = delete;
    JointDisplacementPresentationHelper& operator=(const JointDisplacementPresentationHelper&) = delete;

    /*
       Bind to a BodyItem. The helper subscribes to the item's sigModelUpdated
       so that sigHandlerSetChanged() can notify long-lived owners (such as a
       view that keeps the helper as a member) when the handler set changes.
    */
    void setBodyItem(BodyItem* bodyItem);
    BodyItem* bodyItem() const { return bodyItem_; }

    /*
       Bind to a Body directly, without subscribing to any signal. Intended for
       short-lived helpers created inside a function for a one-shot query, where
       subscribing to a signal would be wasted work and could outlive the helper.
    */
    void setBody(Body* body);

    SignalProxy<void()> sigHandlerSetChanged() { return sigHandlerSetChanged_; }

    double toPresentationValue(const Link* joint, double q) const {
        return (handler_ && handler_->handles(joint))
            ? handler_->toPresentationValue(joint, q) : q;
    }

    double toPresentationValue(const Link* joint) const {
        return toPresentationValue(joint, joint->q());
    }

    double fromPresentationValue(const Link* joint, double value) const {
        return (handler_ && handler_->handles(joint))
            ? handler_->fromPresentationValue(joint, value) : value;
    }

    JointDisplacementPresentationHandler::PresentationType
    getPresentationType(const Link* joint) const {
        if(handler_ && handler_->handles(joint)){
            return handler_->getPresentationType(joint);
        }
        if(joint->isRevoluteJoint()){
            return JointDisplacementPresentationHandler::Angle;
        }
        if(joint->isPrismaticJoint()){
            return JointDisplacementPresentationHandler::Length;
        }
        return JointDisplacementPresentationHandler::Fixed;
    }

    void getPresentationRange(const Link* joint, double& lower, double& upper) const {
        if(handler_ && handler_->handles(joint)){
            handler_->getPresentationRange(joint, lower, upper);
        } else {
            lower = joint->q_lower();
            upper = joint->q_upper();
        }
    }

private:
    BodyItem* bodyItem_;
    Body* body_;
    JointDisplacementPresentationHandler* handler_;
    Signal<void()> sigHandlerSetChanged_;
    ScopedConnection modelUpdateConnection_;

    void refreshHandler();
};

}

#endif
