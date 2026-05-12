#ifndef CNOID_BODY_JOINT_DISPLACEMENT_PRESENTATION_HANDLER_H
#define CNOID_BODY_JOINT_DISPLACEMENT_PRESENTATION_HANDLER_H

#include "BodyHandler.h"
#include "exportdecl.h"

namespace cnoid {

class Link;

/**
   This handler converts a joint's internal displacement value (e.g., a revolute joint's
   angle in radians) to a different presentation value (e.g., the linear distance between
   two specific points on the joint mechanism) for display and editing in GUIs.

   The handler decouples the internal kinematic representation from the user-facing
   representation, allowing domain-specific joints (e.g., a welding gun's stroke axis)
   to appear as a more intuitive quantity to the user without modifying the kinematic core.
*/
class CNOID_EXPORT JointDisplacementPresentationHandler : public virtual BodyHandler
{
public:
    enum PresentationType { Angle, Length, Fixed };

    /**
       Returns true if this handler is responsible for converting the given joint's
       displacement to its presentation value.
    */
    virtual bool handles(const Link* joint) const = 0;

    /**
       Converts the joint's internal q value to a presentation value.
       Called only for joints for which handles() returns true.
    */
    virtual double toPresentationValue(const Link* joint, double q) const = 0;

    /**
       Converts a user-input presentation value back to the joint's internal q value.
    */
    virtual double fromPresentationValue(const Link* joint, double value) const = 0;

    /**
       Returns the category of the presentation value.
       Angle: angle-based, internal unit is radian
       Length: length-based, internal unit is meter
       Fixed: the joint has no presentable displacement
       Implementations may return Fixed for joints they do not handle to give a
       meaningful answer when called for non-target joints.
    */
    virtual PresentationType getPresentationType(const Link* joint) const = 0;

    /**
       Returns the lower and upper bounds of the presentation value.
       These should be expressed in the internal unit of the corresponding
       PresentationType (radian for Angle, meter for Length).
    */
    virtual void getPresentationRange(
        const Link* joint, double& lower, double& upper) const = 0;
};

typedef ref_ptr<JointDisplacementPresentationHandler> JointDisplacementPresentationHandlerPtr;

}

#endif
