/** 
    \file
    \brief Implementations of the LinkTraverse class
    \author Shin'ichiro Nakaoka
*/
  
#include "LinkTraverse.h"
#include "Link.h"

using namespace std;
using namespace cnoid;


LinkTraverse::LinkTraverse()
{

}


LinkTraverse::LinkTraverse(int size)
    : links_(size)
{
    links_.clear();
}


LinkTraverse::LinkTraverse(Link* root, bool doUpward, bool doDownward)
{
    find(root, doUpward, doDownward);
}


LinkTraverse::LinkTraverse(const LinkTraverse& org)
    : links_(org.links_),
      numUpwardConnections(org.numUpwardConnections)
{

}


LinkTraverse::~LinkTraverse()
{

}


void LinkTraverse::clear()
{
    links_.clear();
    numUpwardConnections = 0;
}

void LinkTraverse::find(Link* root, bool doUpward, bool doDownward)
{
    numUpwardConnections = 0;
    links_.clear();
    traverse(root, doUpward, doDownward, false, 0);
}


void LinkTraverse::traverse(Link* link, bool doUpward, bool doDownward, bool isUpward, Link* prev)
{
    links_.push_back(link);
    if(isUpward){
        ++numUpwardConnections;
    }
    
    if(doUpward && link->parent()){
        traverse(link->parent(), doUpward, true, true, link);
    }
    if(doDownward){
        for(Link* child = link->child(); child; child = child->sibling()){
            if(child != prev){
                traverse(child, false, true, false, 0);
            }
        }
    }
}


void LinkTraverse::append(Link* link, bool isDownward)
{
    links_.push_back(link);
    if(!isDownward){
        ++numUpwardConnections;
    }
}


void LinkTraverse::calcForwardKinematics(bool calcVelocity, bool calcAcceleration) const
{
    Vector3 arm;
    int i;
    for(i=1; i <= numUpwardConnections; ++i){

        Link* link = links_[i];
        const Link* child = links_[i-1];

        switch(child->jointType()){

        case Link::ROTATIONAL_JOINT:
            link->R().noalias() = child->R() * AngleAxisd(child->q(), child->a()).inverse();
            arm.noalias() = link->R() * child->b();
            link->p().noalias() = child->p() - arm;

            if(calcVelocity){
                const Vector3 sw(link->R() * child->a());
                link->w() = child->w() - child->dq() * sw;
                link->v() = child->v() - link->w().cross(arm);
                
                if(calcAcceleration){
                    link->dw() = child->dw() - child->dq() * child->w().cross(sw) - (child->ddq() * sw);
                    link->dv() = child->dv() - child->w().cross(child->w().cross(arm)) - child->dw().cross(arm);
                }
            }
            break;
            
        case Link::SLIDE_JOINT:
            link->R() = child->R();
            arm.noalias() = link->R() * (child->b() + child->q() * child->d());
            link->p().noalias() = child->p() - arm;

            if(calcVelocity){
                const Vector3 sv(link->R() * child->d());
                link->w() = child->w();
                link->v().noalias() = child->v() - child->dq() * sv;

                if(calcAcceleration){
                    link->dw() = child->dw();
                    link->dv().noalias() =
                        child->dv() - child->w().cross(child->w().cross(arm)) - child->dw().cross(arm)
                        - 2.0 * child->dq() * child->w().cross(sv) - child->ddq() * sv;
                }
            }
            break;
            
        case Link::FIXED_JOINT:
        default:
            arm.noalias() = link->R() * child->b();
            link->R() = child->R();
            link->p().noalias() = child->p() - arm;

            if(calcVelocity){
                
                link->w() = child->w();
                link->v() = child->v() - link->w().cross(arm);
				
                if(calcAcceleration){
                    link->dw() = child->dw();
                    link->dv() = child->dv() - child->w().cross(child->w().cross(arm)) - child->dw().cross(arm);;
                }
            }
            break;
        }
    }

    const int n = links_.size();
    for( ; i < n; ++i){
        
        Link* link = links_[i];
        const Link* parent = link->parent();

        switch(link->jointType()){
            
        case Link::ROTATIONAL_JOINT:
            link->R().noalias() = parent->R() * AngleAxisd(link->q(), link->a());
            arm.noalias() = parent->R() * link->b();
            link->p().noalias() = parent->p() + arm;

            if(calcVelocity){
                const Vector3 sw(parent->R() * link->a());
                link->w().noalias() = parent->w() + sw * link->dq();
                link->v().noalias() = parent->v() + parent->w().cross(arm);

                if(calcAcceleration){
                    link->dw().noalias() = parent->dw() + link->dq() * parent->w().cross(sw) + (link->ddq() * sw);
                    link->dv().noalias() = parent->dv() + parent->w().cross(parent->w().cross(arm)) + parent->dw().cross(arm);
                }
            }
            break;
            
        case Link::SLIDE_JOINT:
            link->R() = parent->R();
            arm.noalias() = parent->R() * (link->b() + link->q() * link->d());
            link->p() = parent->p() + arm;

            if(calcVelocity){
                const Vector3 sv(parent->R() * link->d());
                link->w() = parent->w();
                link->v().noalias() = parent->v() + sv * link->dq();

                if(calcAcceleration){
                    link->dw() = parent->dw();
                    link->dv().noalias() = parent->dv() + parent->w().cross(parent->w().cross(arm)) + parent->dw().cross(arm)
                        + 2.0 * link->dq() * parent->w().cross(sv) + link->ddq() * sv;
                }
            }
            break;

        case Link::FIXED_JOINT:
        default:
            arm.noalias() = parent->R() * link->b();
            link->R() = parent->R();
            link->p().noalias() = arm + parent->p();

            if(calcVelocity){
                link->w() = parent->w();
                link->v() = parent->v() + parent->w().cross(arm);;

                if(calcAcceleration){
                    link->dw() = parent->dw();
                    link->dv().noalias() = parent->dv() +
                        parent->w().cross(parent->w().cross(arm)) + parent->dw().cross(arm);
                }
            }
            break;
        }
    }
}
