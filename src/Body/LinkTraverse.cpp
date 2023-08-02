#include "LinkTraverse.h"
#include <cnoid/CloneMap>
#include <cnoid/EigenUtil>

using namespace std;
using namespace cnoid;


LinkTraverse::LinkTraverse()
{
    numUpwardConnections = 0;
    hasRootLowerLinks_ = false;
    hasRootUpperLinks_ = false;
}


LinkTraverse::LinkTraverse(int size)
{
    links_.reserve(size);
    numUpwardConnections = 0;
    hasRootLowerLinks_ = false;
    hasRootUpperLinks_ = false;
}


LinkTraverse::LinkTraverse(Link* root, bool toUpper, bool toLower)
{
    find(root, toUpper, toLower);
}


LinkTraverse::LinkTraverse(const LinkTraverse& org, CloneMap* cloneMap)
{
    if(!cloneMap){
        links_ = org.links_;
    } else {
        links_.reserve(org.links_.size());
        for(auto& link : org.links_){
            links_.push_back(cloneMap->getClone<Link>(link));
        }
    }
        
    numUpwardConnections = org.numUpwardConnections;
    hasRootLowerLinks_ = org.hasRootLowerLinks_;
    hasRootUpperLinks_ = org.hasRootUpperLinks_;
}


LinkTraverse::~LinkTraverse()
{

}


void LinkTraverse::clear()
{
    links_.clear();
    numUpwardConnections = 0;
    hasRootLowerLinks_ = false;
    hasRootUpperLinks_ = false;
}

void LinkTraverse::find(Link* root, bool toUpper, bool toLower)
{
    links_.clear();
    numUpwardConnections = 0;
    hasRootLowerLinks_ = false;
    
    traverse(root, toUpper, toLower, false, nullptr);

    hasRootLowerLinks_ = toLower && root->child();
    hasRootUpperLinks_ = numUpwardConnections > 0;
}


void LinkTraverse::traverse(Link* link, bool toUpper, bool toLower, bool isReverseLinkChain, Link* prev)
{
    links_.push_back(link);
    if(isReverseLinkChain){
        ++numUpwardConnections;
    }
    
    if(toUpper){
        if(auto parent = link->parent()){
            if(parent->body() == link->body()){
                traverse(parent, true, true, true, link);
            }
        }
    }
    if(toLower){
        for(Link* child = link->child(); child; child = child->sibling()){
            if(child != prev){
                traverse(child, false, true, false, 0);
            }
        }
    }
}


void LinkTraverse::append(Link* link, bool isLowerLink)
{
    links_.push_back(link);
    if(!isLowerLink){
        ++numUpwardConnections;
    }
}


bool LinkTraverse::remove(Link* link)
{
    int index = -1;
    bool hasSingleAdjacentLink = false;
    for(size_t i=0; i < links_.size(); ++i){
        auto element = links_[i];
        if(element == link){
            index = i;
        } else if(element->parent() == link || link->parent() == element){
            if(!hasSingleAdjacentLink){
                hasSingleAdjacentLink = true;
            } else {
                hasSingleAdjacentLink = false; // Two adjacent links!
                break;
            }
        }
    }
    if(index < 0 || !hasSingleAdjacentLink){
        return false;
    }

    if(numUpwardConnections > 0){
        auto root = rootLink();
        if(link == root || checkIfUpperLink(root, link)){
            --numUpwardConnections;
            if(numUpwardConnections == 0){
                hasRootUpperLinks_ = false;
            }
        }
    }

    links_.erase(links_.begin() + index);

    hasRootLowerLinks_ = (links_.size() - numUpwardConnections) >= 2;

    return true;
}


bool LinkTraverse::checkIfUpperLink(Link* link, Link* upperLink) const
{
    auto parent = link->parent();
    while(parent){
        if(parent == upperLink){
            return true;
        }
        parent = parent->parent();
    }
    return false;
}


void LinkTraverse::calcForwardKinematics(bool calcVelocity, bool calcAcceleration)
{
    Vector3 arm;
    int i;
    for(i=1; i <= numUpwardConnections; ++i){

        Link* link = links_[i];
        const Link* child = links_[i-1];

        switch(child->jointType()){

        case Link::RevoluteJoint:
            link->R().noalias() = child->R() * AngleAxisd(child->q(), child->a()).inverse() * child->Rb().transpose();
            arm.noalias() = link->R() * child->b();
            link->p().noalias() = child->p() - arm;

            if(calcVelocity){
                const Vector3 sw(link->R() * (child->Rb() * child->a()));
                link->w().noalias() = child->w() - child->dq() * sw;
                link->v().noalias() = child->v() - link->w().cross(arm);
                
                if(calcAcceleration){
                    link->dw().noalias() = child->dw() - child->dq() * child->w().cross(sw) - (child->ddq() * sw);
                    link->dv().noalias() = child->dv() - child->w().cross(child->w().cross(arm)) - child->dw().cross(arm);
                }
            }
            break;
            
        case Link::PrismaticJoint:
            link->R().noalias() = child->R() * child->Rb().transpose();
            arm.noalias() = link->R() * (child->b() + child->Rb() * (child->q() * child->d()));
            link->p().noalias() = child->p() - arm;

            if(calcVelocity){
                const Vector3 sv(link->R() * (child->Rb() * child->d()));
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
            
        case Link::FixedJoint:
        default:
            link->R().noalias() = child->R() * child->Rb().transpose();
            arm.noalias() = link->R() * child->b();
            link->p().noalias() = child->p() - arm;

            if(calcVelocity){
                link->w() = child->w();
                link->v().noalias() = child->v() - link->w().cross(arm);
				
                if(calcAcceleration){
                    link->dw() = child->dw();
                    link->dv().noalias() = child->dv() - child->w().cross(child->w().cross(arm)) - child->dw().cross(arm);
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
            
        case Link::RevoluteJoint:
            link->R().noalias() = parent->R() * link->Rb() * AngleAxisd(link->q(), link->a());
            arm.noalias() = parent->R() * link->b();
            link->p().noalias() = parent->p() + arm;

            if(calcVelocity){
                const Vector3 sw(parent->R() * (link->Rb() * link->a()));
                link->w().noalias() = parent->w() + sw * link->dq();
                link->v().noalias() = parent->v() + parent->w().cross(arm);

                if(calcAcceleration){
                    link->dw().noalias() = parent->dw() + link->dq() * parent->w().cross(sw) + (link->ddq() * sw);
                    link->dv().noalias() = parent->dv() + parent->w().cross(parent->w().cross(arm)) + parent->dw().cross(arm);
                }
            }
            break;
            
        case Link::PrismaticJoint:
            link->R().noalias() = parent->R() * link->Rb();
            arm.noalias() = parent->R() * (link->b() + link->Rb() * (link->q() * link->d()));
            link->p().noalias() = parent->p() + arm;

            if(calcVelocity){
                const Vector3 sv(parent->R() * (link->Rb() * link->d()));
                link->w() = parent->w();
                link->v().noalias() = parent->v() + sv * link->dq();

                if(calcAcceleration){
                    link->dw() = parent->dw();
                    link->dv().noalias() = parent->dv() + parent->w().cross(parent->w().cross(arm)) + parent->dw().cross(arm)
                        + 2.0 * link->dq() * parent->w().cross(sv) + link->ddq() * sv;
                }
            }
            break;

        case Link::FixedJoint:
        default:
            link->R().noalias() = parent->R() * link->Rb();
            arm.noalias() = parent->R() * link->b();
            link->p().noalias() = parent->p() + arm;

            if(calcVelocity){
                link->w() = parent->w();
                link->v().noalias() = parent->v() + parent->w().cross(arm);

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


Vector6 LinkTraverse::calcInverseDynamics()
{
    if(links_.empty()){
        return Vector6::Zero();
    }
    Vector3 vo = Vector3::Zero();
    Vector3 dvo = Vector3::Zero();
    
    return calcInverseDynamicsSub(
        rootLink(), vo, dvo, hasRootUpperLinks_, hasRootLowerLinks_, false, nullptr);
}


Vector6 LinkTraverse::calcInverseDynamicsSub
(Link* link, const Vector3& vo_upper, const Vector3& dvo_upper,
 bool goParentDirection, bool goChildDirection, bool isReverseLinkChain, Link* upperLink)
{
    Vector3 dvo, sv, sw;
    Link* jointLink = nullptr;
    double jointSign = 1.0;

    if(!upperLink){
        dvo = link->dv() - link->dw().cross(link->p()) - link->w().cross(link->v());
        sv.setZero();
        sw.setZero();

    } else {
        if(isReverseLinkChain){
            jointLink = upperLink;
            jointSign = -1.0;
        } else {
            jointLink = link;
        }
        switch(link->jointType()){
        case Link::RevoluteJoint:
            sw.noalias() = jointLink->R() * (jointSign * jointLink->a());
            sv.noalias() = jointLink->p().cross(sw);
            break;
        case Link::PrismaticJoint:
            sw.setZero();
            sv.noalias() = jointLink->R() * (jointSign * jointLink->d());
            break;
        case Link::FixedJoint:
        default:
            sw.setZero();
            sv.setZero();
            break;
        }
        const Vector3 dsv = upperLink->w().cross(sv) + vo_upper.cross(sw);
        const Vector3 dsw = upperLink->w().cross(sw);
        link->dw() = upperLink->dw() + jointLink->dq() * dsw + jointLink->ddq() * sw;
        dvo = dvo_upper + jointLink->dq() * dsv + jointLink->ddq() * sv;
    }

    const Vector3 c = link->R() * link->c() + link->p();
    Matrix3 I = link->R() * link->I() * link->R().transpose();
    const Matrix3 c_hat = hat(c);
    I.noalias() += link->m() * c_hat * c_hat.transpose();
    const Vector3 vo = link->v() - link->w().cross(link->p());
    const Vector3 P = link->m() * (vo + link->w().cross(c));
    const Vector3 L = link->m() * c.cross(vo) + I * link->w();

    Vector6 f;
    f.head<3>() = link->m() * (dvo + link->dw().cross(c)) + link->w().cross(P);
    f.tail<3>() = link->m() * c.cross(dvo) + I * link->dw() + vo.cross(P) + link->w().cross(L);

    // TODO: traverse only the links contained in the travese list
    if(goParentDirection){
        if(auto parent = link->parent()){
            if(parent->body() == link->body()){
                f += calcInverseDynamicsSub(parent, vo, dvo, true, true, true, link);
            }
        }
    }
    // TODO: traverse only the links contained in the travese list
    if(goChildDirection){
        for(Link* child = link->child(); child; child = child->sibling()){
            if(child != upperLink){
                f += calcInverseDynamicsSub(child, vo, dvo, false, true, false, link);
            }
        }
    }

    f -= link->F_ext();

    if(jointLink){
        double u = sv.dot(f.head<3>()) + sw.dot(f.tail<3>());
        double Ir = jointLink->ddq() * jointLink->Jm2() /* rotor inertia */;
        jointLink->u() = jointSign * u + Ir;
    }

    return f;
}
