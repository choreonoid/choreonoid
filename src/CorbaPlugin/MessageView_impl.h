/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_CORBA_PLUGIN_MESSAGE_VIEW_IMPL_H_INCLUDED
#define CNOID_CORBA_PLUGIN_MESSAGE_VIEW_IMPL_H_INCLUDED


#include <cnoid/corba/MessageView.hh>

namespace cnoid {

class MessageView_impl : virtual public POA_cnoid::Corba::MessageView
{
protected:
    CORBA::ORB_var orb;
        
public:
        
    MessageView_impl(CORBA::ORB_ptr orb);
    virtual ~MessageView_impl();
        
    virtual void put(const char* message);
};
}

#endif /* CONTROLLER_IMPL */
