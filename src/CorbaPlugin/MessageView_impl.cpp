/**
   @author Shin'ichiro Nakaoka
*/

#include "MessageView_impl.h"
#include <cnoid/MessageView>
#include <iostream>


using namespace std;
using namespace cnoid;

MessageView_impl::MessageView_impl(CORBA::ORB_ptr orb)
    : orb(CORBA::ORB::_duplicate(orb))
{
	
}


MessageView_impl::~MessageView_impl()
{
    PortableServer::POA_var poa = _default_POA();
    PortableServer::ObjectId_var id = poa->servant_to_id(this);
    poa->deactivate_object(id);
}


void MessageView_impl::put(const char* message)
{
    cnoid::MessageView::instance()->put(message);
}
