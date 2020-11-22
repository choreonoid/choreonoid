
#include <cstring>
#include "CosNaming_impl.h"
#include <omniORB4/omniURI.h>
#include <iostream>
#include <fmt/format.h>

using namespace std;
using namespace cnoid;

namespace {

const bool TRACE_FUNCTIONS = false;

}

std::mutex NamingContext_impl::mtx;


BindingNode::BindingNode(const CosNaming::Name& n, CosNaming::BindingType t,  CORBA::Object_ptr o, NamingContext_impl* nc)
{
    binding.binding_name = n;
    binding.binding_type = t;
                
    object = CORBA::Object::_duplicate(o);
    context = nc;
    next = 0;
                
    prev = context->lastNode;
    context->lastNode = this;

    if(prev){
        prev->next = this;
    } else {
        context->firstNode = this;
    }
    ++context->size;
}


BindingNode::~BindingNode()
{
    if(prev) {
        prev->next = next;
    } else {
        context->firstNode = next;
    }
    if(next){
        next->prev = prev;
    } else {
        context->lastNode = prev;
    }
    --context->size;
}


NamingContext_impl::NamingContext_impl(PortableServer::POA_ptr poa, const PortableServer::ObjectId& id)
    : poa(poa)
{
    firstNode = 0;
    lastNode = 0;
    size = 0;
    try {
        poa->activate_object_with_id(id, this);
    } catch (...) {
    }
}


NamingContext_impl::~NamingContext_impl()
{
    if(TRACE_FUNCTIONS){
        cout << "NamingContext_impl::~NamingContext_impl()" << endl;
    }
}


void NamingContext_impl::bind(const CosNaming::Name& n, CORBA::Object_ptr obj)
{
    if(TRACE_FUNCTIONS){
        cout << "NamingContext_impl::bind()" << endl;
    }
    bind_sub(n, obj, CosNaming::nobject, false);
}


void NamingContext_impl::rebind(const CosNaming::Name& n, CORBA::Object_ptr obj)
{
    if(TRACE_FUNCTIONS){
        cout << "NamingContext_impl::rebind()" << endl;
    }
    bind_sub(n, obj, CosNaming::nobject, true);
}


void NamingContext_impl::bind_context(const CosNaming::Name& n, CosNaming::NamingContext_ptr nc)
{
    if(TRACE_FUNCTIONS){
        cout << "NamingContext_impl::bind_context()" << endl;
    }
    bind_sub(n, nc, CosNaming::ncontext, false);
}


void NamingContext_impl::rebind_context(const CosNaming::Name& n, CosNaming::NamingContext_ptr nc)
{
    if(TRACE_FUNCTIONS){
        cout << "NamingContext_impl::rebind_context()" << endl;
    }
    bind_sub(n, nc, CosNaming::ncontext, true);
}


CORBA::Object_ptr NamingContext_impl::resolve(const CosNaming::Name& n)
{
    if(TRACE_FUNCTIONS){
        cout << "NamingContext_impl::resolve()" << endl;
    }
    if(n.length() == 1) {
        std::lock_guard<std::mutex> lock(mtx);
        BindingNode* node = resolve_single(n);
        return CORBA::Object::_duplicate(node->object);
        
    } else {
        CosNaming::Name restOfName;
        CosNaming::NamingContext_var context = resolve_multi(n, restOfName);
        return context->resolve(restOfName);
    }
}


void NamingContext_impl::unbind(const CosNaming::Name& n)
{
    if(TRACE_FUNCTIONS){
        cout << "NamingContext_impl::unbind()" << endl;
    }
    if(n.length() == 1) {
        std::lock_guard<std::mutex> lock(mtx);
        BindingNode* node = resolve_single(n);
        CosNaming::NamingContext_var nc = _this();
        delete node;

    } else {
        CosNaming::Name restOfName;
        CosNaming::NamingContext_var context = resolve_multi(n, restOfName);
        context->unbind(restOfName);
    }
}


CosNaming::NamingContext_ptr NamingContext_impl::new_context()
{
    if(TRACE_FUNCTIONS){
        cout << "NamingContext_impl::new_context()" << endl;
    }
    PortableServer::ObjectId_var an_oid = PortableServer::string_to_ObjectId((string("id") + std::to_string(count++)).c_str());
    CORBA::Object_var ref = nspoa->create_reference_with_id(an_oid, CosNaming::NamingContext::_PD_repoId);

    PortableServer::ObjectId_var id = nspoa->reference_to_id(ref);
    NamingContext_impl* nc = new NamingContext_impl(nspoa, id);
    CosNaming::NamingContext_ptr ncref = nc->_this();
    nc->_remove_ref();
    return ncref;
}


CosNaming::NamingContext_ptr NamingContext_impl::bind_new_context(const CosNaming::Name& n)
{
    if(TRACE_FUNCTIONS){
        cout << "NamingContext_impl::bind_new_context()" << endl;
    }
    if(n.length() == 1) {
        CosNaming::NamingContext_ptr nc = new_context();
        try {
            bind_context(n, nc);
        } catch (...) {
            nc->destroy();
            CORBA::release(nc);
            throw;
        }
        return nc;

    } else {
        CosNaming::Name restOfName;
        CosNaming::NamingContext_var context = resolve_multi(n, restOfName);
        return context->bind_new_context(restOfName);
    }
}


void NamingContext_impl::destroy()
{
    if(TRACE_FUNCTIONS){
        cout << "NamingContext_impl::destroy()" << endl;
    }
    std::lock_guard<std::mutex> lock(mtx);

    if(firstNode){
        throw CosNaming::NamingContext::NotEmpty();
    }

    CosNaming::NamingContext_var nc = _this();

    PortableServer::ObjectId_var id = poa->servant_to_id(this);
    poa->deactivate_object(id);
}


void NamingContext_impl::list(CORBA::ULong how_many, CosNaming::BindingList_out bl, CosNaming::BindingIterator_out bi)
{
    if(TRACE_FUNCTIONS){
        cout << "NamingContext_impl::list()" << endl;
    }
    CosNaming::BindingList* allBindings;
    
    {
        std::lock_guard<std::mutex> lock(mtx);

        allBindings = new CosNaming::BindingList(size);
        allBindings->length(size);
        
        int index = 0;
        for(BindingNode* node = firstNode; node; node = node->next){
            (*allBindings)[index++] = node->binding;
        }
    }

    if(allBindings->length() <= how_many) {
        bi = CosNaming::BindingIterator::_nil();
        bl = allBindings;
        return;
    }

    BindingIterator_impl* bii = new BindingIterator_impl(nspoa, allBindings);

    bi = bii->_this();
    bii->_remove_ref();

    if(!CORBA::is_nil(bi.ptr())) {
        bi->next_n(how_many, bl);
    }
}


char* NamingContext_impl::to_string(const CosNaming::Name& name)
{
    if(TRACE_FUNCTIONS){
        cout << "NamingContext_impl::to_string()" << endl;
    }
    return omni::omniURI::nameToString(name);
}


CosNaming::Name* NamingContext_impl::to_name(const char* sn)
{
    if(TRACE_FUNCTIONS){
        cout << "NamingContext_impl::to_name()" << endl;
    }
    return omni::omniURI::stringToName(sn);
}


char* NamingContext_impl::to_url(const char* addr, const char* sn)
{
    if(TRACE_FUNCTIONS){
        cout << "NamingContext_impl::to_url()" << endl;
    }
    return omni::omniURI::addrAndNameToURI(addr, sn);
}


CORBA::Object_ptr NamingContext_impl::resolve_str(const char* sn)
{
    if(TRACE_FUNCTIONS){
        cout << "NamingContext_impl::resolve_str()" << endl;
    }
    CosNaming::Name_var name = omni::omniURI::stringToName(sn);
    return resolve(name);
}


BindingNode* NamingContext_impl::resolve_single(const CosNaming::Name& n)
{
    if(TRACE_FUNCTIONS){
        cout << "NamingContext_impl::resolve_single()" << endl;
    }
    for(BindingNode* node = firstNode; node; node = node->next){
        if((strcmp(n[0].id, node->binding.binding_name[0].id) == 0) &&
           (strcmp(n[0].kind, node->binding.binding_name[0].kind) == 0)){
            return node;
        }
    }
    throw CosNaming::NamingContext::NotFound(CosNaming::NamingContext::missing_node, n);
    
    return 0;
}


CosNaming::NamingContext_ptr NamingContext_impl::resolve_multi(const CosNaming::Name& n, CosNaming::Name& restOfName)
{
    if(TRACE_FUNCTIONS){
        cout << "NamingContext_impl::resolve_multi()" << endl;
    }
    if(n.length() == 0){
        throw CosNaming::NamingContext::InvalidName();
    }

    CosNaming::Name contextName = n;
    contextName.length(1);
    restOfName.length(n.length() - 1);
    for(unsigned int i = 0; i < n.length() - 1; ++i){
        restOfName[i] = n[i + 1];
    }

    BindingNode* node;

    std::lock_guard<std::mutex> lock(mtx);

    try {
        node = resolve_single(contextName);
    }
    catch (CosNaming::NamingContext::NotFound& ex) {
        ex.rest_of_name = n;
        throw;
    }

    CosNaming::NamingContext_var context = CosNaming::NamingContext::_narrow(node->object);

    if(CORBA::is_nil((CosNaming::NamingContext_ptr)context) ||
       (node->binding.binding_type != CosNaming::ncontext)) {
        throw CosNaming::NamingContext::NotFound(CosNaming::NamingContext::not_context, n);
    }

    return CosNaming::NamingContext::_duplicate(context);
}


void NamingContext_impl::bind_sub(const CosNaming::Name& n, CORBA::Object_ptr obj, CosNaming::BindingType t, CORBA::Boolean rebind)
{
    if(TRACE_FUNCTIONS){
        cout << "NamingContext_impl::bind_sub()" << endl;
    }
    if(n.length() == 1){
        std::lock_guard<std::mutex> lock(mtx);
        
        BindingNode* node = 0;
        
        try {
            node = resolve_single(n);
            if(!rebind){
                throw CosNaming::NamingContext::AlreadyBound();
            }
        }
        catch (CosNaming::NamingContext::NotFound& ex) {
            node = 0;
        }

        CosNaming::NamingContext_var nc = _this();

        if(node){
            delete node;
        }
        
        new BindingNode(n, t, obj, this);
        
    } else {
        CosNaming::Name restOfName;
        CosNaming::NamingContext_var context = resolve_multi(n, restOfName);

        if(t == CosNaming::nobject){
            if(rebind){
                context->rebind(restOfName, obj);
            } else {
                context->bind(restOfName, obj);
            }
        } else {
            if(rebind){
                context->rebind_context(restOfName, CosNaming::NamingContext::_narrow(obj));
            } else {
                context->bind_context(restOfName, CosNaming::NamingContext::_narrow(obj));
            }
        }
    }
}


BindingIterator_impl::BindingIterator_impl(PortableServer::POA_ptr poa, CosNaming::BindingList* list)
    : list(list)
{
    if(TRACE_FUNCTIONS){
        cout << "BindingIterator_impl::BindingIterator_impl()" << endl;
    }
    PortableServer::ObjectId_var id = poa->activate_object(this);
}


BindingIterator_impl::~BindingIterator_impl()
{
    if(TRACE_FUNCTIONS){
        cout << "BindingIterator_impl::~BindingIterator_impl()" << endl;
    }
    delete list;
}


CORBA::Boolean BindingIterator_impl::next_one(CosNaming::Binding_out b)
{
    if(TRACE_FUNCTIONS){
        cout << "BindingIterator_impl::next_one" << endl;
    }
    CosNaming::BindingList* blist;
    CORBA::Boolean ret = next_n(1, blist);
    b = new CosNaming::Binding;
    if(ret){
        *b = (*blist)[0];
    } else {
        b.ptr()->binding_type = CosNaming::nobject;
    }
    delete blist;
    return ret;
}


CORBA::Boolean BindingIterator_impl::next_n(CORBA::ULong how_many, CosNaming::BindingList_out bl)
{
    if(TRACE_FUNCTIONS){
        cout << "BindingIterator_impl::next_n" << endl;
    }
    if(list->length() < how_many){
        how_many = list->length();
    }
    bl = list;
    list = new CosNaming::BindingList(bl->length() - how_many);
    list->length(bl->length() - how_many);
    for(unsigned int i = 0; i < list->length(); ++i){
        (*list)[i] = (*bl)[i + how_many];
    }
    bl->length(how_many);
    
    return (how_many == 0) ? false : true;
}


void BindingIterator_impl::destroy(void)
{
    if(TRACE_FUNCTIONS){
        cout << "BindingIterator_impl::destroy" << endl;
    }
    PortableServer::ObjectId_var id = nspoa->servant_to_id(this);
    nspoa->deactivate_object(id);
}
