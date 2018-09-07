
#ifndef CNOID_CORBA_COS_NAMING_IMPL_H
#define CNOID_CORBA_COS_NAMING_IMPL_H

#include <omniORB4/Naming.hh>
#include <mutex>

namespace cnoid {

extern PortableServer::POA_var nspoa;

class NamingContext_impl;

class BindingNode
{
public:
    CosNaming::Binding binding;
    CORBA::Object_var object;
    NamingContext_impl* context;
    BindingNode* prev;
    BindingNode* next;
        
    BindingNode(const CosNaming::Name& n, CosNaming::BindingType t,  CORBA::Object_ptr o, NamingContext_impl* nc);
    ~BindingNode();
};

class NamingContext_impl : public POA_CosNaming::NamingContextExt, public PortableServer::RefCountServantBase
{
    friend class BindingNode;
        
public:
        
    NamingContext_impl(PortableServer::POA_ptr poa, const PortableServer::ObjectId& id);
        
    virtual void bind(const CosNaming::Name& n, CORBA::Object_ptr obj);
    virtual void rebind(const CosNaming::Name& n, CORBA::Object_ptr obj);
    virtual void bind_context(const CosNaming::Name& n, CosNaming::NamingContext_ptr nc);
    virtual void rebind_context(const CosNaming::Name& n, CosNaming::NamingContext_ptr nc);
    virtual CORBA::Object_ptr resolve(const CosNaming::Name& n);
    virtual void unbind(const CosNaming::Name& n);
    virtual CosNaming::NamingContext_ptr new_context();
    virtual CosNaming::NamingContext_ptr bind_new_context(const CosNaming::Name& n);
    virtual void destroy();
    virtual void list(CORBA::ULong how_many, CosNaming::BindingList_out bl, CosNaming::BindingIterator_out bi);

    virtual char* to_string(const CosNaming::Name& n);
    virtual CosNaming::Name* to_name(const char* sn);
    virtual char* to_url(const char* addr, const char* sn);
    virtual CORBA::Object_ptr resolve_str(const char* n);

private:

    PortableServer::POA_ptr poa;

    // Multiple-readers, single-writer lock
    static std::mutex mtx;

    BindingNode* firstNode;
    BindingNode* lastNode;
    int size;
    int count = 0;

    ~NamingContext_impl();
        
    void bind_sub(const CosNaming::Name& n, CORBA::Object_ptr obj, CosNaming::BindingType t, CORBA::Boolean rebind);
    BindingNode* resolve_single(const CosNaming::Name& name);
    CosNaming::NamingContext_ptr resolve_multi(const CosNaming::Name& name, CosNaming::Name& restOfName);
};


class BindingIterator_impl : public POA_CosNaming::BindingIterator, public PortableServer::RefCountServantBase
{
public:
    BindingIterator_impl(PortableServer::POA_ptr poa, CosNaming::BindingList* list);

    virtual CORBA::Boolean next_one(CosNaming::Binding_out b);
    virtual CORBA::Boolean next_n(CORBA::ULong how_many, CosNaming::BindingList_out bl);
    virtual void destroy(void);

private:
    CosNaming::BindingList* list;

    ~BindingIterator_impl();
};

}

#endif
