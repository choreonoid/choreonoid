from omniORB import CORBA, any, cdrUnmarshal, cdrMarshal
import CosNaming

import cnoid.Corba

cnoid.Corba.helloCorba()

global rootnc, orb

#orb = CORBA.ORB_init(sys.argv, CORBA.ORB_ID)
orb = cnoid.Corba.getORB()

nameserver = orb.resolve_initial_references("NameService");
rootnc = nameserver._narrow(CosNaming.NamingContext)

if(rootnc):
    print "rootnc found!"
