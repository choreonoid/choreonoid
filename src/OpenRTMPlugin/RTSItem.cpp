/*!
 * @brief  This is a definition of RTSystemEditorPlugin.

 * @author Hisashi Ikari 
 * @file
 */
#include "RTSItem.h"
#include <cnoid/ItemManager>
#include <cnoid/Archive>

#include "gettext.h"

#if 0
#include <math.h>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QMenu>
#include <QGridLayout>       // for including diagram-view
#include <QHBoxLayout>       // for including diagram-view
#include <QVBoxLayout>       // for including diagram-view
#include <QGraphicsTextItem> // for including diagram-view
#include <QTreeWidget>       // for including diagram-view
#include <QWidget>           // for including diagram-view

#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <cnoid/EigenArchive>
#include <cnoid/Action>
#include <cnoid/MainWindow>
#include <cnoid/View>
#include <cnoid/Selection>
#include <cnoid/TreeWidget> // for including diagram-view
#include <cnoid/Dialog>     // for including diagram-view
#include <cnoid/View>       // for including diagram-view
#include <cnoid/ViewManager>

#include <rtm/CorbaNaming.h>
#include <rtm/RTObject.h>
#include <rtm/CorbaConsumer.h>                

#include "RTSItem.h"
#include "RTSComponent.h"
#include "RTSCommonImpl.h"
#include "RTSComponentImpl.h"
#include "RTSDiagramViewImpl.h"
#include "RTSNameServerView.h"

#include "gettext.h"

using namespace boost;
using namespace boost::uuids;
using namespace RTC;
#endif

namespace cnoid 
{ 
    //enum {DT_TIMESEQ_FLOAT = 0};
    //enum {_TIMESEQ_FLOAT = 0};

class RTSystemItemImpl
{
//    friend class RTSystemItem;
public:
    RTSystemItem* self;

    RTSystemItemImpl(RTSystemItem* self);
    RTSystemItemImpl(RTSystemItem* self, const RTSystemItemImpl& org);
    ~RTSystemItemImpl();

    //void doPutProperties(PutPropertyFunction& putProperty);
    //void store(Archive& archive);
    //void restore(const Archive& archive);


    /*RTSystemItemImpl(DroppableAbstractionImplPtr abst, DroppablePresentationImplPtr pres, RTCValuePtr value)
    : abst(abst), pres(pres), value(value), dataType(1, CNOID_GETTEXT_DOMAIN_NAME)
    {
        dataType.setSymbol(DT_TIMESEQ_FLOAT, N_("TimedFloatSeq"));
            }
    RTSystemItemImpl() {}

        void doPutProperties    (PutPropertyFunction& putProperty);
        bool storeAbstraction   (RTCControllerPtr conp, MappingPtr view);
        bool storePresentation  (RTCControllerPtr conp, MappingPtr abst);
        bool storeMovableLines  (MovableLinesPtr lines, MappingPtr connm);
        bool storeDirections    (DirectionsPtr directions, MappingPtr dic);
        bool restoreAbstraction (MappingPtr abst, string name, RTCAbstractionPtr* conp);
        bool restorePresentation(MappingPtr view, RTCAbstractionPtr abst, RTCPresentationPtr* conp);
        bool restoreMovableLines(MappingPtr lines, MovableLinesPtr conp);
        bool restoreDirections  (MappingPtr directions, DirectionsPtr rdic);
        void readRTCDiagrams    (const Archive& archive);
        bool convertListing     (MappingPtr view, string name, PortPosPtr items);
        void disconnectByRoot   ();

        static RTSDiagramView*    getDiagramView();
        static RTSNameServerView* getNameServerView();

    public:
        DroppableAbstractionImplPtr  abst;
        DroppablePresentationImplPtr pres;

        map<string, string> convertion; // for switching old-id to new-id
        RTCValuePtr         value;
        string              name; 

        Selection dataType;
        Selection interfaceType;
        Selection dataflowType;
        Selection subscriptionType;
        */
};

void RTSystemItem::initialize(ExtensionManager* ext)
{
    ext->itemManager().registerClass<RTSystemItem>(N_("RTSystemItem"));
    ext->itemManager().addCreationPanel<RTSystemItem>();
}


RTSystemItem::RTSystemItem()
{
    impl = new RTSystemItemImpl(this);
    /*
    RTSDiagramView* diagram = RTSystemItemImpl::getDiagramView();
        if (!diagram) {
            throwException((format(_("RTC Diagram view not found"))).str());
        }
        QGraphicsView* temp = diagram->value->view;
        impl = RTSystemItemImplPtr(new RTSystemItemImpl(static_cast<DroppableGraphicsView*>(temp)->abstImpl,
                                                        static_cast<DroppableGraphicsView*>(temp)->presImpl,
                                                        diagram->value));
        impl->name = lexical_cast<string>(random_generator()());
        */
}


RTSystemItemImpl::RTSystemItemImpl(RTSystemItem* self)
    : self(self)
{

}


RTSystemItem::RTSystemItem(const RTSystemItem& org)
    : Item(org)
{
    impl = new RTSystemItemImpl(this, *org.impl);
}


RTSystemItemImpl::RTSystemItemImpl(RTSystemItem* self, const RTSystemItemImpl& org)
    : self(self)
{

}


RTSystemItem::~RTSystemItem()
{
    delete impl;
}


RTSystemItemImpl::~RTSystemItemImpl()
{

}


ItemPtr RTSystemItem::doDuplicate() const
{
    return new RTSystemItem(*this);
}


#if 0
    ItemPtr RTSystemItem::doDuplicate() const
    {
        RTSDiagramView* diagram = impl->getDiagramView();
        if (!diagram) {
            throwException((format(_("RTC Diagram view not found"))).str());
        }
        impl->pres->enable = true;
        impl->pres->scene->clear();
        diagram->value->view->setEnabled(true);
        diagram->value->view->setBackgroundBrush(QBrush(Qt::white, Qt::SolidPattern));

        RTSystemItem* obj = new RTSystemItem(*this);
        QGraphicsView* temp = diagram->value->view;
        obj->impl = RTSystemItemImplPtr(new RTSystemItemImpl(static_cast<DroppableGraphicsView*>(temp)->abstImpl, 
                                                             static_cast<DroppableGraphicsView*>(temp)->presImpl,
                                                             diagram->value)); 
        obj->impl->name = lexical_cast<string>(random_generator()());

        static_cast<DroppableGraphicsView*>(temp)->switchControllers(obj->impl->name);
        return obj;
    }
#endif

void RTSystemItem::doPutProperties(PutPropertyFunction& putProperty)
{
//    impl->doPutProperties(putProperty);

//    this->sigDisconnectedFromRoot().connect(bind(&RTSystemItemImpl::disconnectByRoot, impl));
}

#if 0
    void RTSystemItemImpl::doPutProperties(PutPropertyFunction& putProperty)
    {
        QGraphicsView* temp = value->view;
        DroppableGraphicsView* view = static_cast<DroppableGraphicsView*>(temp);
        if (!view) {
            throwException((format(_("RTC Diagram view not found"))).str());
        }

        view->switchControllers(name);
        for (RTCControllers::iterator iterc  = view->abstImpl->controllers->begin(); 
                                      iterc != view->abstImpl->controllers->end();
                                      iterc++) {
            iterc->second->view->showDiagram(false); // only delete object without scene
        }
        view->presImpl->replaceRow();
        view->presImpl->connectDiagram();
        view->fitFigure(view->presImpl->mag);

        putProperty(_("Unique Name"), name);
    }


    void RTSystemItemImpl::disconnectByRoot()
    {
        RTSDiagramView* view = getDiagramView();
        if (!view) {
            MessageView::instance()->putln((format(_("RTC Diagram view not found"))).str());
            return;
        }
        view->clearDiagram();

        abst->group->controllers->erase(name);
        abst->group->lines->      erase(name);
        abst->group->arrows->     erase(name);
        abst->group->persistants->erase(name);
        abst->group->mags       ->erase(name);
        abst->group->movables   ->erase(name);
        abst->group->directions ->erase(name);

        for (int i = 0; i < abst->controllers.use_count(); i++) {
            abst->controllers.reset();
        }
               
        view->initView();
    }


    bool RTSystemItem::store(Archive& archive)
    {
        QGraphicsView* temp = impl->value->view;
        DroppableGraphicsView* view = static_cast<DroppableGraphicsView*>(temp);
        if (!view) {
            MessageView::instance()->putln((format(_("RTC Diagram view not found"))).str());
            return false;
        }
        view->switchControllers(impl->name);

        MappingPtr controllers = new Mapping;
        for (RTCControllers::iterator iterc  = view->abstImpl->controllers->begin(); 
                                      iterc != view->abstImpl->controllers->end(); 
                                      iterc++) {

            // put the information of view.
            RTCControllerPtr conp = iterc->second; 
            MappingPtr presentation = new Mapping;
            if (!impl->storePresentation(conp, presentation)) {
                return false;
            }

            // put the corba-server information.
            MappingPtr abst = new Mapping;
            if (!impl->storeAbstraction(conp, abst)) {
                return false;
            }

            // put the abst and view to controller infomation.
            MappingPtr controller = new Mapping;
            controller->insert("abstraction", abst);
            controller->insert("presentation", presentation);
            controllers->insert(iterc->first, controller);
        }
        archive.insert("controllers", controllers);

        // put the connections' lines on diagram view.
        MappingPtr linegroup = new Mapping;
        if (!impl->storeMovableLines(view->presImpl->movables, linegroup)) {
            return false;
        }
        archive.insert("connection-lines", linegroup);

        // put the connections' directions on diagram view.
        MappingPtr dicgroup = new Mapping;
        if (!impl->storeDirections(view->presImpl->directions, dicgroup)) {
            return false;
        }
        archive.insert("connection-directions", dicgroup);

        // record the toolbar settings.
        MappingPtr toolbar = new Mapping;
        toolbar->write("mag", view->presImpl->mag);
        archive.insert("toolbar-setting", toolbar);
 
        // repaint for the aborting crash of qt-graphics-scene.
        view->switchControllers(impl->name);
        for (RTCControllers::iterator iterc  = view->abstImpl->controllers->begin();
                                      iterc != view->abstImpl->controllers->end();
                                      iterc++) {
            iterc->second->view->showDiagram(false); // only delete object without scene
        }
        view->presImpl->replaceRow();
        view->presImpl->connectDiagram();

        return true;
    }
    

    bool RTSystemItemImpl::storeAbstraction(RTCControllerPtr conp, MappingPtr abst)
    {
        try {
            // put the corba-server information.
            abst->write("state", conp->abstraction->impl->state);

            // inport name informations.
            int inportNum = P(conp->abstraction->inports)->size();
            ListingPtr inports = new Listing(inportNum);
            for (int i = 0; i < inportNum; i++) {
                inports->write(i, P(conp->abstraction->inports)->at(i)->name);
            }           
            inports->setFlowStyle(true);
            abst->insert("in-ports", inports);

            ListingPtr inportt = new Listing(inportNum);
            for (int i = 0; i < inportNum; i++) {
                inportt->write(i, P(conp->abstraction->inports)->at(i)->servicePort);
            }           
            inportt->setFlowStyle(true);
            abst->insert("in-ports-type", inportt);

            // outport informations.
            int outportNum = P(conp->abstraction->outports)->size();
            ListingPtr outports = new Listing(outportNum);
            for (int i = 0; i < outportNum; i++) {
                outports->write(i, P(conp->abstraction->outports)->at(i)->name);
            }
            outports->setFlowStyle(true);
            abst->insert("out-ports", outports);

            ListingPtr outportt = new Listing(outportNum);
            for (int i = 0; i < outportNum; i++) {
                outportt->write(i, P(conp->abstraction->outports)->at(i)->servicePort);
            }
            outportt->setFlowStyle(true);
            abst->insert("out-ports-type", outportt);

            // ports' connection information. 
            MappingPtr connsmap = new Mapping;
            for (RTCConnectionMap::iterator iteri  = C(conp->abstraction->connections)->begin();
                                       iteri != C(conp->abstraction->connections)->end();
                                       iteri++) {
                int ic = 0;
                MappingPtr connmap = new Mapping;
                for (RTCConnectionList::iterator iterv = iteri->second->begin(); iterv != iteri->second->end(); iterv++) {
                    // check the persistant conection.
                    Persistants::iterator iterp = std::find(T(conp->abstraction->persistants)->begin(), 
                                                            T(conp->abstraction->persistants)->end(), 
                                                            S((*iterv)->id));
                    if (iterp != T(conp->abstraction->persistants)->end()) {
                        MappingPtr conn = new Mapping;
                        conn->write("id",           S((*iterv)->id));
                        conn->write("name",         S((*iterv)->name));
                        conn->write("source-rtc",   S((*iterv)->sourceRtcName));
                        conn->write("source-port",  S((*iterv)->sourcePortName));
                        conn->write("target-rtc",   S((*iterv)->targetRtcName));
                        conn->write("target-port",  S((*iterv)->targetPortName));
                        conn->write("dataflow",     S((*iterv)->dataflow));
                        conn->write("interface",    S((*iterv)->sinterface));
                        conn->write("subscription", S((*iterv)->subscription));
                        conn->write("push-rate",    F((*iterv)->pushRate));
                        conn->write("push-policy",  S((*iterv)->pushPolicy));
                        connmap->insert("connection" + lexical_cast<string>((++ic)), conn);
                    }
                }
                if (0 < ic) {
                    connsmap->insert(iteri->first, connmap);
                }
            }
            abst->insert("connections", connsmap);

        } catch (...) {
            return false;
        }

        return true;
    }


    bool RTSystemItemImpl::storePresentation(RTCControllerPtr conp, MappingPtr view)
    {
        try {
            // put the base information of view.
            view->write("basex", conp->view->impl->basex);
            view->write("basey", conp->view->impl->basey);
            view->write("rectl", conp->view->impl->rectl);
            view->write("row",   conp->view->impl->row);

            // put the port location of inport. 
            BOOST_ASSERT(conp->view->impl->inports->size() == conp->view->impl->inports->size() && 
                         conp->view->impl->inports->size() == P(conp->abstraction->inports)->size());
            int inportNum  = conp->view->impl->inports->size();
            VectorXd in = VectorXd(inportNum * 2);
            for (int i = 0; i < (inportNum * 2); i++) {
                in[i] = ((i % 2) == 0) ? conp->view->impl->inports->at(floor(static_cast<float>(i / 2))).first 
                                       : conp->view->impl->inports->at(floor(static_cast<float>(i / 2))).second;
            }
            write(*view, "in-pos", in);

            // put the port location of outport.
            BOOST_ASSERT(conp->view->impl->outports->size() == conp->view->impl->outports->size() &&  
                         conp->view->impl->outports->size() == P(conp->abstraction->outports)->size());
            int outportNum = conp->view->impl->outports->size();
            VectorXd out = VectorXd(outportNum * 2);
            for (int i = 0; i < (outportNum * 2); i++) {
                out[i] = ((i % 2) == 0) ? conp->view->impl->outports->at(floor(static_cast<float>(i / 2))).first 
                                        : conp->view->impl->outports->at(floor(static_cast<float>(i / 2))).second;
            }
            write(*view, "ot-pos", out);

        } catch (...) {
            return false;
        }

        return true;
    }


    bool RTSystemItemImpl::storeMovableLines(MovableLinesPtr lines, MappingPtr linegroup)
    {
        // put the connections' lines on diagram view.
        try {
            int j = 0;
            for (MovableLines::iterator iter = lines->begin(); iter != lines->end(); iter++) {
                MappingPtr lines = new Mapping;
                DroppableGraphicsLineGroupPtr group = iter->second;
                for (int i = 0; i < 3; i++) {
                    VectorXd line = VectorXd(4);
                    line << group->msp[i    ][0], group->msp[i    ][1],
                            group->msp[i + 1][0], group->msp[i + 1][1];
                    write(*lines, ("sline" + lexical_cast<string>(i + 1)), line);
                }
                for (int i = 0; i < 3; i++) {
                    VectorXd line = VectorXd(4);
                    line << group->mtp[i    ][0], group->mtp[i    ][1],
                            group->mtp[i + 1][0], group->mtp[i + 1][1];
                    write(*lines, ("tline" + lexical_cast<string>(i + 1)), line);
                }
                linegroup->insert(iter->first, lines);
                j++;
            }
        } catch (...) {
            return false;
        }

        return true;
    }


    bool RTSystemItemImpl::storeDirections(DirectionsPtr directions, MappingPtr dicgroup)
    {
        // put the connections' directions on diagram view.
        try {
            for (Directions::iterator iter = directions->begin(); iter != directions->end(); iter++) {
                MappingPtr dics = new Mapping;
                DirectionPtr dic = iter->second;

                ListingPtr tdics = new Listing(5);
                for (int i = 0; i < 5; i++) {
                    tdics->write(i, (*dic)(i));
                }           
                tdics->setFlowStyle(true);
                dicgroup->insert(iter->first, tdics);
            }
        } catch (...) {
            return false;
        }

        return true;
    }


    bool RTSystemItem::restore(const Archive& archive)
    {
        archive.addPostProcess(bind(&RTSystemItemImpl::readRTCDiagrams, impl, ref(archive)));
        return true;
    }
 

    void RTSystemItemImpl::readRTCDiagrams(const Archive& archive)
    {
        RTSNameServerView* nameserver = getNameServerView();
        if (!nameserver) {
            throwException((format(_("RTC NameServer view not found"))).str());
        }
        nameserver->setConnection();

        RTSDiagramView* diagram = getDiagramView();
        if (!diagram) {
            throwException((format(_("RTC Diagram view not found"))).str());
        }
        QGraphicsView* temp = value->view;
        DroppableGraphicsView* view = static_cast<DroppableGraphicsView*>(temp);

        this->name = lexical_cast<string>(random_generator()());
        view->switchControllers(this->name);

        diagram->clearDiagram();
        Mapping* conts = archive.findMapping("controllers");
        if (!conts->isValid()) {
            MessageView::instance()->putln((format(_("invaid controller found"))).str());
            return;
        }
        convertion.clear();

        RTCAbstractionPtr refabst;
        for (Mapping::iterator iterm = conts->begin(); iterm != conts->end(); iterm++) {
            // read the yaml.
            MappingPtr elements = (iterm->second)->toMapping();
            MappingPtr mabst = elements->findMapping("abstraction");
            MappingPtr mview = elements->findMapping("presentation");

            // create a controller.
            RTCAbstractionPtr  rabst; // instance is created in concrete function.
            RTCPresentationPtr rview;

            if (!restoreAbstraction (mabst, iterm->first, &rabst)) return;
            if (!restorePresentation(mview, rabst,        &rview)) return;

            RTCControllerImplPtr tcontImpl = RTCControllerImplPtr(new RTCControllerImpl); 
            RTCControllerPtr rcont = RTCControllerPtr(new RTCController(tcontImpl, value, 
                                                                        rabst, rview, rabst->impl->rtcName, 
                                                                        rview->impl->basex, rview->impl->basey, false));
            refabst = rabst;

            // regist controllers.
            pair<RTCControllers::iterator, bool> pib = abst->controllers
                ->insert(pair<string, RTCControllerPtr>(iterm->first, rcont));
            if (!pib.second) {
                MessageView::instance()->putln((format(_("duplicate controller:%1%")) % iterm->first).str());
                return;
            }           
           
            // show rtc.
            rview->showDiagram();
        }

        // get the toolbar settings.
        MappingPtr toolbar = archive.findMapping("toolbar-setting");
        view->presImpl->mag = toolbar->get("mag").toDouble();
        if (view->presImpl->mag <= 0.0) view->presImpl->mag = 1.0;
        view->fitFigure(view->presImpl->mag);

        // get the connection-lines.
        MappingPtr conns = archive.findMapping("connection-lines");
        if (!restoreMovableLines(conns, pres->movables)) return;

        // get the connection-directions.
        MappingPtr dics = archive.findMapping("connection-directions");
        if (!restoreDirections(dics, pres->directions)) return;

        // draw the connections.
        view->presImpl->connectDiagram();
 
        // Wait for drawing line of timer.
        // initialize the mouse event.
        pres->drag = false;    
    }


    bool RTSystemItemImpl::restoreAbstraction(MappingPtr abst, string name, RTCAbstractionPtr* conp)
    {
        if (!abst->isValid()) {
            return false;
        }

        RTCAbstractionImplPtr impl = RTCAbstractionImplPtr(new RTCAbstractionImpl);
        RTCAbstractionPtr ret = RTCAbstractionPtr(new RTCAbstraction(impl, value, name, 
                                                                     RTCConnectionMapPtr(new RTCConnectionMap), 
                                                                     PersistantsPtr(new Persistants),
                                                                     IndexesPtr    (new Indexes),
                                                                     PortsPtr      (new Ports),
                                                                     PortsPtr      (new Ports)));
        *conp = ret;

        // get the basic abstraction of rtc.
        ret->impl->state = abst->get("state").toInt();

        // get the in-port informations.
        Listing* inports = abst->get("in-ports").toListing();
        Listing* inportt = abst->get("in-ports-type").toListing();
        for (int i = 0; i < inports->size(); i++) {
            RTCPortPtr port = RTCPortPtr(new RTCPort(inports->at(i)->toString(), inportt->at(i)->toInt()));
            P(ret->inports)->push_back(port);
        }

        // get the out-port infomations.
        Listing* outports = abst->get("out-ports").toListing();
        Listing* outportt = abst->get("out-ports-type").toListing();
        for (int i = 0; i < outports->size(); i++) {
            RTCPortPtr port = RTCPortPtr(new RTCPort(outports->at(i)->toString(), outportt->at(i)->toInt()));
            P(ret->outports)->push_back(port);
        }

        // get the connections.
        Mapping* conns = abst->findMapping("connections");
        if (!conns->isValid()) {
            return true;
        }
        for (Mapping::iterator iterc = conns->begin(); iterc != conns->end(); iterc++) {
            Mapping* connsmap = (iterc->second)->toMapping();

            RTCConnectionListPtr rconns = RTCConnectionListPtr(new RTCConnectionList);
            for (Mapping::iterator iterv = connsmap->begin(); iterv != connsmap->end(); iterv++) {
                Mapping* connmap = (iterv->second)->toMapping();
                string beforeId = connmap->get("id").toString();

                RTCConnectionImplPtr impl = RTCConnectionImplPtr(new RTCConnectionImpl);
                RTCConnectionPtr rcon = RTCConnectionPtr(new RTCConnection(impl, value,
                                                                           connmap->get("id"          ).toString(),
                                                                           connmap->get("name"        ).toString(),
                                                                           connmap->get("source-rtc"  ).toString(),
                                                                           connmap->get("source-port" ).toString(),
                                                                           connmap->get("target-rtc"  ).toString(),
                                                                           connmap->get("target-port" ).toString(),
                                                                           connmap->get("dataflow"    ).toString(),
                                                                           connmap->get("interface"   ).toString(),
                                                                           connmap->get("subscription").toString(),
                                                                           connmap->get("push-rate"   ).toDouble(),
                                                                           connmap->get("push-policy" ).toString()));
                try {
                    rcon->connect();
                    rcon->setId(); // connection is new, old id is recorded. then we should be getting new id.
                    convertion.insert(pair<string, string>(beforeId, rcon->id)); // regist the new-id.
                    
                } catch (...) {
                    // ignore the failure connection.
                }
                rconns->push_back(rcon);
                T(ret->persistants)->push_back(S(rcon->id));

            }
            pair<RTCConnectionMap::iterator, bool> pib = C(ret->connections)->insert(pair<string, RTCConnectionListPtr>(iterc->first, rconns));
            if (!pib.second) {
                MessageView::instance()->putln((format(_("duplicate connection:%1%")) % iterc->first).str());
                return false;
            }                      
        }
        return true;
    }


    bool RTSystemItemImpl::restorePresentation(MappingPtr view, RTCAbstractionPtr abst, RTCPresentationPtr* conp)
    {
        if (!view->isValid()) {
            return false;
        }
        RTCPresentationImplPtr impl = RTCPresentationImplPtr(new RTCPresentationImpl);
        RTCPresentationPtr ret = RTCPresentationPtr(new RTCPresentation(impl, value, NULL, abst, 
                                                                        PortPosPtr(new PortPos),
                                                                        PortPosPtr(new PortPos),
                                                                        abst->rtcName, 0, 0));
        *conp = ret;

        ret->impl->rtcName = abst->rtcName;
        ret->impl->basex   = view->get("basex").toInt();
        ret->impl->basey   = view->get("basey").toInt();
        ret->impl->rectl   = view->get("rectl").toInt();
        ret->impl->row     = view->get("row"  ).toInt();

        if (!convertListing(view, "in-pos", ret->impl->inports )) return false;       
        if (!convertListing(view, "ot-pos", ret->impl->outports)) return false;       

        return true;
    }


    bool RTSystemItemImpl::restoreMovableLines(MappingPtr conns, MovableLinesPtr conp)
    {
        if (!conns->isValid()) {
            return false;
        }
        for (Mapping::iterator iterp = conns->begin(); iterp != conns->end(); iterp++) {
            Mapping* lines = (iterp->second)->toMapping();
            if (!lines->isValid()) {
                return false;
            }

            int u = 0;
            DroppableGraphicsLineGroupPtr group = DroppableGraphicsLineGroupPtr(new DroppableGraphicsLineGroup);
            for (Mapping::iterator iterc = lines->begin(); iterc != lines->end(); iterc++) {
                Listing* line = (iterc->second)->toListing();
                if (!line->isValid()) {
                    return false;
                }
                BOOST_ASSERT(line->size() == 4);
                int vline[4];
                for (int i = 0; i < line->size(); i++) {
                    ValueNodePtr value = line->at(i);
                    int result = 0;
                    try { result = value->toInt(); } catch (...) { /* NaN can not convert to int! */ }
                    vline[i] = result;
                }

                if (istarts_with(iterc->first, "sline")) {
                    // source port to target port
                    group->msp[u    ][0] = vline[0];       
                    group->msp[u    ][1] = vline[1];       
                    group->msp[u + 1][0] = vline[2];       
                    group->msp[u + 1][1] = vline[3];       
                } else {
                    // target port to source port
                    group->mtp[u    ][0] = vline[0];       
                    group->mtp[u    ][1] = vline[1];       
                    group->mtp[u + 1][0] = vline[2];       
                    group->mtp[u + 1][1] = vline[3];       
                }
                u++;
                if (2 < u) u = 0;
            }
            map<string, string>::iterator iterj = convertion.find(iterp->first);
            if (iterj != convertion.end()) {
                conp->insert(pair<string, DroppableGraphicsLineGroupPtr>(iterj->second, group));
            }
        }
        return true;
    }


    bool RTSystemItemImpl::restoreDirections(MappingPtr dics, DirectionsPtr rdic)
    {
        for (Mapping::iterator iter = dics->begin(); iter != dics->end(); iter++) {
            Listing* dic = dics->get(iter->first).toListing();
            int rsy  = dic->at(0)->toInt(); 
            int rty  = dic->at(1)->toInt(); 
            int same = dic->at(2)->toInt(); 
            int srow = dic->at(3)->toInt(); 
            int trow = dic->at(4)->toInt(); 

            DirectionPtr ndic = DirectionPtr(new Direction(5));
            (*ndic) << rsy, rty, same, srow, trow;
            rdic->insert(pair<string, DirectionPtr>(iter->first, ndic));
        }
        return true;
    } 



    bool RTSystemItemImpl::convertListing(MappingPtr view, string name, PortPosPtr items)
    {
        items->clear();

        Listing* listing = view->get(name).toListing();
        if (!listing->isValid()) {
            MessageView::instance()->putln((format(_("invalid %1% on RTSystem")) % name).str());    
            return false;
        }
        int bef = 0;
        for (int i = 0; i < listing->size(); i++) {
            ValueNodePtr value = listing->at(i);
            int result = 0;
            try { result = value->toInt(); } catch (...) { /*NaN can not convert to int!*/ }
            if ((i % 2) != 0) {
                items->push_back(pair<int, int>(bef, result));    
            }
            bef = result;            
        }

       return true;
    }


    RTSDiagramView* RTSystemItemImpl::getDiagramView()
    {
        vector<View*> views = ViewManager::allViews();
        for (vector<View*>::iterator iterv = views.begin(); iterv != views.end(); iterv++) {
            const string& viewName = (*iterv)->name();
            if (iequals(viewName, N_("RTC Diagram"))) {
                RTSDiagramView* diaView = static_cast<RTSDiagramView*>(*iterv);
                return diaView;
            }       
        }
        throwException((format(_("RTC Diagram view not found"))).str());       
        return NULL;
    }


    RTSNameServerView* RTSystemItemImpl::getNameServerView()
    {
        vector<View*> views = ViewManager::allViews();
        for (vector<View*>::iterator iterv = views.begin(); iterv != views.end(); iterv++) {
            const string& viewName = (*iterv)->name();
            if (iequals(viewName, N_("RTC List"))) {
                RTSNameServerView* diaView = static_cast<RTSNameServerView*>(*iterv);
                return diaView;
            }       
        }
        throwException((format(_("RTC NameServer view not found"))).str());       
        return NULL;
    }

#endif
}; // end of namespace
