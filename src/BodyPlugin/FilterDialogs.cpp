/**
   @file
   @author Shin'ichiro NAKAOKA
*/

#include "FilterDialogs.h"
#include "BodyItem.h"
#include "LinkSelectionView.h"
#include <cnoid/MainWindow>
#include <cnoid/MenuManager>
#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <cnoid/MultiValueSeqItem>
#include <cnoid/BodyMotionUtil>
#include <glibmm/i18n.h>
#include <gtkmm/dialog.h>
#include <gtkmm/spinbutton.h>
#include <gtkmm/box.h>
#include <gtkmm/label.h>

#include <iostream>

using namespace std;
using namespace cnoid;

namespace {

class VelocityLimitFilterDialog : public Gtk::Dialog
{
public:
    VelocityLimitFilterDialog();
protected:
    virtual void on_response(int id);
private:
    Gtk::CheckButton onlySelectedCheck;
    Gtk::SpinButton percentSpin;
    Gtk::CheckButton pollardCheck;
    Gtk::SpinButton ksSpin;
    Gtk::CheckButton gaussianCheck;
    Gtk::SpinButton gaussianSigmaSpin;
    Gtk::SpinButton gaussianRangeSpin;
};


VelocityLimitFilterDialog::VelocityLimitFilterDialog()
    : Gtk::Dialog(_("Velocity Limiting Filter"), MainWindow::instance())
{
    set_modal(false);
        
    Gtk::VBox* vbox = get_vbox();

    Gtk::HBox* hbox = Gtk::manage(new Gtk::HBox());
    onlySelectedCheck.set_label(_("Selected joints only"));
    onlySelectedCheck.set_active(false);
    hbox->pack_start(onlySelectedCheck, Gtk::PACK_SHRINK);
    vbox->pack_start(*hbox, Gtk::PACK_SHRINK);

    hbox = Gtk::manage(new Gtk::HBox());
    hbox->pack_start(*Gtk::manage(new Gtk::Label("Ratio")), Gtk::PACK_SHRINK);
    percentSpin.set_range(1.0, 100.0);
    percentSpin.set_increments(1.0, 10.0);
    percentSpin.set_value(100.0);
    hbox->pack_start(percentSpin, Gtk::PACK_SHRINK);
    hbox->pack_start(*Gtk::manage(new Gtk::Label("%")), Gtk::PACK_SHRINK);
    vbox->pack_start(*hbox, Gtk::PACK_SHRINK);
        
    hbox = Gtk::manage(new Gtk::HBox());

    pollardCheck.set_label(_("Pollard method"));
    pollardCheck.set_active(false);
    hbox->pack_start(pollardCheck, Gtk::PACK_SHRINK);

    hbox->pack_start(*Gtk::manage(new Gtk::Label(",")), Gtk::PACK_SHRINK);
        
    hbox->pack_start(*Gtk::manage(new Gtk::Label("ks")), Gtk::PACK_SHRINK);
    ksSpin.set_digits(1);
    ksSpin.set_range(0.0, 9.9);
    ksSpin.set_increments(0.1, 1.0);
    ksSpin.set_value(2.0);
    hbox->pack_start(ksSpin, Gtk::PACK_SHRINK);
        
    vbox->pack_start(*hbox, Gtk::PACK_SHRINK);

    hbox = Gtk::manage(new Gtk::HBox());

    gaussianCheck.set_label(_("Gaussian Filter"));
    gaussianCheck.set_active(true);
    hbox->pack_start(gaussianCheck, Gtk::PACK_SHRINK);

    hbox->pack_start(*Gtk::manage(new Gtk::Label("sigma")), Gtk::PACK_SHRINK);
    gaussianSigmaSpin.set_digits(1);
    gaussianSigmaSpin.set_range(0.1, 9.9);
    gaussianSigmaSpin.set_increments(0.1, 1.0);
    gaussianSigmaSpin.set_value(3.0);
    hbox->pack_start(gaussianSigmaSpin, Gtk::PACK_SHRINK);

    hbox->pack_start(*Gtk::manage(new Gtk::Label("range")), Gtk::PACK_SHRINK);
    gaussianRangeSpin.set_digits(0);
    gaussianRangeSpin.set_range(1, 99);
    gaussianRangeSpin.set_increments(1, 5);
    gaussianRangeSpin.set_value(7);
    hbox->pack_start(gaussianRangeSpin, Gtk::PACK_SHRINK);

    vbox->pack_start(*hbox, Gtk::PACK_SHRINK);

    show_all_children();
            
    add_button(_("Apply"), Gtk::RESPONSE_OK);
    add_button(_("Cancel"), Gtk::RESPONSE_CANCEL);
}


void VelocityLimitFilterDialog::on_response(int id)
{
    if(id == Gtk::RESPONSE_OK){
        ItemList<MultiValueSeqItem> items =
            ItemTreeView::mainInstance()->selectedItems<MultiValueSeqItem>();
        for(size_t i=0; i < items.size(); ++i){
            BodyItemPtr bodyItem = items[i]->findOwnerItem<BodyItem>();
            if(bodyItem){
                bool result;
                std::ostream& os = MessageView::mainInstance()->cout();

                // \todo The following code has not completely been implemented.
                if(onlySelectedCheck.get_active()){
                    auto linkSelection = LinkSelectionView::mainInstance()->getLinkSelection(bodyItem);
                    BodyPtr body = bodyItem->body();
                    double r = percentSpin.get_value() / 100.0;
                    int n = body->numJoints();
                    for(int j=0; j < n; ++j){
                        Link* joint = body->joint(j);
                        if(linkSelection[joint->index]){
                            std::cout << "joint " << joint->name() << ", limit = " << joint->uvlimit << ", ratio = " << r << endl;
                            result = applyVelocityLimitFilter2(*items[i]->seq(), j, joint->uvlimit * r);
                        }
                    }

                } else {

                    if(pollardCheck.get_active()){
                        result = applyPollardVelocityLimitFilter(
                            *items[i]->seq(), bodyItem->body(), ksSpin.get_value(), os);
                    } else {
                        result = applyVelocityLimitFilter(
                            *items[i]->seq(), bodyItem->body(), os);
                    }
                        
                    if(gaussianCheck.get_active()){
                        applyGaussianFilter(
                            *items[i]->seq(), gaussianSigmaSpin.get_value(), gaussianRangeSpin.get_value_as_int(), os);
                        result = true;
                    }
                }
                            
                if(result){
                    items[i]->notifyUpdate();
                }
            }
        }
    }

    hide();
}


class RangeLimitFilterDialog : public Gtk::Dialog
{
public:
    RangeLimitFilterDialog();
protected:
    virtual void on_response(int id);
private:
    Gtk::SpinButton limitGradSpin;
    Gtk::SpinButton edgeGradRatioSpin;
    Gtk::SpinButton marginSpin;
};

RangeLimitFilterDialog::RangeLimitFilterDialog()
    : Gtk::Dialog(_("Range Limiting Filter"), MainWindow::instance())
{
    set_modal(false);

    Gtk::VBox* vbox = get_vbox();

    Gtk::HBox* hbox = Gtk::manage(new Gtk::HBox());

    hbox->pack_start(*Gtk::manage(new Gtk::Label(_("limit grad."))), Gtk::PACK_SHRINK);
    limitGradSpin.set_digits(1);
    limitGradSpin.set_range(0.1, 0.9);
    limitGradSpin.set_increments(0.1, 0.2);
    limitGradSpin.set_value(0.4);
    hbox->pack_start(limitGradSpin, Gtk::PACK_SHRINK);

    hbox->pack_start(*Gtk::manage(new Gtk::Label(_("edge grad. ratio"))), Gtk::PACK_SHRINK);
    edgeGradRatioSpin.set_digits(1);
    edgeGradRatioSpin.set_range(0.1, 1.0);
    edgeGradRatioSpin.set_increments(0.1, 0.2);
    edgeGradRatioSpin.set_value(0.5);
    hbox->pack_start(edgeGradRatioSpin, Gtk::PACK_SHRINK);

    hbox->pack_start(*Gtk::manage(new Gtk::Label(_("margin"))), Gtk::PACK_SHRINK);
    marginSpin.set_digits(3);
    marginSpin.set_range(0.0, 9.999);
    marginSpin.set_increments(0.001, 0.1);
    marginSpin.set_value(0.0);
    hbox->pack_start(marginSpin, Gtk::PACK_SHRINK);

    vbox->pack_start(*hbox, Gtk::PACK_SHRINK);

    show_all_children();
            
    add_button(_("Apply"), Gtk::RESPONSE_OK);
    add_button(_("Cancel"), Gtk::RESPONSE_CANCEL);
}


void RangeLimitFilterDialog::on_response(int id)
{
    if(id == Gtk::RESPONSE_OK){
        ItemList<MultiValueSeqItem> items =
            ItemTreeView::mainInstance()->selectedItems<MultiValueSeqItem>();
        for(size_t i=0; i < items.size(); ++i){
            BodyItemPtr bodyItem = items[i]->findOwnerItem<BodyItem>();
            if(bodyItem){
                std::ostream& os = MessageView::mainInstance()->cout();
                applyRangeLimitFilter(*items[i]->seq(), bodyItem->body(), limitGradSpin.get_value(),
                                      edgeGradRatioSpin.get_value(), marginSpin.get_value(), os);
                items[i]->notifyUpdate();
            }
        }
    }
    hide();
}


void mergeSelectedMultiValueSeqItems()
{
    ItemList<MultiValueSeqItem> items =
        ItemTreeView::mainInstance()->selectedItems<MultiValueSeqItem>();
        
    if(!items.empty()){
            
        string name;
        ItemList<> commonPath;
        MultiValueSeqItemPtr mergedItem = new MultiValueSeqItem();
        MultiValueSeqPtr merged = mergedItem->seq();
            
        for(size_t i=0; i < items.size(); ++i){
                
            MultiValueSeqItemPtr item = items[i];
            MultiValueSeqPtr seq = item->seq();
            merged->setFrameRate(seq->frameRate());

            for(int j=0; j < seq->numParts(); ++j){
                //if(seq->isSeqValid(j)){
                if(j >= merged->numParts() || seq->numFrames() > merged->numFrames()){
                    merged->setDimension(seq->numFrames(), j+1);
                }
                MultiValueSeq::View src = seq->part(j);
                std::copy(src.begin(), src.end(), merged->part(j).begin());
                //}
            }
                
            ItemList<> path;
            for(Item* parent = item->parent(); parent; parent = parent->parent()){
                path.push_front(parent);
            }
            if(i==0){
                name = item->name();
                commonPath = path;
            } else {
                name += " + " + item->name();
                ItemList<> prev(commonPath);
                commonPath.clear();
                ItemList<>::iterator p = prev.begin();
                ItemList<>::iterator q = path.begin();
                while(p != prev.end() && q != path.end()){
                    if(*p == *q){
                        commonPath.push_back(*p);
                    }
                    ++p; ++q;
                }
            }
        }
            
        mergedItem->setName(name);
        ItemPtr parentItem = commonPath.back();
        parentItem->addChildItem(mergedItem);
    }
}
}


void cnoid::initializeFilterDialogs(ExtensionManager& ext)
{
    Gtk::Dialog* dialog;

    MenuManager& mm = ext.menuManager();
    
    mm.setPath("/Filters");

    dialog = ext.manage(new VelocityLimitFilterDialog());
    mm.addItem(_("Velocity Limiting Filter"))
        ->signal_activate().connect(sigc::mem_fun(*dialog, &Gtk::Widget::show));

    dialog = ext.manage(new RangeLimitFilterDialog());
    mm.addItem(_("Range Limiting Filter"))
        ->signal_activate().connect(sigc::mem_fun(*dialog, &Gtk::Widget::show));

    mm.addItem(_("Merge selected MultiValueSeqItems (Temporary version)"))
        ->signal_activate().connect(sigc::ptr_fun(&mergeSelectedMultiValueSeqItems));
}
