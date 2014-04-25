/** \file
    \author Shin'ichiro Nakaoka
*/

#include "PoseSeqView.h"
#include "PoseSeqViewBase.h"
#include "PronunSymbol.h"
#include <cnoid/Body>
#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <cnoid/MenuManager>
#include <gtkmm/treeview.h>
#include <gtkmm/liststore.h>
#include <gtkmm/cellrendererspin.h>
#include <gtkmm/cellrenderercombo.h>
#include <gtkmm/treerowreference.h>
#include <gtkmm/scrolledwindow.h>
#include <gtkmm/paned.h>
#include <gtkmm/separator.h>
#include <gtkmm/menu.h>
#include <gtkmm/dialog.h>
#include <glibmm/i18n.h>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <map>
#include <cmath>
#include <string>
#include <iostream>

using namespace std;
using namespace Glib;
using namespace boost;
using namespace cnoid;

namespace {

const bool TRACE_FUNCTIONS = false;
    
inline double degree(double rad) { return (180.0 * rad / 3.14159265358979); }
inline double radian(double deg) { return (3.14159265358979 * deg / 180.0); }

inline optional<double> convertTextToDouble(const Glib::ustring& text)
{
    optional<double> v;
    try {
        v = lexical_cast<double>(text);
    }
    catch(const boost::bad_lexical_cast& ){
            
    }
    return v;
}

class MoveOperationDialog : public Gtk::Dialog
{
public:
    MoveOperationDialog();
    Gtk::Label timeSpinLabel;
    Gtk::SpinButton timeSpin;
};
}


namespace cnoid {

class PoseSeqViewImpl : public PoseSeqViewBase
{
public:
    PoseSeqViewImpl(PoseSeqView* self, ExtensionManager& ext);
    ~PoseSeqViewImpl();

    PoseSeqView* self;

    Gtk::VPaned vpaned;
    Gtk::HBox hSplitBox;
    Gtk::VSeparator separator;
    Gtk::RadioMenuItem* verticalSplitRadio;
    Gtk::RadioMenuItem* horizontalSplitRadio;

    Gtk::ScrolledWindow swLinkTreeWidget;

    Gtk::ScrolledWindow swPoseListView;
    Gtk::TreeView poseListView;
    Gtk::TreeViewColumn* timeViewColumn;
    Gtk::TreeViewColumn* labelViewColumn;
    Gtk::TreeViewColumn* actualPronunPoseViewColumn;
    Gtk::TreeViewColumn* transitionTimeViewColumn;
    Gtk::TreeModelColumn<PoseSeq::iterator> poseIterColumn;
    Gtk::TreeModel::ColumnRecord record;
    Glib::RefPtr<Gtk::ListStore> listStore;
    Glib::RefPtr<Gtk::TreeSelection> listSelection;
    sigc::connection listSelectionChangedConnection;
    Gtk::TreeIter tmpLastIter;

    typedef std::map<PoseRef*, Gtk::TreeIter> PoseRefToRowIterMap;
    PoseRefToRowIterMap poseRefToRowIterMap;

    Gtk::RadioMenuItem* normalModeRadio;
    Gtk::RadioMenuItem* humanoidModeRadio;
    Gtk::RadioMenuItem* lipsyncModeRadio;

    bool isLipSyncMode;
    Gtk::TreeModel::Path pressedPath;
    bool isPressedPathValid;
    Gtk::Menu popupMenu;

    MoveOperationDialog moveOperationDialog;

    void setupMainMenu(ExtensionManager& ext);
    void setupPopupMenu();
    void layoutOperationParts();
    void setupPoseListView();
    void updatePlacementOfPoseListViewAndLinkTreeView();
            
    void onModeMenuItemToggled(Gtk::RadioMenuItem* menuItem);
    void onModeToggled();

    void onSplitModeMenuItemToggled(Gtk::RadioMenuItem* menuItem);

    virtual void setCurrentPoseSeqItem(PoseSeqItemPtr poseSeqItem);
    void resetPoseListViewModel();
        
    void nameCellDataFunction(Gtk::CellRenderer* cell, const Gtk::TreeModel::iterator& iter);
    void actualPronunPoseCellDataFunction(Gtk::CellRenderer* cell, const Gtk::TreeModel::iterator& iter);
    void timeCellDataFunction(
        Gtk::CellRenderer* cell, const Gtk::TreeModel::iterator& iter);
    void transitionTimeCellDataFunction(
        Gtk::CellRenderer* cell, const Gtk::TreeModel::iterator& iter);
        
    virtual void onPoseInserted(PoseSeq::iterator it, bool isMoving);
    virtual void onPoseRemoving(PoseSeq::iterator it, bool isMoving);
    bool getSelectedPoseIters(vector<PoseSeq::iterator>& iters);
    void onListSelectionChanged();
    virtual bool onTimeChanged(double time);
    virtual void onInsertPoseButtonClicked();

    void onPoseTimeEdited(const Glib::ustring& path, const Glib::ustring& text);
    void onPoseNameEdited(const Glib::ustring& path, const Glib::ustring& text);
    void onPoseTransitionTimeEdited(const Glib::ustring& path, const Glib::ustring& text);

    bool onPoseListViewKeyPressEvent(GdkEventKey* event);
    bool onPoseListViewButtonPressEvent(GdkEventButton* event);

    void onCutActivated();
    void onCopyActivated();
    void onPasteActivated();
    void onMoveActivated();

    Gtk::TreeIter getListLastIter();
    bool getListLastIterCallback(const Gtk::TreeModel::iterator& iter);

    bool restoreState(const Archive& archive);
    bool storeState(Archive& archive);
};
}


PoseSeqView::PoseSeqView(ExtensionManager& ext)
{
    impl = new PoseSeqViewImpl(this, ext);
}


PoseSeqViewImpl::PoseSeqViewImpl(PoseSeqView* self, ExtensionManager& ext)
    : PoseSeqViewBase(self),
      self(self)
{
    self->set_name(N_("Pose Seq"));
    self->setDefaultLayoutArea(View::CENTER);

    setupMainMenu(ext);
    setupPopupMenu();
    layoutOperationParts();
    setupPoseListView();

    swLinkTreeWidget.set_policy(Gtk::POLICY_NEVER, Gtk::POLICY_ALWAYS);
    swLinkTreeWidget.add(linkTreeWidget);
    swLinkTreeWidget.show();

    mainVBox.pack_start(hSplitBox, Gtk::PACK_SHRINK);
    mainVBox.pack_start(vpaned);
    updatePlacementOfPoseListViewAndLinkTreeView();

    onModeToggled();
}


void PoseSeqViewImpl::setupMainMenu(ExtensionManager& ext)
{
    MenuManager& mm = ext.menuManager();
    
    mm.setPath("/Options").setPath(N_("Pose Seq View"));

    mm.requestNewRadioGroup();
    normalModeRadio = mm.addRadioItem(_("Normal mode"));
    normalModeRadio->set_active(true);
    normalModeRadio->signal_toggled().connect(
        sigc::bind(sigc::mem_fun(*this, &PoseSeqViewImpl::onModeMenuItemToggled), normalModeRadio));
    
    humanoidModeRadio = mm.addRadioItem(_("Humanoid mode"));
    humanoidModeRadio->signal_toggled().connect(
        sigc::bind(sigc::mem_fun(*this, &PoseSeqViewImpl::onModeMenuItemToggled), humanoidModeRadio));
    
    lipsyncModeRadio = mm.addRadioItem(_("Lip-sync mode"));
    lipsyncModeRadio->signal_toggled().connect(
        sigc::bind(sigc::mem_fun(*this, &PoseSeqViewImpl::onModeMenuItemToggled), lipsyncModeRadio));

    mm.addSeparator();

    mm.requestNewRadioGroup();

    verticalSplitRadio = mm.addRadioItem(_("Vertical split mode"));
    verticalSplitRadio->set_active(true);
    verticalSplitRadio->signal_toggled().connect(
        sigc::bind(sigc::mem_fun(*this, &PoseSeqViewImpl::onSplitModeMenuItemToggled), verticalSplitRadio));

    horizontalSplitRadio = mm.addRadioItem(_("Horizontal split mode"));
    horizontalSplitRadio->signal_toggled().connect(
        sigc::bind(sigc::mem_fun(*this, &PoseSeqViewImpl::onSplitModeMenuItemToggled), horizontalSplitRadio));
}


void PoseSeqViewImpl::setupPopupMenu()
{
    Gtk::Menu::MenuList& menulist = popupMenu.items();
    menulist.push_back(
        Gtk::Menu_Helpers::MenuElem(_("_Cut"), sigc::mem_fun(*this, &PoseSeqViewImpl::onCutActivated)));
    menulist.push_back(
        Gtk::Menu_Helpers::MenuElem(_("_Copy"), sigc::mem_fun(*this, &PoseSeqViewImpl::onCopyActivated)));
    menulist.push_back(
        Gtk::Menu_Helpers::MenuElem(_("_Paste"), sigc::mem_fun(*this, &PoseSeqViewImpl::onPasteActivated)));    
    menulist.push_back(
        Gtk::Menu_Helpers::MenuElem(_("_Move"), sigc::mem_fun(*this, &PoseSeqViewImpl::onMoveActivated)));    
}


void PoseSeqViewImpl::layoutOperationParts()
{
    Gtk::HBox* hbox = Gtk::manage(new Gtk::HBox());
    hbox->set_border_width(3);
    hbox->pack_start(*Gtk::manage(new Gtk::Label(_("Target Data: "))), Gtk::PACK_SHRINK);
    hbox->pack_start(currentItemLabel);
    mainVBox.pack_start(*hbox, Gtk::PACK_SHRINK);

    hbox = Gtk::manage(new Gtk::HBox());
    hbox->pack_start(insertPoseButton, Gtk::PACK_SHRINK);
    hbox->pack_start(*Gtk::manage(new Gtk::Label(_("t.time"))), Gtk::PACK_SHRINK);
    hbox->pack_start(transitionTimeSpin, Gtk::PACK_SHRINK);
    hbox->pack_start(updateButton, Gtk::PACK_SHRINK);
    hbox->pack_start(timeSyncCheck, Gtk::PACK_SHRINK);
    mainVBox.pack_start(*hbox, Gtk::PACK_SHRINK);

    mainVBox.show_all_children();
}


void PoseSeqViewImpl::setupPoseListView()
{
    record.add(poseIterColumn);

    int columnIndex = 0;

    Gtk::CellRendererSpin* timeCell = Gtk::manage(new Gtk::CellRendererSpin());
    poseListView.append_column(_("time"), *timeCell);
    timeViewColumn = poseListView.get_column(columnIndex++);
    timeViewColumn->set_cell_data_func(
        *timeCell, sigc::mem_fun(*this, &PoseSeqViewImpl::timeCellDataFunction));
    timeCell->property_adjustment() = new Gtk::Adjustment(0.0, 0.0, 999.99, 0.01, 0.1, 0);
    timeCell->property_digits() = 2;
    timeCell->property_editable() = true;
    timeCell->signal_edited().connect
        (sigc::mem_fun(*this, &PoseSeqViewImpl::onPoseTimeEdited));

    Gtk::CellRendererText* labelCell = Gtk::manage(new Gtk::CellRendererText());
    poseListView.append_column(_("label"), *labelCell);
    labelViewColumn = poseListView.get_column(columnIndex++);
    labelViewColumn->set_expand(true);
    labelViewColumn->set_cell_data_func(
        *labelCell, sigc::mem_fun(*this, &PoseSeqViewImpl::nameCellDataFunction));
    labelCell->signal_edited().connect(sigc::mem_fun(*this, &PoseSeqViewImpl::onPoseNameEdited));

    Gtk::CellRendererText* pronunCell = Gtk::manage(new Gtk::CellRendererText());
    poseListView.append_column(_("actual pose"), *pronunCell);
    actualPronunPoseViewColumn = poseListView.get_column(columnIndex++);
    actualPronunPoseViewColumn->set_cell_data_func(
        *pronunCell, sigc::mem_fun(*this, &PoseSeqViewImpl::actualPronunPoseCellDataFunction));
    
    Gtk::CellRendererSpin* transitionTimeCell = Gtk::manage(new Gtk::CellRendererSpin());
    poseListView.append_column(_("t.time"), *transitionTimeCell);
    transitionTimeViewColumn = poseListView.get_column(columnIndex++);
    transitionTimeViewColumn->set_cell_data_func(
        *transitionTimeCell, sigc::mem_fun(*this, &PoseSeqViewImpl::transitionTimeCellDataFunction));
    transitionTimeCell->property_adjustment() = new Gtk::Adjustment(0.0, 0.0, 999.99, 0.01, 0.1, 0);
    transitionTimeCell->property_digits() = 2;
    transitionTimeCell->signal_edited().connect
        (sigc::mem_fun(*this, &PoseSeqViewImpl::onPoseTransitionTimeEdited));

    //listStore = Gtk::ListStore::create(record);
    //poseListView.set_model(listStore);
    
    poseListView.set_grid_lines(Gtk::TREE_VIEW_GRID_LINES_HORIZONTAL);

    poseListView.signal_key_press_event().connect(
        sigc::mem_fun(*this, &PoseSeqViewImpl::onPoseListViewKeyPressEvent));
    poseListView.signal_button_press_event().connect(
        sigc::mem_fun(*this, &PoseSeqViewImpl::onPoseListViewButtonPressEvent), false);

    listSelection = poseListView.get_selection();
    listSelection->set_mode(Gtk::SELECTION_MULTIPLE);
    listSelectionChangedConnection = listSelection->signal_changed().connect
        (sigc::mem_fun(*this, &PoseSeqViewImpl::onListSelectionChanged));

    swPoseListView.set_policy(Gtk::POLICY_NEVER, Gtk::POLICY_ALWAYS);
    swPoseListView.add(poseListView);

    poseListView.show();
    swPoseListView.show();
}


void PoseSeqViewImpl::updatePlacementOfPoseListViewAndLinkTreeView()
{
    if(verticalSplitRadio->get_active()){
        hSplitBox.hide();
        if(swPoseListView.get_parent() == &hSplitBox){
            hSplitBox.remove(swPoseListView);
            hSplitBox.remove(separator);
            hSplitBox.remove(swLinkTreeWidget);
            separator.hide();
        }
        vpaned.pack1(swPoseListView, true, false);
        vpaned.pack2(swLinkTreeWidget, true, false);
        vpaned.show();
    } else if(horizontalSplitRadio->get_active()){
        vpaned.hide();
        if(swPoseListView.get_parent() == &vpaned){
            vpaned.remove(swPoseListView);
            vpaned.remove(swLinkTreeWidget);
        }
        hSplitBox.pack_start(swPoseListView);
        hSplitBox.pack_start(separator, Gtk::PACK_SHRINK);
        hSplitBox.pack_start(swLinkTreeWidget, Gtk::PACK_SHRINK);
        separator.show();
        hSplitBox.show();
    }
}


PoseSeqView::~PoseSeqView()
{
    delete impl;
}


PoseSeqViewImpl::~PoseSeqViewImpl()
{

}


void PoseSeqViewImpl::onModeMenuItemToggled(Gtk::RadioMenuItem* menuItem)
{
    if(menuItem->get_active()){
        onModeToggled();
    }
}


void PoseSeqViewImpl::onModeToggled()
{
    isLipSyncMode = lipsyncModeRadio->get_active();
    actualPronunPoseViewColumn->set_visible(isLipSyncMode);
    transitionTimeViewColumn->set_visible(!isLipSyncMode);
    labelViewColumn->set_title(isLipSyncMode ? _("pronunciation / label") : _("label"));

    if(!isLipSyncMode){
        swLinkTreeWidget.show();
    } else {
        swLinkTreeWidget.hide();
    }

    resetPoseListViewModel();
}


void PoseSeqViewImpl::onSplitModeMenuItemToggled(Gtk::RadioMenuItem* menuItem)
{
    if(menuItem->get_active()){
        updatePlacementOfPoseListViewAndLinkTreeView();
    }
}


void PoseSeqViewImpl::setCurrentPoseSeqItem(PoseSeqItemPtr poseSeqItem)
{
    PoseSeqViewBase::setCurrentPoseSeqItem(poseSeqItem);
    resetPoseListViewModel();
}


void PoseSeqViewImpl::resetPoseListViewModel()
{
    listSelectionChangedConnection.block();
    
    poseListView.unset_model();
    poseRefToRowIterMap.clear();

    if(seq){
        listStore = Gtk::ListStore::create(record);
 
        for(PoseSeq::iterator p = seq->begin(); p != seq->end(); ++p){
            if((!isLipSyncMode && p->get<Pose>()) || (isLipSyncMode && p->get<PronunSymbol>())){
                Gtk::TreeIter iter = listStore->append();
                Gtk::TreeRow row = *iter;
                row[poseIterColumn] = p;
                poseRefToRowIterMap[&(*p)] = iter;
            }
        }
        poseListView.set_model(listStore);
        onListSelectionChanged();
    }

    listSelectionChangedConnection.unblock();
}


void PoseSeqViewImpl::nameCellDataFunction(Gtk::CellRenderer* cell, const Gtk::TreeModel::iterator& iter)
{
    const Gtk::TreeRow& row = *iter;
    PoseSeq::iterator it = row[poseIterColumn];
    cell->set_property("text", ustring(it->name()));
    PronunSymbolPtr pronun = it->get<PronunSymbol>();
    if((!isLipSyncMode && !pronun) || (isLipSyncMode && pronun)){
        cell->set_property("editable", true);
        cell->set_property("foreground-gdk", poseListView.get_style()->get_text(Gtk::STATE_NORMAL));
    } else {
        cell->set_property("editable", false);
        cell->set_property("foreground-gdk", poseListView.get_style()->get_text(Gtk::STATE_INSENSITIVE));
    }
}


void PoseSeqViewImpl::actualPronunPoseCellDataFunction(Gtk::CellRenderer* cell, const Gtk::TreeModel::iterator& iter)
{
    const Gtk::TreeRow& row = *iter;
    PoseSeq::iterator it = row[poseIterColumn];
    PronunSymbolPtr pronun = it->get<PronunSymbol>();
    if(pronun){
        PoseUnitPtr actualPoseUnit = pronun->actualPoseUnit();
        if(actualPoseUnit){
            cell->set_property("text", actualPoseUnit->name());
        } else {
            cell->set_property("text", ustring());
        }
        //cell->set_property("foreground-gdk", poseListView.get_style()->get_text(Gtk::STATE_NORMAL));
    } else {
        cell->set_property("text", ustring());
        //cell->set_property("foreground-gdk", poseListView.get_style()->get_text(Gtk::STATE_INSENSITIVE));
    }
}


void PoseSeqViewImpl::timeCellDataFunction
(Gtk::CellRenderer* cell, const Gtk::TreeModel::iterator& iter)
{
    const Gtk::TreeRow& row = *iter;
    PoseSeq::iterator it = row[poseIterColumn];
    static format f(" % 6.2f  ");
    string text(str(f % it->time()));
    cell->set_property("text", text);
}


void PoseSeqViewImpl::transitionTimeCellDataFunction
(Gtk::CellRenderer* cell, const Gtk::TreeModel::iterator& iter)
{
    const Gtk::TreeRow& row = *iter;
    PoseSeq::iterator it = row[poseIterColumn];
    string text(str(format(" % 6.2f  ") % it->maxTransitionTime()));
    cell->set_property("text", text);
    cell->set_property("editable", true);
}


void PoseSeqViewImpl::onPoseInserted(PoseSeq::iterator it, bool isMoving)
{
    if(TRACE_FUNCTIONS){
        cout << "PoseSeqViewImpl::onPoseInserted()" << endl;
    }

    PoseSeqViewBase::onPoseInserted(it, isMoving);
    
    if((!isLipSyncMode && it->get<Pose>()) || (isLipSyncMode && it->get<PronunSymbol>())){
        
        if(TRACE_FUNCTIONS){
            cout << "PoseSeqViewImpl::onPoseInserted(): 2" << endl;
        }
        
        Gtk::TreeIter newTreeIter;

        PoseSeq::iterator next = it;
        next++;

        while(next != seq->end()){
            if((!isLipSyncMode && next->get<Pose>()) || (isLipSyncMode && next->get<PronunSymbol>())){
                break;
            }
            next++;
        }
        
        if(next != seq->end()){
            PoseRefToRowIterMap::iterator p = poseRefToRowIterMap.find(&(*next));
            if(p != poseRefToRowIterMap.end()){
                Gtk::TreeIter pos = p->second;
                newTreeIter = listStore->insert(pos);
            }
        } else {
            newTreeIter = listStore->append();
        }
        
        if(newTreeIter){
            Gtk::TreeRow row = *newTreeIter;
            row[poseIterColumn] = it;
            poseRefToRowIterMap[&(*it)] = newTreeIter;
        }
    }
}


void PoseSeqViewImpl::onPoseRemoving(PoseSeq::iterator it, bool isMoving)
{
    if(TRACE_FUNCTIONS){
        cout << "PoseSeqViewImpl::onPoseRemoving()" << endl;
    }

    PoseSeqViewBase::onPoseRemoving(it, isMoving);

    PoseRefToRowIterMap::iterator p = poseRefToRowIterMap.find(&(*it));
    if(p != poseRefToRowIterMap.end()){
        Gtk::TreeIter treeIter = p->second;
        poseRefToRowIterMap.erase(p);
        listStore->erase(treeIter);
    }
}


bool PoseSeqViewImpl::getSelectedPoseIters(vector<PoseSeq::iterator>& iters)
{
    iters.clear();
    vector<Gtk::TreeModel::Path> selected = listSelection->get_selected_rows();
    for(size_t i=0; i < selected.size(); ++i){
        const Gtk::TreeRow& row = *listStore->get_iter(selected[i]);
        iters.push_back(row[poseIterColumn]);
    }
    return !iters.empty();
}


void PoseSeqViewImpl::onListSelectionChanged()
{
    if(TRACE_FUNCTIONS){
        cout << "PoseSeqViewImpl::onListSelectionChanged()" << endl;
    }
    
    selectedPoseIters.clear();
    
    vector<Gtk::TreeModel::Path> selected = listSelection->get_selected_rows();

    if(!selected.empty()){
        for(size_t i=0; i < selected.size(); ++i){
            const Gtk::TreeModel::Row& row = *listStore->get_iter(selected[i]);
            PoseSeq::iterator p = row[poseIterColumn];
            selectedPoseIters.insert(p);
            currentPoseIter = p;
        }
    }

    updateLinkTreeModel();
    onSelectedPosesModified();

    if(!selectedPoseIters.empty()){
        currentTime = (*selectedPoseIters.begin())->time();
        if(timeSyncCheck.get_active() && selectedPoseIters.size() == 1){
            timeBar->setTime(currentTime);
        }
    }
}


bool PoseSeqViewImpl::onTimeChanged(double time)
{
    if(TRACE_FUNCTIONS){
        cout << "PoseSeqViewImpl::onTimeChanged(" << time << ")" << endl;
    }
    
    bool doContinue = false;
    
    if(seq && timeSyncCheck.get_active()){

        PoseSeq::iterator prevPoseIter = currentPoseIter;

        currentPoseIter = seq->seek(currentPoseIter, time);

        if(currentPoseIter == seq->end() || currentPoseIter->time() > time){
            if(currentPoseIter == seq->begin()){
                currentPoseIter = seq->end();
            } else {
                currentPoseIter--;
            }
        }

        if(toggleSelection(currentPoseIter, false, false)){

            if(TRACE_FUNCTIONS){
                cout << "PoseSeqViewImpl::actually do pose selection change" << endl;
            }

            listSelectionChangedConnection.block();
            listSelection->unselect_all();
            
            if(currentPoseIter != seq->end()){
                PoseRefToRowIterMap::iterator p = poseRefToRowIterMap.find(&(*currentPoseIter));
                if(p != poseRefToRowIterMap.end()){
                    Gtk::TreeIter treeIter = p->second;
                    listSelection->select(treeIter);
                    poseListView.scroll_to_row(Gtk::TreePath(treeIter));
                }
                doContinue = true;
            }
            listSelectionChangedConnection.unblock();
            
        }
    }

    currentTime = time;
    
    return false;
}


void PoseSeqViewImpl::onInsertPoseButtonClicked()
{
    if(seq){
        PoseSeq::iterator poseIter;
        
        if(isLipSyncMode){
            poseIter = insertPronunSymbol();
        } else if(currentBodyItem){
            poseIter = insertPose();
        }

        if(poseIter != seq->end()){
            Gtk::TreeIter treeIter = poseRefToRowIterMap[&(*poseIter)];
            listSelectionChangedConnection.block();
            listSelection->unselect_all();
            listSelectionChangedConnection.unblock();
            listSelection->select(treeIter);
        }
    }
}


Gtk::TreeIter PoseSeqViewImpl::getListLastIter()
{
    tmpLastIter = Gtk::TreeIter(); // clear
    listStore->foreach_iter(sigc::mem_fun(*this, &PoseSeqViewImpl::getListLastIterCallback));
    return tmpLastIter;
}


bool PoseSeqViewImpl::getListLastIterCallback(const Gtk::TreeModel::iterator& iter)
{
    tmpLastIter = iter;
    return false;
}


void PoseSeqViewImpl::onPoseTimeEdited(const Glib::ustring& path, const Glib::ustring& text)
{
    if(TRACE_FUNCTIONS){
        cout << "PoseSeqViewImpl::onPoseTimeEdited()" << endl;
    }
    
    if(currentPoseSeqItem){
        optional<double> newTime = convertTextToDouble(text);
        if(newTime){
            Gtk::TreeRow row = *listStore->get_iter(path);
            PoseSeq::iterator poseIter = row[poseIterColumn];
            seq->changeTime(poseIter, *newTime);
        }
    }
}


void PoseSeqViewImpl::onPoseNameEdited(const Glib::ustring& path, const Glib::ustring& text)
{
    if(currentPoseSeqItem){

        poseSeqConnections.block();

        Gtk::TreeIter iter = listStore->get_iter(path);
        Gtk::TreeRow row = *iter;
        PoseSeq::iterator poseIter = row[poseIterColumn];
        seq->beginPoseModification(poseIter);
        seq->rename(poseIter, text);
        seq->endPoseModification(poseIter);
        
        poseSeqConnections.unblock();
        
        if(isLipSyncMode){
            
            bool nextFound = false;
            ++poseIter;
            while(poseIter != seq->end()){
                if(poseIter->get<PronunSymbol>()){
                    nextFound = true;
                    break;
                }
                ++poseIter;
            }
            
            // begin to edit the next symbol
            if(nextFound){
                PoseRefToRowIterMap::iterator p = poseRefToRowIterMap.find(&(*poseIter));
                if(p != poseRefToRowIterMap.end()){
                    Gtk::TreeIter treeIter = p->second;
                    
                    listSelectionChangedConnection.block();
                    listSelection->unselect_all();
                    listSelection->select(treeIter);
                    Gtk::TreePath path(treeIter);
                    poseListView.scroll_to_row(path);
                    
                    listSelectionChangedConnection.unblock();

                    toggleSelection(poseIter, false, true);
                    
                    poseListView.set_cursor(path, *labelViewColumn, true);
                }
            }
        }
    }
}


void PoseSeqViewImpl::onPoseTransitionTimeEdited(const Glib::ustring& path, const Glib::ustring& text)
{
    if(currentPoseSeqItem){
        optional<double> newMaxTransitionTime = convertTextToDouble(text);
        if(newMaxTransitionTime){
            Gtk::TreeRow row = *listStore->get_iter(path);
            PoseSeq::iterator poseIter = row[poseIterColumn];
            poseSeqConnections.block();
            seq->beginPoseModification(poseIter);
            poseIter->setMaxTransitionTime(*newMaxTransitionTime);
            seq->endPoseModification(poseIter);
            doAutomaticInterpolationUpdate(); // This should be done by slots
            poseSeqConnections.unblock();
        }
    }
}


bool PoseSeqViewImpl::onPoseListViewKeyPressEvent(GdkEventKey* event)
{
    switch(event->keyval){

    case GDK_Escape:
        // see the comment in onPoseListViewKeyPressEvent()
        //listSelection->unselect_all();
        poseListView.unset_model();
        poseListView.set_model(listStore);
        return true;
    }
    
    return false;
}


bool PoseSeqViewImpl::onPoseListViewButtonPressEvent(GdkEventButton* event)
{
    if(TRACE_FUNCTIONS){
        cout << "PoseSeqViewImpl::onPoseListViewButtonPressEvent()" << endl;
    }
    
    bool doDefaultHandler = true;

    poseListView.grab_focus();

    int x, y;
    Gtk::TreeViewColumn* column = 0;
    isPressedPathValid = poseListView.get_path_at_pos(event->x, event->y, pressedPath, column, x, y);

    if(event->type == GDK_BUTTON_PRESS){
        if(!isPressedPathValid && (event->button == 1 || event->button == 3)){
            // @bug Here we want to unselect all the rows when a user presses a point not on a valid row.
            // TreeSelection::unselect_all() cannot be used for this purpose because
            // after unselect_all(), clicking a cell which was selected before doing unselect_all() results
            // in not selecting the row but immediately entering the edit mode of the cell.
            // I could not find any solution to avoid this behavior as far as unselect_all() is used.
            // (For example, setting 'false' to the 'editable' attribute of the cell before calling unselect_all()
            //  and setting 'true' to the attribute again after the call falls into the same result.
            // This behavior seems a bug of GTK+, so currently realizing unselect_all() should be
            // done by the code like the followings.
            poseListView.unset_model();
            poseListView.set_model(listStore);
            //restoreExpansionState();
        }
        if(event->button == 3){
            popupMenu.popup(event->button, event->time);
            if(listSelection->count_selected_rows() > 0){
                doDefaultHandler = false;
            }
        }
    }

    return !doDefaultHandler;
}


void PoseSeqViewImpl::onCutActivated()
{
    if(seq){
        listSelectionChangedConnection.block();
        cutSelectedPoses();
        listSelectionChangedConnection.unblock();
        onListSelectionChanged();
    }
}


void PoseSeqViewImpl::onCopyActivated()
{
    if(seq){
        copySelectedPoses();
    }
}


void PoseSeqViewImpl::onPasteActivated()
{
    if(seq){
        pasteCopiedPoses(timeBar->time());
    }
}


MoveOperationDialog::MoveOperationDialog()
    : Gtk::Dialog(_("Move Selected Poses"), true),
      timeSpinLabel(_("New time of the first pose: "))
{
    Gtk::VBox* vbox = get_vbox();
    Gtk::HBox* hbox = Gtk::manage(new Gtk::HBox());

    hbox->pack_start(timeSpinLabel, Gtk::PACK_SHRINK);

    timeSpin.set_digits(2);
    timeSpin.set_range(-999.99, 999.99);
    timeSpin.set_increments(0.01, 0.1);
    hbox->pack_start(timeSpin, Gtk::PACK_SHRINK);

    vbox->pack_start(*hbox, Gtk::PACK_SHRINK);
    
    add_button(_("Ok"), Gtk::RESPONSE_OK);
    add_button(_("Cancel"), Gtk::RESPONSE_CANCEL);

    show_all_children();
}


void PoseSeqViewImpl::onMoveActivated()
{
    if(seq){
        vector<PoseSeq::iterator> selectedPoseIters;
        if(getSelectedPoseIters(selectedPoseIters)){

            const double orgTime = selectedPoseIters.front()->time();
            moveOperationDialog.timeSpin.set_value(orgTime);

            if(moveOperationDialog.run() == Gtk::RESPONSE_OK){

                const double newTime = moveOperationDialog.timeSpin.get_value();
                const double timeDiff = newTime - orgTime;

                if(timeDiff != 0.0){
                    for(size_t i=0; i < selectedPoseIters.size(); ++i){
                        PoseSeq::iterator& iter = selectedPoseIters[i];
                        seq->changeTime(iter, iter->time() + timeDiff);
                    }
                }
            }
            moveOperationDialog.hide();
        }
    }
}


bool PoseSeqView::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool PoseSeqViewImpl::storeState(Archive& archive)
{
    if(PoseSeqViewBase::storeState(archive)){

        archive.write("mode", (isLipSyncMode ? "lipsync" : "normal"));
        archive.write("splitMode", (verticalSplitRadio->get_active() ? "vertical" : "horizontal"));
        if(verticalSplitRadio->get_active()){
            archive.write("splitPosition", vpaned.get_position());
        }
        return true;
    }
    return false;
}


bool PoseSeqView::restoreState(const Archive& archive)
{
    return impl->restoreState(archive);
}


bool PoseSeqViewImpl::restoreState(const Archive& archive)
{
    if(PoseSeqViewBase::restoreState(archive)){

        string mode = archive.get("mode", (isLipSyncMode ? "lipsync" : "normal"));
        if(mode == "normal"){
            normalModeRadio->set_active(true);
        } else if(mode == "lipsync"){
            lipsyncModeRadio->set_active(true);
        }
        string splitMode = archive.get("splitMode", (verticalSplitRadio->get_active() ? "vertical" : "horizontal"));
        if(splitMode == "vertical"){
            verticalSplitRadio->set_active(true);
            int position;
            if(archive.read("splitPosition", position)){
                vpaned.set_position(position);
            }
        } else if(splitMode == "horizontal"){
            horizontalSplitRadio->set_active(true);
        }
        return true;
    }
    return false;
}
