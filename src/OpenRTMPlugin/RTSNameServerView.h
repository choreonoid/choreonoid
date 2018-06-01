/**
@author Shin'ichiro Nakaoka
@author Hisashi Ikari
*/

#ifndef CNOID_OPENRTM_PLUGIN_RTS_NAME_SERVER_VIEW_H_INCLUDED
#define CNOID_OPENRTM_PLUGIN_RTS_NAME_SERVER_VIEW_H_INCLUDED

#include <QtWidgets>
#include <cnoid/Dialog>
#include <cnoid/View>
#include <cnoid/CorbaUtil>
#include <cnoid/TreeWidget>
#include <cnoid/MenuManager>
#include "OpenRTMItem.h"

using namespace cnoid;

namespace cnoid {
	class RTSNameServerViewImpl;
	class RTSVItem;

	/*!
	* @brief It is a screen of RTC list.
	*/
	class RTSNameServerView : public View {
	public:
		static void initializeClass(ExtensionManager* ext);
		static RTSNameServerView* instance();

		RTSNameServerView();
		virtual ~RTSNameServerView();

		SignalProxy<void(const std::list<NamingContextHelper::ObjectInfo>&)>
			sigSelectionChanged();
		SignalProxy<void(std::string, int)> sigLocationChanged();

		std::list<NamingContextHelper::ObjectInfo> getSelection();

		//Proxy to RTSNameServerViewImpl
		const std::string getHost();
		int getPort();
		void updateView();
		void setSelection(std::string RTCname, std::string RTCfullPath);
		NamingContextHelper getNCHelper();

    protected:
        virtual void onActivated() override;
        virtual bool storeState(Archive& archive) override;
        virtual bool restoreState(const Archive& archive) override;

	private:
		RTSNameServerViewImpl * impl;
	};

	class RTSNameTreeWidget : public TreeWidget {
		Q_OBJECT

	private:
		MenuManager menuManager;

		void mouseMoveEvent(QMouseEvent* event);
		void mousePressEvent(QMouseEvent* event);

		void showIOR();
		void deleteFromView();
		void deleteFromNameService();
		void addContext();
		void addObject();

		void activateComponent();
		void deactivateComponent();
		void resetComponent();
		void finalizeComponent();
		void startExecutionContext();
		void stopExecutionContext();
	};

	enum CORBA_KIND { KIND_CATEGORY, KIND_HOST, KIND_MANAGER, KIND_MODULE, KIND_SERVER, KIND_RTC, KIND_FOLDER, KIND_OTHER };

	class RTSVItem : public QTreeWidgetItem, public RTCWrapper {
	public:
		RTSVItem();
		RTSVItem(const NamingContextHelper::ObjectInfo& info, RTC::RTObject_ptr rtc = 0);

		NamingContextHelper::ObjectInfo info_;
		CORBA_KIND kind_;
	};

	class AddContextDialog : public Dialog {
		Q_OBJECT
	public:
		AddContextDialog(RTSVItem* target);

		private Q_SLOTS:
		void okClicked();

	private:
		RTSVItem * target_;

		QLineEdit* nameEdit_;
		QComboBox* kindCombo_;
	};

	class AddObjectDialog : public Dialog {
		Q_OBJECT
	public:
		AddObjectDialog(RTSVItem* target);

		private Q_SLOTS:
		void okClicked();

	private:
		RTSVItem * target_;

		QLineEdit* nameEdit_;
		QLineEdit* kindEdit_;
		QTextEdit* iorText_;
	};
}

#endif
