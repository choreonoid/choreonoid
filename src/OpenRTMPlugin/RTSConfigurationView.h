#ifndef CNOID_OPENRTM_PLUGIN_RTS_CONFIGURATION_VIEW_H
#define CNOID_OPENRTM_PLUGIN_RTS_CONFIGURATION_VIEW_H

#include "RTCWrapper.h"
#include <cnoid/View>
#include <cnoid/CorbaUtil>
#include <QTableWidget>
#include <cnoid/CheckBox>
#include <QStyledItemDelegate>
#include <memory>

class QRadioButton;

namespace cnoid {

class ConfigurationParam;
class RTSConfigurationView;
class RTSConfigurationViewImpl;

enum ParamMode
{
    MODE_NORMAL = 1,
    MODE_UPDATE,
    MODE_DELETE,
    MODE_INSERT,
    MODE_IGNORE
};

typedef std::shared_ptr<ConfigurationParam> ConfigurationParamPtr;


class ConfigurationParam
{
public:
    ConfigurationParam(int id, QString name, QString value)
        : id_(id), name_(name), nameOrg_(name), value_(value), valueOrg_(value), mode_(MODE_NORMAL)
    {}
    ConfigurationParam(const ConfigurationParamPtr source)
        : id_(source->id_), name_(source->name_), nameOrg_(source->nameOrg_),
        value_(source->value_), valueOrg_(source->valueOrg_), mode_(MODE_NORMAL)
    {}

    inline ParamMode getMode() const { return this->mode_; }

    inline int getId() const { return this->id_; }
    inline void setId(int value) { this->id_ = value; }

    inline QString getName() const { return this->name_; }
    inline void setName(QString value) { this->name_ = value; }

    inline QString getValue() const { return this->value_; }
    inline void setValue(QString value) { this->value_ = value; }

    inline bool isChangedName()
    {
        if (name_ != nameOrg_ || mode_ == ParamMode::MODE_INSERT) return true;
        return false;
    }
    inline bool isChangedValue()
    {
        if (value_ != valueOrg_ || mode_ == ParamMode::MODE_INSERT) return true;
        return false;
    }

    inline void setDelete()
    {
        if (mode_ == ParamMode::MODE_NORMAL || mode_ == ParamMode::MODE_UPDATE) {
            mode_ = ParamMode::MODE_DELETE;

        } else if (mode_ == ParamMode::MODE_INSERT) {
            mode_ = ParamMode::MODE_IGNORE;
        }
    }
    inline void setNew()
    {
        if (this->mode_ == ParamMode::MODE_NORMAL) {
            this->mode_ = ParamMode::MODE_INSERT;
        }
    }

private:
    ParamMode mode_;
    int id_;
    QString name_;
    QString value_;
    QString nameOrg_;
    QString valueOrg_;
};


class ConfigurationSetParam
{
public:
    ConfigurationSetParam(int id, QString name);
    int getId() const { return id_; }
    void setId(int value) { id_ = value; }
    QString getName() const { return name_; }
    void setName(QString value) { name_ = value; }
    QString getNameOrg() const { return nameOrg_; }
    void setDelete();
    void setNew();
    bool getActive() const { return active_; }
    void setActive(bool value);
    void setRadio(QRadioButton* value) { radioActive_ = value; }
    int getMode() const { return this->mode_; }
    void setChanged();
    std::vector<ConfigurationParamPtr> getConfigurationList() const { return configurationList_; }
    void addConfiguration(ConfigurationParamPtr target) { configurationList_.push_back(target); }
    void updateActive();
    bool isChangedActive() const { return (active_ != activeOrg_); }
    bool isChangedName() const { return (name_ != nameOrg_ || mode_ == ParamMode::MODE_INSERT); }

private:
    int id_;
    ParamMode mode_;
    QString name_;
    QString nameOrg_;
    bool active_;
    bool activeOrg_;
    QRadioButton* radioActive_;
    std::vector<ConfigurationParamPtr> configurationList_;
};

typedef std::shared_ptr<ConfigurationSetParam> ConfigurationSetParamPtr;


class DetailDelegate : public QStyledItemDelegate
{
public:
    DetailDelegate(RTSConfigurationViewImpl* view, QWidget *parent = 0);
    virtual void setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const;

private:
    RTSConfigurationViewImpl* view_;
};


class ConfigSetDelegate : public QStyledItemDelegate
{
public:
    ConfigSetDelegate(RTSConfigurationViewImpl* view, QWidget *parent = 0);
    virtual void setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const;

private:
    RTSConfigurationViewImpl* view_;
};


class RTSConfigurationView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static RTSConfigurationView* instance();
    RTSConfigurationView();
    ~RTSConfigurationView();
    void updateConfigurationSet();

private:
    RTSConfigurationViewImpl* impl;
};


class RTSConfigurationViewImpl : public QWidget
{
    Q_OBJECT
public:
    RTSConfigurationViewImpl(RTSConfigurationView* self);
    ~RTSConfigurationViewImpl();

    RTSConfigurationView* self;
    Connection selectionChangedConnection;

    void onItemSelectionChanged(const std::list<NamingContextHelper::ObjectInfo>& items);

    void updateConfigSet();
    void updateDetail();

    void updateConfigurationSet();

private Q_SLOTS:
    void configSetSelectionChanged();
    void activeSetChanged();

private:
    NamingContextHelper::ObjectInfo currentItem_;
    RTCWrapperPtr currentRtc_;

    QLineEdit* txtCompName_;
    QTableWidget* lstConfigSet_;
    CheckBox* chkSetDetail_;

    QLineEdit* txtConfSetName_;
    QTableWidget* lstDetail_;
    CheckBox* chkDetail_;

    std::vector<ConfigurationSetParamPtr> configSetList_;
    ConfigurationSetParamPtr currentSet_;

    void getConfigurationSet();
    void showConfigurationSetView();
    void showConfigurationView();
    ///
    void setCopyClicked();
    void setAddClicked();
    void setDeleteClicked();
    void setDetailClicked();

    void addClicked();
    void deleteClicked();
    void detailClicked();

    void applyClicked();
    void cancelClicked();
};

}
#endif
