/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_BUTTONS_H
#define CNOID_BASE_BUTTONS_H

#include <cnoid/Signal>
#include <QPushButton>
#include <QRadioButton>
#include <QButtonGroup>
#include <QToolButton>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT PushButton : public QPushButton
{
    Q_OBJECT

public:
    PushButton(QWidget* parent = 0);
    PushButton(const QString& text, QWidget* parent = 0);
    PushButton(const QIcon& icon, const QString& text, QWidget* parent = 0);

    SignalProxy<void()> sigClicked() {
        return sigClicked_;
    }
    SignalProxy<void(bool)> sigToggled() {
        return sigToggled_;
    }

private Q_SLOTS:
    void onClicked(bool checked);
    void onToggled(bool checked);

private:
    Signal<void()> sigClicked_;
    Signal<void(bool)> sigToggled_;

    void initialize();
};

class CNOID_EXPORT ToggleButton : public PushButton
{
public:
    ToggleButton(QWidget* parent = 0);
    ToggleButton(const QString& text, QWidget* parent = 0);
    ToggleButton(const QIcon& icon, const QString& text, QWidget* parent = 0);
};
    
class CNOID_EXPORT RadioButton : public QRadioButton
{
    Q_OBJECT

public:
    RadioButton(QWidget* parent = 0);
    RadioButton(const QString & text, QWidget* parent = 0);

    SignalProxy<void(bool)> sigToggled() {
        return sigToggled_;
    }

private Q_SLOTS:
    void onToggled(bool checked);

private:
    Signal<void(bool)> sigToggled_;

    void initialize();
};

class CNOID_EXPORT ToolButton : public QToolButton
{
    Q_OBJECT

public:
    ToolButton(QWidget* parent = 0);
    ToolButton(const QString& text, QWidget* parent = 0);
    
    SignalProxy<void()> sigClicked() {
        return sigClicked_;
    }
    SignalProxy<void(bool)> sigToggled() {
        return sigToggled_;
    }

    SignalProxy<void()> sigPressed();
    SignalProxy<void()> sigReleased();

private Q_SLOTS:
    void onClicked(bool checked);
    void onToggled(bool checked);
    void onPressed();
    void onReleased();

private:
    Signal<void()> sigClicked_;
    Signal<void(bool)> sigToggled_;
    std::unique_ptr<Signal<void()>> sigPressed_;
    std::unique_ptr<Signal<void()>> sigReleased_;
};


class CNOID_EXPORT ToggleToolButton : public ToolButton
{
public:
    ToggleToolButton(QWidget* parent = 0);
    //ToggleButton(const QString& text, QWidget* parent = 0);
    //ToggleButton(const QIcon& icon, const QString& text, QWidget* parent = 0);
};

}

#endif
