/**
   @author Shin'ichiro NAKAOKA
*/

#ifndef CNOID_BASE_EXTENSION_MANAGER_H
#define CNOID_BASE_EXTENSION_MANAGER_H

#include <cnoid/Signal>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class Item;
class View;
class ToolBar;
class Archive;
class ItemManager;
class ViewManager;
class TimeSyncItemEngineManager;
class MenuManager;
class OptionManager;

class ExtensionManagerImpl;

class CNOID_EXPORT ExtensionManager
{
public:

    ExtensionManager(const std::string& moduleName, bool isPlugin);
    ExtensionManager(const std::string& moduleName, const std::string& version, bool isPlugin);
    virtual ~ExtensionManager();

    const std::string& name() const;
    const std::string& textDomain() const;

    ItemManager& itemManager();
    TimeSyncItemEngineManager& timeSyncItemEngineManger();
    ViewManager& viewManager();
    MenuManager& menuManager();
    OptionManager& optionManager();

private:

    struct CNOID_EXPORT PtrHolderBase {
        virtual ~PtrHolderBase();
    };

    // smart pointer version
    template <class PointerType> struct PtrHolder : public PtrHolderBase {
        PtrHolder(PointerType pointer) : pointer(pointer) { }
        virtual ~PtrHolder() { }
        PointerType pointer;
    };

    // raw pointer version
    template <class Object> struct PtrHolder<Object*> : public PtrHolderBase {
        PtrHolder(Object* pointer) : pointer(pointer) { }
        virtual ~PtrHolder() { delete pointer; }
        Object* pointer;
    };

    void manageSub(PtrHolderBase* holder);

public:

    void addToolBar(ToolBar* toolBar);

    template <class PointerType> PointerType manage(PointerType pointer) {
        manageSub(new PtrHolder<PointerType>(pointer));
        return pointer;
    }

    /**
       @if jp
       起動時にオブジェクトの初期生成が全て終わった時や、  
       新たにプラグインの読み込み・解放があったときなど、  
       システムで利用可能なオブジェクトの状態に更新があったときに  
       発行される 'SystemUpdated' シグナルと接続する。  
       @endif
    */
    SignalProxy<void()> sigSystemUpdated();
        
    /**
       @if jp
       'SystemUpdated' シグナルを発行する。  
	   
       シグナルの発行は本メソッド呼び出し時ではなく、後ほどイベントループ内にて行われる。  
       これにより、スロットの処理は現在処理中の他のイベントが処理された後になる。  
       また、本メソッドが同時期に複数回呼ばれた場合でも、シグナルの発行はひとつにまとめられる。  
       @endif
    */
    static void notifySystemUpdate();

    SignalProxy<void()> sigReleaseRequest();

    void setProjectArchiver(
        const std::string& name,
        boost::function<bool(Archive&)> storeFunction,
        boost::function<void(const Archive&)> restoreFunction);

    void setProjectArchiver(
        boost::function<bool(Archive&)> storeFunction,
        boost::function<void(const Archive&)> restoreFunction);
        
private:
    ExtensionManager(const ExtensionManager& org); // disable the copy constructor
        
    ExtensionManagerImpl* impl;

    friend class ExtensionManagerImpl;
};

}

#endif
