/**
@author Shin'ichiro Nakaoka
*/

#ifndef CNOID_CORBA_CORBA_UTIL_H
#define CNOID_CORBA_CORBA_UTIL_H

#include <vector>
#include <string>
#include <omniORB4/CORBA.h>
#include "exportdecl.h"

namespace cnoid {

	CNOID_EXPORT void initializeCorbaUtil(bool activatePOAManager = false, int listeningPort = -1);

	/**
	do initialization with an existing orb
	*/
	CNOID_EXPORT void initializeCorbaUtil(CORBA::ORB_ptr orb, bool activatePOAManager = false);

    CNOID_EXPORT bool isObjectAlive(CORBA::Object_ptr obj);

	CNOID_EXPORT CORBA::ORB_ptr getORB();

	class CNOID_EXPORT NamingContextHelper {
	public:
		NamingContextHelper();
		NamingContextHelper(const std::string& host, int port);

		void setLocation(const std::string& host, int port);

                bool updateConnection();

		struct ObjectPath {
			std::string id;
			std::string kind;

			ObjectPath(std::string strId) {
				this->id = strId;
			};

			ObjectPath(std::string strId, std::string strKind) {
				this->id = strId;
				this->kind = strKind;
			};
		};

		template <class T> typename T::_ptr_type findObject(const std::string& name, const std::string& kind = "") {
			ObjectPath path(name, kind);
			std::vector<ObjectPath> pathList;
			pathList.push_back(path);
			CORBA::Object_ptr obj = findObjectSub(pathList);
			if (CORBA::is_nil(obj)) {
				return T::_nil();
			} else {
                typename T::_ptr_type narrowed = T::_nil();
                if(isObjectAlive(obj)){
                    narrowed = T::_narrow(obj);
                }
				CORBA::release(obj);
				return narrowed;
			}
		}

		template <class T> typename T::_ptr_type findObject(std::vector<ObjectPath>& pathList) {
			CORBA::Object_ptr obj = findObjectSub(pathList);
			if (CORBA::is_nil(obj)) {
				return T::_nil();
			} else {
                typename T::_ptr_type narrowed = T::_nil();
                if(isObjectAlive(obj)){
                    narrowed = T::_narrow(obj);
                }
				CORBA::release(obj);
				return narrowed;
			}
		}

		CORBA::Object::_ptr_type findObject(const std::string& name, const std::string& kind = "") {
			ObjectPath path(name, kind);
			std::vector<ObjectPath> pathList;
			pathList.push_back(path);

			return findObjectSub(pathList);
		}

		CORBA::Object::_ptr_type findObject(std::vector<ObjectPath>& pathList) {
			return findObjectSub(pathList);
		}

		const std::string& host() { return host_; }
		int port() { return port_; }
		const std::string& errorMessage();

		bool isAlive(bool doRescan = true);

        /**
           \deprecated Use the cnoid::isObjectAlive function.
        */
		static bool isObjectAlive(CORBA::Object_ptr obj) {
            return cnoid::isObjectAlive(obj);
        }

		struct ObjectInfo {
			std::string id;
			std::string kind;
			bool isAlive;
			bool isContext;
			std::string ior;
			std::vector<ObjectPath> fullPath;

			const std::string getFullPath() const {
				std::string result;
				for (int index = 0; index < fullPath.size(); index++) {
					ObjectPath path = fullPath[index];
					result = result + "/" + path.id + "." + path.kind;
				}
				return result;
			}

		};

		typedef std::vector<ObjectInfo> ObjectInfoList;

		ObjectInfoList getObjectList();
		ObjectInfoList getObjectList(std::vector<ObjectPath>& pathList);

		bool bindObject(CORBA::Object_ptr object, const std::string& name);
		bool bindObject(std::vector<ObjectPath>& pathList, std::string& ior);

		void unbind(const std::string& name);
		void unbind(std::vector<ObjectPath> pathList);

		bool bind_new_context(std::vector<ObjectPath>& pathList);

		std::string getRootIOR();

	private:

		bool checkOrUpdateNamingContext();
		CORBA::Object_ptr findObjectSub(std::vector<ObjectPath>& pathList);
		void appendBindingList(CosNaming::BindingList_var& bList, ObjectInfoList& objects);
		void appendBindingList(CosNaming::BindingList_var& bList, std::vector<ObjectPath> pathList, ObjectInfoList& objects);

		CosNaming::NamingContext_var namingContext;
		std::string errorMessage_;
		std::string namingContextLocation;
		std::string host_;
		int port_;
                bool failedInLastAccessToNamingContext;
	};

	CNOID_EXPORT NamingContextHelper* getDefaultNamingContextHelper();
}


#endif
