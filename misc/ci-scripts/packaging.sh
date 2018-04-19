#!/bin/bash

## Usage: packaging.sh [CI_SERVICE]
##
##	CI_SERVICE: travis or circle
##		    if not specified, try to detect automatically.

set -e
SCRIPT_DIR=$(cd $(dirname $0); pwd)
source ${SCRIPT_DIR}/defs.sh

if [ ${WITHOUT_PACKAGING:-0} -ne 0 ]; then
	print_info "WITHOUT_PACKAGING=${WITHOUT_PACKAGING}"
	print_info "exit without building and uploading package."
	exit 0
fi

[ $# -ge 1 ] && CI_SERVICE="$1" || CI_SERVICE=""
case "${CI_SERVICE}" in
	"travis" ) print_info "Travis CI specified";;
	"circle" ) print_info "Circle CI specified";;
	* )
		[ ! -z "${CI_SERVICE}" ] &&
			print_wran "Warning: unkown CI service specified"

		if [ ! -z "${TRAVIS_BRANCH}" ]; then
			CI_SERVICE="travis"
			print_info "Travis CI detected"
		elif [ ! -z "${CIRCLE_BRANCH}" ]; then
			CI_SERVICE="circle"
			print_info "Circle CI detected"
		else
			print_err "could not detect CI service"
			exit 1
		fi
esac

## if there is the config file, environment variables are overridden with
## setthing values in the file.  if you do not prepare config file, you
## need to specify the necessary environment variables on CI service
## (Web interface or .travis.yml/circle.yml).
if [ -r ${SCRIPT_DIR}/config ]; then
	source ${SCRIPT_DIR}/config
fi

source ${SCRIPT_DIR}/${CI_SERVICE}/environment.sh

print_debug "branch: ${GIT_BRANCH}"

PACKAGE_NAME="${PACKAGE_NAME}-${VERSION_NAME}"

print_debug "var PACKAGE_NAME: ${PACKAGE_NAME}"
print_debug "var PPA: ${PPA}"
print_debug "var TARGET_DISTROS: ${TARGET_DISTROS}"

## decrypt secret files.
[ -r ${SCRIPT_DIR}/${CI_SERVICE}/secret.tar.enc ] &&
	ENCRYPTED_FILE=${SCRIPT_DIR}/${CI_SERVICE}/secret.tar.enc ||
	ENCRYPTED_FILE=${SCRIPT_DIR}/secret.tar.enc

if [ "${CI_SERVICE}" = "travis" ] && [ ! -z "${TRAVIS_ENCRYPT_HASH}" ]; then
	ENCRYPT_KEY=$(eval echo \${encrypted_${TRAVIS_ENCRYPT_HASH}_key})
	ENCRYPT_IV=$(eval echo \${encrypted_${TRAVIS_ENCRYPT_HASH}_iv})
	openssl aes-256-cbc -K ${ENCRYPT_KEY} -iv ${ENCRYPT_IV} \
		-in ${ENCRYPTED_FILE} -out /tmp/secret.tar -d
else
	openssl aes-256-cbc -md sha256 -k ${ENCRYPT_KEY} \
		-in ${ENCRYPTED_FILE} -out /tmp/secret.tar -d
fi
tar xvf /tmp/secret.tar -C /tmp/

## prepare passphrase file.
## NOTE: write the passphrase several times, because debuild requires
## passphrase more than once.
DEB_PASSPHRASE=$(cat /tmp/passphrase)
echo -n > /tmp/passphrase
for i in $(seq 10); do
	echo "${DEB_PASSPHRASE}" >> /tmp/passphrase
done

print_info "Importing GPG key for debuild"
gpg --import /tmp/pub.key /tmp/sec.key 2>&1 | tee /tmp/gpg-output

## get Key ID of imported key
GPG_KEYID=$(grep "key\ [^:]\+: public" /tmp/gpg-output | \
	    head -1 | sed -e "s/^.*key\ \([^:]\+\):.*$/\1/")
rm /tmp/gpg-output
if [ -z "${GPG_KEYID}" ]; then
	print_err "failed to get GPG key ID"; exit 1
fi

## get one of User IDs of imported key.
GPG_UID=$(gpg --list-keys ${GPG_KEYID} | grep "^uid" | \
	  head -1 | sed -E "s/^uid\s+//")
if [ -z "${GPG_KEYID}" ]; then
	print_err "failed to get GPG user ID"; exit 1
fi

cd ${BUILD_DIR}
## get latest git log infomations for version name and changelog.
GIT_SRCREV=$(git rev-parse HEAD)
GIT_COMMIT_MSG=$(git log -1 --pretty=format:"%s" HEAD)
GIT_DATE_RFC=$(git log -1 --date=rfc --pretty=format:"%cd" HEAD)
GIT_DATE=$(git log -1 --date=format:"%Y%m%d%H%M" --pretty=format:"%cd" HEAD)

if [ -d ${BUILD_DIR}/debian ]; then
	PREPARED_DEBDIR=${BUILD_DIR}/debian
	DEB_IS_READY=true
else
	PREPARED_DEBDIR=${SCRIPT_DIR}/debian
	DEB_IS_READY=false
fi

## initialize script for launchpad deployment.
DEPLOY_LAUNCHPAD="${SCRIPT_DIR}/deploy-launchpad.sh"
echo -n > ${DEPLOY_LAUNCHPAD}

for DISTRO in ${TARGET_DISTROS}; do

	VERSION="${GIT_DATE}~${DISTRO}"
	print_info "Package: ${PACKAGE_NAME}_${VERSION}"

	cd ${BUILD_DIR}

	PACKAGING_DIR=${BUILD_DIR}/packaging-${DISTRO}
	SRCDIR=${PACKAGING_DIR}/${PACKAGE_NAME}
	mkdir -p ${SRCDIR}

	print_info "Exporting sources to ${SRCDIR}"
	git archive --format=tar HEAD | tar -x -C ${SRCDIR} -f -
	if [ ${REMOVE_SCRIPTS:-0} -ne 0 ]; then
		rm -rf ${SRCDIR}/.travis.yml
		rm -rf ${SRCDIR}/circle.yml
	fi

	if ! ${DEB_IS_READY}; then
		print_info "Copying debian directory to ${SRCDIR}"
		[ -d ${SCRIPT_DIR}/debian-${DISTRO} ] && \
			PREPARED_DEBDIR=${SCRIPT_DIR}/debian-${DISTRO}
		cp -r ${PREPARED_DEBDIR} ${SRCDIR}/debian
	fi

	cd ${SRCDIR}

	## replacing package name
	${SCRIPT_DIR}/rename-debfiles.sh ${SRCDIR}/debian ${VERSION_NAME}

	CHANGELOG=${SRCDIR}/debian/changelog
	print_info "Creating changelog from the latest git log"
	## package infomation
	echo -e "${PACKAGE_NAME} (${VERSION}) ${DISTRO}; urgency=${URGENCY}\n" > \
		${CHANGELOG}
	## log entry
	echo "  * commit ${GIT_SRCREV}" >> ${CHANGELOG}
	echo "  * ${GIT_COMMIT_MSG}" >> ${CHANGELOG}
	## author
	echo " -- ${GPG_UID}  ${GIT_DATE_RFC}" >> ${CHANGELOG}

	if [ -r ${PREPARED_DEBDIR}/changelog ]; then
		print_info "Append add existing changelog"
		echo >> ${CHANGELOG}
		cat ${PREPARED_DEBDIR}/changelog >> ${CHANGELOG}
	fi

	print_info "Building source package"
	debuild -k${GPG_KEYID} -p"gpg --no-tty --batch --passphrase-fd 0" -S \
		< /tmp/passphrase

	print_info "Append dput entry to script for launchpad deployment"
	CHANGES_FILE=${PACKAGE_NAME}_${VERSION}_source.changes
	DEPLOY_CMD="dput"
	## in debug, only do upload simulation with dput.
	[ ${SIMULATE_UPLOAD:-0} -ne 0 ] && DEPLOY_CMD="${DEPLOY_CMD} -s"
	echo "${DEPLOY_CMD} ${PPA} ${PACKAGING_DIR}/${CHANGES_FILE}" >> \
		${DEPLOY_LAUNCHPAD}
done

rm /tmp/passphrase
rm /tmp/pub.key
rm /tmp/sec.key
