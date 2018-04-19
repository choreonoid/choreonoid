BUILD_DIR=${TRAVIS_BUILD_DIR}
GIT_BRANCH=${TRAVIS_BRANCH}

## check and set undefined variables.
ERROR=0
[ -z "${GIT_BRANCH}" ] &&
	print_err "could not detect branch. check CI service type." &&
	ERROR=1
[ -z "${PACKAGE_NAME}" ] &&
	print_err "PACKAGE_NAME is not set." &&
	ERROR=1
[ -z "${PPA}" ] &&
	print_err "PPA is not set." &&
	ERROR=1

if [ -z "${TRAVIS_ENCRYPT_HASH}" ] && [ -z "${ENCRYPT_KEY}" ]; then
	print_err "both TRAVIS_ENCRYPT_HASH and ENCRYPT_KEY are not set."
	ERROR=1
fi

[ ${ERROR} -gt 0 ] && exit 1

## if VERSION_NUM is not set, it expects that the name of stable branches
## are "release-VERSION_NAME".
[ -z "${VERSION_NAME}" ] && \
	VERSION_NAME=$(echo ${GIT_BRANCH} | sed -e "s/^release-//")

[ -z "${TARGET_DISTROS}" ] && \
	TARGET_DISTROS="xenial trusty"

[ -z "${URGENCY}" ] && \
	URGENCY="medium"
