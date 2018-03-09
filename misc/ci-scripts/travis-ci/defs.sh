print_info() {
	if [ ${QUIET:-0} -eq 0 ]; then echo "$@"; fi
}

print_err() {
	if [ ${QUIET:-0} -eq 0 ]; then echo "Error: $@" 1>&2; fi
}

print_debug() {
	if [ ${DEBUG:-0} -ne 0 ]; then echo "DEBUG: $@"; fi
}

## check and set undefined variables.
if [ -z "${PACKAGE_NAME}" ]; then
	print_err "variable PACKAGE_NAME is not set."; exit 1
elif [ -z "${PPA}" ]; then
	print_err "variable PACKAGE_NAME is not set."; exit 1
elif [ -z "${ENCRYPT_ENV_HASH}" ]; then
	print_err "variable ENCRYPT_ENV_HASH is not set."; exit 1
fi
[ -z "${TARGET_DISTROS}" ] && TARGET_DISTROS="xenial trusty"
[ -z "${URGENCY}" ] && URGENCY="medium"
