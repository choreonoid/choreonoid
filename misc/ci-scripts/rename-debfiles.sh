#!/bin/bash

## Usage:
##	rename-debfiles.sh <DEBIAN_DIR> [VERSION_NAME]

set -e
SCRIPT_DIR=$(cd $(dirname $0); pwd)
source ${SCRIPT_DIR}/defs.sh

usage() {
	echo "Usage:"
	echo "	rename-debfiles.sh <DEBIAN_DIR> <VERSION_NAME>"
}

if [ $# -lt 1 ]; then
	print_err "too few arguments"
	usage; exit 1
elif [ ! -d $1 ]; then
	print_err "$1 is not directory"
	usage; exit 1
fi
DEBIAN_DIR="$1"
if [ $# -gt 1 ]; then
	VERSION_NAME="$2"
else
	## When version name is not specified, just remove %VERSION% from
	## control and rules file.
	print_info "Removing placeholdor %VERSION% in package names"
	sed -i -e "s/%VERSION%//" ${DEBIAN_DIR}/control
	sed -i -e "s/%VERSION%//" ${DEBIAN_DIR}/rules
	exit 0
fi

PACKAGES=$(grep "^Package:\ " ${DEBIAN_DIR}/control | sed -e "s/^Package:\ \+//")

print_info "Renaming files under debian"
cd ${DEBIAN_DIR}
for package in ${PACKAGES}; do
	SOURCE=$(echo ${package} | sed -e "s/%VERSION%//g")
	DEST=$(echo ${package} | sed -e "s/%VERSION%/-${VERSION_NAME}/g")

	TARGET_FILES=$(ls -1 ${SOURCE}.*)
	for target in ${TARGET_FILES}; do
		destfile=$(echo ${target} | sed -e "s/^${SOURCE}/${DEST}/")
		if [ -r ${destfile} ]; then
			print_info "${destfile} already exists"
			continue
		fi
		print_info "rename ${target} with ${destfile}"
		cp ${target} ${destfile}
	done
done

print_info "Replacing placeholdor %VERSION% in package names"
sed -i -e "s/%VERSION%/-${VERSION_NAME}/" control
sed -i -e "s/%VERSION%/-${VERSION_NAME}/" rules
