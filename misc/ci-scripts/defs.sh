print_info() {
	if [ ${QUIET:-0} -eq 0 ]; then echo "INFO: $@"; fi
}
print_warn() {
	if [ ${QUIET:-0} -eq 0 ]; then echo "Warning: $@" 1>&2; fi
}
print_err() {
	if [ ${QUIET:-0} -eq 0 ]; then echo "Error: $@" 1>&2; fi
}
print_debug() {
	if [ ${DEBUG:-0} -ne 0 ]; then echo "DEBUG: $@"; fi
}
