#!/bin/bash

# Function to display usage information
usage() {
    echo "Usage: $0 SOURCE_FILE REMOTE_HOST [REMOTE_FILE]"
    echo "  SOURCE_FILE: Path to the local source file"
    echo "  REMOTE_HOST: Remote host in the format user@hostname"
    echo "  REMOTE_FILE: (Optional) Path to the remote destination file. If omitted, same as SOURCE_FILE"
    exit 1
}

# Check command-line arguments
if [ $# -lt 2 ]; then
    usage
fi

# Set parameters
SOURCE_FILE="$1"
REMOTE_HOST="$2"
REMOTE_FILE="${3:-$SOURCE_FILE}"

# Establish SSH connection and run in the background
ssh -N -f -M -S /tmp/ssh_socket "$REMOTE_HOST"

cleanup() {
    echo "Cleaning up and closing SSH connection..."
    ssh -S /tmp/ssh_socket -O exit "$REMOTE_HOST"
    exit 0
}

# Execute cleanup on script termination
trap cleanup EXIT

# Initial sync: Replace the remote file with the source file at script start
initial_sync() {
    echo "Performing initial sync..."
    scp -o "ControlPath=/tmp/ssh_socket" "$SOURCE_FILE" "$REMOTE_HOST:$REMOTE_FILE"
    if [ $? -eq 0 ]; then
        echo "Initial sync completed successfully."
    else
        echo "Error during initial sync. Exiting."
        cleanup
    fi
}

# Function to sync specified byte range
sync_range() {
    local start=$1
    local end=$2
    local length=$((end - start))

    if [ $length -gt 0 ]; then
        dd if="$SOURCE_FILE" bs=1 skip=$start count=$length 2>/dev/null | 
        ssh -S /tmp/ssh_socket "$REMOTE_HOST" "cat >> $REMOTE_FILE"
        
        return $length
    fi
    return 0
}

# Perform initial sync
initial_sync

# Record the last synced file size
last_size=$(stat -c %s "$SOURCE_FILE")

# Get current time in seconds with nanosecond precision
get_time() {
    date +%s.%N
}

start_time=$(get_time)
last_output_time=$start_time
total_synced_bytes=0
sync_count=0

while true; do
    current_time=$(get_time)
    
    # Get current file size
    current_size=$(stat -c %s "$SOURCE_FILE")

    # Sync if file size has changed
    if [ "$current_size" != "$last_size" ]; then
        if [ "$current_size" -lt "$last_size" ]; then
            # If file size decreased, replace the entire file
            scp -o "ControlPath=/tmp/ssh_socket" "$SOURCE_FILE" "$REMOTE_HOST:$REMOTE_FILE"
            echo "File size decreased. Replaced entire file. New size: $current_size bytes"
            synced_bytes=$current_size
        else
            # If file size increased, send only the newly added data
            sync_range "$last_size" "$current_size"
            synced_bytes=$?
        fi
        
        # Update statistics
        total_synced_bytes=$((total_synced_bytes + synced_bytes))
        sync_count=$((sync_count + 1))
        
        # Update the last synced size
        last_size=$current_size
    fi

    # Output statistics approximately once per second, only if there were syncs
    if (( $(echo "$current_time - $last_output_time >= 1" | bc -l) )); then
        if [ $sync_count -gt 0 ]; then
            echo "Sync count: $sync_count, Total bytes synced: $total_synced_bytes"
        fi
        last_output_time=$current_time
        sync_count=0
        total_synced_bytes=0
    fi

    # Minimal sleep to reduce CPU usage
    sleep 0.001
done
