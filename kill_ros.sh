#!/bin/bash

# ANSI color codes
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'  # No Color

# Get all ROS 2-related PIDs
PIDS=($(ps aux | grep -E 'ros|rviz|rosbag' | grep -v grep | awk '{print $2}'))

if [ ${#PIDS[@]} -eq 0 ]; then
  echo "No ROS 2 processes found."
  exit 0
fi

echo "Found ${#PIDS[@]} ROS 2 process(es) to kill:"
echo "[${PIDS[*]}]"
echo "" 

# Kill each process and record success/failure
SUCCESS_COUNT=0
FAIL_COUNT=0

for pid in "${PIDS[@]}"; do
  if kill -9 "$pid" 2>/dev/null; then
    echo -e "${GREEN}✓ Successfully killed PID $pid${NC}"
    ((SUCCESS_COUNT++))
  else
    echo -e "${RED}✗ Failed to kill PID $pid (permission denied or already dead)${NC}"
    ((FAIL_COUNT++))
  fi
done

# Summary
echo ""
echo "Summary: ${SUCCESS_COUNT} succeeded, ${FAIL_COUNT} failed."

# Exit status
if [ $FAIL_COUNT -eq 0 ]; then
  exit 0  # All succeeded
else
  exit 1  # Some failed
fi