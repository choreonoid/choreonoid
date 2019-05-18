This directory contains a new implementation of RTSystemItem and RTSDiagramView.
The implementation is used by enabling ENABLE_NEW_RT_SYSTEM_ITEM_IMPLEMENTATION in the CMake configuration.
In the new implementation, the state detection of RTCs can be processed in the background thread
so that the state detection can be done without blocking the main (GUI) thread.
However, the implementation has a bug that somtimes causes the segmentation fault.
Until this bug is fixed, the implementation is stored in this directory and is not used by default.
