## Dependencies
 - DynaSLAM library (libDynaSLAM): this is linked by hard code path in the CMakeLists.txt, make sure the DynaSLAM is built locally
 - Other dependencies required same as YOLO_DynaSLAM

## Other settings

Set ld path to find the shared library:
`export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/nvidia/workspace/sdcard/SP_Scheduler_Stack/YOLO-DynaSLAM/lib`