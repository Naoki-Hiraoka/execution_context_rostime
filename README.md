```
rosrun <package> <rtcComp> -o manager.modules.load_path:`pkg-config execution_context_rostime --variable prefix`/lib  -o manager.modules.preload:ROSTimePeriodicExecutionContext.so -o exec_cxt.periodic.type:ROSTimePeriodicExecutionContext -o exec_cxt.periodic.rate:10
```
