{
  "build_systems": [
   {
 "name": "List",
 "shell_cmd": "ls -l"
   },
   {
 "name": "Build with arduino-mk",
 "shell_cmd": "make -C Inductance_Capacitance_Meter",
 "working_dir": "${project_path:${folder:${file_path}}}"
   },
   {
 "name": "Upload with arduino-mk",
 "shell_cmd": "make -C Inductance_Capacitance_Meter do_upload",
 "working_dir": "${project_path:${folder:${file_path}}}"
   },
   {
 "name": "clang-format",
 "shell_cmd": "clang-format -verbose -style=file -i `find . -name '*.ino' -or -name '*.cpp'`",
 "working_dir": "${project_path:${folder:${file_path}}}"
   },
   {
 "name": "Open in Arduino IDE",
 "shell_cmd": "test -e ${file_path}/${file_base_name}.ino || ln -svf ${file_name} ${file_path}/${file_base_name}.ino;  arduino ${file_path}/${file_base_name}.ino",
 "working_dir": "${project_path:${folder:${file_path}}}"
   }
   ],
"folders": [
   {
 "path": "/home/roman/Dokumente/Sources/Arduino-Projects"
   }
   ]
}
