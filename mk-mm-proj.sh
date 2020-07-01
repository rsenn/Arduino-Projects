out_mariamole() {
  cat <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<mm_project current="1" serial_port="" programmer="" board="Arduino Uno" includePaths="" libPaths="" srcPaths="" libs="" useCodeAutomation="0" linkPrintfVersion="0" serialPortSpeed="" isLibrary="51">
    <files>
        <file name="$1" open="no"/>
        <file name="Makefile" open="no"/>
    </files>
    <externals/>
</mm_project>
EOF
}
