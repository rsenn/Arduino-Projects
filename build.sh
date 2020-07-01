#!/bin/sh
set -x
set -e
ARDUINO_DIR=/opt/arduino-1.8.12

builddir=build/uno
mkdir -p $builddir

$ARDUINO_DIR/arduino-builder -dump-prefs -logger=machine -hardware $ARDUINO_DIR/hardware -hardware /home/roman/.arduino15/packages -tools $ARDUINO_DIR/tools-builder -tools $ARDUINO_DIR/hardware/tools/avr -tools /home/roman/.arduino15/packages -built-in-libraries $ARDUINO_DIR/libraries -libraries /home/roman/Arduino/libraries -fqbn=arduino:avr:uno -vid-pid=0X2341_0X0001 -ide-version=10812 -build-path $builddir -warnings=more -prefs=build.warn_data_percentage=75 -prefs=runtime.tools.avr-gcc.path=$ARDUINO_DIR/hardware/tools/avr -prefs=runtime.tools.avr-gcc-7.3.0-atmel3.6.1-arduino5.path=$ARDUINO_DIR/hardware/tools/avr -prefs=runtime.tools.arduinoOTA.path=$ARDUINO_DIR/hardware/tools/avr -prefs=runtime.tools.arduinoOTA-1.3.0.path=$ARDUINO_DIR/hardware/tools/avr -prefs=runtime.tools.avrdude.path=$ARDUINO_DIR/hardware/tools/avr -prefs=runtime.tools.avrdude-6.3.0-arduino17.path=$ARDUINO_DIR/hardware/tools/avr -verbose $PWD/Inductance_Capacitance_Meter/Inductance_Capacitance_Meter.ino
$ARDUINO_DIR/arduino-builder -compile -logger=machine -hardware $ARDUINO_DIR/hardware -hardware /home/roman/.arduino15/packages -tools $ARDUINO_DIR/tools-builder -tools $ARDUINO_DIR/hardware/tools/avr -tools /home/roman/.arduino15/packages -built-in-libraries $ARDUINO_DIR/libraries -libraries /home/roman/Arduino/libraries -fqbn=arduino:avr:uno -vid-pid=0X2341_0X0001 -ide-version=10812 -build-path $builddir -warnings=more -prefs=build.warn_data_percentage=75 -prefs=runtime.tools.avr-gcc.path=$ARDUINO_DIR/hardware/tools/avr -prefs=runtime.tools.avr-gcc-7.3.0-atmel3.6.1-arduino5.path=$ARDUINO_DIR/hardware/tools/avr -prefs=runtime.tools.arduinoOTA.path=$ARDUINO_DIR/hardware/tools/avr -prefs=runtime.tools.arduinoOTA-1.3.0.path=$ARDUINO_DIR/hardware/tools/avr -prefs=runtime.tools.avrdude.path=$ARDUINO_DIR/hardware/tools/avr -prefs=runtime.tools.avrdude-6.3.0-arduino17.path=$ARDUINO_DIR/hardware/tools/avr -verbose $PWD/Inductance_Capacitance_Meter/Inductance_Capacitance_Meter.ino
$ARDUINO_DIR/hardware/tools/avr/bin/avrdude -C$ARDUINO_DIR/hardware/tools/avr/etc/avrdude.conf -v -patmega328p -carduino -P/dev/ttyACM0 -b115200 -D -Uflash:w:./build/uno/Inductance_Capacitance_Meter.ino.hex

