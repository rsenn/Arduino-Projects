# Makefile for leonardo

BOARD := leonardo
ARDUINODIR = /opt/arduino-1.8.10/
LIBRARIES := Capacitor
LIBRARYPATH += $(HOME)/Arduino/libraries

include ~/Sources/arduino/arduino-mk/arduino.mk

CPPFLAGS += -std=c++17 -I $(LIBRARYPATH)/Capacitor/src
