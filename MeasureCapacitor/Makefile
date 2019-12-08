# Makefile for leonardo

BOARD := leonardo
ARDUINODIR = /opt/arduino-1.8.10/
LIBRARIES := Capacitor
LIBRARYPATH += $(HOME)/Arduino/libraries

include ../build/arduino-mk/arduino.mk

CPPFLAGS += -std=c++1y -I $(LIBRARYPATH)/Capacitor/src
