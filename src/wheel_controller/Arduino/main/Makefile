ARDUINO_DIR = /usr/share/arduino
ARDMK_DIR = /usr/share/arduino


USER_LIB_PATH := $(realpath ../libraries)
ARDUINO_LIB_PATH := $(realpath ../libraries)

BOARD_TAG    = mega2560
MONITOR_PORT = /dev/ttyACM*
MNITOR_BAUDRATE = 57600
ARDUINO_SKETCHBOOK := $(realpath ../libraries)
CXXFLAGS_STD = -std=gnu++14

ARDUINO_LIBS += QTRSensors
ARDUINO_LIBS += qtr_msg
ARDUINO_LIBS += ros_lib
ARDUINO_LIBS += motor_controller

include $(ARDMK_DIR)/Arduino.mk
