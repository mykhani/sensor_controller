PROGRAM=sensors_controller
EXTRA_COMPONENTS = extras/paho_mqtt_c extras/i2c
CPPFLAGS += -DCONSOLE_UART=0
include ${RTOS_ROOTDIR}/common.mk
