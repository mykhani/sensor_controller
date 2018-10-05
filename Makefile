PROGRAM=sensors_controller
EXTRA_COMPONENTS = extras/paho_mqtt_c extras/i2c extras/sht3x
CPPFLAGS += -DCONSOLE_UART=0
include ${RTOS_ROOTDIR}/common.mk
