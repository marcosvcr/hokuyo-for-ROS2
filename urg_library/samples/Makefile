TARGET = sensor_parameter get_distance get_distance_intensity get_multiecho get_multiecho_intensity sync_time_stamp calculate_xy find_port hokuyo_visualizer

URG_LIB = ../src/liburg_c.a

include ../build_rule.mk

CFLAGS = -O2 $(INCLUDES) -I../include
LDLIBS = -lm `/bin/sh ld_wsock.sh` `/bin/sh ld_setupapi.sh`

all : $(TARGET)

clean :
	$(RM) *.o $(TARGET)

$(TARGET) : open_urg_sensor.o $(URG_LIB)

$(URG_LIB) :
	cd $(@D)/ && $(MAKE) $(@F)
