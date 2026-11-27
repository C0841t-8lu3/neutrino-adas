
ifndef QCONFIG
QCONFIG=qconfig.mk
endif
include $(QCONFIG)

NAME=helloworld

EXTRA_SRCVPATH += $(PROJECT_ROOT)/src
EXTRA_INCVPATH += $(PROJECT_ROOT)/include

LIBS += socket

#This has to be included last
include $(MKFILES_ROOT)/qtargets.mk
