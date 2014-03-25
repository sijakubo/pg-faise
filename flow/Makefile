# -*- Den Contiki-Pfad hier eintragen -*-

CONTIKI =D:/Entwicklung/FAISE/contiki-2.7

DEFINES+=PROJECT_CONF_H=\"project-conf.h\"
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
SMALL=1

all: IOTest

PROJECT_SOURCEFILES += drivers/bolt_drv.c interface/bolt_int.c drivers/photosensor_drv.c drivers/extflash_drv.c interface/photosensor_int.c drivers/uart_drv.c service/uart_service.c

include $(CONTIKI)/Makefile.include

