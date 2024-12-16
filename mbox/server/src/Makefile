PKGDIR ?= ../..
L4DIR  ?= $(PKGDIR)/../../..

TARGET  = bcm2835-mbox
SRC_CC  = main.cc \
          bcm2835_mbox.cc \
          client.cc \
          client_factory.cc \
          client_collection.cc \
          util.cc
SRC_DATA := mbox.lua

REQUIRES_LIBS  := libio-vbus lua++ libstdc++

include $(L4DIR)/mk/prog.mk
