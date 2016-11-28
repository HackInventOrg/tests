QT += core
QT -= gui

TARGET = bossaqt
CONFIG += console
CONFIG -= app_bundle

LIBS += -lsetupapi

TEMPLATE = app

DEFINES += VERSION="\"1.2\""\

SOURCES += \
    Applet.cpp \
    bossac.cpp \
    CmdOpts.cpp \
    EefcFlash.cpp \
    EfcFlash.cpp \
    Flash.cpp \
    Flasher.cpp \
    FlashFactory.cpp \    
    NvmFlash.cpp \
    Samba.cpp \
    ShumaTechLogo.cpp \
    WordCopyApplet.cpp \
    WordCopyArm.cpp \
    WinPortFactory.cpp \
    WinSerialPort.cpp


DISTFILES += \


HEADERS += \
    Applet.h \
    CmdOpts.h \
    Devices.h \
    Driver.h \
    EefcFlash.h \
    EfcFlash.h \
    FileError.h \
    Flash.h \
    Flasher.h \
    FlashFactory.h \
    NvmFlash.h \
    PortFactory.h \
    Samba.h \
    SerialPort.h \
    WordCopyApplet.h \
    WordCopyArm.h \
    WinPortFactory.h \
    WinSerialPort;h


