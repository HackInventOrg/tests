QT += core
QT -= gui

TARGET = bossaqt
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

DEFINES += VERSION="\"1.2\""\

SOURCES += \
    Applet.cpp \
    bossac.cpp \
    BSDPortFactory.cpp \
    CmdOpts.cpp \
    EefcFlash.cpp \
    EfcFlash.cpp \
    Flash.cpp \
    Flasher.cpp \
    FlashFactory.cpp \
    LinuxPortFactory.cpp \
    NvmFlash.cpp \
    OSXPortFactory.cpp \
    PosixSerialPort.cpp \
    Samba.cpp \
    ShumaTechLogo.cpp \
    WordCopyApplet.cpp \
    WordCopyArm.cpp


DISTFILES += \


HEADERS += \
    Applet.h \
    BSDPortFactory.h \
    CmdOpts.h \
    Devices.h \
    Driver.h \
    EefcFlash.h \
    EfcFlash.h \
    FileError.h \
    Flash.h \
    Flasher.h \
    FlashFactory.h \
    LinuxPortFactory.h \
    NvmFlash.h \
    OSXPortFactory.h \
    PortFactory.h \
    PosixSerialPort.h \
    Samba.h \
    SerialPort.h \
    WordCopyApplet.h \
    WordCopyArm.h


