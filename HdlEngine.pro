TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    HdlEngine.cpp \
    ../ugv-share/config.cpp \
    HdlCorrection.cpp \
    ../ugv-share/defines.cpp

include(deployment.pri)
qtcAddDeployment()

HEADERS += \
    HdlEngine.h \
    ../ugv-share/config.h \
    ../ugv-share/defines.h \
    HdlCorrection.h

LIBS += `pkg-config opencv --cflags --libs` \
    -lglog

INCLUDEPATH += /usr/include/opencv \
             /usr/include/opencv2
CONFIG += c++14
DEFINES += DEBUG
DEFINES += OFFLINE

DISTFILES += \
    ../ugv-share/ugv.conf
