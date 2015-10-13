#TEMPLATE = app
TEMPLATE = lib
#CONFIG += console
#CONFIG += staticlib
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += c++14

SOURCES += \
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
        -lglog\
       -lboost_system\
       -lboost_thread\
       -lboost_program_options

INCLUDEPATH += /usr/include/opencv \
             /usr/include/opencv2
#            /home/victor/workspace/zoulu
DEFINES += DEBUG
DEFINES += OFFLINE

DISTFILES += \
    ../ugv-share/ugv.conf

unix {
#    CONFIG += staticlib
    CONFIG += dll
    target.path = /usr/local/lib
    INSTALLS += target
}
