QT       += core gui widgets
QT       += network
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    src/main.cpp \
    src/main_window.cpp\
    src/ausfuehrend.cpp\
    src/Beobachter.cpp\
    src/Bildanalysator_Proxy.cpp\
    src/dialogzustand.cpp\
    src/koordinator.cpp\
    src/roboter.cpp\
    src/Subjekt.cpp\
    src/suchend.cpp\
    src/timer.cpp\
    src/verabschiedend.cpp\
    src/wartend.cpp\
    src/rosnode.cpp\
    src/udpnode.cpp\

HEADERS += \
    src/main_window.hpp\
    src/ausfuehrend.h\
    src/Beobachter.h\
    src/Bildanalysator_Proxy.h\
    src/dialogzustand.h\
    src/koordinator.hpp\
    src/roboter.h\
    src/Subjekt.h\
    src/suchend.h\
    src/timer.h\
    src/verabschiedend.h\
    src/wartend.h\
    src/rosnode.h\
    src/udpnode.hpp\

FORMS += \
    ui/main_window.ui

@
LIBS += -L/opt/ros/noetic
@
# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
