#-------------------------------------------------
#
# Project created by QtCreator 2011-01-02T20:30:24
#
#-------------------------------------------------

QT       += core gui opengl

TARGET = stickman
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    glwidget.cpp \
    signaldata.cpp \
    samplingthread.cpp \
    gldebugdrawer.cpp \
    gldebugfont.cpp

HEADERS  += mainwindow.h \
    glwidget.h \
    signaldata.h \
    samplingthread.h \
    sample.h \
    gldebugdrawer.h \
    gldebugfont.h

FORMS    += mainwindow.ui

LIBS += -lQtSerialPort -lBulletDynamics -lBulletCollision -lLinearMath

unix {
	LIBS += -L/usr/local/qserialport/lib/
	INCLUDEPATH += /usr/local/qserialport/include/QtSerialPort
	INCLUDEPATH += /usr/local/include/bullet
}
