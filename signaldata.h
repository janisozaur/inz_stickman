#ifndef SIGNALDATA_H
#define SIGNALDATA_H

#include <QObject>
#include <QRectF>
#include <QVector>
#include <QMutex>
#include <QSerialPort>

#include "sample.h"
#include "samplingthread.h"

class SignalData : QObject
{
	Q_OBJECT
public:
	static SignalData &instance();

	Sample value(Marker which) const;

public slots:
	void fetchSamples();
	void start(QString portName, QPortSettings::BaudRate baudRate);

signals:
	void dataArrived();

private:
	SignalData();
	~SignalData();
	Sample mBlueSample, mYellowSample;
	mutable QMutex mMutex;
	SamplingThread mSampler;
};

#endif // SIGNALDATA_H
