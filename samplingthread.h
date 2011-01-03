#ifndef SAMPLINGTHREAD_H
#define SAMPLINGTHREAD_H

#include <QThread>
#include <QVector>
#include <QMutexLocker>
#include <QFile>
#include <QSerialPort>
#include <QTime>

using namespace TNX;

#include "sample.h"

#define SAMPLES_COUNT 2
#define WEIGHT 1.0

class SamplingThread : public QThread
{
	Q_OBJECT
public:
	explicit SamplingThread(QObject *parent = 0);
	~SamplingThread();
	QVector<Sample> takeSamples();
	void open(QString fileName, QPortSettings::BaudRate baudRate);
	void close();

protected:
	void sample(double elapsed);
	void append(Sample mySample);
	void append(const QByteArray &data, double elapsed);
	void timerEvent(QTimerEvent *event);
	void run();

signals:
	void dataArrived();

private:
	QVector<Sample> mSamples, mFilteredSamples;
	QByteArray mTempData;
	QMutex mSampleMutex, mStopMutex;
	volatile bool mRun;
	QSerialPort *mpSerport;
	int mTimerId, mIndex, mPrevIndex;
	QTime mTime;
};

#endif // SAMPLINGTHREAD_H
