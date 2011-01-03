#include "signaldata.h"

#include <QDebug>

SignalData &SignalData::instance()
{
	static SignalData valueVector;
	return valueVector;
}

SignalData::SignalData()
{
	qDebug() << "SignalData ctor" << this;
	// FIXME: should this really be connected?
	connect(&mSampler, SIGNAL(dataArrived()), SLOT(fetchSamples()));
}

SignalData::~SignalData()
{
	qDebug() << "SignalData dtor" << this;
	// TODO: stop sampling thread
}

Sample SignalData::value(Marker which) const
{
	QMutexLocker locker(&mMutex);
	//qDebug() << "asking for value of sample" << index;
	switch (which) {
		case Blue:
			return mBlueSample;
		case Yellow:
			return mYellowSample;
	}
	// shouldn't reach here
	Q_ASSERT_X(false, "SignalData::value", "marker was not set to one of supported values");
	// this return never happens, but is needed to shut up compile time warnings
	return Sample();
}

void SignalData::fetchSamples()
{
	QMutexLocker locker(&mMutex);
	//qDebug() << "fetching samples";
	QVector<Sample> newSamples = mSampler.takeSamples();
	//newSamples = newSamples.mid(qMax(0, newSamples.count() - 2));
	for (int i = 0; i < 2 && !newSamples.isEmpty(); i++) {
		Sample sample = newSamples.last();
		newSamples.erase(newSamples.end() - 1);
		switch (sample.marker) {
			case Blue:
				mBlueSample = sample;
				break;
			case Yellow:
				mYellowSample = sample;
				break;
		}
	}
	emit dataArrived();
}

void SignalData::start(QString portName, QPortSettings::BaudRate baudRate)
{
	qDebug() << "starting sampler" << QThread::currentThreadId();
	mSampler.open(portName, baudRate);
	mSampler.start();
}
