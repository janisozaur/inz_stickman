/*
 * stickman
 * Copyright (C) 2011 Michał Janiszewski
 *
 * This work is a thesis. Due to nature of law you are obliged to
 * get a written permission from Technical University of Lodz prior to
 * using any part of this work. If you do get a permission, you are able
 * to use the work, which is thereafter licensed using GNU GPL v3 or
 * (at your option) any later.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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
	bool open(QString fileName, QPortSettings::BaudRate baudRate);
	void stop();

protected:
	void sample(double elapsed);
	void append(Sample mySample);
	void append(const QByteArray &data, double elapsed);
	void run();

signals:
	void dataArrived();
	void error(QString);

private:
	QVector<Sample> mSamples, mFilteredSamples;
	QByteArray mTempData;
	QMutex mSampleMutex, mStopMutex;
	volatile bool mRun;
	QSerialPort *mpSerport;
	int mIndex, mPrevIndex;
	QTime mTime;
};

#endif // SAMPLINGTHREAD_H
