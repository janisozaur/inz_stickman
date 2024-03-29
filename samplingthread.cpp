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

#include "samplingthread.h"

#include <QTimerEvent>
#include <QMatrix4x4>
#include <cmath>

#include <QDebug>

#ifndef Q_OS_LINUX
inline bool isnan(double x) {
	return x != x;
}
#endif


SamplingThread::SamplingThread(QObject *parent) :
	QThread(parent),
	mSamples(SAMPLES_COUNT),
	mFilteredSamples(SAMPLES_COUNT),
	mpSerport(NULL),
	mIndex(0),
	mPrevIndex(0)
{
	qDebug() << "SamplingThread ctor" << this;
}

SamplingThread::~SamplingThread()
{
	qDebug() << "SamplingThread dtor" << this;
	stop();
	wait();
	qDebug() << "finished";
}

void SamplingThread::sample(double elapsed)
{
	qint64 avail = mpSerport->bytesAvailable();
	QByteArray readData;
	readData.reserve(avail);
	//mpSerport->errorString();
	qint64 bytesRead = mpSerport->read(readData.data(), avail);
	//qDebug() << "Read" << bytesRead << "bytes of available" << avail << mpSerport->isOpen() << mpSerport->isReadable();
	if (bytesRead != avail) {
		//qWarning() << "Warning! Read" << bytesRead << "bytes instead of" <<
		//			  avail << "bytes available.";
	}
	if (bytesRead == -1) {
		qDebug() << "No further data can be read. Stopping sampler.";
		stop();
		return;
	}
	readData.resize(bytesRead);
	append(readData, elapsed);
}

bool SamplingThread::open(QString fileName, QPortSettings::BaudRate baudRate)
{
	qDebug() << "Opening" << fileName << fileName.toLatin1();
	if (mpSerport == NULL) {
		QPortSettings settings;
		settings.setBaudRate(baudRate);
		settings.setDataBits(QPortSettings::DB_8);
		settings.setFlowControl(QPortSettings::FLOW_OFF);
		settings.setParity(QPortSettings::PAR_NONE);
		settings.setStopBits(QPortSettings::STOP_1);
		qDebug() << "settings:" << settings.toString();
		mpSerport = new QSerialPort(fileName, settings);
		if (!mpSerport->open()) {
			qDebug() << "failed to open serial port" << fileName << "("
					 << mpSerport->errorString() << ")";
			emit error(QString("Failed to open serial port %1 (%2)")
					   .arg(fileName, mpSerport->errorString()));
			// cleanup
			delete mpSerport;
			// allow another attempt to open to be made
			mpSerport = NULL;
			return false;
		} else {
			qDebug() << "port" << fileName << "successfully opened";
		}
		if (!mpSerport->setCommTimeouts(QSerialPort::CtScheme_NonBlockingRead)) {
			qWarning("Cannot set communications timeout values at port %s.", qPrintable(fileName));
		}
		return true;
	} else {
		emit error(QString("Already opened serial port %1")
				   .arg(mpSerport->portName()));
		return false;
	}
}

void SamplingThread::append(Sample mySample)
{
	qDebug() << "append(Sample)";
	QMutexLocker locker(&mSampleMutex);
	mSamples.append(mySample);
	emit dataArrived();
}

void SamplingThread::append(const QByteArray &data, double elapsed)
{
	QMutexLocker locker(&mSampleMutex);
	mTempData.append(data);
	int count = 0;
	while (mTempData.size() >= 8) {
		// skip any leading malformed data
		while (mTempData.size() >= 8 && (
				(mTempData.at(0) != Yellow && mTempData.at(0) != Blue) ||
				mTempData.at(7) != (char)0xFF)
		) {
			//qDebug() << "****************** removing data";
			mTempData.remove(0, 1);
		}
		if (mTempData.size() < 8) {
			return;
		}
		// 8MHz with prescaler clk/8, premultiplied by ovfCounter
		float up = 256 * (unsigned char)mTempData.at(2) + (unsigned char)mTempData.at(1);
		float right = 256 * (unsigned char)mTempData.at(4) + (unsigned char)mTempData.at(3);
		float left = 256 * (unsigned char)mTempData.at(6) + (unsigned char)mTempData.at(5);
		Sample mySample;

		// multiplied by speed of sound in air and divided by 100cm/m
		// results in distance in cm from receiver
		mySample.left = left * 333 / 10000;
		mySample.right = right * 333 / 10000;
		mySample.up = up * 333 / 10000;
		mySample.marker = (Marker)mTempData.at(0);
		mySample.time = elapsed;

		// distance on x axis of right receiver
		float d = 33;
		// distance on x axis of up receiver
		float i = 16.5f;
		// distance on y axis of 2nd receiver (sqrt(3)/2 * d)
		float j = 28.5788383f;

		float x = (pow(mySample.left, 2) - pow(mySample.right, 2) + pow(d, 2)) / (2 * d);
		float y = ((pow(mySample.left, 2) - pow(mySample.up, 2) + pow(i, 2) + pow(j, 2)) / (2 * j)) - ((i * x) / j);
		float z = sqrt(pow(mySample.left, 2) - pow(x, 2) - pow(y, 2));

		// make the coordinate system relative to user
		mySample.x = -x;
		mySample.y = -y;
		mySample.z = -z;

		//qDebug() << mTempData.left(8).toHex();
		//qDebug() << "Sample(" << mySample.left << "," << mySample.right << "," <<
		//			mySample.up << "," << mySample.time << ")";

		mSamples[(mPrevIndex + count) % SAMPLES_COUNT] = mySample;
		mTempData.remove(0, 8);
		count++;
	}
	//qDebug() << "index:" << mIndex << "count:" << count << "previdx:" << mPrevIndex;
	for (int i = mPrevIndex; i < mPrevIndex + count; i++) {
		Sample s = mSamples.at(i % SAMPLES_COUNT);
		QVector3D v(s.x, s.y, s.z);
		QMatrix4x4 rot;
		rot.rotate(60, 0, 0, 1);
		s.filteredPos = v * rot;
		// handle c++ stupid bug that could cause modulo result to be negative
		int pIdx = (i + SAMPLES_COUNT - 2) % SAMPLES_COUNT;
		QVector3D p = mFilteredSamples[pIdx].filteredPos;
		// if any value is nan, consider previous position to be (0, 0, 0)
		if (isnan(p.x()) || isnan(p.y()) || isnan(p.z())) {
			p = QVector3D(0, 0, 0);
		}
		//s.filteredPos = v * WEIGHT + p * (1.0 - WEIGHT);
		//qDebug() << v << v * WEIGHT << p << p * (1.0 - WEIGHT) << s.filteredPos;
		mFilteredSamples[i % SAMPLES_COUNT] = s;
	}
	mPrevIndex = mIndex;
	mIndex = (mPrevIndex + count) % SAMPLES_COUNT;
	emit dataArrived();
}

QVector<Sample> SamplingThread::takeSamples()
{
	QMutexLocker locker(&mSampleMutex);
	QVector<Sample> result;
	result << mFilteredSamples.at((mIndex + SAMPLES_COUNT - 2) % SAMPLES_COUNT);
	result << mFilteredSamples.at((mIndex + SAMPLES_COUNT - 1) % SAMPLES_COUNT);
	return result;
}

void SamplingThread::run()
{
	mTime.start();
	mRun = true;
	mStopMutex.lock();
	bool run = mRun;
	mStopMutex.unlock();
	while (run) {
		sample(mTime.elapsed() / 1000.0f);
		msleep(25);
		mStopMutex.lock();
		run = mRun;
		mStopMutex.unlock();
	}
	qDebug() << "thread succesfully finished";
}

void SamplingThread::stop()
{
	mStopMutex.lock();
	mRun = false;
	mStopMutex.unlock();
}
