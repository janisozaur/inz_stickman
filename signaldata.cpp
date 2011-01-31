/*
 * stickman
 * Copyright (C) 2011 Michał Janiszewski
 *
 * This work is a master thesis. Due to nature of law you are obliged to
 * get a written permission prior to using any part of this work. If you
 * do get a permission, you are able to use the work, which is thereafter
 * licensed using GNU GPL v3 or (at your option) any later.
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
	connect(&mSampler, SIGNAL(dataArrived()), SLOT(fetchSamples()));
	connect(&mSampler, SIGNAL(started()), this, SIGNAL(started()));
	connect(&mSampler, SIGNAL(finished()), this, SIGNAL(finished()));
	connect(&mSampler, SIGNAL(terminated()), this, SIGNAL(finished()));
	connect(&mSampler, SIGNAL(error(QString)), this, SIGNAL(error(QString)));
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
	if (mSampler.open(portName, baudRate)) {
		mSampler.start();
	}
}
