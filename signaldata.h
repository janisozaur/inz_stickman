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

#ifndef SIGNALDATA_H
#define SIGNALDATA_H

#include <QObject>
#include <QRectF>
#include <QVector>
#include <QMutex>
#include <QSerialPort>

#include "sample.h"
#include "samplingthread.h"

class SignalData : public QObject
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
	void started();
	void finished();
	void error(QString);

private:
	SignalData();
	~SignalData();
	Sample mBlueSample, mYellowSample;
	mutable QMutex mMutex;
	SamplingThread mSampler;
};

#endif // SIGNALDATA_H
