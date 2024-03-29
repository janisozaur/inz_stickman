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

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "glwidget.h"
#include "signaldata.h"

#include <QSerialPort>

#include <QDebug>

using namespace TNX;

MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow)
{
	ui->setupUi(this);
	ui->menuView->addAction(ui->optionsDockWidget->toggleViewAction());
	showMaximized();
	ui->displayWidget->toggleDebugEnable(ui->debugEnableCheckBox->isChecked());
	ui->displayWidget->setDebugInterval(ui->debugIntervalSpinBox->value());

	connect(&SignalData::instance(), SIGNAL(started()), this, SLOT(threadStarted()));
	connect(&SignalData::instance(), SIGNAL(finished()), this, SLOT(threadFinished()));
	connect(&SignalData::instance(), SIGNAL(error(QString)), ui->statusBar, SLOT(showMessage(QString)));

	// use map to sort values
	QMap<QString, QPortSettings::BaudRate> map;
	// values stolen from qportsettings.h
#ifdef TNX_POSIX_SERIAL_PORT
	map.insert("BAUDR_50", QPortSettings::BAUDR_50);
	map.insert("BAUDR_75", QPortSettings::BAUDR_75);
	map.insert("BAUDR_134", QPortSettings::BAUDR_134);
	map.insert("BAUDR_150", QPortSettings::BAUDR_150);
	map.insert("BAUDR_200", QPortSettings::BAUDR_200);
	map.insert("BAUDR_1800", QPortSettings::BAUDR_1800);
#endif
#ifdef Q_OS_LINUX
	map.insert("BAUDR_230400", QPortSettings::BAUDR_230400);
	map.insert("BAUDR_460800", QPortSettings::BAUDR_460800);
	map.insert("BAUDR_500000", QPortSettings::BAUDR_500000);
	map.insert("BAUDR_576000", QPortSettings::BAUDR_576000);
	map.insert("BAUDR_921600", QPortSettings::BAUDR_921600);
#endif
#ifdef TNX_WINDOWS_SERIAL_PORT
	map.insert("BAUDR_14400", QPortSettings::BAUDR_14400);
	map.insert("BAUDR_56000", QPortSettings::BAUDR_56000);
	map.insert("BAUDR_128000", QPortSettings::BAUDR_128000);
	map.insert("BAUDR_256000", QPortSettings::BAUDR_256000);
#endif
	// baud rates supported by all OSs
	map.insert("BAUDR_110", QPortSettings::BAUDR_110);
	map.insert("BAUDR_300", QPortSettings::BAUDR_300);
	map.insert("BAUDR_600", QPortSettings::BAUDR_600);
	map.insert("BAUDR_1200", QPortSettings::BAUDR_1200);
	map.insert("BAUDR_2400", QPortSettings::BAUDR_2400);
	map.insert("BAUDR_4800", QPortSettings::BAUDR_4800);
	map.insert("BAUDR_9600", QPortSettings::BAUDR_9600);
	map.insert("BAUDR_19200", QPortSettings::BAUDR_19200);
	map.insert("BAUDR_38400", QPortSettings::BAUDR_38400);
	map.insert("BAUDR_57600", QPortSettings::BAUDR_57600);
	map.insert("BAUDR_115200", QPortSettings::BAUDR_115200);

	for (int i = 0; i < map.count(); i++) {
		ui->portBaudRateComboBox->addItem(map.keys().at(i), map.values().at(i));
	}
	ui->portBaudRateComboBox->setCurrentIndex(
				ui->portBaudRateComboBox->findText("BAUDR_9600"));

#ifdef Q_OS_LINUX
	ui->portNameLineEdit->setText("/dev/ttyS0");
#endif
#ifdef Q_OS_WIN
	ui->portNameLineEdit->setText("COM1");
#endif
}

MainWindow::~MainWindow()
{
	qDebug() << "MainWindow::~MainWindow";
	delete ui->displayWidget;
	delete ui;
}

void MainWindow::on_startPushButton_clicked()
{
	QVariant baudVariant = ui->portBaudRateComboBox->itemData(
				ui->portBaudRateComboBox->currentIndex());
	QPortSettings::BaudRate baud = (QPortSettings::BaudRate)baudVariant.toInt();
	SignalData::instance().start(ui->portNameLineEdit->text(), baud);
	mTimerId = this->startTimer(25);
}

void MainWindow::timerEvent(QTimerEvent *event)
{
	if (mTimerId == event->timerId()) {
		dataArrived();
	}
}

void MainWindow::dataArrived()
{
	QVector3D posYellow = SignalData::instance().value(Yellow).filteredPos;
	//pos.setY(-pos.y());
	ui->displayWidget->moveRight(posYellow);

	QVector3D posBlue = SignalData::instance().value(Blue).filteredPos;
	ui->displayWidget->moveLeft(posBlue);
}

void MainWindow::on_rightNearPushButton_clicked()
{
	QVector3D pos = SignalData::instance().value(Yellow).filteredPos;
	ui->displayWidget->setYellowNearPos(pos);
	ui->rightNearXLcdNumber->display(pos.x());
	ui->rightNearYLcdNumber->display(pos.y());
	ui->rightNearZLcdNumber->display(pos.z());
}

void MainWindow::on_rightFarPushButton_clicked()
{
	QVector3D pos = SignalData::instance().value(Yellow).filteredPos;
	ui->displayWidget->setYellowFarPos(pos);
	ui->rightFarXLcdNumber->display(pos.x());
	ui->rightFarYLcdNumber->display(pos.y());
	ui->rightFarZLcdNumber->display(pos.z());
}

void MainWindow::on_leftNearPushButton_clicked()
{
	QVector3D pos = SignalData::instance().value(Blue).filteredPos;
	ui->displayWidget->setBlueNearPos(pos);
	ui->leftNearXLcdNumber->display(pos.x());
	ui->leftNearYLcdNumber->display(pos.y());
	ui->leftNearZLcdNumber->display(pos.z());
}

void MainWindow::on_leftFarPushButton_clicked()
{
	QVector3D pos = SignalData::instance().value(Blue).filteredPos;
	ui->displayWidget->setBlueFarPos(pos);
	ui->leftFarXLcdNumber->display(pos.x());
	ui->leftFarYLcdNumber->display(pos.y());
	ui->leftFarZLcdNumber->display(pos.z());
}

void MainWindow::on_rightFrontPushButton_clicked()
{
	QVector3D pos = SignalData::instance().value(Yellow).filteredPos;
	ui->displayWidget->calibrateRightFront(pos);
}

void MainWindow::on_rightZeroPushButton_clicked()
{
	QVector3D pos = SignalData::instance().value(Yellow).filteredPos;
	ui->displayWidget->calibrateRightZero(pos);
}

void MainWindow::on_rightRightPushButton_clicked()
{
	QVector3D pos = SignalData::instance().value(Yellow).filteredPos;
	ui->displayWidget->calibrateRightRight(pos);
}

void MainWindow::on_rightExperimentalCalibratePushButton_clicked()
{
	ui->displayWidget->calibrateRightGo();
	ui->displayWidget->setRightCalibration(GLWidget::Experimental);
}

void MainWindow::on_rightResetPushButton_clicked()
{
	ui->displayWidget->rightReset();
	ui->displayWidget->setRightCalibration(GLWidget::None);

	ui->rightFarXLcdNumber->display(0);
	ui->rightFarYLcdNumber->display(0);
	ui->rightFarZLcdNumber->display(0);

	ui->rightNearXLcdNumber->display(0);
	ui->rightNearYLcdNumber->display(0);
	ui->rightNearZLcdNumber->display(0);
}

void MainWindow::on_rightRegularCalibratePushButton_clicked()
{
	ui->displayWidget->setRightCalibration(GLWidget::Regular);
}

void MainWindow::on_benchmarkPushButton_clicked()
{
	QVector3D rightFrontPos = QVector3D(-15.968, 33.3038, -65.4952);
	QVector3D rightRightPos = QVector3D(-37.6838, -63.9283, -72.0952);
	QVector3D rightZeroPos = QVector3D(29.4606, -70.0331, -57.7565);
	QTime benchTime;
	benchTime.start();
	int times = 100000;
	for (int i = 0; i < times; i++) {
		ui->displayWidget->rightReset();
		ui->displayWidget->calibrateRightFront(rightFrontPos);
		ui->displayWidget->calibrateRightRight(rightRightPos);
		ui->displayWidget->calibrateRightZero(rightZeroPos);
		ui->displayWidget->calibrateRightGo(false);
	}
	int time = benchTime.elapsed();
	on_rightResetPushButton_clicked();
	qDebug() << "doing transform" << times << "times took" << time << "ms";
}

void MainWindow::threadStarted()
{
	ui->statusBar->showMessage("Thread started!");
}

void MainWindow::threadFinished()
{
	ui->statusBar->showMessage("Thread terminated!");
}

void MainWindow::on_leftRegularCalibratePushButton_clicked()
{
	ui->displayWidget->setLeftCalibration(GLWidget::Regular);
}
