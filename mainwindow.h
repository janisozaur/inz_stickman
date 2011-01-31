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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
	class MainWindow;
}

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit MainWindow(QWidget *parent = 0);
	~MainWindow();

private slots:
	void on_startPushButton_clicked();
	void dataArrived();
	void threadStarted();
	void threadFinished();

	void on_rightNearPushButton_clicked();

	void on_rightFarPushButton_clicked();

	void on_leftNearPushButton_clicked();

	void on_leftFarPushButton_clicked();

	void on_rightFrontPushButton_clicked();

	void on_rightZeroPushButton_clicked();

	void on_rightRightPushButton_clicked();

	void on_rightExperimentalCalibratePushButton_clicked();

	void on_rightResetPushButton_clicked();

	void on_rightRegularCalibratePushButton_clicked();

	void on_benchmarkPushButton_clicked();

	void on_leftRegularCalibratePushButton_clicked();

private:
	Ui::MainWindow *ui;
	int mTimerId;

protected:
	void timerEvent(QTimerEvent *);
};

#endif // MAINWINDOW_H
