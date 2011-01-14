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

private:
	Ui::MainWindow *ui;
	int mTimerId;

protected:
	void timerEvent(QTimerEvent *);
};

#endif // MAINWINDOW_H
