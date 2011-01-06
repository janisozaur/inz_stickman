#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
	class MainWindow;
}

class GLWidget;

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit MainWindow(QWidget *parent = 0);
	~MainWindow();

private slots:
	void on_startPushButton_clicked();
	void dataArrived();

	void on_rightNearPushButton_clicked();

	void on_rightFarPushButton_clicked();

	void on_leftNearPushButton_clicked();

	void on_leftFarPushButton_clicked();

	void on_rightFrontPushButton_clicked();

	void on_rightZeroPushButton_clicked();

	void on_rightRightPushButton_clicked();

	void on_rightCalibratePushButton_clicked();

private:
	Ui::MainWindow *ui;
	GLWidget *mGLWidget;
	int mTimerId;

protected:
	void timerEvent(QTimerEvent *);
};

#endif // MAINWINDOW_H
