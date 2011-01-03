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

private:
	Ui::MainWindow *ui;
	GLWidget *mGLWidget;
};

#endif // MAINWINDOW_H
