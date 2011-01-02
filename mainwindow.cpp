#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "glwidget.h"

MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow)
{
	ui->setupUi(this);
	mGLWidget = new GLWidget(this);
	ui->verticalLayout->addWidget(mGLWidget);
}

MainWindow::~MainWindow()
{
	delete mGLWidget;
	delete ui;
}
