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
	mGLWidget = new GLWidget(this);
	ui->verticalLayout->addWidget(mGLWidget);

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
	ui->portNameLineEdit->setText("/dev/ttyUSB0");
#endif
}

MainWindow::~MainWindow()
{
	delete mGLWidget;
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
	QVector3D pos = SignalData::instance().value(Yellow).filteredPos;
	//pos.setY(-pos.y());
	mGLWidget->move(pos);
	static int count = 0;
	if (count++ == 20) {
		//qDebug() << "pos:" << pos;
		count = 0;
	}
}

void MainWindow::on_rightNearPushButton_clicked()
{
	QVector3D pos = SignalData::instance().value(Yellow).filteredPos;
	mGLWidget->setYellowNearPos(pos);
	ui->rightNearXLcdNumber->display(pos.x());
	ui->rightNearYLcdNumber->display(pos.y());
	ui->rightNearZLcdNumber->display(pos.z());
}

void MainWindow::on_rightFarPushButton_clicked()
{
	QVector3D pos = SignalData::instance().value(Yellow).filteredPos;
	mGLWidget->setYellowFarPos(pos);
	ui->rightFarXLcdNumber->display(pos.x());
	ui->rightFarYLcdNumber->display(pos.y());
	ui->rightFarZLcdNumber->display(pos.z());
}

void MainWindow::on_leftNearPushButton_clicked()
{
	QVector3D pos = SignalData::instance().value(Blue).filteredPos;
	mGLWidget->setBlueNearPos(pos);
	ui->leftNearXLcdNumber->display(pos.x());
	ui->leftNearYLcdNumber->display(pos.y());
	ui->leftNearZLcdNumber->display(pos.z());
}

void MainWindow::on_leftFarPushButton_clicked()
{
	QVector3D pos = SignalData::instance().value(Blue).filteredPos;
	mGLWidget->setBlueFarPos(pos);
	ui->leftFarXLcdNumber->display(pos.x());
	ui->leftFarYLcdNumber->display(pos.y());
	ui->leftFarZLcdNumber->display(pos.z());
}

void MainWindow::on_rightFrontPushButton_clicked()
{
	QVector3D pos = SignalData::instance().value(Yellow).filteredPos;
	mGLWidget->calibrateRightFront(pos);
}

void MainWindow::on_rightZeroPushButton_clicked()
{
	QVector3D pos = SignalData::instance().value(Yellow).filteredPos;
	mGLWidget->calibrateRightZero(pos);
}

void MainWindow::on_rightRightPushButton_clicked()
{
	QVector3D pos = SignalData::instance().value(Yellow).filteredPos;
	mGLWidget->calibrateRightRight(pos);
}

void MainWindow::on_rightCalibratePushButton_clicked()
{
	mGLWidget->calibrateRightGo();
}
