#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <QTime>
#include <QTimer>
#include <QVector3D>
#include <QMatrix4x4>
#include <QFlags>

class GLWidget : public QGLWidget
{
	Q_OBJECT
public:
	enum Calibration {None, Regular, Experimental};

	explicit GLWidget(QWidget *parent = 0);
	~GLWidget();
	void moveRight(const QVector3D &pos);
	void moveLeft(const QVector3D &pos);
	void setYellowNearPos(const QVector3D &pos);
	void setYellowFarPos(const QVector3D &pos);
	void setBlueNearPos(const QVector3D &pos);
	void setBlueFarPos(const QVector3D &pos);
	QVector3D yellowNearPos() const;
	QVector3D yellowFarPos() const;
	QVector3D blueNearPos() const;
	QVector3D blueFarPos() const;
	void calibrateRightFront(const QVector3D &pos);
	void calibrateRightRight(const QVector3D &pos);
	void calibrateRightZero(const QVector3D &pos);
	QVector3D getRightFrontPos() const;
	QVector3D getRightRightPos() const;
	QVector3D getRightZeroPos() const;
	Calibration getRightCalibration() const;
	void calibrateRightGo(bool debug = true);
	void setRightCalibration(Calibration c);
	void setLeftCalibration(Calibration c);
	void rightReset();

signals:

public slots:
	void timeout();
	void toggleDebugEnable(bool enabled);
	void setDebugInterval(int interval);
	void setDrawStickman(bool draw);
	void setDrawLeftMarker(bool draw);
	void setDrawRightMarker(bool draw);

private:
	void drawStickman();

	GLfloat *light_ambient, *light_ambient_position, *whiteDiffuseLight,
			*blackAmbientLight, *whiteSpecularLight;
	GLUquadric *mQuadric;
	int mWidth, mHeight;
	int mRotation;
	QTime mTime;
	QTimer mUpdateTimer;
	QVector3D mRightPos, mLeftPos;
	QVector3D mYellowNearPos, mYellowFarPos, mBlueNearPos, mBlueFarPos;
	QVector3D mRightFrontPos, mRightRightPos, mRightZeroPos;
	float mRightArmUpDownDegrees, mRightArmFoldDegrees, mRightArmLeftRightDegrees;
	float mLeftArmUpDownDegrees, mLeftArmFoldDegrees, mLeftArmLeftRightDegrees;
	QMatrix4x4 mRightTransform, mLeftTransform;
	int mDebugInterval;
	bool mDebugEnabled;
	Calibration mRightCalibration, mLeftCalibration;
	bool mDoDrawStickman, mDoDrawLeftMarker, mDoDrawRightMarker;

protected:
	void initializeGL();
	void paintGL();
	void resizeGL(int w, int h);
};

#endif // GLWIDGET_H
