#include "glwidget.h"

#include <cmath>

#include <QDebug>

#define PI 3.14159265

inline int fuzzySign(double d)
{
	int sign;
	if (qFuzzyIsNull(d)) {
		sign = 0;
	} else if (d > 0) {
		sign = 1;
	} else {
		sign = -1;
	}
	qDebug() << "fuzzy sign of" << d << "is" << sign;
	return sign;
}

inline int sign(double d)
{
	int sign;
	if (d == 0) {
		sign = 0;
	} else if (d > 0) {
		sign = 1;
	} else {
		sign = -1;
	}
	qDebug() << "sign of" << d << "is" << sign;
	return sign;
}

GLWidget::GLWidget(QWidget *parent) :
	QGLWidget(parent),
	mQuadric(gluNewQuadric()),
	mRotation(0)
{
	qDebug() << "GLWidget ctor";
	gluQuadricNormals(mQuadric, GLU_SMOOTH);
	light_ambient = new GLfloat[4];
	light_ambient[0] = 0.5f;
	light_ambient[1] = 0.5f;
	light_ambient[2] = 0.5f;
	light_ambient[3] = 1.0;
	light_ambient_position = new GLfloat[4];
	light_ambient_position[0] = 0.0;
	light_ambient_position[1] = 0.0;
	light_ambient_position[2] = 2.0;
	light_ambient_position[3] = 1.0;
	whiteDiffuseLight = new GLfloat[4];
	whiteDiffuseLight[0] = 0.7;
	whiteDiffuseLight[1] = 0.7;
	whiteDiffuseLight[2] = 0.7;
	whiteDiffuseLight[3] = 1.0;
	blackAmbientLight = new GLfloat[3];
	blackAmbientLight[0] = 1.0;
	blackAmbientLight[1] = 0.0;
	blackAmbientLight[2] = 0.0;
	whiteSpecularLight = new GLfloat[4];
	whiteSpecularLight[0] = 0.8;
	whiteSpecularLight[1] = 0.8;
	whiteSpecularLight[2] = 0.8;
	whiteSpecularLight[3] = 1.0;
	connect(&mUpdateTimer, SIGNAL(timeout()), this, SLOT(timeout()));
	connect(&mUpdateTimer, SIGNAL(timeout()), this, SLOT(updateGL()));
}

GLWidget::~GLWidget()
{
	gluDeleteQuadric(mQuadric);
	delete light_ambient;
	delete light_ambient_position;
	delete whiteDiffuseLight;
	delete blackAmbientLight;
	delete whiteSpecularLight;
}

void GLWidget::initializeGL()
{
	qDebug() << "GLWidget::initializeGL";
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glShadeModel(GL_SMOOTH);
	glLightfv(GL_LIGHT0, GL_POSITION, light_ambient_position);
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, whiteDiffuseLight);
	glLightfv(GL_LIGHT0, GL_SPECULAR, whiteSpecularLight);
	mTime.start();
	mUpdateTimer.start(20);
	qDebug() << "GLWidget::initializeGL done";
}

void GLWidget::resizeGL(int w, int h)
{
	mWidth = w;
	mHeight = h;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glViewport(0, 0, (GLint)w, (GLint)h);
	gluPerspective(45.0f, (GLfloat)w / (GLfloat)h, 0.1f, 1000.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	qDebug() << "resize (" << w << ", " << h << ")";
}

void GLWidget::paintGL()
{
	glMatrixMode(GL_MODELVIEW);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLoadIdentity();

	gluLookAt(0, 60, 80,
			  0, 0, 0,
			  0, 1, 0);

	glPushMatrix();
	glRotatef(90, 1, 0, 0);
	//glRotatef(mRotation, 1, 0, 0);

	// torso
	gluCylinder(mQuadric, 3, 4, 9, 32, 2);

	// one leg
	glPushMatrix();
	glTranslatef(1.5, 0, 9);
	gluCylinder(mQuadric, 1, 1.3, 9, 20, 2);
	glPopMatrix();

	// other leg
	glPushMatrix();
	glTranslatef(-1.5, 0, 9);
	gluCylinder(mQuadric, 1, 1.3, 9, 20, 2);
	glPopMatrix();

	// one arm
	glPushMatrix();
	glRotatef(90, 1, 0, 0);
	if (!std::isnan(mDegrees)) {
		glRotatef(mDegrees, 1, 0, 0);
	}
	glRotatef(90, 0, 1, 0);
	glTranslatef(0, 0, 3);
	gluCylinder(mQuadric, 1, 0.9, 4.5, 20, 2);

	glPushMatrix();
	glTranslatef(0, 0, 4.5);
	glRotatef(-90, 0, 1, 0);
	gluCylinder(mQuadric, 0.9, 0.8, 4.5, 20, 2);
	glPopMatrix();

	glPopMatrix();

	// other arm
	glPushMatrix();
	glRotatef(-90, 0, 1, 0);
	glTranslatef(0, 0, 3);
	gluCylinder(mQuadric, 1, 0.9, 4.5, 20, 2);

	glPushMatrix();
	glTranslatef(0, 0, 4.5);
	glRotatef(-90, 0, 1, 0);
	gluCylinder(mQuadric, 0.9, 0.8, 4.5, 20, 2);
	glPopMatrix();

	glPopMatrix();

	// head
	glTranslatef(0, 0, -4);
	gluSphere(mQuadric, 4, 32, 32);

	glPopMatrix();

	glPushMatrix();
	glTranslatef(mPos.x(), mPos.y(), mPos.z());
	gluSphere(mQuadric, 3, 5, 5);
	glPopMatrix();

	QVector3D xAxis(5, 0, 0), yAxis(0, 5, 0), zAxis(0, 0, 5);
	xAxis = mTransform * xAxis;
	yAxis = mTransform * yAxis;
	zAxis = mTransform * zAxis;

	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glBegin(GL_LINES);
		glColor3f(1, 0, 0);
		glVertex3f(0, 0, 0);
		glVertex3f(xAxis.x(), xAxis.y(), xAxis.z());

		glColor3f(0, 1, 0);
		glVertex3f(0, 0, 0);
		glVertex3f(yAxis.x(), yAxis.y(), yAxis.z());

		glColor3f(0, 0, 1);
		glVertex3f(0, 0, 0);
		glVertex3f(zAxis.x(), zAxis.y(), zAxis.z());
	glEnd();
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
}

void GLWidget::timeout()
{
	int elapsed = mTime.restart();
	mRotation += (elapsed / (float)mUpdateTimer.interval()) * 2.0f;
	mRotation = mRotation % 360;

	float y = mPos.z() - mYellowNearPos.z();
	QVector3D pos = mPos;
	pos.setX(mYellowNearPos.x());
	float l = (pos - mYellowNearPos).length();
	mDegrees = 360 * asin(y / l) * M_1_PI / 2;
	static int count = 0;
	if (count++ == 20) {
		//qDebug() << "degrees" << mDegrees;
		count = 0;
	}
}

void GLWidget::move(const QVector3D &pos)
{
	mPos = mTransform * pos;
	static int count = 0;
	if (mDebugEnabled && count++ >= mDebugInterval) {
		qDebug() << "pos:" << pos << "transformed:" << mPos;
		count = 0;
	}
}

void GLWidget::setYellowNearPos(const QVector3D &pos)
{
	mYellowNearPos = pos;
	qDebug() << "set yellow near pos to" << pos;
}

void GLWidget::setYellowFarPos(const QVector3D &pos)
{
	mYellowFarPos = pos;
	qDebug() << "set yellow far pos to" << pos;
}

void GLWidget::setBlueNearPos(const QVector3D &pos)
{
	mBlueNearPos = pos;
	qDebug() << "set blue near pos to" << pos;
}

void GLWidget::setBlueFarPos(const QVector3D &pos)
{
	mBlueFarPos = pos;
	qDebug() << "set blue far pos to" << pos;
}

QVector3D GLWidget::yellowNearPos() const
{
	return mYellowNearPos;
}

QVector3D GLWidget::yellowFarPos() const
{
	return mYellowFarPos;
}

QVector3D GLWidget::blueNearPos() const
{
	return mBlueNearPos;
}

QVector3D GLWidget::blueFarPos() const
{
	return mBlueFarPos;
}

void GLWidget::calibrateRightFront(const QVector3D &pos)
{
	mRightFrontPos = pos;
	qDebug() << "set right front pos to" << pos;
}

void GLWidget::calibrateRightRight(const QVector3D &pos)
{
	mRightRightPos = pos;
	qDebug() << "set right right pos to" << pos;
}

void GLWidget::calibrateRightZero(const QVector3D &pos)
{
	mRightZeroPos = pos;
	qDebug() << "set right zero pos to" << pos;
}

void GLWidget::calibrateRightGo()
{
	//mRightFrontPos = QVector3D(-15.2956, 22.7808, 74.4234);
	//mRightRightPos = QVector3D(40.0147, -26.0087, 75.293);
	//mRightZeroPos = QVector3D(52.4254, 29.1463, 35.6232);
	qDebug() << "mRightFrontPos" << mRightFrontPos;
	qDebug() << "mRightRightPos" << mRightRightPos;
	qDebug() << "mRightZeroPos" << mRightZeroPos;

	float xRot, yRot;

	QVector3D d = mRightRightPos - mRightFrontPos;
	QMatrix4x4 rotationX, rotationY, tempMatrix;

	// plane's normal vector
	QVector3D n = QVector3D::crossProduct(mRightFrontPos - mRightZeroPos, mRightRightPos - mRightZeroPos);
	n.normalize();

	QVector3D nx = n;
	nx.setX(0);
	xRot = acos(nx.z() / nx.length());
	xRot *= sign(n.y()) * 360 * M_1_PI / 2;
	rotationX.rotate(xRot, 1, 0, 0);

	qDebug() << "normal:" << n;

	QVector3D rotatedN = rotationX * n;
	qDebug() << "rotated normal:" << rotatedN;

	yRot = acos(rotatedN.z() / rotatedN.length());
	yRot *= sign(rotatedN.x()) * -360 * M_1_PI / 2;
	rotationY.rotate(yRot, 0, 1, 0);

	rotatedN = rotationY * rotatedN;
	qDebug() << "rotated normal:" << rotatedN;

	mTransform = rotationY * rotationX;

	qDebug() << "rotated diagon:" << mTransform * d;

	// rotate everything to XZ plane (from XY)
	tempMatrix.rotate(90, 1, 0, 0);
	mTransform = tempMatrix * mTransform;

	qDebug() << "rotation:" << xRot << yRot;
	qDebug() << "rotated mRightFrontPos:" << mTransform * mRightFrontPos;
	qDebug() << "rotated mRightRightPos:" << mTransform * mRightRightPos;
	qDebug() << "rotated diagon:" << mTransform * d;

	d = mTransform * d;
	d.normalize();
	qDebug() << d << qFuzzyIsNull(d.y());

	QVector3D xAxis(5, 0, 0), yAxis(0, 5, 0), zAxis(0, 0, 5);
	xAxis = mTransform * xAxis;
	yAxis = mTransform * yAxis;
	zAxis = mTransform * zAxis;

	qDebug() << "new coordinate system:" << endl
			 << "\t" << xAxis << endl
			 << "\t" << yAxis << endl
			 << "\t" << zAxis << endl;
}

void GLWidget::toggleDebugEnable(bool enabled)
{
	qDebug() << "debug" << enabled;
	mDebugEnabled = enabled;
}

void GLWidget::setDebugInterval(int interval)
{
	qDebug() << "interval" << interval;
	mDebugInterval = interval;
}
