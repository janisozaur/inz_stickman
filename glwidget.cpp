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
	//mRightArmDegrees = 90;
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
	if (!std::isnan(mRightArmDegrees)) {
		glRotatef(-(90 - mRightArmDegrees), 0, 1, 0);
	}
	gluCylinder(mQuadric, 1, 0.9, 4.5, 20, 2);

	glPushMatrix();
	glTranslatef(0, 0, 4.5);
	//glRotatef(-90, 0, 1, 0);
	if (!std::isnan(mRightArmDegrees)) {
		glRotatef(-2 * mRightArmDegrees, 0, 1, 0);
	}
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
	xAxis = mRightTransform * xAxis;
	yAxis = mRightTransform * yAxis;
	zAxis = mRightTransform * zAxis;
	QVector3D zero = mRightTransform * QVector3D();

	glDisable(GL_LIGHTING);

	/*glBegin(GL_TRIANGLES); {
		QVector3D temp;
		glColor3f(1, 0, 0);
		temp = mTransform * mRightRightPos * 5;
		glVertex3f(temp.x(), temp.y(), temp.z());

		glColor3f(0, 1, 0);
		temp = mTransform * mRightFrontPos * 5;
		glVertex3f(temp.x(), temp.y(), temp.z());

		glColor3f(0, 0, 1);
		temp = mTransform * mRightZeroPos * 5;
		glVertex3f(temp.x(), temp.y(), temp.z());
	} glEnd();*/

	glDisable(GL_DEPTH_TEST);

	glBegin(GL_LINES);
		glColor3f(1, 0, 0);
		glVertex3f(zero.x(), zero.y(), zero.z());
		glVertex3f(xAxis.x(), xAxis.y(), xAxis.z());

		glColor3f(0, 1, 0);
		glVertex3f(zero.x(), zero.y(), zero.z());
		glVertex3f(yAxis.x(), yAxis.y(), yAxis.z());

		glColor3f(0, 0, 1);
		glVertex3f(zero.x(), zero.y(), zero.z());
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

}

void GLWidget::move(const QVector3D &pos)
{
	switch (mRightCalibration) {
		case Regular:
			{
				// FIXME: this should use Y axis and is the current state only
				// for testing
				float y = pos.z() - mYellowNearPos.z();
				float l = (pos - mYellowNearPos).length();
				mDegrees = 360 * -acos(y / l) * M_1_PI / 2;
				mDegrees = 90 - mDegrees + 180;
				static int count = 0;
				float armLength = (mYellowFarPos - mYellowNearPos).length();
				mRightArmDegrees = 360 * acos(l / armLength) * M_1_PI / 2;
				mRightArmDegrees = qBound(0.0f, mRightArmDegrees, 90.0f);
				if (mDebugEnabled && count++ >= mDebugInterval) {
					qDebug() << "y:" << y << "l:" << l << "length" << armLength;
					qDebug() << "degrees" << mDegrees << "arm degrees" << mRightArmDegrees;
					count = 0;
				}
			}
		// fall-through
		case None:
			mPos = pos;
			break;
		case Experimental:
			mPos = mRightTransform * pos;
			break;
	}
	static int count = 0;
	if (mDebugEnabled && count++ >= mDebugInterval) {
		qDebug() << "pos:" << pos << "transformed:" << mRightTransform * pos << "inverted:" << mRightTransform.inverted() * pos;
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
	//mRightFrontPos = QVector3D(-15.968, 33.3038, -65.4952);
	//mRightRightPos = QVector3D(-37.6838, -63.9283, -72.0952);
	//mRightZeroPos = QVector3D(29.4606, -70.0331, -57.7565);
	qDebug() << "mRightFrontPos" << mRightFrontPos;
	qDebug() << "mRightRightPos" << mRightRightPos;
	qDebug() << "mRightZeroPos" << mRightZeroPos;

	float xRot, yRot;

	QVector3D d = mRightRightPos - mRightFrontPos;
	QMatrix4x4 rotationX, rotationY, tempMatrix;

	// plane's normal vector
	QVector3D n = QVector3D::crossProduct(mRightFrontPos - mRightZeroPos, mRightRightPos - mRightZeroPos);
	n.normalize();
	qDebug() << "normal:" << n;

	QVector3D nx = n;
	nx.setX(0);
	xRot = acos(nx.z() / nx.length());
	xRot *= sign(n.y()) * 360 * M_1_PI / 2;
	rotationX.rotate(xRot, 1, 0, 0);
	qDebug() << "xRot:" << xRot;

	QVector3D rotatedN = rotationX * n;
	qDebug() << "rotated normal:" << rotatedN;

	yRot = acos(rotatedN.z() / rotatedN.length());
	yRot *= sign(rotatedN.x()) * -360 * M_1_PI / 2;
	rotationY.rotate(yRot, 0, 1, 0);
	qDebug() << "yRot:" << yRot;

	rotatedN = rotationY * rotatedN;
	qDebug() << "rotated normal:" << rotatedN;

	mRightTransform = rotationY * rotationX;

	qDebug() << "rotated diagon:" << mRightTransform * d;

	// rotate everything to XZ plane (from XY)
	tempMatrix.rotate(90, 1, 0, 0);
	mRightTransform = tempMatrix * mRightTransform;

	qDebug() << "rotation:" << xRot << yRot;
	qDebug() << "rotated mRightFrontPos:" << mRightTransform * mRightFrontPos;
	qDebug() << "rotated mRightRightPos:" << mRightTransform * mRightRightPos;
	qDebug() << "rotated mRightZeroPos:"  << mRightTransform * mRightZeroPos;
	qDebug() << "*** diagon:" << d << "rotated:" << mRightTransform * d;

	d.normalize();
	qreal cosVal = (mRightTransform * d).x();
	if (d.x() <= 0) {
		if (d.z() <= 0) {
			// rr.x <= rf.x && rr.z <= rf.z
			// 45 + alfa
			yRot = 45 + acos(fabs(cosVal)) * 360 * M_1_PI / 2;
		} else {
			// rr.x <= rf.x && rr.z > rf.z
			// 45 - alfa
			yRot = 45 - acos(fabs(cosVal)) * 360 * M_1_PI / 2;
		}
	} else {
		if (d.z() <= 0) {
			// rr.x > rf.x && rr.z <= rf.z
			// alfa - 135
			yRot = acos(cosVal) * 360 * M_1_PI / 2 - 135;
		} else {
			// rr.x > rf.x && rr.z > rf.z
			// 225 - alfa
			yRot = 225 - acos(cosVal) * 360 * M_1_PI / 2;
		}
	}
	qDebug() << "rotated diagon:" << d << mRightTransform * d;
	tempMatrix = QMatrix4x4();
	tempMatrix.rotate(yRot, 0, 1, 0);
	mRightTransform = tempMatrix * mRightTransform;

	d = mRightTransform * d;

	qDebug() << "rotation:" << xRot << yRot;
	qDebug() << "rotated mRightFrontPos:" << mRightTransform * mRightFrontPos;
	qDebug() << "rotated mRightRightPos:" << mRightTransform * mRightRightPos;
	qDebug() << "rotated mRightZeroPos:"  << mRightTransform * mRightZeroPos;
	qDebug() << "rotated diagon:" << d;
	qDebug() << yRot << d << qFuzzyIsNull(d.y());


	QVector3D xAxis(5, 0, 0), yAxis(0, 5, 0), zAxis(0, 0, 5);
	xAxis = mRightTransform * xAxis;
	yAxis = mRightTransform * yAxis;
	zAxis = mRightTransform * zAxis;

	qDebug() << "new coordinate system:" << endl
			 << "\t" << xAxis << endl
			 << "\t" << yAxis << endl
			 << "\t" << zAxis << endl;

	// move coordinate system origin to the new position based on a square
	// diagon rr - rf
	QVector3D zero = mRightTransform * mRightFrontPos;
	zero.setX((mRightTransform * mRightRightPos).x());
	QVector3D translation = -zero;
	QMatrix4x4 translationMatrix;
	translationMatrix.translate(translation);
	mRightTransform = translationMatrix * mRightTransform;

	if (mRightTransform.determinant() == 0) {
		qDebug() << "FATAL! matrix not invertible!";
	}
	//mTransform = mTransform.inverted();

	QList<QVector3D> list;
	list << QVector3D(-16.113, -99.127, -68.146);
	list << QVector3D(-37.684, -63.928, -72.095);
	list << QVector3D(-74.407, -4.004, -78.818);
	list << QVector3D(-15.968, 33.304, -65.495);
	list << QVector3D(20.755, -26.620, -58.772);
	list << QVector3D(42.327, -61.819, -54.823);
	list << QVector3D(29.461, -70.033, -57.756);

	for (int i = 0; i < list.count(); i++) {
		qDebug() << "i:" << i << list.at(i) << mRightTransform * list.at(i) << mRightTransform.inverted() * list.at(i);
	}

	qDebug() << "rotated mRightFrontPos:" << mRightFrontPos << mRightTransform * mRightFrontPos << mRightTransform.inverted() * mRightTransform * mRightFrontPos;
	qDebug() << "rotated mRightRightPos:" << mRightRightPos << mRightTransform * mRightRightPos << mRightTransform.inverted() * mRightTransform * mRightRightPos;
	qDebug() << "rotated mRightZeroPos:"  << mRightZeroPos  << mRightTransform * mRightZeroPos  << mRightTransform.inverted() * mRightTransform * mRightZeroPos;
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

void GLWidget::setRightCalibration(Calibration c)
{
	mRightCalibration = c;
}

void GLWidget::setLeftCalibration(Calibration c)
{
	mLeftCalibration = c;
}

void GLWidget::rightReset()
{
	mRightTransform = QMatrix4x4();
	mDegrees = 0;
}
