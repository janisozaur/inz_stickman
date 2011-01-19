#include "glwidget.h"
#include "gldebugdrawer.h"

#include <cmath>

#include <QDebug>

#define PI 3.14159265
#define ARM_LENGTH 4.5

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
	//qDebug() << "fuzzy sign of" << d << "is" << sign;
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
	//qDebug() << "sign of" << d << "is" << sign;
	return sign;
}

GLWidget::GLWidget(QWidget *parent) :
	QGLWidget(parent),
	mQuadric(gluNewQuadric()),
	mRotation(0),
	mRightArmLeftRightDegrees(0),
	mLeftArmLeftRightDegrees(180),
	mDoDrawStickman(false),
	mDoDrawLeftMarker(false),
	mDoDrawRightMarker(false),

	// physics
	mBroadphase(new btDbvtBroadphase()),
	mCollisionConfiguration(new btDefaultCollisionConfiguration()),
	mDispatcher(new btCollisionDispatcher(mCollisionConfiguration)),
	mSolver(new btSequentialImpulseConstraintSolver),
	mDynamicsWorld(new btDiscreteDynamicsWorld(mDispatcher, mBroadphase,
			mSolver, mCollisionConfiguration)),
	mLeftHandMotionState(new btDefaultMotionState(
			btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)))),
	mRightHandMotionState(new btDefaultMotionState(
			btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)))),
	mSphereShape(new btSphereShape(1)),

	// physics debug
	mDebugLevel(1),
	mDebugDrawer(new GLDebugDrawer),

	mDefaultContactProcessingThreshold(BT_LARGE_FLOAT)
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

	connect(&mUpdateTimer, SIGNAL(timeout()), this, SLOT(updateGL()));

	// left hand physics
	mLeftHandRigidBodyCI = new btRigidBody::btRigidBodyConstructionInfo(0,
									mLeftHandMotionState, mSphereShape);
	mLeftHandRigidBody = new btRigidBody(*mLeftHandRigidBodyCI);
	mLeftHandRigidBody->setCollisionFlags(
									mLeftHandRigidBody->getCollisionFlags() |
									btCollisionObject::CF_KINEMATIC_OBJECT);
	mLeftHandRigidBody->setActivationState(DISABLE_DEACTIVATION);
	mDynamicsWorld->addRigidBody(mLeftHandRigidBody);

	// right hand physics
	mRightHandRigidBodyCI = new btRigidBody::btRigidBodyConstructionInfo(0,
									mRightHandMotionState, mSphereShape);
	mRightHandRigidBody = new btRigidBody(*mRightHandRigidBodyCI);
	mRightHandRigidBody->setCollisionFlags(
									mRightHandRigidBody->getCollisionFlags() |
									btCollisionObject::CF_KINEMATIC_OBJECT);
	mRightHandRigidBody->setActivationState(DISABLE_DEACTIVATION);
	mDynamicsWorld->addRigidBody(mLeftHandRigidBody);

	mDynamicsWorld->setDebugDrawer(mDebugDrawer);

	{
		// create a universal joint using generic 6DOF constraint
		// create two rigid bodies
		// static bodyA (parent) on top:
		btCollisionShape *shape = new btBoxShape(btVector3(1, 1, 1));
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(btVector3(btScalar(0.), btScalar(4.), btScalar(-5.)));
		btRigidBody *pBodyA = localCreateRigidBody(0.0, tr, shape);
		pBodyA->setActivationState(DISABLE_DEACTIVATION);
		// dynamic bodyB (child) below it :
		tr.setIdentity();
		tr.setOrigin(btVector3(btScalar(0.), btScalar(0.), btScalar(-5.)));
		btRigidBody *pBodyB = localCreateRigidBody(1.0, tr, shape);
		pBodyB->setActivationState(DISABLE_DEACTIVATION);
		// add some (arbitrary) data to build constraint frames
		btVector3 parentAxis(1.f, 0.f, 0.f);
		btVector3 childAxis(0.f, 0.f, 1.f);
		btVector3 anchor(0.f, 2.f, 0.f);

		btUniversalConstraint *pUniv = new btUniversalConstraint(*pBodyA, *pBodyB, anchor, parentAxis, childAxis);
		pUniv->setLowerLimit(-SIMD_HALF_PI * 0.5f, -SIMD_HALF_PI * 0.5f);
		pUniv->setUpperLimit(SIMD_HALF_PI * 0.5f,  SIMD_HALF_PI * 0.5f);
		// add constraint to world
		mDynamicsWorld->addConstraint(pUniv, true);
		// draw constraint frames and limits for debugging
		pUniv->setDbgDrawSize(btScalar(5.f));
	}
	setDebugLevel(mDebugLevel);
	connect(&mPhysicsTimer, SIGNAL(timeout()), this, SLOT(timeout()));
	mPhysicsTimer.start(15);
	mPhysicsTime.start();
}

GLWidget::~GLWidget()
{
	gluDeleteQuadric(mQuadric);
	delete [] light_ambient;
	delete [] light_ambient_position;
	delete [] whiteDiffuseLight;
	delete [] blackAmbientLight;
	delete [] whiteSpecularLight;

	delete mRightHandMotionState;
	delete mLeftHandMotionState;
	delete mSphereShape;
	delete mBroadphase;
	delete mCollisionConfiguration;
	delete mDispatcher;
	delete mSolver;
	delete mDynamicsWorld;
	delete mDebugDrawer;
}

void GLWidget::timeout()
{
	int elapsed = mPhysicsTime.restart();
	float coef = float(elapsed) / mPhysicsTimer.interval();
	mDynamicsWorld->stepSimulation((float)elapsed / 1000.f, 5);
}

btRigidBody *GLWidget::localCreateRigidBody(float mass, const btTransform &startTransform, btCollisionShape *shape)
{
	btAssert(!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
	if (isDynamic) {
		shape->calculateLocalInertia(mass, localInertia);
	}

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState *myMotionState = new btDefaultMotionState(startTransform);

	btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);

	btRigidBody *body = new btRigidBody(cInfo);
	body->setContactProcessingThreshold(mDefaultContactProcessingThreshold);

	mDynamicsWorld->addRigidBody(body);

	return body;
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

void GLWidget::drawStickman()
{
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

		// right arm
		glPushMatrix();
			glRotatef(90, 1, 0, 0);
			glTranslatef(3, 0, 0);
			glRotatef(90 - mRightArmLeftRightDegrees, 0, 1, 0);
			if (!std::isnan(mRightArmUpDownDegrees)) {
				glRotatef(mRightArmUpDownDegrees, 1, 0, 0);
			}
			glRotatef(90, 0, 1, 0);
			if (!std::isnan(mRightArmFoldDegrees)) {
				glRotatef(-(90 - mRightArmFoldDegrees), 0, 1, 0);
			}
			gluCylinder(mQuadric, 1, 0.9, ARM_LENGTH, 20, 2);

			glPushMatrix();
				glTranslatef(0, 0, ARM_LENGTH);
				//glRotatef(-90, 0, 1, 0);
				if (!std::isnan(mRightArmFoldDegrees)) {
					glRotatef(-2 * mRightArmFoldDegrees, 0, 1, 0);
				}
				gluCylinder(mQuadric, 0.9, 0.8, ARM_LENGTH, 20, 2);
			glPopMatrix();

		glPopMatrix();

		// left arm
		glPushMatrix();
			glRotatef(-90, 1, 0, 0);
			glTranslatef(-3, 0, 0);
			glRotatef(90, 0, 1, 0);
			glRotatef(mLeftArmLeftRightDegrees, 0, 1, 0);
			if (!std::isnan(mLeftArmUpDownDegrees)) {
				glRotatef(-mLeftArmUpDownDegrees, 1, 0, 0);
			}
			if (!std::isnan(mLeftArmFoldDegrees)) {
				glRotatef(mLeftArmFoldDegrees, 0, 1, 0);
			}
			gluCylinder(mQuadric, 1, 0.9, ARM_LENGTH, 20, 2);

			glPushMatrix();
				glTranslatef(0, 0, ARM_LENGTH);
				//glRotatef(-90, 0, 1, 0);
				if (!std::isnan(mLeftArmFoldDegrees)) {
					glRotatef(-2 * mLeftArmFoldDegrees, 0, 1, 0);
				}
				gluCylinder(mQuadric, 0.9, 0.8, ARM_LENGTH, 20, 2);
			glPopMatrix();

		glPopMatrix();

		// head
		glTranslatef(0, 0, -4);
		//gluSphere(mQuadric, 4, 32, 32);

	glPopMatrix();
}

void GLWidget::setDrawStickman(bool draw)
{
	mDoDrawStickman = draw;
}

void GLWidget::setDrawRightMarker(bool draw)
{
	mDoDrawRightMarker = draw;
}

void GLWidget::setDrawLeftMarker(bool draw)
{
	mDoDrawLeftMarker = draw;
}

void GLWidget::paintGL()
{
	glMatrixMode(GL_MODELVIEW);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLoadIdentity();

	gluLookAt(0, 60, 80,
			  0, 0, 0,
			  0, 1, 0);

	if (mDoDrawStickman) {
		drawStickman();
	}

	if (mDoDrawLeftMarker) {
		glPushMatrix();
		glTranslatef(mLeftPos.x(), mLeftPos.y(), mLeftPos.z());
		gluSphere(mQuadric, 3, 5, 5);
		glPopMatrix();
	}

	if (mDoDrawRightMarker) {
		glPushMatrix();
		glTranslatef(mRightPos.x(), mRightPos.y(), mRightPos.z());
		gluSphere(mQuadric, 3, 5, 5);
		glPopMatrix();
	}

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

	/*glBegin(GL_LINES);
		glColor3f(1, 0, 0);
		glVertex3f(zero.x(), zero.y(), zero.z());
		glVertex3f(xAxis.x(), xAxis.y(), xAxis.z());

		glColor3f(0, 1, 0);
		glVertex3f(zero.x(), zero.y(), zero.z());
		glVertex3f(yAxis.x(), yAxis.y(), yAxis.z());

		glColor3f(0, 0, 1);
		glVertex3f(zero.x(), zero.y(), zero.z());
		glVertex3f(zAxis.x(), zAxis.y(), zAxis.z());
	glEnd();*/
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);

	mDynamicsWorld->debugDrawWorld();
}

void GLWidget::setDebugLevel(int level)
{
	mDebugDrawer->setDebugMode(level);
}

void GLWidget::moveRight(const QVector3D &pos)
{
	switch (mRightCalibration) {
		case Regular:
			{
				// FIXME: this should use Y axis and is the current state only
				// for testing
				float y = pos.z() - mYellowNearPos.z();
				float l = (pos - mYellowNearPos).length();
				mRightArmUpDownDegrees = 360 * -acos(y / l) * M_1_PI / 2;
				mRightArmUpDownDegrees = 90 - mRightArmUpDownDegrees + 180;
				static int count = 0;
				float armLength = (mYellowFarPos - mYellowNearPos).length();
				mRightArmFoldDegrees = 360 * acos(l / armLength) * M_1_PI / 2;
				mRightArmFoldDegrees = qBound(0.0f, mRightArmFoldDegrees, 90.0f);
				// FIXME: this should perhaps use some other axis and is the
				// current state only for testing
				float desiredZ = (mYellowNearPos - mYellowFarPos).x();
				desiredZ = acos(desiredZ / armLength) * 360 * M_1_PI / 2;
				float z = (pos - mYellowFarPos).y();
				z = asin(z / armLength) * 360 * M_1_PI / 2;
				mRightArmLeftRightDegrees = z - desiredZ + 90;
				if (mDebugEnabled && count++ >= mDebugInterval) {
					qDebug() << "y:" << y << "l:" << l << "length" << armLength;
					qDebug() << "right arm up/down" << mRightArmUpDownDegrees << "right arm fold"
							 << mRightArmFoldDegrees << "right arm left/right"
							 << mRightArmLeftRightDegrees;
					count = 0;
				}

				QMatrix4x4 m;
				m.rotate(90, 1, 0, 0);
				m.translate(3, 0, 0);
				m.rotate(90 - mRightArmLeftRightDegrees, 0, 1, 0);
				m.rotate(mRightArmUpDownDegrees, 1, 0, 0);
				m.rotate(90, 0, 1, 0);
				m.rotate(-(90 - mRightArmFoldDegrees), 0, 1, 0);
				m.translate(0, 0, ARM_LENGTH);
				m.rotate(-2 * mRightArmFoldDegrees, 0, 1, 0);
				m.translate(0, 0, ARM_LENGTH);
				QVector3D v;
				mRightHandPos = m * v;
			}
		// fall-through
		case None:
			mRightPos = pos;
			break;
		case Experimental:
			mRightPos = mRightTransform * pos;
			break;
	}

	static int count = 0;
	if (mDebugEnabled && count++ >= mDebugInterval) {
		qDebug() << "right pos:" << pos << "transformed:" << mRightTransform * pos << "inverted:" << mRightTransform.inverted() * pos;
		count = 0;
	}
}

void GLWidget::moveLeft(const QVector3D &pos)
{
	switch (mLeftCalibration) {
			case Regular:
				{
					// FIXME: this should use Y axis and is the current state only
					// for testing
					float y = pos.z() - mBlueNearPos.z();
					float l = (pos - mBlueNearPos).length();
					mLeftArmUpDownDegrees = 360 * -acos(y / l) * M_1_PI / 2;
					mLeftArmUpDownDegrees = 90 - mLeftArmUpDownDegrees + 180;
					static int count = 0;
					float armLength = (mBlueFarPos - mBlueNearPos).length();
					mLeftArmFoldDegrees = 360 * acos(l / armLength) * M_1_PI / 2;
					mLeftArmFoldDegrees = qBound(0.0f, mLeftArmFoldDegrees, 90.0f);
					// FIXME: this should perhaps use some other axis and is the
					// current state only for testing
					float desiredZ = (mBlueNearPos - mBlueFarPos).x();
					desiredZ = acos(desiredZ / armLength) * 360 * M_1_PI / 2;
					float z = (pos - mBlueFarPos).y();
					z = asin(z / armLength) * 360 * M_1_PI / 2;
					mLeftArmLeftRightDegrees = z - desiredZ + 90;
					if (mDebugEnabled && count++ >= mDebugInterval) {
						qDebug() << "y:" << y << "l:" << l << "length" << armLength;
						qDebug() << "left arm up/down" << mLeftArmUpDownDegrees << " left arm fold"
								 << mLeftArmFoldDegrees << "left arm left/right"
								 << mLeftArmLeftRightDegrees;
						count = 0;
					}

					QMatrix4x4 m;
					m.rotate(-90, 1, 0, 0);
					m.translate(-3, 0, 0);
					m.rotate(90, 0, 1, 0);
					m.rotate(mLeftArmLeftRightDegrees, 0, 1, 0);
					m.rotate(-mLeftArmUpDownDegrees, 1, 0, 0);
					m.rotate(mLeftArmFoldDegrees, 0, 1, 0);
					m.translate(0, 0, ARM_LENGTH);
					m.rotate(-2 * mLeftArmFoldDegrees, 0, 1, 0);
					m.translate(0, 0, ARM_LENGTH);
					QVector3D v;
					mLeftHandPos = m * v;
				}
			// fall-through
			case None:
				mLeftPos = pos;
				break;
			case Experimental:
				mRightPos = mLeftTransform * pos;
				break;
		}

		static int count = 0;
		if (mDebugEnabled && count++ >= mDebugInterval) {
			qDebug() << "left pos:" << pos << "transformed:" << mRightTransform * pos << "inverted:" << mRightTransform.inverted() * pos;
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
	//qDebug() << "set right front pos to" << pos;
}

void GLWidget::calibrateRightRight(const QVector3D &pos)
{
	mRightRightPos = pos;
	//qDebug() << "set right right pos to" << pos;
}

void GLWidget::calibrateRightZero(const QVector3D &pos)
{
	mRightZeroPos = pos;
	//qDebug() << "set right zero pos to" << pos;
}

void GLWidget::calibrateRightGo(bool debug)
{
	//mRightFrontPos = QVector3D(-15.968, 33.3038, -65.4952);
	//mRightRightPos = QVector3D(-37.6838, -63.9283, -72.0952);
	//mRightZeroPos = QVector3D(29.4606, -70.0331, -57.7565);
	if (debug) {
		qDebug() << "mRightFrontPos" << mRightFrontPos;
		qDebug() << "mRightRightPos" << mRightRightPos;
		qDebug() << "mRightZeroPos" << mRightZeroPos;
	}

	float xRot, yRot;

	QVector3D d = mRightRightPos - mRightFrontPos;
	QMatrix4x4 rotationX, rotationY, tempMatrix;

	// plane's normal vector
	QVector3D n = QVector3D::crossProduct(mRightFrontPos - mRightZeroPos, mRightRightPos - mRightZeroPos);
	n.normalize();
	if (debug) {
		qDebug() << "normal:" << n;
	}

	QVector3D nx = n;
	nx.setX(0);
	xRot = acos(nx.z() / nx.length());
	xRot *= sign(n.y()) * 360 * M_1_PI / 2;
	rotationX.rotate(xRot, 1, 0, 0);
	if (debug) {
		qDebug() << "xRot:" << xRot;
	}

	QVector3D rotatedN = rotationX * n;
	if (debug) {
		qDebug() << "rotated normal:" << rotatedN;
	}

	yRot = acos(rotatedN.z() / rotatedN.length());
	yRot *= sign(rotatedN.x()) * -360 * M_1_PI / 2;
	rotationY.rotate(yRot, 0, 1, 0);
	if (debug) {
		qDebug() << "yRot:" << yRot;
	}

	rotatedN = rotationY * rotatedN;
	if (debug) {
		qDebug() << "rotated normal:" << rotatedN;
	}

	mRightTransform = rotationY * rotationX;

	if (debug) {
		qDebug() << "rotated diagon:" << mRightTransform * d;
	}

	// rotate everything to XZ plane (from XY)
	tempMatrix.rotate(90, 1, 0, 0);
	mRightTransform = tempMatrix * mRightTransform;

	if (debug) {
		qDebug() << "rotation:" << xRot << yRot;
		qDebug() << "rotated mRightFrontPos:" << mRightTransform * mRightFrontPos;
		qDebug() << "rotated mRightRightPos:" << mRightTransform * mRightRightPos;
		qDebug() << "rotated mRightZeroPos:"  << mRightTransform * mRightZeroPos;
		qDebug() << "*** diagon:" << d << "rotated:" << mRightTransform * d;
	}

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
	if (debug) {
		qDebug() << "rotated diagon:" << d << mRightTransform * d;
	}
	tempMatrix = QMatrix4x4();
	tempMatrix.rotate(yRot, 0, 1, 0);
	mRightTransform = tempMatrix * mRightTransform;

	d = mRightTransform * d;

	if (debug) {
		qDebug() << "rotation:" << xRot << yRot;
		qDebug() << "rotated mRightFrontPos:" << mRightTransform * mRightFrontPos;
		qDebug() << "rotated mRightRightPos:" << mRightTransform * mRightRightPos;
		qDebug() << "rotated mRightZeroPos:"  << mRightTransform * mRightZeroPos;
		qDebug() << "rotated diagon:" << d;
		qDebug() << yRot << d << qFuzzyIsNull(d.y());
	}


	QVector3D xAxis(5, 0, 0), yAxis(0, 5, 0), zAxis(0, 0, 5);
	xAxis = mRightTransform * xAxis;
	yAxis = mRightTransform * yAxis;
	zAxis = mRightTransform * zAxis;

	if (debug) {
		qDebug() << "new coordinate system:" << endl
				 << "\t" << xAxis << endl
				 << "\t" << yAxis << endl
				 << "\t" << zAxis << endl;
	}

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

	if (debug) {
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
	mRightArmUpDownDegrees = 0;
}

QVector3D GLWidget::getRightFrontPos() const
{
	return mRightFrontPos;
}

QVector3D GLWidget::getRightRightPos() const
{
	return mRightRightPos;
}

QVector3D GLWidget::getRightZeroPos() const
{
	return mRightZeroPos;
}

GLWidget::Calibration GLWidget::getRightCalibration() const
{
	return mRightCalibration;
}
