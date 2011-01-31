/*
 * stickman
 * Copyright (C) 2011 Michał Janiszewski
 *
 * This work is a master thesis. Due to nature of law you are obliged to
 * get a written permission prior to using any part of this work. If you
 * do get a permission, you are able to use the work, which is thereafter
 * licensed using GNU GPL v3 or (at your option) any later.
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

#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <QTime>
#include <QTimer>
#include <QVector3D>
#include <QMatrix4x4>
#include <QFlags>
#include <bullet/btBulletDynamicsCommon.h>

class GLDebugDrawer;

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
	void setDrawBoxes(bool draw);
	void setDebugLevel(int level);
	void resetBoxes();

private:
	void drawStickman();
	void removeBoxes();
	btRigidBody *localCreateRigidBody(float mass, const btTransform& startTransform, btCollisionShape *shape);

	GLfloat *light_ambient, *light_ambient_position, *whiteDiffuseLight,
			*blackAmbientLight, *whiteSpecularLight;
	GLUquadric *mQuadric;
	int mWidth, mHeight;
	int mRotation;
	QTime mTime;
	QTimer mUpdateTimer;
	QVector3D mRightPos, mLeftPos, mRightHandPos, mLeftHandPos;
	QVector3D mYellowNearPos, mYellowFarPos, mBlueNearPos, mBlueFarPos;
	QVector3D mRightFrontPos, mRightRightPos, mRightZeroPos;
	float mRightArmUpDownDegrees, mRightArmFoldDegrees, mRightArmLeftRightDegrees;
	float mLeftArmUpDownDegrees, mLeftArmFoldDegrees, mLeftArmLeftRightDegrees;
	QMatrix4x4 mRightTransform, mLeftTransform;
	int mDebugInterval;
	bool mDebugEnabled;
	Calibration mRightCalibration, mLeftCalibration;
	bool mDoDrawStickman, mDoDrawLeftMarker, mDoDrawRightMarker, mDoDrawBoxes;

	// Bullet Physics variables, as seen on
	// http://www.bulletphysics.org/mediawiki-1.5.8/index.php?title=Hello_World
	btBroadphaseInterface *mBroadphase;
	btDefaultCollisionConfiguration *mCollisionConfiguration;
	btCollisionDispatcher *mDispatcher;
	btSequentialImpulseConstraintSolver *mSolver;
	btDiscreteDynamicsWorld *mDynamicsWorld;
	btDefaultMotionState *mLeftHandMotionState, *mRightHandMotionState;
	btRigidBody::btRigidBodyConstructionInfo *mLeftHandRigidBodyCI, *mRightHandRigidBodyCI;
	btRigidBody *mLeftHandRigidBody, *mRightHandRigidBody, *mLowerBag, *mHook;
	btCollisionShape *mSphereShape;
	btCollisionShape* mGroundShape;
	btDefaultMotionState* mGroundMotionState;
	btRigidBody::btRigidBodyConstructionInfo mGroundRigidBodyCI;
	btRigidBody* mGroundRigidBody;
	QVector<btRigidBody *> mBoxes;
	btCollisionShape *mColShape;
	QTime mPhysicsTime;
	QTimer mPhysicsTimer;

	int mDebugLevel;
	GLDebugDrawer *mDebugDrawer;

	btScalar mDefaultContactProcessingThreshold;

protected:
	void initializeGL();
	void paintGL();
	void resizeGL(int w, int h);
};

#endif // GLWIDGET_H
