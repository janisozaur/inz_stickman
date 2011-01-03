#include "glwidget.h"

#include <QDebug>

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
	glTranslatef(-1.5, 0, 9);
	gluCylinder(mQuadric, 1, 1.3, 9, 20, 2);
	glPopMatrix();

	// one arm
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
}

void GLWidget::timeout()
{
	int elapsed = mTime.restart();
	mRotation += (elapsed / (float)mUpdateTimer.interval()) * 2.0f;
	mRotation = mRotation % 360;
}

void GLWidget::move(const QVector3D &pos)
{
	mPos = pos;
}
