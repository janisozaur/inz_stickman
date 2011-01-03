#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <QTime>
#include <QTimer>
#include <QVector3D>

class GLWidget : public QGLWidget
{
	Q_OBJECT
public:
	explicit GLWidget(QWidget *parent = 0);
	~GLWidget();
	void move(const QVector3D &pos);

signals:

public slots:
	void timeout();

private:
	GLfloat *light_ambient, *light_ambient_position, *whiteDiffuseLight,
			*blackAmbientLight, *whiteSpecularLight;
	GLUquadric *mQuadric;
	int mWidth, mHeight;
	int mRotation;
	QTime mTime;
	QTimer mUpdateTimer;
	QVector3D mPos;

protected:
	void initializeGL();
	void paintGL();
	void resizeGL(int w, int h);
};

#endif // GLWIDGET_H
