#ifndef SAMPLE_H
#define SAMPLE_H

#include <QVector3D>

enum Marker {Blue = 1, Yellow = 2};

struct Sample
{
	Marker marker;
	float right, up, left;
	float x, y, z;
	float time;
	QVector3D filteredPos;
};

#endif // SAMPLE_H
