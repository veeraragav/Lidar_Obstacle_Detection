#ifndef RENDER_H
#define RENDER_H
#include <pcl/visualization/pcl_visualizer.h>
#include "box.h"
#include <iostream>
#include <vector>
#include <string>

struct Color
{

	float r, g, b;

	Color(float setR, float setG, float setB)
		: r(setR), g(setG), b(setB)
	{}
};

struct Vect3
{

	double x, y, z;

	Vect3(double setX, double setY, double setZ)
		: x(setX), y(setY), z(setZ)
	{}

	Vect3 operator+(const Vect3& vec)
	{
		Vect3 result(x+vec.x,y+vec.y,z+vec.z);
		return result;
	}
};

enum CameraAngle
{
	XY, TopDown, Side, FPS
};


void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color = Color(1,1,1));
void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::string name, Color color = Color(-1,-1,-1));
void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, int id, Color color = Color(1,0,0), float opacity=1);
void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, BoxQ box, int id, Color color = Color(1,0,0), float opacity=1);

#endif
