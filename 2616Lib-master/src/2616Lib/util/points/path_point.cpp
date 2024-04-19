#include "main.h"


Path_Point::Path_Point():
  pose(Pose(0, 0, 0)), curvature(0), angular_velocity(0), s(0), velocity(0) {}

Path_Point::Path_Point(Pose pose, double curvature, double angular_velocity, double s, double velocity, double acceleration):
  pose(pose), curvature(curvature), angular_velocity(angular_velocity), s(s), velocity(velocity), acceleration(acceleration) {}

Path_Point::Path_Point(double x, double y, double angle, double curvature, double angular_velocity, double s, double velocity, double acceleration):
  pose(Pose(x, y, angle)), curvature(curvature), angular_velocity(angular_velocity), s(s), velocity(velocity), acceleration(acceleration) {}
