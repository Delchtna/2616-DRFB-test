#include "main.h"

Point::Point():
  x(0), y(0) {}

Point::Point(double x, double y):
  x(x), y(y) {}

//Check if a Point equals another
bool Point::operator==(const Point &other) const {
  return this->x == other.x && this->y == other.y;
}

//Check if a Point is different from another
bool Point::operator!=(const Point &other)const{
  return !operator==(other);
}

//Add two Points together
Point Point::operator+(const Point &other) const{
  return Point(this->x + other.x, this->y + other.y);
}

//Subtract two Points from each other
Point Point::operator-(const Point &other) const{
  return Point(this->x - other.x, this->y - other.y);
}

//Multiply a Point by a constant
Point Point::operator*(double other) const{
  return Point(this->x * other, this->y * other);
}

//Divide a Point by a constant
Point Point::operator/(double other) const{
  return Point(this->x / other, this->y / other);
}
