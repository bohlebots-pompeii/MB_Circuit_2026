// -------------------------------------------------------------------
//////////////////////////////////////////////////////////////////////
/**
*    BOHLEBOTS VECTOR CPP-FILE
*   
*    Version: 0.1.0
*    Date: 7. Mai 2023
*/
//////////////////////////////////////////////////////////////////////
// -------------------------------------------------------------------

#include "Vector2.hpp"
#include <string>
#include <cmath>
#include <algorithm>
#include <numbers>
#include <ostream>

////////////////////////////////////////////////////////////////
//--- CONSTRUCTOR --------------------------------------------//
////////////////////////////////////////////////////////////////
Vector2::Vector2()
    : _x(0.0), _y(0.0) {
    this->update_PhiRad();
}

Vector2::Vector2(const double &a, const double &b)
    : _x(a), _y(b) {
    this->update_PhiRad();
}

Vector2::Vector2(const std::uint8_t v[2]) : _x(static_cast<int>(v[0])), _y(static_cast<int>(v[1])) {
    this->update_PhiRad();
}


////////////////////////////////////////////////////////////////
//--- GETTER FUNCTIONS ---------------------------------------//
////////////////////////////////////////////////////////////////
double Vector2::getX() const { return this->_x; }

double Vector2::getY() const { return this->_y; }

double Vector2::getAngle() const { return this->_angle; }

double Vector2::getMagnitude() const { return this->_magnitude; }

////////////////////////////////////////////////////////////////
//--- CLONING ------------------------------------------------//
////////////////////////////////////////////////////////////////
Vector2 Vector2::clone() const {
    return Vector2(this->_x, this->_y);
}

Vector2 Vector2::clone(const Vector2 &v) {
    return Vector2(v._x, v._y);
}

////////////////////////////////////////////////////////////////
//--- OPERATORS ----------------------------------------------//
////////////////////////////////////////////////////////////////

std::ostream& operator<<(std::ostream& os, const Vector2& v) {
    os << "(" << v._x << ", " << v._y << ")";
    return os;
}

void Vector2::operator+=(const Vector2 &v) { this->add(v); }

void Vector2::operator-=(const Vector2 &v) { this->subtract(v); }

void Vector2::operator*=(const double &d) { this->multiply(d); }

bool Vector2::operator==(const Vector2 &v) const { return _x == v.getX() && _y == v.getY(); }

Vector2 Vector2::operator+(const Vector2 &v) { return Vector2::add(*this, v); }

Vector2 Vector2::operator-(const Vector2 &v) { return Vector2::subtract(*this, v); }

Vector2 Vector2::operator*(const double &d) { return Vector2::multiply(*this, d); }

double Vector2::operator*(const Vector2 &v) { return Vector2::dotProduct(*this, v); }

////////////////////////////////////////////////////////////////
//--- ADDITION --- intern ------------------------------------//
////////////////////////////////////////////////////////////////
void Vector2::add(const Vector2 &v) {
    this->_x += v._x;
    this->_y += v._y;
    this->update_PhiRad();
}

////////////////////////////////////////////////////////////////
//--- ADDITION --- extern ------------------------------------//
////////////////////////////////////////////////////////////////
Vector2 Vector2::add(const Vector2 &v1, const Vector2 &v2) {
    return Vector2(v1._x + v2._x, v1._y + v2._y);
}

////////////////////////////////////////////////////////////////
//--- SUBTRACTION --- intern ---------------------------------//
////////////////////////////////////////////////////////////////
void Vector2::subtract(const Vector2 &v) {
    this->_x -= v._x;
    this->_y -= v._y;
    update_PhiRad();
}

////////////////////////////////////////////////////////////////
//--- SUBTRACTION --- extern ---------------------------------//
////////////////////////////////////////////////////////////////
Vector2 Vector2::subtract(const Vector2 &v1, const Vector2 &v2) {
    return Vector2(v1._x - v2._x, v1._y - v2._y);
}

////////////////////////////////////////////////////////////////
//--- MULTIPLICATION --- intern ------------------------------//
////////////////////////////////////////////////////////////////
void Vector2::multiply(const double &d) {
    this->_x *= d;
    this->_y *= d;
    this->update_PhiRad();
}

////////////////////////////////////////////////////////////////
//--- MULTIPLICATION --- extern ------------------------------//
////////////////////////////////////////////////////////////////
Vector2 Vector2::multiply(const Vector2 &v, const double &d) {
    return Vector2(v._x * d, v._y * d);
}

double Vector2::dotProduct(const Vector2 &v1, const Vector2 &v2) {
    return (v1._x * v2._x) + (v1._y * v2._y);
}

////////////////////////////////////////////////////////////////
//--- SCALING --- intern -------------------------------------//
////////////////////////////////////////////////////////////////
void Vector2::scale_XY(const double &a, const double &b) {
    this->_x *= a;
    this->_y *= b;
    this->update_PhiRad();
}

////////////////////////////////////////////////////////////////
//--- SCALING --- extern -------------------------------------//
////////////////////////////////////////////////////////////////
Vector2 Vector2::scale_XY(const Vector2 &v, const double &a, const double &b) {
    return Vector2(v._x * a, v._y * b);
}

////////////////////////////////////////////////////////////////
//--- ROTATION --- intern ------------------------------------//
////////////////////////////////////////////////////////////////
void Vector2::rotate(const double &angle_rad) {
    this->_angle += angle_rad;
    this->update_XY();
    this->update_PhiRad();
}

////////////////////////////////////////////////////////////////
//--- ROTATION --- extern ------------------------------------//
///////////////////////////////////////////////////////////////
Vector2 Vector2::rotate(const Vector2 &v, const double &angle_rad) {
    Vector2 _v = Vector2::clone(v);
    _v._angle += angle_rad;
    _v.update_XY();
    _v.update_PhiRad();
    return _v;
}

////////////////////////////////////////////////////////////////
//--- ANGLES -------------------------------------------------//
////////////////////////////////////////////////////////////////
double Vector2::angleOffsetTo(const Vector2 &v) const {
    return acos(Vector2::dotProduct(*this, v) / (this->_magnitude * v._magnitude));
}

double Vector2::angleOffsetBetween(const Vector2 &v1, const Vector2 &v2) {
    return acos(Vector2::dotProduct(v1, v2) / (v1._magnitude * v2._magnitude));
}

////////////////////////////////////////////////////////////////
//--- SECTORS ------------------------------------------------//
////////////////////////////////////////////////////////////////
bool Vector2::isBetween(const Vector2 &v1, const Vector2 &v2) const {
    int angleThisV1 = (int) (this->angleOffsetTo(v1) * 1000);
    int angleThisV2 = (int) (this->angleOffsetTo(v2) * 1000);
    int angleV1V2 = (int) (Vector2::angleOffsetBetween(v1, v2) * 1000);
    return ((angleThisV1 + angleThisV2) == angleV1V2);
}

bool Vector2::vectorIsInSector(const Vector2 &v, const Vector2 &a, const Vector2 &b) {
    int av = (int) (Vector2::angleOffsetBetween(a, v) * 1000);
    int vb = (int) (Vector2::angleOffsetBetween(v, b) * 1000);
    int ab = (int) (Vector2::angleOffsetBetween(a, b) * 1000);
    return ((av + vb) == ab);
}

////////////////////////////////////////////////////////////////
//--- DEBUG --------------------------------------------------//
////////////////////////////////////////////////////////////////

std::string Vector2::debugString() const {
    std::string cartesian = "[ x = " + std::to_string(this->_x) + ", y = " + std::to_string(this->_y) + " ]";
    std::string polar = "[ rad = " + std::to_string(this->_magnitude) + ", angle = " +
                        std::to_string(this->_angle / std::numbers::pi) + " * PI ]";
    return cartesian + " <=> " + polar;
}

void Vector2::normalize() {
    this->_x = this->_x / this->_magnitude;
    this->_y = this->_y / this->_magnitude;
    this->_magnitude = 1.0;
}

Vector2 Vector2::normalize(Vector2 &v) {
    double magnitude = v.getMagnitude();
    return Vector2(v._x / magnitude, v._y / magnitude);
}

void Vector2::setX(double x) {
    this->_x = x;
    this->update_PhiRad();
}

void Vector2::setY(double y) {
    this->_y = y;
    this->update_PhiRad();
}

int Vector2::getSignX() {
    return getSign(this->_x);
}

int Vector2::getSignY() {
    return getSign(this->_y);
}

////////////////////////////////////////////////////////////////
//--- PRIVATE ------------------------------------------------//
////////////////////////////////////////////////////////////////

void Vector2::update_XY() {
    this->_x = this->_magnitude * cos(this->_angle);
    this->_y = this->_magnitude * sin(this->_angle);
}

void Vector2::update_PhiRad() {
    this->_angle = atan2(this->_y, this->_x);
    this->_magnitude = sqrt((this->_x * this->_x) + (this->_y * this->_y));
}

int Vector2::getSign(double x) {
    return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
}
