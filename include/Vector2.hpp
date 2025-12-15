// -------------------------------------------------------------------
//////////////////////////////////////////////////////////////////////
/**
*    BOHLEBOTS VECTOR HEADER-FILE
*   
*    Version: 0.1.0
*    Date: 7. Mai 2023
*     
*    Users Guide:
*       this header can do everything of the basic vector math 
*       for which the function names and other comments should be 
*       enough IMPORTANT is that this vector header is optimized 
*       for the BohleBots therefore their way of thinking about 
*       directions is provided here
*                  ^ x
*                  |           
*                  |     @1
*                  |   /
*                  | /
*        ----------|----------> y
*                / |
*              /   |
*             @2   |
*                  *
*        the diagram shows that this header provides a view
*        in which the axis from front to back of the robot is 
*        the x-axis and the axis from left to right is the y-axis
*        therefore also @1 has an angle value of around +45° 
*        and @2 of around -135°
*/
//////////////////////////////////////////////////////////////////////
// -------------------------------------------------------------------
#pragma once

#include <string>
#include <cstdint>
//////////////////////////////////////////////////////////////////////
/*
*   INCLUDES / DEPENDENCIES
*/
//////////////////////////////////////////////////////////////////////

/*
*   DECLARATIONS
*/
//////////////////////////////////////////////////////////////////////
/*
*   BB_VECTOR
*/
class Vector2 {
public:
    /*
    *   PUBLIC FUNCTIONS
    */
    /************************************************************
    *
    *   CONSTRUCTORS
    *
    ************************************************************/
    Vector2();

    Vector2(const double &a, const double &b);

    Vector2(const std::uint8_t v[2]);

    /************************************************************
    *
    *   GETTER FUNCTIONS
    *
    ************************************************************/
    double getX() const;

    double getY() const;

    double getAngle() const;

    double getMagnitude() const;

    /************************************************************
    *
    *   CLONING
    *
    ************************************************************/
    Vector2 clone() const;

    static Vector2 clone(const Vector2 &v);

    /************************************************************
    *
    *   OPERATORS
    *
    ************************************************************/
    friend std::ostream& operator<<(std::ostream& os, const Vector2& v);

    void operator +=(const Vector2 &v);

    void operator -=(const Vector2 &v);

    void operator *=(const double &d);

    bool operator==(const Vector2 &v) const;

    Vector2 operator +(const Vector2 &v);

    Vector2 operator -(const Vector2 &v);

    Vector2 operator *(const double &d);

    double operator *(const Vector2 &v);

    /************************************************************
    *
    *   ADDITION intern
    *
    ************************************************************/
    void add(const Vector2 &v);

    /************************************************************
    *
    *   ADDITION extern
    *
    ************************************************************/
    static Vector2 add(const Vector2 &v1, const Vector2 &v2);

    /************************************************************
    *
    *   SUBTRACTION intern
    *
    ************************************************************/
    void subtract(const Vector2 &v);

    /************************************************************
    *
    *   SUBTRACTION extern
    *
    ************************************************************/
    static Vector2 subtract(const Vector2 &v1, const Vector2 &v2);

    /************************************************************
    *
    *   MULTIPLICATION intern
    *
    ************************************************************/
    void multiply(const double &d);

    /************************************************************
    *
    *   MULTIPLICATION extern
    *
    ************************************************************/
    static Vector2 multiply(const Vector2 &v, const double &d);

    static double dotProduct(const Vector2 &v1, const Vector2 &v2);

    /************************************************************
    *
    *   SCALING intern
    *
    ************************************************************/
    void scale_XY(const double &a, const double &b);

    /************************************************************
    *
    *   SCALING extern
    *
    ************************************************************/
    static Vector2 scale_XY(const Vector2 &v, const double &a, const double &b);

    /************************************************************
    *
    *   ROTATION intern
    *
    ************************************************************/
    void rotate(const double &angle_rad);

    /************************************************************
    *
    *   ROTATION extern
    *
    ************************************************************/
    static Vector2 rotate(const Vector2 &z, const double &angle_rad);

    /************************************************************
    *
    *   ANGLES
    *
    ************************************************************/
    double angleOffsetTo(const Vector2 &v) const;

    static double angleOffsetBetween(const Vector2 &v1, const Vector2 &v2);

    /************************************************************
    *
    *   SECTORS
    *
    ************************************************************/
    bool isBetween(const Vector2 &v1, const Vector2 &v2) const;

    static bool vectorIsInSector(const Vector2 &v, const Vector2 &a, const Vector2 &b);

    /************************************************************
    *
    *   DEBUG
    *
    ************************************************************/
    std::string debugString() const;

    static Vector2 normalize(Vector2 &v);

    void normalize();

    void setX(double x);

    void setY(double y);

    int getSignX();

    int getSignY();

    /*
    *  PRIVATE IMPLEMENTATIONS
    */
private:
    double _x, _y, _angle, _magnitude;

    void update_XY();

    void update_PhiRad();

    int getSign(double x);
};
