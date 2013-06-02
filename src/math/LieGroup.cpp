#include "utils/UtilsCode.h"
#include "math/LieGroup.h"
#include <iomanip>
#include <cmath>

namespace dart {
namespace math {

Vec3::Vec3()
{
    _v[0] = _v[1] = _v[2] = 0.0;
}

Vec3::Vec3(double d)
{
    _v[0] = _v[1] = _v[2] = d;
}

Vec3::Vec3(const double v[])
{
    _v[0] = v[0];
    _v[1] = v[1];
    _v[2] = v[2];
}

Vec3::Vec3(const Eigen::Vector3d& v)
{
    _v[0] = v[0];
    _v[1] = v[1];
    _v[2] = v[2];
}

Vec3::~Vec3()
{
}

Vec3::Vec3(double v0, double v1, double v2)
{
    _v[0] = v0;
    _v[1] = v1;
    _v[2] = v2;
}

const Vec3& Vec3::operator+(void) const
{
    return *this;
}

Vec3 Vec3::operator-(void) const
{
    return Vec3(-_v[0], -_v[1], -_v[2]);
}

double& Vec3::operator[](int i)
{
    return _v[i];
}

const double& Vec3::operator[](int i) const
{
    return _v[i];
}

const Vec3& Vec3::operator = (const Vec3& v)
{
    _v[0] = v[0];
    _v[1] = v[1];
    _v[2] = v[2];
    return *this;
}

const Vec3& Vec3::operator = (double d)
{
    _v[0] = _v[1] = _v[2] = d;
    return *this;
}

const Vec3& Vec3::operator += (const Vec3& v)
{
    _v[0] += v[0];
    _v[1] += v[1];
    _v[2] += v[2];
    return *this;
}

const Vec3& Vec3::operator-= (const Vec3& v)
{
    _v[0] -= v[0];
    _v[1] -= v[1];
    _v[2] -= v[2];
    return *this;
}

const Vec3& Vec3::operator *= (double d)
{
    _v[0] *= d;
    _v[1] *= d;
    _v[2] *= d;
    return *this;
}

bool Vec3::operator==(const Vec3& v) const
{
    if ((_v[0] != v._v[0])
            || (_v[1] != v._v[1])
            || (_v[2] != v._v[2]))
        return false;

    return true;
}

bool Vec3::operator!=(const Vec3& v) const
{
    return !(*this == v);
}

Vec3 Vec3::operator*(double d) const
{
    return Vec3(d * _v[0], d * _v[1], d * _v[2]);
}

Vec3 Vec3::operator+(const Vec3& v) const
{
    return Vec3(_v[0] + v[0], _v[1] + v[1], _v[2] + v[2]);
}

Vec3 Vec3::operator-(const Vec3& v) const
{
    return Vec3(_v[0] - v[0], _v[1] - v[1], _v[2] - v[2]);
}

double Vec3::Normalize(void)
{
    double mag = sqrt(_v[0] * _v[0] + _v[1] * _v[1] + _v[2] * _v[2]);
    if ( mag < LIE_EPS )	// make a unit vector in z-direction
    {
        _v[0] = _v[1] = SCALAR_0;
        _v[2] = SCALAR_1;
    } else
    {
        _v[0] /= mag;
        _v[1] /= mag;
        _v[2] /= mag;
    }
    return mag;
}

Eigen::Vector3d Vec3::getEigenVector() const
{
    Eigen::Vector3d vector3d;
    vector3d << _v[0], _v[1], _v[2];

    return vector3d;
}

//==============================================================================
//
//==============================================================================
Vec3 Rotate(const SE3& T, const Vec3& v)
{
    return Vec3(T._T[0] * v[0] + T._T[3] * v[1] + T._T[6] * v[2],
                T._T[1] * v[0] + T._T[4] * v[1] + T._T[7] * v[2],
                T._T[2] * v[0] + T._T[5] * v[1] + T._T[8] * v[2]);
}

Vec3 InvRotate(const SE3& T, const Vec3& v)
{
    return Vec3(T._T[0] * v[0] + T._T[1] * v[1] + T._T[2] * v[2],
                T._T[3] * v[0] + T._T[4] * v[1] + T._T[5] * v[2],
                T._T[6] * v[0] + T._T[7] * v[1] + T._T[8] * v[2]);
}

Vec3 operator*(double d, const Vec3& v)
{
    return Vec3(d * v[0], d * v[1], d * v[2]);
}

double Norm(const Vec3& v)
{
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

Vec3 Normalize(const Vec3& v)
{
    double mag = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if ( mag < LIE_EPS )	// make a unit vector in z-direction
        return Vec3(SCALAR_0, SCALAR_0, SCALAR_1);

    mag = SCALAR_1 / mag;
    return Vec3(mag * v[0], mag * v[1], mag * v[2]);
}

Vec3 Cross(const Vec3& p, const Vec3& q)
{
    return Vec3(p[1] * q[2] - p[2] * q[1],
                p[2] * q[0] - p[0] * q[2],
                p[0] * q[1] - p[1] * q[0]);
}

double Inner(const Vec3& p, const Vec3& q)
{
    return (p[0] * q[0] + p[1] * q[1] + p[2] * q[2]);
}

double SquareSum(const Vec3& p)
{
    return (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
}

Vec3 MinusLinearAd(const Vec3& p, const se3& s)
{
    return Vec3(p[2] * s[1] - p[1] * s[2] + s[3],
                p[0] * s[2] - p[2] * s[0] + s[4],
                p[1] * s[0] - p[0] * s[1] + s[5]);
}

Vec3 InvAd(const SE3& T, const Vec3& v)
{
    return Vec3(T._T[0] * v[0] + T._T[1] * v[1] + T._T[2] * v[2],
                T._T[3] * v[0] + T._T[4] * v[1] + T._T[5] * v[2],
                T._T[6] * v[0] + T._T[7] * v[1] + T._T[8] * v[2]);
}

Vec3 iEulerZYX(const SE3& T)
{
    return Vec3(atan2(T._T[1], T._T[0]),
                atan2(-T._T[2], sqrt(T._T[0] * T._T[0] + T._T[1] * T._T[1])),
                atan2(T._T[5], T._T[8]));
}

Vec3 iEulerZYZ(const SE3& T)
{
    return Vec3(atan2(T._T[7], T._T[6]),
                atan2(sqrt(T._T[2] * T._T[2] + T._T[5] * T._T[5]), T._T[8]),
                atan2(T._T[5], -T._T[2]));
}

Vec3 ad(const Vec3& s1, const se3& s2)
{
    return Vec3(s2[2] * s1[1] - s2[1] * s1[2],
                s2[0] * s1[2] - s2[2] * s1[0],
                s2[1] * s1[0] - s2[0] * s1[1]);
}

//==============================================================================
//
//==============================================================================
se3::se3()
{
    _w[0] = _w[1] = _w[2] = _w[3] = _w[4] = _w[5] = 0.0;
}

se3::se3(double k)
{
    _w[0] = _w[1] = _w[2] = _w[3] = _w[4] = _w[5] = k;
}

se3::se3(double w0, double w1, double w2, double w3, double w4, double w5)
{
    _w[0] = w0;
    _w[1] = w1;
    _w[2] = w2;
    _w[3] = w3;
    _w[4] = w4;
    _w[5] = w5;
}

se3::se3(const Axis& w, const Vec3& v)
{
    _w[0] = w[0];
    _w[1] = w[1];
    _w[2] = w[2];
    _w[3] = v[0];
    _w[4] = v[1];
    _w[5] = v[2];
}

const se3& se3::operator+(void) const
{
    return *this;
}

se3 se3::operator-(void) const
{
    return se3(-_w[0], -_w[1], -_w[2], -_w[3], -_w[4], -_w[5]);
}

const se3& se3::operator = (const se3& s)
{
    _w[0] = s[0];
    _w[1] = s[1];
    _w[2] = s[2];
    _w[3] = s[3];
    _w[4] = s[4];
    _w[5] = s[5];
    return *this;
}

const se3& se3::operator = (const Vec3& s)
{
    _w[0] = _w[1] = _w[2] = SCALAR_0;
    _w[3] = s[0];
    _w[4] = s[1];
    _w[5] = s[2];
    return *this;
}

const se3& se3::operator = (const Axis& s)
{
    _w[0] = s[0];
    _w[1] = s[1];
    _w[2] = s[2];
    _w[3] = _w[4] = _w[5] = SCALAR_0;
    return *this;
}

const se3& se3::operator = (double d)
{
    _w[0] = _w[1] = _w[2] = _w[3] = _w[4] = _w[5] = d;
    return *this;
}

const se3& se3::operator += (const se3& s)
{
    _w[0] += s[0];
    _w[1] += s[1];
    _w[2] += s[2];
    _w[3] += s[3];
    _w[4] += s[4];
    _w[5] += s[5];
    return *this;
}

const se3& se3::operator += (const Axis& s)
{
    _w[0] += s[0];
    _w[1] += s[1];
    _w[2] += s[2];
    return *this;
}

const se3& se3::operator += (const Vec3& s)
{
    _w[3] += s[0];
    _w[4] += s[1];
    _w[5] += s[2];
    return *this;
}

const se3& se3::operator-= (const se3& s)
{
    _w[0] -= s[0];
    _w[1] -= s[1];
    _w[2] -= s[2];
    _w[3] -= s[3];
    _w[4] -= s[4];
    _w[5] -= s[5];
    return *this;
}

const se3& se3::operator *= (double d)
{
    _w[0] *= d;
    _w[1] *= d;
    _w[2] *= d;
    _w[3] *= d;
    _w[4] *= d;
    _w[5] *= d;
    return *this;
}

bool se3::operator==(const se3& S) const
{
    for (int i = 0; i < 6; ++i)
        if (_w[i] != S._w[i])
            return false;

    return true;
}

bool se3::operator!=(const se3& S) const
{
    return !(*this == S);
}

se3 se3::operator+(const se3& s) const
{
    return se3(_w[0] + s[0], _w[1] + s[1], _w[2] + s[2], _w[3] + s[3], _w[4] + s[4], _w[5] + s[5]);
}

se3 se3::operator-(const se3& s) const
{
    return se3(_w[0] - s[0], _w[1] - s[1], _w[2] - s[2], _w[3] - s[3], _w[4] - s[4], _w[5] - s[5]);
}

se3 se3::operator*(double d) const
{
    return se3(d * _w[0], d * _w[1], d * _w[2], d * _w[3], d * _w[4], d * _w[5]);
}

double& se3::operator[](int i)
{
    return _w[i];
}

const double& se3::operator[](int i) const
{
    return _w[i];
}

void se3::setZero()
{
    _w[0] = _w[1] = _w[2] = _w[3] = _w[4] = _w[5] = 0.0;
}

void se3::setFromMatrixForm(const Eigen::Matrix4d& mat)
{
    // Assume that _M is 4x4 matrix as:
    // _M = | [w] v |
    //      |   0 0 |
    //    = |   0 -w3  w2  v1 |
    //      |  w3   0 -w1  v2 |
    //      | -w2  w1   0  v3 |
    //      |   0   0   0   0 |

    _w[0] = mat(2,1);
    _w[1] = mat(0,2);
    _w[2] = mat(1,0);
    _w[3] = mat(0,3);
    _w[4] = mat(1,3);
    _w[5] = mat(2,3);
}

// *this = T * s * Inv(T)
void se3::setAd(const SE3& T, const se3& s)
{
    _w[0] = T._T[0] * s[0] + T._T[3] * s[1] + T._T[6] * s[2];
    _w[1] = T._T[1] * s[0] + T._T[4] * s[1] + T._T[7] * s[2];
    _w[2] = T._T[2] * s[0] + T._T[5] * s[1] + T._T[8] * s[2];
    _w[3] = T._T[10] * _w[2] - T._T[11] * _w[1] + T._T[0] * s[3] + T._T[3] * s[4] + T._T[6] * s[5];
    _w[4] = T._T[11] * _w[0] - T._T[9] * _w[2] + T._T[1] * s[3] + T._T[4] * s[4] + T._T[7] * s[5];
    _w[5] = T._T[9] * _w[1] - T._T[10] * _w[0] + T._T[2] * s[3] + T._T[5] * s[4] + T._T[8] * s[5];
}

// re = Inv(T) * s * T
void se3::setInvAd(const SE3& T, const se3& s)
{
    double _tmp[3] = {	s[3] + s[1] * T._T[11] - s[2] * T._T[10],
                        s[4] + s[2] * T._T[9] - s[0] * T._T[11],
                        s[5] + s[0] * T._T[10] - s[1] * T._T[9] };
    _w[0] = T._T[0] * s[0] + T._T[1] * s[1] + T._T[2] * s[2];
    _w[1] = T._T[3] * s[0] + T._T[4] * s[1] + T._T[5] * s[2];
    _w[2] = T._T[6] * s[0] + T._T[7] * s[1] + T._T[8] * s[2];
    _w[3] = T._T[0] * _tmp[0] + T._T[1] * _tmp[1] + T._T[2] * _tmp[2];
    _w[4] = T._T[3] * _tmp[0] + T._T[4] * _tmp[1] + T._T[5] * _tmp[2];
    _w[5] = T._T[6] * _tmp[0] + T._T[7] * _tmp[1] + T._T[8] * _tmp[2];
}

void se3::setad(const se3& s1, const se3& s2)
{
    _w[0] =	s1[1] * s2[2] - s1[2] * s2[1];
    _w[1] =	s1[2] * s2[0] - s1[0] * s2[2];
    _w[2] =	s1[0] * s2[1] - s1[1] * s2[0];
    _w[3] =	s1[1] * s2[5] - s1[2] * s2[4] - s2[1] * s1[5] + s2[2] * s1[4];
    _w[4] =	s1[2] * s2[3] - s1[0] * s2[5] - s2[2] * s1[3] + s2[0] * s1[5];
    _w[5] =	s1[0] * s2[4] - s1[1] * s2[3] - s2[0] * s1[4] + s2[1] * s1[3];
}

void se3::setad(const se3& s1, const Axis& s2)
{
    _w[0] =	s1[1] * s2[2] - s1[2] * s2[1];
    _w[1] =	s1[2] * s2[0] - s1[0] * s2[2];
    _w[2] =	s1[0] * s2[1] - s1[1] * s2[0];
    _w[3] =	s2[2] * s1[4] - s2[1] * s1[5];
    _w[4] =	s2[0] * s1[5] - s2[2] * s1[3];
    _w[5] = s2[1] * s1[3] - s2[0] * s1[4];
}

void se3::setEigenVector(const Eigen::Matrix<double,6,1>& vec6)
{
    _w[0] = vec6[0];
    _w[1] = vec6[1];
    _w[2] = vec6[2];
    _w[3] = vec6[3];
    _w[4] = vec6[4];
    _w[5] = vec6[5];
}

Eigen::Matrix<double,6,1> se3::getEigenVector() const
{
    Eigen::Matrix<double,6,1> vec6;

    vec6 << _w[0], _w[1], _w[2], _w[3], _w[4], _w[5];

    return vec6;
}

//==============================================================================
//
//==============================================================================
se3 operator*(double d, const se3& s)
{
    return se3(d * s[0], d * s[1], d * s[2], d * s[3], d * s[4], d * s[5]);
}

double operator*(const dse3& t, const se3& s)
{
    return (t[0] * s[0] + t[1] * s[1] + t[2] * s[2] + t[3] * s[3] + t[4] * s[4] + t[5] * s[5]);
}

double operator*(const dse3& t, const Axis& s)
{
    return (t[0] * s[0] + t[1] * s[1] + t[2] * s[2]);
}

//double operator*(const dse3& t, const Vec3& s)
//{
//    return (t[3] * s[0] + t[4] * s[1] + t[5] * s[2]);
//}

//double operator*(const se3& s, const dse3& t)
//{
//    return (t[0] * s[0] + t[1] * s[1] + t[2] * s[2] + t[3] * s[3] + t[4] * s[4] + t[5] * s[5]);
//}

/*
    T = (R, p) = exp([w, v]), t = ||w||
    v = beta * p + gamma * w + 1 / 2 * cross(p, w)
    , beta = t * (1 + cos(t)) / (2 * sin(t)), gamma = <w, p> * (1 - beta) / t^2
*/
se3 Log(const SE3& T)
{
    double theta = acos(max(min(SCALAR_1_2 * (T[0] + T[4] + T[8] - SCALAR_1), SCALAR_1), -SCALAR_1)), alpha, beta, gamma;

    if ( theta > M_PI - LIE_EPS )
    {
        const double c1 = 0.10132118364234;		// 1 / pi^2
        const double c2 = 0.01507440267955;		// 1 / 4 / pi - 2 / pi^3
        const double c3 = 0.00546765085347;		// 3 / pi^4 - 1 / 4 / pi^2

        double phi = M_PI - theta;
        double delta = SCALAR_1_2 + SCALAR_1_8 * phi * phi;

        double w[] = {	T[5] > T[7] ? theta * sqrt(SCALAR_1 + (T[0] - SCALAR_1) * delta) : -theta * sqrt(SCALAR_1 + (T[0] - SCALAR_1) * delta),
                        T[6] > T[2] ? theta * sqrt(SCALAR_1 + (T[4] - SCALAR_1) * delta) : -theta * sqrt(SCALAR_1 + (T[4] - SCALAR_1) * delta),
                        T[1] > T[3] ? theta * sqrt(SCALAR_1 + (T[8] - SCALAR_1) * delta) : -theta * sqrt(SCALAR_1 + (T[8] - SCALAR_1) * delta) };

        beta = SCALAR_1_4 * theta * (M_PI - theta);
        gamma = (w[0] * T[9] + w[1] * T[10] + w[2] * T[11]) * (c1 -  c2 * phi + c3 * phi * phi);

        return se3(	w[0], w[1], w[2],
                    beta * T[ 9] - SCALAR_1_2 * (w[1] * T[11] - w[2] * T[10]) + gamma * w[0],
                    beta * T[10] - SCALAR_1_2 * (w[2] * T[ 9] - w[0] * T[11]) + gamma * w[1],
                    beta * T[11] - SCALAR_1_2 * (w[0] * T[10] - w[1] * T[ 9]) + gamma * w[2]);
    } else
    {
        if ( theta > LIE_EPS )
        {
            alpha = SCALAR_1_2 * theta / sin(theta);
            beta = (SCALAR_1 + cos(theta)) * alpha;
            gamma = (SCALAR_1 - beta) / theta / theta;
        } else
        {
            alpha = SCALAR_1_2 + SCALAR_1_12 * theta * theta;
            beta = SCALAR_1 - SCALAR_1_12 * theta * theta;
            gamma = SCALAR_1_12 + SCALAR_1_720 * theta * theta;
        }

        double w[] = { alpha * (T[5] - T[7]), alpha * (T[6] - T[2]), alpha * (T[1] - T[3]) };
        gamma *= w[0] * T[9] + w[1] * T[10] + w[2] * T[11];

        return se3(	w[0], w[1], w[2],
                    beta * T[ 9] + SCALAR_1_2 * (w[2] * T[10] - w[1] * T[11]) + gamma * w[0],
                    beta * T[10] + SCALAR_1_2 * (w[0] * T[11] - w[2] * T[ 9]) + gamma * w[1],
                    beta * T[11] + SCALAR_1_2 * (w[1] * T[ 9] - w[0] * T[10]) + gamma * w[2]);
    }
}

Axis LogR(const SE3& T)
{
    double theta = acos(max(min(SCALAR_1_2 * (T[0] + T[4] + T[8] - SCALAR_1), SCALAR_1), -SCALAR_1)), alpha;

    if ( theta > M_PI - LIE_EPS )
    {
        double delta = SCALAR_1_2 + SCALAR_1_8 * (M_PI - theta) * (M_PI - theta);

        return Axis(T[5] > T[7] ? theta * sqrt(SCALAR_1 + (T[0] - SCALAR_1) * delta) : -theta * sqrt(SCALAR_1 + (T[0] - SCALAR_1) * delta),
                    T[6] > T[2] ? theta * sqrt(SCALAR_1 + (T[4] - SCALAR_1) * delta) : -theta * sqrt(SCALAR_1 + (T[4] - SCALAR_1) * delta),
                    T[1] > T[3] ? theta * sqrt(SCALAR_1 + (T[8] - SCALAR_1) * delta) : -theta * sqrt(SCALAR_1 + (T[8] - SCALAR_1) * delta));
    } else
    {
        if ( theta > LIE_EPS )
            alpha = SCALAR_1_2 * theta / sin(theta);
        else
            alpha = SCALAR_1_2 + SCALAR_1_12 * theta * theta;

        return Axis(alpha * (T[5] - T[7]), alpha * (T[6] - T[2]), alpha * (T[1] - T[3]));
    }
}

// re = T * s * Inv(T)
se3 Ad(const SE3& T, const se3& s)
{
    double tmp[3] = {	T[0] * s[0] + T[3] * s[1] + T[6] * s[2],
                        T[1] * s[0] + T[4] * s[1] + T[7] * s[2],
                        T[2] * s[0] + T[5] * s[1] + T[8] * s[2] };
    return se3(	tmp[0], tmp[1], tmp[2],
                T[10] * tmp[2] - T[11] * tmp[1] + T[0] * s[3] + T[3] * s[4] + T[6] * s[5],
                T[11] * tmp[0] - T[9] * tmp[2] + T[1] * s[3] + T[4] * s[4] + T[7] * s[5],
                T[9] * tmp[1] - T[10] * tmp[0] + T[2] * s[3] + T[5] * s[4] + T[8] * s[5]);
}

se3 Ad(const SE3& T, const Axis& s)
{
    double tmp[3] = {	T[0] * s[0] + T[3] * s[1] + T[6] * s[2],
                        T[1] * s[0] + T[4] * s[1] + T[7] * s[2],
                        T[2] * s[0] + T[5] * s[1] + T[8] * s[2] };
    return se3(	tmp[0], tmp[1], tmp[2],
                T[10] * tmp[2] - T[11] * tmp[1],
                T[11] * tmp[0] - T[9] * tmp[2],
                T[9] * tmp[1] - T[10] * tmp[0]);
}

se3 Ad(const SE3& T, const Vec3& v)
{
    return se3(	SCALAR_0, SCALAR_0, SCALAR_0,
                T[0] * v[0] + T[3] * v[1] + T[6] * v[2],
                T[1] * v[0] + T[4] * v[1] + T[7] * v[2],
                T[2] * v[0] + T[5] * v[1] + T[8] * v[2]);
}

// re = Inv(T) * s * T
se3 InvAd(const SE3& T, const se3& s)
{
    double tmp[3] = {	s[3] + s[1] * T[11] - s[2] * T[10],
                        s[4] + s[2] * T[9] - s[0] * T[11],
                        s[5] + s[0] * T[10] - s[1] * T[9] };
    return se3(	T[0] * s[0] + T[1] * s[1] + T[2] * s[2],
                T[3] * s[0] + T[4] * s[1] + T[5] * s[2],
                T[6] * s[0] + T[7] * s[1] + T[8] * s[2],
                T[0] * tmp[0] + T[1] * tmp[1] + T[2] * tmp[2],
                T[3] * tmp[0] + T[4] * tmp[1] + T[5] * tmp[2],
                T[6] * tmp[0] + T[7] * tmp[1] + T[8] * tmp[2]);
}

se3 ad(const se3& s1, const se3& s2)
{
    return se3(	s1[1] * s2[2] - s1[2] * s2[1],
                s1[2] * s2[0] - s1[0] * s2[2],
                s1[0] * s2[1] - s1[1] * s2[0],
                s1[1] * s2[5] - s1[2] * s2[4] - s2[1] * s1[5] + s2[2] * s1[4],
                s1[2] * s2[3] - s1[0] * s2[5] - s2[2] * s1[3] + s2[0] * s1[5],
                s1[0] * s2[4] - s1[1] * s2[3] - s2[0] * s1[4] + s2[1] * s1[3]);
}

double SquareSum(const se3& s)
{
    return (s[0] * s[0] + s[1] * s[1] + s[2] * s[2] + s[3] * s[3] + s[4] * s[4] + s[5] * s[5]);
}

se3 Rotate(const SE3& T, const se3& v)
{
    return se3(	T[0] * v[0] + T[3] * v[1] + T[6] * v[2],
                T[1] * v[0] + T[4] * v[1] + T[7] * v[2],
                T[2] * v[0] + T[5] * v[1] + T[8] * v[2],
                T[0] * v[3] + T[3] * v[4] + T[6] * v[5],
                T[1] * v[3] + T[4] * v[4] + T[7] * v[5],
                T[2] * v[3] + T[5] * v[4] + T[8] * v[5]);
}

se3 InvRotate(const SE3& T, const se3& v)
{
    return se3(	T[0] * v[0] + T[1] * v[1] + T[2] * v[2],
                T[3] * v[0] + T[4] * v[1] + T[5] * v[2],
                T[6] * v[0] + T[7] * v[1] + T[8] * v[2],
                T[0] * v[3] + T[1] * v[4] + T[2] * v[5],
                T[3] * v[3] + T[4] * v[4] + T[5] * v[5],
                T[6] * v[3] + T[7] * v[4] + T[8] * v[5]);
}

//==============================================================================
//
//==============================================================================
dse3::dse3()
{
    _m[0] = _m[1] = _m[2] = _m[3] = _m[4] = _m[5] = 0.0;
}

dse3::dse3(double k)
{
    _m[0] = _m[1] = _m[2] = _m[3] = _m[4] = _m[5] = k;
}

dse3::dse3(double m0, double m1, double m2, double m3, double m4, double m5)
{
    _m[0] = m0;
    _m[1] = m1;
    _m[2] = m2;
    _m[3] = m3;
    _m[4] = m4;
    _m[5] = m5;
}

dse3::dse3(const Axis& m, const Vec3& f)
{
    _m[0] = m[0];
    _m[1] = m[1];
    _m[2] = m[2];
    _m[3] = f[0];
    _m[4] = f[1];
    _m[5] = f[2];
}

dse3::dse3(double mass, const se3& dV)
{
    _m[0] = mass * dV[0];
    _m[1] = mass * dV[1];
    _m[2] = mass * dV[2];
    _m[3] = mass * dV[3];
    _m[4] = mass * dV[4];
    _m[5] = mass * dV[5];
}

const dse3& dse3::operator+(void) const
{
    return *this;
}

dse3 dse3::operator-(void) const
{
    return dse3(-_m[0], -_m[1], -_m[2], -_m[3], -_m[4], -_m[5]);
}

const dse3& dse3::operator = (const dse3& t)
{
    _m[0] = t[0];
    _m[1] = t[1];
    _m[2] = t[2];
    _m[3] = t[3];
    _m[4] = t[4];
    _m[5] = t[5];
    return *this;
}

const dse3& dse3::operator = (const Axis& t)
{
    _m[0] = t[0];
    _m[1] = t[1];
    _m[2] = t[2];
    _m[3] = _m[4] = _m[5] = SCALAR_0;
    return *this;
}

const dse3& dse3::operator = (const Vec3& t)
{
    _m[0] = _m[1] =  _m[2] = SCALAR_0;
    _m[3] = t[0];
    _m[4] = t[1];
    _m[5] = t[2];
    return *this;
}

const dse3& dse3::operator = (double d)
{
    _m[0] = _m[1] = _m[2] = _m[3] = _m[4] = _m[5] = d;
    return *this;
}

const dse3& dse3::operator += (const dse3& t)
{
    _m[0] += t[0];
    _m[1] += t[1];
    _m[2] += t[2];
    _m[3] += t[3];
    _m[4] += t[4];
    _m[5] += t[5];
    return *this;
}

const dse3& dse3::operator += (const Axis& t)
{
    _m[0] += t[0];
    _m[1] += t[1];
    _m[2] += t[2];
    return *this;
}

const dse3& dse3::operator-= (const dse3& t)
{
    _m[0] -= t[0];
    _m[1] -= t[1];
    _m[2] -= t[2];
    _m[3] -= t[3];
    _m[4] -= t[4];
    _m[5] -= t[5];
    return *this;
}

const dse3& dse3::operator *= (double d)
{
    _m[0] *= d;
    _m[1] *= d;
    _m[2] *= d;
    _m[3] *= d;
    _m[4] *= d;
    _m[5] *= d;
    return *this;
}

dse3 dse3::operator+(const dse3& t) const
{
    return dse3(_m[0] + t[0], _m[1] + t[1], _m[2] + t[2], _m[3] + t[3], _m[4] + t[4], _m[5] + t[5]);
}

dse3 dse3::operator-(const dse3& t) const
{
    return dse3(_m[0] - t[0], _m[1] - t[1], _m[2] - t[2], _m[3] - t[3], _m[4] - t[4], _m[5] - t[5]);
}

dse3 dse3::operator*(double d) const
{
    return dse3(d * _m[0], d * _m[1], d * _m[2], d * _m[3], d * _m[4], d * _m[5]);
}

double& dse3::operator[](int i)
{
    return _m[i];
}

const double& dse3::operator[](int i) const
{
    return _m[i];
}

dse3 dad(const se3& s, const dse3& t)
{
    return dse3(t[1] * s[2] - t[2] * s[1] + t[4] * s[5] - t[5] * s[4],
                t[2] * s[0] - t[0] * s[2] + t[5] * s[3] - t[3] * s[5],
                t[0] * s[1] - t[1] * s[0] + t[3] * s[4] - t[4] * s[3],
                t[4] * s[2] - t[5] * s[1],
                t[5] * s[0] - t[3] * s[2],
                t[3] * s[1] - t[4] * s[0]);
}

void dse3::dad(const se3& s, const dse3& t)
{
    _m[0] =	t[1] * s[2] - t[2] * s[1] + t[4] * s[5] - t[5] * s[4];
    _m[1] =	t[2] * s[0] - t[0] * s[2] + t[5] * s[3] - t[3] * s[5];
    _m[2] =	t[0] * s[1] - t[1] * s[0] + t[3] * s[4] - t[4] * s[3];
    _m[3] =	t[4] * s[2] - t[5] * s[1];
    _m[4] =	t[5] * s[0] - t[3] * s[2];
    _m[5] =	t[3] * s[1] - t[4] * s[0];
}

void dse3::dAd(const SE3& T, const dse3& t)
{
    double tmp[3] = {	t[0] - T[10] * t[5] + T[11] * t[4],
                        t[1] - T[11] * t[3] + T[9] * t[5],
                        t[2] - T[9] * t[4] + T[10] * t[3] };
    _m[0] = T[0] * tmp[0] + T[1] * tmp[1] + T[2] * tmp[2];
    _m[1] = T[3] * tmp[0] + T[4] * tmp[1] + T[5] * tmp[2];
    _m[2] = T[6] * tmp[0] + T[7] * tmp[1] + T[8] * tmp[2];
    _m[3] = T[0] * t[3] + T[1] * t[4] + T[2] * t[5];
    _m[4] = T[3] * t[3] + T[4] * t[4] + T[5] * t[5];
    _m[5] = T[6] * t[3] + T[7] * t[4] + T[8] * t[5];
}

//==============================================================================
//
//==============================================================================
dse3 operator*(double d, const dse3& t)
{
    return dse3(d * t[0], d * t[1], d * t[2], d * t[3], d * t[4], d * t[5]);
}

dse3 dAd(const SE3& T, const dse3& t)
{
    double tmp[3] = {	t[0] - T[10] * t[5] + T[11] * t[4],
                        t[1] - T[11] * t[3] + T[9] * t[5],
                        t[2] - T[9] * t[4] + T[10] * t[3] };
    return dse3(T[0] * tmp[0] + T[1] * tmp[1] + T[2] * tmp[2],
                T[3] * tmp[0] + T[4] * tmp[1] + T[5] * tmp[2],
                T[6] * tmp[0] + T[7] * tmp[1] + T[8] * tmp[2],
                T[0] * t[3] + T[1] * t[4] + T[2] * t[5],
                T[3] * t[3] + T[4] * t[4] + T[5] * t[5],
                T[6] * t[3] + T[7] * t[4] + T[8] * t[5]);
}

dse3 dAd(const SE3& T, const Vec3& f)
{
    double tmp[3] = {	- T[10] * f[2] + T[11] * f[1],
                        - T[11] * f[0] + T[9] * f[2],
                        - T[9] * f[1] + T[10] * f[0] };
    return dse3(T[0] * tmp[0] + T[1] * tmp[1] + T[2] * tmp[2],
                T[3] * tmp[0] + T[4] * tmp[1] + T[5] * tmp[2],
                T[6] * tmp[0] + T[7] * tmp[1] + T[8] * tmp[2],
                T[0] * f[0] + T[1] * f[1] + T[2] * f[2],
                T[3] * f[0] + T[4] * f[1] + T[5] * f[2],
                T[6] * f[0] + T[7] * f[1] + T[8] * f[2]);
}

dse3 InvdAd(const SE3& T, const dse3& t)
{
    double tmp[3] = {	T[0] * t[3] + T[3] * t[4] + T[6] * t[5],
                        T[1] * t[3] + T[4] * t[4] + T[7] * t[5],
                        T[2] * t[3] + T[5] * t[4] + T[8] * t[5] };
    return dse3(T[10] * tmp[2] - T[11] * tmp[1] + T[0] * t[0] + T[3] * t[1] + T[6] * t[2],
                T[11] * tmp[0] - T[9] * tmp[2] + T[1] * t[0] + T[4] * t[1] + T[7] * t[2],
                T[9] * tmp[1] - T[10] * tmp[0] + T[2] * t[0] + T[5] * t[1] + T[8] * t[2],
                tmp[0], tmp[1], tmp[2]);
}

dse3 InvdAd(const Vec3& p, const Vec3& f)
{
    return dse3(p[1] * f[2] - p[2] * f[1],
                p[2] * f[0] - p[0] * f[2],
                p[0] * f[1] - p[1] * f[0],
                f[0],
                f[1],
                f[2]);
}

double SquareSum(const dse3& t)
{
    return (t[0] * t[0] + t[1] * t[1] + t[2] * t[2] + t[3] * t[3] + t[4] * t[4] + t[5] * t[5]);
}

//==============================================================================
//
//==============================================================================
SE3::SE3()
{
    _T[0] = _T[4] = _T[8] = SCALAR_1;
    _T[1] = _T[2] = _T[3] = _T[5] = _T[6] = _T[7] = _T[9] = _T[10] = _T[11] = SCALAR_0;
}

SE3::SE3(const SE3& T)
{
    _T[0] = T._T[0];
    _T[1] = T._T[1];
    _T[2] = T._T[2];
    _T[3] = T._T[3];
    _T[4] = T._T[4];
    _T[5] = T._T[5];
    _T[6] = T._T[6];
    _T[7] = T._T[7];
    _T[8] = T._T[8];
    _T[9] = T._T[9];
    _T[10] = T._T[10];
    _T[11] = T._T[11];
}

SE3::SE3(double T0, double T1, double T2, double T4, double T5, double T6, double T8, double T9, double T10, double T12, double T13, double T14)
{
    _T[0] = T0;
    _T[1] = T1;
    _T[2] = T2;
    _T[3] = T4;
    _T[4] = T5;
    _T[5] = T6;
    _T[6] = T8;
    _T[7] = T9;
    _T[8] = T10;
    _T[9] = T12;
    _T[10] = T13;
    _T[11] = T14;
}

SE3::SE3(double T0, double T1, double T2, double T4, double T5, double T6, double T8, double T9, double T10)
{
    _T[0] = T0;
    _T[1] = T1;
    _T[2] = T2;
    _T[3] = T4;
    _T[4] = T5;
    _T[5] = T6;
    _T[6] = T8;
    _T[7] = T9;
    _T[8] = T10;
    _T[9] = _T[10] = _T[11] = SCALAR_0;
}

SE3::SE3(const Vec3& p)
{
    _T[0] = _T[4] = _T[8] = SCALAR_1;
    _T[1] = _T[2] = _T[3] = _T[5] = _T[6] = _T[7] = SCALAR_0;
    _T[9] = p._v[0];
    _T[10] = p._v[1];
    _T[11] = p._v[2];
}

SE3::SE3(const Vec3& Rx, const Vec3& Ry, const Vec3& Rz, const Vec3& p)
{
    _T[0] = Rx[0];
    _T[1] = Rx[1];
    _T[2] = Rx[2];
    _T[3] = Ry[0];
    _T[4] = Ry[1];
    _T[5] = Ry[2];
    _T[6] = Rz[0];
    _T[7] = Rz[1];
    _T[8] = Rz[2];
    _T[9] = p[0];
    _T[10] = p[1];
    _T[11] = p[2];
}

SE3::SE3(double p)
{
    _T[0] = _T[4] = _T[8] = SCALAR_1;
    _T[1] = _T[2] = _T[3] = _T[5] = _T[6] = _T[7] = SCALAR_0;
    _T[9] = _T[10] = _T[11] = p;
}

SE3::SE3(int p)
{
    _T[0] = _T[4] = _T[8] = SCALAR_1;
    _T[1] = _T[2] = _T[3] = _T[5] = _T[6] = _T[7] = SCALAR_0;
    _T[9] = _T[10] = _T[11] = (double)p;
}

SE3::SE3(const double T[])
{
    _T[0] = T[0];
    _T[1] = T[1];
    _T[2] = T[2];
    _T[3] = T[4];
    _T[4] = T[5];
    _T[5] = T[6];
    _T[6] = T[8];
    _T[7] = T[9];
    _T[8] = T[10];
    _T[9] = T[12];
    _T[10] = T[13];
    _T[11] = T[14];
}

SE3::~SE3()
{
}

double SE3::operator()(int i, int j) const
{
    if (i == 3)
        return j == 3 ? SCALAR_1 : SCALAR_0;

    return _T[i + (3 * j)];
}

const double& SE3::operator[](int i) const
{
    return _T[i];
}

double& SE3::operator[](int i)
{
    return _T[i];
}

const SE3& SE3::operator=(const SE3& T)
{
    _T[0] = T[0];
    _T[1] = T[1];
    _T[2] = T[2];
    _T[3] = T[3];
    _T[4] = T[4];
    _T[5] = T[5];
    _T[6] = T[6];
    _T[7] = T[7];
    _T[8] = T[8];
    _T[9] = T[9];
    _T[10] = T[10];
    _T[11] = T[11];
    return *this;
}

const SE3& SE3::operator=(const Vec3& p)
{
    _T[0] = _T[4] = _T[8] = SCALAR_1;
    _T[1] = _T[2] = _T[3] = _T[5] = _T[6] = _T[7] = SCALAR_0;
    _T[9] = p[0];
    _T[10] = p[1];
    _T[11] = p[2];
    return *this;
}

SE3 SE3::operator*(const SE3& T) const
{
    return SE3(	_T[0] * T[0] + _T[3] * T[1] + _T[6] * T[2],
                _T[1] * T[0] + _T[4] * T[1] + _T[7] * T[2],
                _T[2] * T[0] + _T[5] * T[1] + _T[8] * T[2],
                _T[0] * T[3] + _T[3] * T[4] + _T[6] * T[5],
                _T[1] * T[3] + _T[4] * T[4] + _T[7] * T[5],
                _T[2] * T[3] + _T[5] * T[4] + _T[8] * T[5],
                _T[0] * T[6] + _T[3] * T[7] + _T[6] * T[8],
                _T[1] * T[6] + _T[4] * T[7] + _T[7] * T[8],
                _T[2] * T[6] + _T[5] * T[7] + _T[8] * T[8],
                _T[9] + _T[0] * T[9] + _T[3] * T[10] + _T[6] * T[11],
                _T[10] + _T[1] * T[9] + _T[4] * T[10] + _T[7] * T[11],
                _T[11] + _T[2] * T[9] + _T[5] * T[10] + _T[8] * T[11] );
}

SE3 SE3::operator / (const SE3& T) const
{
    double tmp[] = {_T[0] * T[0] + _T[3] * T[3] + _T[6] * T[6],
                    _T[1] * T[0] + _T[4] * T[3] + _T[7] * T[6],
                    _T[2] * T[0] + _T[5] * T[3] + _T[8] * T[6],
                    _T[0] * T[1] + _T[3] * T[4] + _T[6] * T[7],
                    _T[1] * T[1] + _T[4] * T[4] + _T[7] * T[7],
                    _T[2] * T[1] + _T[5] * T[4] + _T[8] * T[7],
                    _T[0] * T[2] + _T[3] * T[5] + _T[6] * T[8],
                    _T[1] * T[2] + _T[4] * T[5] + _T[7] * T[8],
                    _T[2] * T[2] + _T[5] * T[5] + _T[8] * T[8] };
    return SE3(	tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5], tmp[6], tmp[7], tmp[8],
                _T[9] - tmp[0] * T[9] - tmp[3] * T[10] - tmp[6] * T[11],
                _T[10] - tmp[1] * T[9] - tmp[4] * T[10] - tmp[7] * T[11],
                _T[11] - tmp[2] * T[9] - tmp[5] * T[10] - tmp[8] * T[11] );
}

SE3 SE3::operator % (const SE3& T) const
{
    return SE3(	_T[0] * T[0] + _T[1] * T[1] + _T[2] * T[2],
                _T[3] * T[0] + _T[4] * T[1] + _T[5] * T[2],
                _T[6] * T[0] + _T[7] * T[1] + _T[8] * T[2],
                _T[0] * T[3] + _T[1] * T[4] + _T[2] * T[5],
                _T[3] * T[3] + _T[4] * T[4] + _T[5] * T[5],
                _T[6] * T[3] + _T[7] * T[4] + _T[8] * T[5],
                _T[0] * T[6] + _T[1] * T[7] + _T[2] * T[8],
                _T[3] * T[6] + _T[4] * T[7] + _T[5] * T[8],
                _T[6] * T[6] + _T[7] * T[7] + _T[8] * T[8],
                _T[0] * (T[9] - _T[9]) + _T[1] * (T[10] - _T[10]) + _T[2] * (T[11] - _T[11]),
                _T[3] * (T[9] - _T[9]) + _T[4] * (T[10] - _T[10]) + _T[5] * (T[11] - _T[11]),
                _T[6] * (T[9] - _T[9]) + _T[7] * (T[10] - _T[10]) + _T[8] * (T[11] - _T[11]) );
}

Vec3 SE3::operator % (const Vec3& p) const
{
    return Vec3(_T[0] * (p[0] - _T[9]) + _T[1] * (p[1] - _T[10]) + _T[2] * (p[2] - _T[11]),
                _T[3] * (p[0] - _T[9]) + _T[4] * (p[1] - _T[10]) + _T[5] * (p[2] - _T[11]),
                _T[6] * (p[0] - _T[9]) + _T[7] * (p[1] - _T[10]) + _T[8] * (p[2] - _T[11]) );
}

Vec3 SE3::operator*(const Vec3& p) const
{
    return Vec3(_T[9] + _T[0] * p[0] + _T[3] * p[1] + _T[6] * p[2],
                _T[10] + _T[1] * p[0] + _T[4] * p[1] + _T[7] * p[2],
                _T[11] + _T[2] * p[0] + _T[5] * p[1] + _T[8] * p[2] );
}

const SE3& SE3::operator *= (const SE3& T)
{
    double x0, x1, x2;
    _T[9] += _T[0] * T[9] + _T[3] * T[10] + _T[6] * T[11];
    _T[10] += _T[1] * T[9] + _T[4] * T[10] + _T[7] * T[11];
    _T[11] += _T[2] * T[9] + _T[5] * T[10] + _T[8] * T[11];
    x0 = _T[0] * T[0] + _T[3] * T[1] + _T[6] * T[2];
    x1 = _T[0] * T[3] + _T[3] * T[4] + _T[6] * T[5];
    x2 = _T[0] * T[6] + _T[3] * T[7] + _T[6] * T[8];
    _T[0] = x0;	_T[3] = x1;	_T[6] = x2;
    x0 = _T[1] * T[0] + _T[4] * T[1] + _T[7] * T[2];
    x1 = _T[1] * T[3] + _T[4] * T[4] + _T[7] * T[5];
    x2 = _T[1] * T[6] + _T[4] * T[7] + _T[7] * T[8];
    _T[1] = x0;	_T[4] =x1;	_T[7] = x2;
    x0 = _T[2] * T[0] + _T[5] * T[1] + _T[8] * T[2];
    x1 = _T[2] * T[3] + _T[5] * T[4] + _T[8] * T[5];
    x2 = _T[2] * T[6] + _T[5] * T[7] + _T[8] * T[8];
    _T[2] = x0; _T[5] = x1; _T[8] = x2;
    return  *this;
}

const SE3& SE3::operator /= (const SE3& T)
{
    double tmp[] = {_T[0] * T[0] + _T[3] * T[3] + _T[6] * T[6],
                    _T[1] * T[0] + _T[4] * T[3] + _T[7] * T[6],
                    _T[2] * T[0] + _T[5] * T[3] + _T[8] * T[6],
                    _T[0] * T[1] + _T[3] * T[4] + _T[6] * T[7],
                    _T[1] * T[1] + _T[4] * T[4] + _T[7] * T[7],
                    _T[2] * T[1] + _T[5] * T[4] + _T[8] * T[7],
                    _T[0] * T[2] + _T[3] * T[5] + _T[6] * T[8],
                    _T[1] * T[2] + _T[4] * T[5] + _T[7] * T[8],
                    _T[2] * T[2] + _T[5] * T[5] + _T[8] * T[8] };
    _T[0] = tmp[0]; _T[1] = tmp[1]; _T[2] = tmp[2];
    _T[3] = tmp[3]; _T[4] = tmp[4]; _T[5] = tmp[5];
    _T[6] = tmp[6]; _T[7] = tmp[7]; _T[8] = tmp[8],
    _T[9] -= tmp[0] * T[9] + tmp[3] * T[10] + tmp[6] * T[11];
    _T[10] -= tmp[1] * T[9] + tmp[4] * T[10] + tmp[7] * T[11];
    _T[11] -= tmp[2] * T[9] + tmp[5] * T[10] + tmp[8] * T[11];
    return *this;
}

const SE3& SE3::operator %= (const SE3& T)
{
    double tmp[12] = { _T[0], _T[1], _T[2], _T[3], _T[4], _T[5], _T[6], _T[7], _T[8], T[9] - _T[9], T[10] - _T[10], T[11] - _T[11] };
    _T[0] = tmp[0] * T[0] + tmp[1] * T[1] + tmp[2] * T[2];
    _T[1] = tmp[3] * T[0] + tmp[4] * T[1] + tmp[5] * T[2];
    _T[2] = tmp[6] * T[0] + tmp[7] * T[1] + tmp[8] * T[2];
    _T[3] = tmp[0] * T[3] + tmp[1] * T[4] + tmp[2] * T[5];
    _T[4] = tmp[3] * T[3] + tmp[4] * T[4] + tmp[5] * T[5];
    _T[5] = tmp[6] * T[3] + tmp[7] * T[4] + tmp[8] * T[5];
    _T[6] = tmp[0] * T[6] + tmp[1] * T[7] + tmp[2] * T[8];
    _T[7] = tmp[3] * T[6] + tmp[4] * T[7] + tmp[5] * T[8];
    _T[8] = tmp[6] * T[6] + tmp[7] * T[7] + tmp[8] * T[8];
    _T[9] = tmp[0] * tmp[9] + tmp[1] * tmp[10] + tmp[2] * tmp[11];
    _T[10] = tmp[3] * tmp[9] + tmp[4] * tmp[10] + tmp[5] * tmp[11];
    _T[11] = tmp[6] * tmp[9] + tmp[7] * tmp[10] + tmp[8] * tmp[11];
    return *this;
}

bool SE3::operator==(const SE3& T) const
{
    for (int i = 0; i < 12; ++i)
        if (_T[i] != T._T[i])
            return false;

    return true;
}

bool SE3::operator!=(const SE3& T) const
{
    return !(*this == T);
}

void SE3::setIdentity(void)
{
    _T[0] = _T[4] = _T[8] = SCALAR_1;
    _T[1] = _T[2] = _T[3] = _T[5] = _T[6] = _T[7] = _T[9] = _T[10] = _T[11] = SCALAR_0;
}

void SE3::setOrientationPosition(const SE3& T, const Vec3& p)
{
    _T[0] = T[0];
    _T[1] = T[1];
    _T[2] = T[2];
    _T[3] = T[3];
    _T[4] = T[4];
    _T[5] = T[5];
    _T[6] = T[6];
    _T[7] = T[7];
    _T[8] = T[8];
    _T[9] = p[0];
    _T[10] = p[1];
    _T[11] = p[2];
}

void SE3::setOrientation(const SE3& T)
{
    _T[0] = T[0];
    _T[1] = T[1];
    _T[2] = T[2];
    _T[3] = T[3];
    _T[4] = T[4];
    _T[5] = T[5];
    _T[6] = T[6];
    _T[7] = T[7];
    _T[8] = T[8];
    _T[9] = T[9];
}

void SE3::setPosition(const Vec3& Pos)
{
    _T[9] = Pos[0];
    _T[10] = Pos[1];
    _T[11] = Pos[2];
}

Vec3 SE3::getPosition(void) const
{
    return Vec3(_T[9], _T[10], _T[11]);
}

void SE3::ToDoubleArray(double M[]) const
{
    M[0] = _T[0];
    M[1] = _T[1];
    M[2] = _T[2];
    M[3] = SCALAR_0;
    M[4] = _T[3];
    M[5] = _T[4];
    M[6] = _T[5];
    M[7] = SCALAR_0;
    M[8] = _T[6];
    M[9] = _T[7];
    M[10] = _T[8];
    M[11] = SCALAR_0;
    M[12] = _T[9];
    M[13] = _T[10];
    M[14] = _T[11];
    M[15] = SCALAR_1;
}

Eigen::Matrix4d SE3::getEigenMatrix() const
{
    Eigen::Matrix4d M = Eigen::Matrix4d::Identity();

    M(0,0) = _T[0];
    M(1,0) = _T[1];
    M(2,0) = _T[2];

    M(0,1) = _T[3];
    M(1,1) = _T[4];
    M(2,1) = _T[5];

    M(0,2) = _T[6];
    M(1,2) = _T[7];
    M(2,2) = _T[8];

    M(0,3) = _T[9];
    M(1,3) = _T[10];
    M(2,3) = _T[11];

    return M;
}

void SE3::Rectify(void)
{
    double _S[9];
    while ( true )
    {
        double idet = SCALAR_1 / (_T[0] * _T[4] * _T[8] - _T[0] * _T[7] * _T[5] - _T[1] * _T[3] * _T[8] + _T[1] * _T[6] * _T[5] + _T[2] * _T[3] * _T[7] - _T[2] * _T[6] * _T[4]);

        _S[0] = idet * (_T[4] * _T[8] - _T[7] * _T[5]);
        _S[1] = idet * (_T[6] * _T[5] - _T[3] * _T[8]);
        _S[2] = idet * (_T[3] * _T[7] - _T[6] * _T[4]);
        _S[3] = idet * (_T[7] * _T[2] - _T[1] * _T[8]);
        _S[4] = idet * (_T[0] * _T[8] - _T[6] * _T[2]);
        _S[5] = idet * (_T[6] * _T[1] - _T[0] * _T[7]);
        _S[6] = idet * (_T[1] * _T[5] - _T[4] * _T[2]);
        _S[7] = idet * (_T[3] * _T[2] - _T[0] * _T[5]);
        _S[8] = idet * (_T[0] * _T[4] - _T[3] * _T[1]);

        _T[0] = SCALAR_1_2 * (_T[0] + _S[0]);
        _T[1] = SCALAR_1_2 * (_T[1] + _S[1]);
        _T[2] = SCALAR_1_2 * (_T[2] + _S[2]);
        _T[3] = SCALAR_1_2 * (_T[3] + _S[3]);
        _T[4] = SCALAR_1_2 * (_T[4] + _S[4]);
        _T[5] = SCALAR_1_2 * (_T[5] + _S[5]);
        _T[6] = SCALAR_1_2 * (_T[6] + _S[6]);
        _T[7] = SCALAR_1_2 * (_T[7] + _S[7]);
        _T[8] = SCALAR_1_2 * (_T[8] + _S[8]);

        if ( abs(_S[0] - _T[0]) + abs(_S[1] - _T[1]) + abs(_S[2] - _T[2]) + abs(_S[3] - _T[3]) + abs(_S[4] - _T[4]) + abs(_S[5] - _T[5]) + abs(_S[6] - _T[6]) + abs(_S[7] - _T[7]) + abs(_S[8] - _T[8]) < LIE_EPS ) break;
    }
}

void SE3::ToQuaternion(double q[4]) const
{
    q[0] = SCALAR_1_2 * sqrt(_T[0] + _T[4] + _T[8] + SCALAR_1);
    double demon = SCALAR_1_4 / q[0];
    q[1] = demon * (_T[5] - _T[7]);
    q[2] = demon * (_T[6] - _T[2]);
    q[3] = demon * (_T[1] - _T[3]);
}

//==============================================================================
//
//==============================================================================
SE3 EulerZYX(const Vec3& angle)
{
    double ca = cos(angle[0]), sa = sin(angle[0]), cb = cos(angle[1]), sb = sin(angle[1]), cg = cos(angle[2]), sg = sin(angle[2]);
    return SE3(ca * cb, sa * cb, -sb, ca * sb * sg - sa * cg, sa * sb * sg + ca * cg, cb * sg, ca * sb * cg + sa * sg, sa * sb * cg - ca * sg, cb * cg);
}

SE3 EulerZYZ(const Vec3& angle)
{
    double ca = cos(angle[0]), sa = sin(angle[0]), cb = cos(angle[1]), sb = sin(angle[1]), cg = cos(angle[2]), sg = sin(angle[2]);
    return SE3(ca * cb * cg - sa * sg, sa * cb * cg + ca * sg, -sb * cg,	-ca * cb * sg - sa * cg, ca * cg - sa * cb * sg, sb * sg, ca * sb, sa * sb, cb);
}

SE3 EulerXYZ(const Vec3& angle, const Vec3& pos)
{
    SE3 T = RotX(angle[0]) * RotY(angle[1]) * RotZ(angle[2]);
    T.setPosition(pos);
    return T;
}

SE3 EulerZYX(const Vec3& angle, const Vec3& pos)
{
    double ca = cos(angle[0]);
    double sa = sin(angle[0]);
    double cb = cos(angle[1]);
    double sb = sin(angle[1]);
    double cg = cos(angle[2]);
    double sg = sin(angle[2]);

    return SE3(ca * cb, sa * cb, -sb,
               ca * sb * sg - sa * cg, sa * sb * sg + ca * cg, cb * sg,
               ca * sb * cg + sa * sg, sa * sb * cg - ca * sg, cb * cg,
               pos[0], pos[1], pos[2]);
}

SE3 EulerZYZ(const Vec3& angle, const Vec3& pos)
{
    double ca = cos(angle[0]), sa = sin(angle[0]), cb = cos(angle[1]), sb = sin(angle[1]), cg = cos(angle[2]), sg = sin(angle[2]);
    return SE3(ca * cb * cg - sa * sg, sa * cb * cg + ca * sg, -sb * cg,	-ca * cb * sg - sa * cg, ca * cg - sa * cb * sg, sb * sg, ca * sb, sa * sb, cb, pos[0], pos[1], pos[2]);
}

// R = Exp(w)
// p = sin(t) / t * v + (t - sin(t)) / t^3 * <w, v> * w + (1 - cos(t)) / t^2 * (w X v)
// , when S = (w, v), t = |w|
SE3 Exp(const se3& S)
{
    double s2[] = { S[0] * S[0], S[1] * S[1], S[2] * S[2] };
    double s3[] = { S[0] * S[1], S[1] * S[2], S[2] * S[0] };
    double theta = sqrt(s2[0] + s2[1] + s2[2]), cos_t = cos(theta), alpha, beta, gamma;

    if ( theta > LIE_EPS )
    {
        double sin_t = sin(theta);
        alpha = sin_t / theta;
        beta = (SCALAR_1 - cos_t) / theta / theta;
        gamma = (S[0] * S[3] + S[1] * S[4] + S[2] * S[5]) * (theta - sin_t) / theta / theta / theta;
    } else
    {
        alpha = SCALAR_1 - SCALAR_1_6 * theta * theta;
        beta = SCALAR_1_2 - SCALAR_1_24 * theta * theta;
        gamma = (S[0] * S[3] + S[1] * S[4] + S[2] * S[5]) * SCALAR_1_6 - SCALAR_1_120 * theta * theta;
    }

    return SE3(	beta * s2[0] + cos_t, beta * s3[0] + alpha * S[2], beta * s3[2] - alpha * S[1],
                beta * s3[0] - alpha * S[2], beta * s2[1] + cos_t, beta * s3[1] + alpha * S[0],
                beta * s3[2] + alpha * S[1], beta * s3[1] - alpha * S[0], beta * s2[2] + cos_t,
                alpha * S[3] + beta * (S[1] * S[5] - S[2] * S[4]) + gamma * S[0],
                alpha * S[4] + beta * (S[2] * S[3] - S[0] * S[5]) + gamma * S[1],
                alpha * S[5] + beta * (S[0] * S[4] - S[1] * S[3]) + gamma * S[2]);
}

// I + sin(t) / t * [S] + (1 - cos(t)) / t^2 * [S]^2, where t = |S|
SE3 Exp(const Axis& S)
{
    double s2[] = { S[0] * S[0], S[1] * S[1], S[2] * S[2] };
    double s3[] = { S[0] * S[1], S[1] * S[2], S[2] * S[0] };
    double theta = sqrt(s2[0] + s2[1] + s2[2]), cos_t = cos(theta), alpha, beta;

    if ( theta > LIE_EPS )
    {
        alpha = sin(theta) / theta;
        beta = (SCALAR_1 - cos_t) / theta / theta;
    } else
    {
        alpha = SCALAR_1 - SCALAR_1_6 * theta * theta;
        beta = SCALAR_1_2 - SCALAR_1_24 * theta * theta;
    }

    return SE3( beta * s2[0] + cos_t, beta * s3[0] + alpha * S[2], beta * s3[2] - alpha * S[1],
                beta * s3[0] - alpha * S[2], beta * s2[1] + cos_t, beta * s3[1] + alpha * S[0],
                beta * s3[2] + alpha * S[1], beta * s3[1] - alpha * S[0], beta * s2[2] + cos_t);
}

// I + sin(t) * [S] + (1 - cos(t)) * [S]^2,, where |S| = 1
SE3 Exp(const Axis& S, double theta)
{
    double s2[] = { S[0] * S[0], S[1] * S[1], S[2] * S[2] };

    if ( abs(s2[0] + s2[1] + s2[2] - SCALAR_1) > LIE_EPS ) return Exp(theta * S);

    double s3[] = { S[0] * S[1], S[1] * S[2], S[2] * S[0] };
    double alpha = sin(theta), cos_t = cos(theta), beta = SCALAR_1 - cos_t;

    return SE3( beta * s2[0] + cos_t, beta * s3[0] + alpha * S[2], beta * s3[2] - alpha * S[1],
                beta * s3[0] - alpha * S[2], beta * s2[1] + cos_t, beta * s3[1] + alpha * S[0],
                beta * s3[2] + alpha * S[1], beta * s3[1] - alpha * S[0], beta * s2[2] + cos_t);
}

SE3 Inv(const SE3& T)
{
    return SE3(	T[0], T[3], T[6], T[1], T[4], T[7], T[2], T[5], T[8],
                -T[0] * T[9] - T[1] * T[10] - T[2] * T[11],
                -T[3] * T[9] - T[4] * T[10] - T[5] * T[11],
                -T[6] * T[9] - T[7] * T[10] - T[8] * T[11]);
}

SE3 RotX(double t)
{
    double c = cos(t), s = sin(t);
    return SE3(SCALAR_1, SCALAR_0, SCALAR_0, SCALAR_0, c, s, SCALAR_0, -s, c);
}

SE3 RotY(double t)
{
    double c = cos(t), s = sin(t);
    return SE3(c, SCALAR_0, -s, SCALAR_0, SCALAR_1, SCALAR_0, s, SCALAR_0, c);
}

SE3 RotZ(double t)
{
    double c = cos(t), s = sin(t);
    return SE3(c, s, SCALAR_0, -s, c, SCALAR_0, SCALAR_0, SCALAR_0, SCALAR_1);
}

// invskew(T - I)
se3 Linearize(const SE3& T)
{
    return se3(SCALAR_1_2 * (T[5] - T[7]), SCALAR_1_2 * (T[6] - T[2]), SCALAR_1_2 * (T[1] - T[3]), T[9], T[10], T[11]);
}

SE3 Normalize(const SE3& T)
{
    double idet = SCALAR_1 / (T._T[0] * (T._T[4] * T._T[8] - T._T[5] * T._T[7]) + T._T[3] * (T._T[2] * T._T[7] - T._T[1] * T._T[8]) + T._T[6] * (T._T[1] * T._T[5] - T._T[2] * T._T[4]));

    return SE3(	SCALAR_1_2 * (T._T[0] + idet * (T._T[4] * T._T[8] - T._T[5] * T._T[7])),
                SCALAR_1_2 * (T._T[1] + idet * (T._T[5] * T._T[6] - T._T[3] * T._T[8])),
                SCALAR_1_2 * (T._T[2] + idet * (T._T[3] * T._T[7] - T._T[4] * T._T[6])),
                SCALAR_1_2 * (T._T[3] + idet * (T._T[2] * T._T[7] - T._T[1] * T._T[8])),
                SCALAR_1_2 * (T._T[4] + idet * (T._T[0] * T._T[8] - T._T[2] * T._T[6])),
                SCALAR_1_2 * (T._T[5] + idet * (T._T[1] * T._T[6] - T._T[0] * T._T[7])),
                SCALAR_1_2 * (T._T[6] + idet * (T._T[1] * T._T[5] - T._T[2] * T._T[4])),
                SCALAR_1_2 * (T._T[7] + idet * (T._T[2] * T._T[3] - T._T[0] * T._T[5])),
                SCALAR_1_2 * (T._T[8] + idet * (T._T[0] * T._T[4] - T._T[1] * T._T[3])),
                T._T[9], T._T[10], T._T[11]);
}

SE3 Quaternion2SE3(const double q[4])
{
    double q11 = q[1] * q[1];
    double q22 = q[2] * q[2];
    double q33 = q[3] * q[3];
    double q12 = q[1] * q[2];
    double q13 = q[1] * q[3];
    double q23 = q[2] * q[3];
    double q01 = q[0] * q[1];
    double q02 = q[0] * q[2];
    double q03 = q[0] * q[3];

    assert(abs(q[0] * q[0] + q11 + q22 + q33 - SCALAR_1) < LIE_EPS && "Quaternion2SE3() --> not unit quaternion");

    return SE3( SCALAR_1 - SCALAR_2 * (q22 + q33), SCALAR_2 * (q12 + q03), SCALAR_2 * (q13 - q02),
                SCALAR_2 * (q12 - q03), SCALAR_1 - SCALAR_2 * (q11 + q33), SCALAR_2 * (q01 + q23),
                SCALAR_2 * (q02 + q13), SCALAR_2 * (q23 - q01), SCALAR_1 - SCALAR_2 * (q11 + q22));
}

//==============================================================================
//
//==============================================================================
Inertia::Inertia()
{
    _I[9] = 1.0;
    _I[0] = _I[1] = _I[2] = 1.0;
    _I[3] = _I[4] = _I[5] = _I[6] = _I[7] = _I[8] = SCALAR_0;
}

Inertia::Inertia(double mass, double Ixx, double Iyy, double Izz)
{
    assert(mass >= 0 && Ixx >= 0 && Iyy >= 0 && Izz >= 0
          && "A mass of this inertia tensor is not positive definate.");

    _I[9] = mass;
    _I[0] = Ixx; _I[1] = Iyy; _I[2] = Izz;
    _I[3] = _I[4] = _I[5] = _I[6] = _I[7] = _I[8] = SCALAR_0;
}

Inertia::Inertia(double Ixx, double Iyy, double Izz,
                        double Ixy, double Iyz, double Izx,
                        double r0, double r1, double r2,
                        double mass)
{
    _I[0] = Ixx;
    _I[1] = Iyy;
    _I[2] = Izz;

    _I[3] = Ixy;
    _I[4] = Izx;
    _I[5] = Iyz;

    _I[6] = r0;
    _I[7] = r1;
    _I[8] = r2;

    _I[9] = mass;
}

Inertia::~Inertia()
{
}

Inertia Inertia::Transform(const SE3& T) const
{
    double j0 = _I[0] + _I[9] * T[11] * T[11] + _I[9] * T[10] * T[10] - SCALAR_2 * _I[8] * T[11] - SCALAR_2 * _I[7] * T[10];
    double j1 = _I[1] + _I[9] * T[11] * T[11] + _I[9] * T[9] * T[9] - SCALAR_2 * _I[8] * T[11] - SCALAR_2 * _I[6] * T[9];
    double j2 = _I[2] + _I[9] * T[10] * T[10] + _I[9] * T[9] * T[9] - SCALAR_2 * _I[7] * T[10] - SCALAR_2 * _I[6] * T[9];
    double j3 = _I[3] + T[10] * _I[6] + T[9] * _I[7] - _I[9] * T[10] * T[9];
    double j4 = _I[4] + T[11] * _I[6] + T[9] * _I[8] - _I[9] * T[11] * T[9];
    double j5 = _I[5] + T[11] * _I[7] + T[10] * _I[8] - _I[9] * T[11] * T[10];
    double t0 = T[0] * j0 + T[1] * j3 + T[2] * j4;
    double t1 = T[3] * j0 + T[4] * j3 + T[5] * j4;
    double t2 = T[6] * j0 + T[7] * j3 + T[8] * j4;
    double t3 = T[0] * j3 + T[1] * j1 + T[2] * j5;
    double t4 = T[3] * j3 + T[4] * j1 + T[5] * j5;
    double t5 = T[6] * j3 + T[7] * j1 + T[8] * j5;
    double t6 = T[0] * j4 + T[1] * j5 + T[2] * j2;
    double t7 = T[3] * j4 + T[4] * j5 + T[5] * j2;
    double t8 = T[6] * j4 + T[7] * j5 + T[8] * j2;

    return Inertia(	t0 * T[0] + t3 * T[1] + t6 * T[2],
                    t1 * T[3] + t4 * T[4] + t7 * T[5],
                    t2 * T[6] + t5 * T[7] + t8 * T[8],
                    t1 * T[0] + t4 * T[1] + t7 * T[2],
                    t2 * T[0] + t5 * T[1] + t8 * T[2],
                    t2 * T[3] + t5 * T[4] + t8 * T[5],
                    T[0] * (_I[6] - _I[9] * T[9]) + T[1] * (_I[7] - _I[9] * T[10]) + T[2] * (_I[8] - _I[9] * T[11]),
                    T[3] * (_I[6] - _I[9] * T[9]) + T[4] * (_I[7] - _I[9] * T[10]) + T[5] * (_I[8] - _I[9] * T[11]),
                    T[6] * (_I[6] - _I[9] * T[9]) + T[7] * (_I[7] - _I[9] * T[10]) + T[8] * (_I[8] - _I[9] * T[11]),
            _I[9]);
}

dse3 Inertia::operator*(const se3& s) const
{
    // M = I * w + m * (r X v - r X (r X w))
    // F = m * (v - r X w)

    // r X w
    double rw0 =_I[7] * s[2] - _I[8] * s[1];
    double rw1 =_I[8] * s[0] - _I[6] * s[2];
    double rw2 =_I[6] * s[1] - _I[7] * s[0];

    // r X v - r X (r x w)
    double R0 = (_I[7] * s[5] - _I[8] * s[4]) - (_I[7] * rw2 - _I[8] * rw1);
    double R1 = (_I[8] * s[3] - _I[6] * s[5]) - (_I[8] * rw0 - _I[6] * rw2);
    double R2 = (_I[6] * s[4] - _I[7] * s[3]) - (_I[6] * rw1 - _I[7] * rw0);

    // M = I * w + m * R
    // F = m * (v - rw)
    return dse3(_I[0] * s[0] + _I[3] * s[1] + _I[4] * s[2] + _I[9] * R0, // M[0]
                _I[3] * s[0] + _I[1] * s[1] + _I[5] * s[2] + _I[9] * R1, // M[1]
                _I[4] * s[0] + _I[5] * s[1] + _I[2] * s[2] + _I[9] * R2, // M[2]
                _I[9] * (s[3] - rw0),									 // F[0]
                _I[9] * (s[4] - rw1),									 // F[1]
                _I[9] * (s[5] - rw2));									 // F[2]
}

dse3 Inertia::operator*(const Axis& s) const
{
    return dse3(_I[0] * s[0] + _I[3] * s[1] + _I[4] * s[2],
                _I[3] * s[0] + _I[1] * s[1] + _I[5] * s[2],
                _I[4] * s[0] + _I[5] * s[1] + _I[2] * s[2],
                s[1] * _I[8] - s[2] * _I[7],
                s[2] * _I[6] - s[0] * _I[8],
                s[0] * _I[7] - s[1] * _I[6]);
}

dse3 Inertia::operator*(const Vec3& s) const
{
    return dse3(_I[7] * s[2] - _I[8] * s[1],
                _I[8] * s[0] - _I[6] * s[2],
                _I[6] * s[1] - _I[7] * s[0],
                _I[9] * s[0], _I[9] * s[1], _I[9] * s[2]);
}

double& Inertia::operator[](int i)
{
    return _I[i];
}

const double	&Inertia::operator[](int i) const
{
    return _I[i];
}

void Inertia::toDoubleArray(double M[]) const
{
    // G = | I - m * [r] * [r]   m * [r] |
    //     |          -m * [r]     m * 1 |

    // m * r
    double mr0 = _I[9] * _I[6];
    double mr1 = _I[9] * _I[7];
    double mr2 = _I[9] * _I[8];

    // m * [r] * [r]
    double mr0r0 = mr0 * _I[6];
    double mr1r1 = mr1 * _I[7];
    double mr2r2 = mr2 * _I[8];
    double mr0r1 = mr0 * _I[7];
    double mr1r2 = mr1 * _I[8];
    double mr2r0 = mr2 * _I[6];

    M[0] =  _I[0] + mr1r1 + mr2r2;   M[6]  =  _I[3] - mr0r1;           M[12] =  _I[4] - mr2r0;           M[18] =    0.0;   M[24] =   -mr2;   M[30] =    mr1;
    M[1] =  M[6];                    M[7]  =  _I[1] + mr2r2 + mr0r0;   M[13] =  _I[5] - mr1r2;           M[19] =    mr2;   M[25] =    0.0;   M[31] =   -mr0;
    M[2] =  M[12];                   M[8]  =  M[13];                   M[14] =  _I[2] + mr0r0 + mr1r1;   M[20] =   -mr1;   M[26] =    mr0;   M[32] =    0.0;

    M[3] =   0.0;                    M[9]  =    mr2;                   M[15] =   -mr1;                   M[21] =  _I[9];   M[27] =    0.0;   M[33] =    0.0;
    M[4] =  -mr2;                    M[10] =    0.0;                   M[16] =    mr0;                   M[22] =    0.0;   M[28] =  _I[9];   M[34] =    0.0;
    M[5] =   mr1;                    M[11] =   -mr0;                   M[17] =    0.0;                   M[23] =    0.0;   M[29] =    0.0;   M[35] =  _I[9];
}

const Inertia& Inertia::operator=(const Inertia& I)
{
    _I[0] = I._I[0];
    _I[1] = I._I[1];
    _I[2] = I._I[2];
    _I[3] = I._I[3];
    _I[4] = I._I[4];
    _I[5] = I._I[5];
    _I[6] = I._I[6];
    _I[7] = I._I[7];
    _I[8] = I._I[8];
    _I[9] = I._I[9];

    return *this;
}

Inertia Inertia::operator+(const Inertia& I) const
{
    dterr << "Not implemented.\n";
    return Inertia();
}

void Inertia::setMass(double mass)
{
    assert(0.0 <= mass);

    _I[9] = mass;
}

double Inertia::getMass(void) const
{
    return _I[9];
}

void Inertia::setAngularMomentDiag(double Ixx, double Iyy, double Izz)
{
    setIxx(Ixx);
    setIyy(Iyy);
    setIzz(Izz);
}

Vec3 Inertia::getAngularMomentDiag() const
{
    return Vec3(getIxx(), getIyy(), getIzz());
}

void Inertia::setAngularMomentOffDiag(double Ixy, double Ixz, double Iyz)
{
    setIxy(Ixy);
    setIxz(Ixz);
    setIyz(Iyz);
}

Vec3 Inertia::getAngularMomentOffDiag() const
{
    return Vec3(getIxy(), getIxz(), getIxz());
}

void Inertia::setIxx(double Ixx)
{
    _I[0] = Ixx;
}

void Inertia::setIyy(double Iyy)
{
    _I[1] = Iyy;
}

void Inertia::setIzz(double Izz)
{
    _I[2] = Izz;
}

void Inertia::setIxy(double Ixy)
{
    _I[3] = Ixy;
}

void Inertia::setIxz(double Ixz)
{
    _I[4] = Ixz;
}

void Inertia::setIyz(double Iyz)
{
    _I[5] = Iyz;
}

double Inertia::getIxx() const
{
    return _I[0];
}

double Inertia::getIyy() const
{
    return _I[1];
}

double Inertia::getIzz() const
{
    return _I[2];
}

double Inertia::getIxy() const
{
    return _I[3];
}

double Inertia::getIxz() const
{
    return _I[4];
}

double Inertia::getIyz() const
{
    return _I[5];
}

void Inertia::setOffset(const Vec3& offset)
{
    _I[6] = offset[0];
    _I[7] = offset[1];
    _I[8] = offset[2];
}

Vec3 Inertia::getOffset() const
{
    return Vec3(_I[6], _I[7], _I[8]);
}

Inertia BoxInertia(double density, const Vec3& size)
{
    double mass = (double)8.0 * density * size[0] * size[1] * size[2];
    double ix = mass * (size[1] * size[1] + size[2] * size[2]) / SCALAR_3;
    double iy = mass * (size[0] * size[0] + size[2] * size[2]) / SCALAR_3;
    double iz = mass * (size[0] * size[0] + size[1] * size[1]) / SCALAR_3;
    return Inertia(mass, ix, iy, iz);
}

Inertia SphereInertia(double density, double rad)
{
    rad *= rad;
    double mass = density * M_PI * rad;
    double i = (double)0.4 * mass * rad;
    return Inertia(mass, i, i, i);
}

Inertia CylinderInertia(double density, double rad, double height)
{
    rad *= rad;
    double mass = density * M_PI * rad * height;
    double ix = mass * height * height  / (double)12.0 + SCALAR_1_4 * mass * rad;
    double iy = ix;
    double iz = SCALAR_1_2 * mass * rad;
    return Inertia(mass, ix, iy, iz);
}

Inertia TorusInertia(double density, double ring_rad, double tube_rad)
{
    double mass = density * SCALAR_2 * M_PI_SQR * ring_rad * tube_rad * tube_rad;
    double ix = mass * ((double)0.625 * tube_rad * tube_rad + SCALAR_1_2 * ring_rad + ring_rad);
    double iy = ix;
    double iz = mass * ((double)0.75 * tube_rad * tube_rad + ring_rad + ring_rad);
    return Inertia(mass, ix, iy, iz);
}

//==============================================================================
//
//==============================================================================
AInertia::AInertia()
{
    // Off-diagoanl terms
    _J[1] = _J[2] = _J[3] = _J[4]  = _J[5]  =
    _J[7] = _J[8] = _J[9] = _J[10] =
    _J[12] = _J[13] = _J[14] =
    _J[16] = _J[17] =
    _J[19] = 0.0;

    // Diagonal terms: Ixx, Iyy, Izz, mass are all 1.0.
    _J[0] = _J[6] = _J[11] = _J[15] = _J[18] = _J[20] = 1.0;
}

AInertia::AInertia(double d)
{
    _J[0] = _J[1] = _J[2] = _J[3] = _J[4] = _J[5] = _J[6] = _J[7] = _J[8] = _J[9] = _J[10] = _J[11] = _J[12] = _J[13] = _J[14] = _J[15] = _J[16] = _J[17] = _J[18] = _J[19] = _J[20] = d;
}

AInertia::AInertia(const Inertia& I)
{
    _J[0] = I[0];	_J[1] = I[3];	_J[2] = I[4];	_J[3] = SCALAR_0;	_J[4] = -I[8];		_J[5] = I[7];
                    _J[6] = I[1];	_J[7] = I[5];	_J[8] = I[8];		_J[9] = SCALAR_0;	_J[10] = -I[6];
                                    _J[11] = I[2];	_J[12] = -I[7];		_J[13] = I[6];		_J[14] = SCALAR_0;
                                                    _J[15] = I[9];		_J[16] = SCALAR_0;	_J[17] = SCALAR_0;
                                                                        _J[18] = I[9];		_J[19] = SCALAR_0;
                                                                                            _J[20] = I[9];
}

AInertia::AInertia(double a0, double a1, double a2,
                          double a3, double a4, double a5,
                          double a6, double a7, double a8,
                          double a9, double a10, double a11,
                          double a12, double a13, double a14,
                          double a15, double a16, double a17,
                          double a18, double a19, double a20)
{
    _J[0] = a0;		_J[1] = a1;		_J[2] = a2;
    _J[3] = a3;		_J[4] = a4;		_J[5] = a5;
    _J[6] = a6;		_J[7] = a7;		_J[8] = a8;
    _J[9] = a9;		_J[10] = a10;	_J[11] = a11;
    _J[12] = a12;	_J[13] = a13;	_J[14] = a14;
    _J[15] = a15;	_J[16] = a16;	_J[17] = a17;
    _J[18] = a18;	_J[19] = a19;	_J[20] = a20;
}

const AInertia& AInertia::operator+(void) const
{
    return *this;
}

AInertia AInertia::operator-(void) const
{
    return AInertia(-_J[0], -_J[1], -_J[2], -_J[3], -_J[4], -_J[5], -_J[6], -_J[7], -_J[8], -_J[9], -_J[10], -_J[11], -_J[12], -_J[13], -_J[14], -_J[15], -_J[16], -_J[17], -_J[18], -_J[19], -_J[20]);
}

dse3 AInertia::operator*(const se3& a) const
{
    return dse3(_J[0] * a[0] + _J[1] * a[1] + _J[2] * a[2] + _J[3] * a[3] + _J[4] * a[4] + _J[5] * a[5],
                _J[1] * a[0] + _J[6] * a[1] + _J[7] * a[2] + _J[8] * a[3] + _J[9] * a[4] + _J[10] * a[5],
                _J[2] * a[0] + _J[7] * a[1] + _J[11] * a[2] + _J[12] * a[3] + _J[13] * a[4] + _J[14] * a[5],
                _J[3] * a[0] + _J[8] * a[1] + _J[12] * a[2] + _J[15] * a[3] + _J[16] * a[4] + _J[17] * a[5],
                _J[4] * a[0] + _J[9] * a[1] + _J[13] * a[2] + _J[16] * a[3] + _J[18] * a[4] + _J[19] * a[5],
                _J[5] * a[0] + _J[10] * a[1] + _J[14] * a[2] + _J[17] * a[3] + _J[19] * a[4] + _J[20] * a[5]);
}

dse3 AInertia::operator*(const Vec3& a) const
{
    return dse3(_J[3] * a[0] + _J[4] * a[1] + _J[5] * a[2],
                _J[8] * a[0] + _J[9] * a[1] + _J[10] * a[2],
                _J[12] * a[0] + _J[13] * a[1] + _J[14] * a[2],
                _J[15] * a[0] + _J[16] * a[1] + _J[17] * a[2],
                _J[16] * a[0] + _J[18] * a[1] + _J[19] * a[2],
                _J[17] * a[0] + _J[19] * a[1] + _J[20] * a[2]);
}

dse3 AInertia::operator*(const Axis& a) const
{
    return dse3(_J[0] * a[0] + _J[1] * a[1] + _J[2] * a[2],
                _J[1] * a[0] + _J[6] * a[1] + _J[7] * a[2],
                _J[2] * a[0] + _J[7] * a[1] + _J[11] * a[2],
                _J[3] * a[0] + _J[8] * a[1] + _J[12] * a[2],
                _J[4] * a[0] + _J[9] * a[1] + _J[13] * a[2],
                _J[5] * a[0] + _J[10] * a[1] + _J[14] * a[2]);
}

double& AInertia::operator[](int i)
{
    return _J[i];
}

const double	&AInertia::operator[](int i) const
{
    return _J[i];
}

AInertia AInertia::operator+(const AInertia& J) const
{
    return AInertia(_J[0] + J[0], _J[1] + J[1], _J[2] + J[2], _J[3] + J[3], _J[4] + J[4], _J[5] + J[5], _J[6] + J[6], _J[7] + J[7], _J[8] + J[8], _J[9] + J[9], _J[10] + J[10], _J[11] + J[11], _J[12] + J[12], _J[13] + J[13], _J[14] + J[14], _J[15] + J[15], _J[16] + J[16], _J[17] + J[17], _J[18] + J[18], _J[19] + J[19], _J[20] + J[20]);
}

AInertia AInertia::operator+(const Inertia& J) const
{
    return AInertia(_J[0] + J[0], _J[1] + J[3], _J[2] + J[4], _J[3], _J[4] - J[8], _J[5] + J[7], _J[6] + J[1], _J[7] + J[5], _J[8] + J[8], _J[9], _J[10] - J[6], _J[11] + J[2], _J[12] - J[7], _J[13] + J[6], _J[14], _J[15] + J[9], _J[16], _J[17], _J[18] + J[9], _J[19], _J[20] + J[9]);
}

AInertia AInertia::operator-(const AInertia& J) const
{
    return AInertia(_J[0] - J[0], _J[1] - J[1], _J[2] - J[2], _J[3] - J[3], _J[4] - J[4], _J[5] - J[5], _J[6] - J[6], _J[7] - J[7], _J[8] - J[8], _J[9] - J[9], _J[10] - J[10], _J[11] - J[11], _J[12] - J[12], _J[13] - J[13], _J[14] - J[14], _J[15] - J[15], _J[16] - J[16], _J[17] - J[17], _J[18] - J[18], _J[19] - J[19], _J[20] - J[20]);
}

AInertia AInertia::operator-(const Inertia& J) const
{
    return AInertia(_J[0] - J[0], _J[1] - J[3], _J[2] - J[4], _J[3], _J[4] + J[8], _J[5] - J[7], _J[6] - J[1], _J[7] - J[5], _J[8] - J[8], _J[9], _J[10] + J[6], _J[11] - J[2], _J[12] + J[7], _J[13] - J[6], _J[14], _J[15] - J[9], _J[16], _J[17], _J[18] - J[9], _J[19], _J[20] - J[9]);
}

const AInertia& AInertia::operator += (const AInertia& J)
{
    _J[0] += J[0]; _J[1] += J[1]; _J[2] += J[2]; _J[3] += J[3]; _J[4] += J[4]; _J[5] += J[5]; _J[6] += J[6]; _J[7] += J[7]; _J[8] += J[8]; _J[9] += J[9]; _J[10] += J[10]; _J[11] += J[11]; _J[12] += J[12]; _J[13] += J[13]; _J[14] += J[14]; _J[15] += J[15]; _J[16] += J[16]; _J[17] += J[17]; _J[18] += J[18]; _J[19] += J[19]; _J[20] += J[20];
    return *this;
}

const AInertia& AInertia::operator-= (const AInertia& J)
{
    _J[0] -= J[0]; _J[1] -= J[1]; _J[2] -= J[2]; _J[3] -= J[3]; _J[4] -= J[4]; _J[5] -= J[5]; _J[6] -= J[6]; _J[7] -= J[7]; _J[8] -= J[8]; _J[9] -= J[9]; _J[10] -= J[10]; _J[11] -= J[11]; _J[12] -= J[12]; _J[13] -= J[13]; _J[14] -= J[14]; _J[15] -= J[15]; _J[16] -= J[16]; _J[17] -= J[17]; _J[18] -= J[18]; _J[19] -= J[19]; _J[20] -= J[20];
    return *this;
}

const AInertia& AInertia::operator += (const Inertia& J)
{
    _J[0] += J[0]; _J[1] += J[3]; _J[2] += J[4]; _J[4] -= J[8]; _J[5] += J[7]; _J[6] += J[1]; _J[7] += J[5]; _J[8] += J[8]; _J[10] -= J[6]; _J[11] += J[2]; _J[12] -= J[7]; _J[13] += J[6]; _J[15] += J[9]; _J[18] += J[9]; _J[20] += J[9];
    return *this;
}

const AInertia& AInertia::operator-= (const Inertia& J)
{
    _J[0] -= J[0]; _J[1] -= J[3]; _J[2] -= J[4]; _J[4] -= J[8]; _J[5] -= J[7]; _J[6] -= J[1]; _J[7] -= J[5]; _J[8] -= J[8]; _J[10] -= J[6]; _J[11] -= J[2]; _J[12] -= J[7]; _J[13] -= J[6]; _J[15] -= J[9]; _J[18] -= J[9]; _J[20] -= J[9];
    return *this;
}

template <class TYPE>
void AInertia::ToArray(TYPE M[]) const
{
    M[0] = (TYPE)_J[0]; M[6] = (TYPE)_J[1];  M[12] = (TYPE)_J[2];  M[18] = (TYPE)_J[3];  M[24] = (TYPE)_J[4];  M[30] = (TYPE)_J[5];
    M[1] = (TYPE)_J[1]; M[7] = (TYPE)_J[6];  M[13] = (TYPE)_J[7];  M[19] = (TYPE)_J[8];  M[25] = (TYPE)_J[9];  M[31] = (TYPE)_J[10];
    M[2] = (TYPE)_J[2]; M[8] = (TYPE)_J[7];  M[14] = (TYPE)_J[11]; M[20] = (TYPE)_J[12]; M[26] = (TYPE)_J[13]; M[32] = (TYPE)_J[14];
    M[3] = (TYPE)_J[3]; M[9] = (TYPE)_J[8];  M[15] = (TYPE)_J[12]; M[21] = (TYPE)_J[15]; M[27] = (TYPE)_J[16]; M[33] = (TYPE)_J[17];
    M[4] = (TYPE)_J[4]; M[10] = (TYPE)_J[9];  M[16] = (TYPE)_J[13]; M[22] = (TYPE)_J[16]; M[28] = (TYPE)_J[18]; M[34] = (TYPE)_J[19];
    M[5] = (TYPE)_J[5]; M[11] = (TYPE)_J[10]; M[17] = (TYPE)_J[14]; M[23] = (TYPE)_J[17]; M[29] = (TYPE)_J[19]; M[35] = (TYPE)_J[20];
}

AInertia AInertia::Transform(const SE3& T) const
{
    double d0 = _J[3] + T[11] * _J[16] - T[10] * _J[17];
    double d1 = _J[8] - T[11] * _J[15] + T[9] * _J[17];
    double d2 = _J[12] + T[10] * _J[15] - T[9] * _J[16];
    double d3 = _J[4] + T[11] * _J[18] - T[10] * _J[19];
    double d4 = _J[9] - T[11] * _J[16] + T[9] * _J[19];
    double d5 = _J[13] + T[10] * _J[16] - T[9] * _J[18];
    double d6 = _J[5] + T[11] * _J[19] - T[10] * _J[20];
    double d7 = _J[10] - T[11] * _J[17] + T[9] * _J[20];
    double d8 = _J[14] + T[10] * _J[17] - T[9] * _J[19];
    double e0 = _J[0] + T[11] * _J[4] - T[10] * _J[5] + d3 * T[11] - d6 * T[10];
    double e3 = _J[1] + T[11] * _J[9] - T[10] * _J[10] - d0 * T[11] + d6 * T[9];
    double e4 = _J[6] - T[11] * _J[8] + T[9] * _J[10] - d1 * T[11] + d7 * T[9];
    double e6 = _J[2] + T[11] * _J[13] - T[10] * _J[14] + d0 * T[10] - d3 * T[9];
    double e7 = _J[7] - T[11] * _J[12] + T[9] * _J[14] + d1 * T[10] - d4 * T[9];
    double e8 = _J[11] + T[10] * _J[12] - T[9] * _J[13] + d2 * T[10] - d5 * T[9];
    double f0 = T[0] * e0 + T[1] * e3 + T[2] * e6;
    double f1 = T[0] * e3 + T[1] * e4 + T[2] * e7;
    double f2 = T[0] * e6 + T[1] * e7 + T[2] * e8;
    double f3 = T[0] * d0 + T[1] * d1 + T[2] * d2;
    double f4 = T[0] * d3 + T[1] * d4 + T[2] * d5;
    double f5 = T[0] * d6 + T[1] * d7 + T[2] * d8;
    double f6 = T[3] * e0 + T[4] * e3 + T[5] * e6;
    double f7 = T[3] * e3 + T[4] * e4 + T[5] * e7;
    double f8 = T[3] * e6 + T[4] * e7 + T[5] * e8;
    double g0 = T[3] * d0 + T[4] * d1 + T[5] * d2;
    double g1 = T[3] * d3 + T[4] * d4 + T[5] * d5;
    double g2 = T[3] * d6 + T[4] * d7 + T[5] * d8;
    double g3 = T[6] * d0 + T[7] * d1 + T[8] * d2;
    double g4 = T[6] * d3 + T[7] * d4 + T[8] * d5;
    double g5 = T[6] * d6 + T[7] * d7 + T[8] * d8;
    double h0 = T[0] * _J[15] + T[1] * _J[16] + T[2] * _J[17];
    double h1 = T[0] * _J[16] + T[1] * _J[18] + T[2] * _J[19];
    double h2 = T[0] * _J[17] + T[1] * _J[19] + T[2] * _J[20];
    double h3 = T[3] * _J[15] + T[4] * _J[16] + T[5] * _J[17];
    double h4 = T[3] * _J[16] + T[4] * _J[18] + T[5] * _J[19];
    double h5 = T[3] * _J[17] + T[4] * _J[19] + T[5] * _J[20];

    return AInertia(f0 * T[0] + f1 * T[1] + f2 * T[2],
                    f0 * T[3] + f1 * T[4] + f2 * T[5],
                    f0 * T[6] + f1 * T[7] + f2 * T[8],
                    f3 * T[0] + f4 * T[1] + f5 * T[2],
                    f3 * T[3] + f4 * T[4] + f5 * T[5],
                    f3 * T[6] + f4 * T[7] + f5 * T[8],
                    f6 * T[3] + f7 * T[4] + f8 * T[5],
                    f6 * T[6] + f7 * T[7] + f8 * T[8],
                    g0 * T[0] + g1 * T[1] + g2 * T[2],
                    g0 * T[3] + g1 * T[4] + g2 * T[5],
                    g0 * T[6] + g1 * T[7] + g2 * T[8],
                    (T[6] * e0 + T[7] * e3 + T[8] * e6) * T[6] + (T[6] * e3 + T[7] * e4 + T[8] * e7) * T[7] + (T[6] * e6 + T[7] * e7 + T[8] * e8) * T[8],
                    g3 * T[0] + g4 * T[1] + g5 * T[2],
                    g3 * T[3] + g4 * T[4] + g5 * T[5],
                    g3 * T[6] + g4 * T[7] + g5 * T[8],
                    h0 * T[0] + h1 * T[1] + h2 * T[2],
                    h0 * T[3] + h1 * T[4] + h2 * T[5],
                    h0 * T[6] + h1 * T[7] + h2 * T[8],
                    h3 * T[3] + h4 * T[4] + h5 * T[5],
                    h3 * T[6] + h4 * T[7] + h5 * T[8],
                    (T[6] * _J[15] + T[7] * _J[16] + T[8] * _J[17]) * T[6] + (T[6] * _J[16] + T[7] * _J[18] + T[8] * _J[19]) * T[7] + (T[6] * _J[17] + T[7] * _J[19] + T[8] * _J[20]) * T[8]);
}

void AInertia::AddTransform(const AInertia& J, const SE3& T)
{
    double d0 = J[3] + T[11] * J[16] - T[10] * J[17];
    double d1 = J[8] - T[11] * J[15] + T[9] * J[17];
    double d2 = J[12] + T[10] * J[15] - T[9] * J[16];
    double d3 = J[4] + T[11] * J[18] - T[10] * J[19];
    double d4 = J[9] - T[11] * J[16] + T[9] * J[19];
    double d5 = J[13] + T[10] * J[16] - T[9] * J[18];
    double d6 = J[5] + T[11] * J[19] - T[10] * J[20];
    double d7 = J[10] - T[11] * J[17] + T[9] * J[20];
    double d8 = J[14] + T[10] * J[17] - T[9] * J[19];
    double e0 = J[0] + T[11] * J[4] - T[10] * J[5] + d3 * T[11] - d6 * T[10];
    double e3 = J[1] + T[11] * J[9] - T[10] * J[10] - d0 * T[11] + d6 * T[9];
    double e4 = J[6] - T[11] * J[8] + T[9] * J[10] - d1 * T[11] + d7 * T[9];
    double e6 = J[2] + T[11] * J[13] - T[10] * J[14] + d0 * T[10] - d3 * T[9];
    double e7 = J[7] - T[11] * J[12] + T[9] * J[14] + d1 * T[10] - d4 * T[9];
    double e8 = J[11] + T[10] * J[12] - T[9] * J[13] + d2 * T[10] - d5 * T[9];
    double f0 = T[0] * e0 + T[1] * e3 + T[2] * e6;
    double f1 = T[0] * e3 + T[1] * e4 + T[2] * e7;
    double f2 = T[0] * e6 + T[1] * e7 + T[2] * e8;
    double f3 = T[0] * d0 + T[1] * d1 + T[2] * d2;
    double f4 = T[0] * d3 + T[1] * d4 + T[2] * d5;
    double f5 = T[0] * d6 + T[1] * d7 + T[2] * d8;
    double f6 = T[3] * e0 + T[4] * e3 + T[5] * e6;
    double f7 = T[3] * e3 + T[4] * e4 + T[5] * e7;
    double f8 = T[3] * e6 + T[4] * e7 + T[5] * e8;
    double g0 = T[3] * d0 + T[4] * d1 + T[5] * d2;
    double g1 = T[3] * d3 + T[4] * d4 + T[5] * d5;
    double g2 = T[3] * d6 + T[4] * d7 + T[5] * d8;
    double g3 = T[6] * d0 + T[7] * d1 + T[8] * d2;
    double g4 = T[6] * d3 + T[7] * d4 + T[8] * d5;
    double g5 = T[6] * d6 + T[7] * d7 + T[8] * d8;
    double h0 = T[0] * J[15] + T[1] * J[16] + T[2] * J[17];
    double h1 = T[0] * J[16] + T[1] * J[18] + T[2] * J[19];
    double h2 = T[0] * J[17] + T[1] * J[19] + T[2] * J[20];
    double h3 = T[3] * J[15] + T[4] * J[16] + T[5] * J[17];
    double h4 = T[3] * J[16] + T[4] * J[18] + T[5] * J[19];
    double h5 = T[3] * J[17] + T[4] * J[19] + T[5] * J[20];

    _J[0] += f0 * T[0] + f1 * T[1] + f2 * T[2];
    _J[1] += f0 * T[3] + f1 * T[4] + f2 * T[5];
    _J[2] += f0 * T[6] + f1 * T[7] + f2 * T[8];
    _J[3] += f3 * T[0] + f4 * T[1] + f5 * T[2];
    _J[4] += f3 * T[3] + f4 * T[4] + f5 * T[5];
    _J[5] += f3 * T[6] + f4 * T[7] + f5 * T[8];
    _J[6] += f6 * T[3] + f7 * T[4] + f8 * T[5];
    _J[7] += f6 * T[6] + f7 * T[7] + f8 * T[8];
    _J[8] += g0 * T[0] + g1 * T[1] + g2 * T[2];
    _J[9] += g0 * T[3] + g1 * T[4] + g2 * T[5];
    _J[10] += g0 * T[6] + g1 * T[7] + g2 * T[8];
    _J[11] += (T[6] * e0 + T[7] * e3 + T[8] * e6) * T[6] + (T[6] * e3 + T[7] * e4 + T[8] * e7) * T[7] + (T[6] * e6 + T[7] * e7 + T[8] * e8) * T[8];
    _J[12] += g3 * T[0] + g4 * T[1] + g5 * T[2];
    _J[13] += g3 * T[3] + g4 * T[4] + g5 * T[5];
    _J[14] += g3 * T[6] + g4 * T[7] + g5 * T[8];
    _J[15] += h0 * T[0] + (T[0] * J[16] + T[1] * J[18] + T[2] * J[19]) * T[1] + (T[0] * J[17] + T[1] * J[19] + T[2] * J[20]) * T[2];
    _J[16] += h0 * T[3] + (T[0] * J[16] + T[1] * J[18] + T[2] * J[19]) * T[4] + (T[0] * J[17] + T[1] * J[19] + T[2] * J[20]) * T[5];
    _J[17] += h0 * T[6] + (T[0] * J[16] + T[1] * J[18] + T[2] * J[19]) * T[7] + (T[0] * J[17] + T[1] * J[19] + T[2] * J[20]) * T[8];
    _J[18] += h3 * T[3] + (T[3] * J[16] + T[4] * J[18] + T[5] * J[19]) * T[4] + (T[3] * J[17] + T[4] * J[19] + T[5] * J[20]) * T[5];
    _J[19] += h3 * T[6] + (T[3] * J[16] + T[4] * J[18] + T[5] * J[19]) * T[7] + (T[3] * J[17] + T[4] * J[19] + T[5] * J[20]) * T[8];
    _J[20] += (T[6] * J[15] + T[7] * J[16] + T[8] * J[17]) * T[6] + (T[6] * J[16] + T[7] * J[18] + T[8] * J[19]) * T[7] + (T[6] * J[17] + T[7] * J[19] + T[8] * J[20]) * T[8];
}

se3 AInertia::operator*(const dse3& f) const
{
    return se3(	_J[0] * f[0] + _J[1] * f[1] + _J[2] * f[2] + _J[3] * f[3] + _J[4] * f[4] + _J[5] * f[5],
                _J[1] * f[0] + _J[6] * f[1] + _J[7] * f[2] + _J[8] * f[3] + _J[9] * f[4] + _J[10] * f[5],
                _J[2] * f[0] + _J[7] * f[1] + _J[11] * f[2] + _J[12] * f[3] + _J[13] * f[4] + _J[14] * f[5],
                _J[3] * f[0] + _J[8] * f[1] + _J[12] * f[2] + _J[15] * f[3] + _J[16] * f[4] + _J[17] * f[5],
                _J[4] * f[0] + _J[9] * f[1] + _J[13] * f[2] + _J[16] * f[3] + _J[18] * f[4] + _J[19] * f[5],
                _J[5] * f[0] + _J[10] * f[1] + _J[14] * f[2] + _J[17] * f[3] + _J[19] * f[4] + _J[20] * f[5]);
}

// SCALAR_1_2 * ( x * ~y + y * ~x )
AInertia KroneckerProduct(const dse3& x, const dse3& y)
{
    double y_m0 = SCALAR_1_2 * y[0];
    double y_m1 = SCALAR_1_2 * y[1];
    double y_m2 = SCALAR_1_2 * y[2];
    double y_m3 = SCALAR_1_2 * y[3];
    double y_m4 = SCALAR_1_2 * y[4];
    double y_m5 = SCALAR_1_2 * y[5];

    return AInertia(x[0] * y[0],
                    x[0] * y_m1 + x[1] * y_m0,
                    x[0] * y_m2 + x[2] * y_m0,
                    x[0] * y_m3 + x[3] * y_m0,
                    x[0] * y_m4 + x[4] * y_m0,
                    x[0] * y_m5 + x[5] * y_m0,
                    x[1] * y[1],
                    x[1] * y_m2 + x[2] * y_m1,
                    x[1] * y_m3 + x[3] * y_m1,
                    x[1] * y_m4 + x[4] * y_m1,
                    x[1] * y_m5 + x[5] * y_m1,
                    x[2] * y[2],
                    x[2] * y_m3 + x[3] * y_m2,
                    x[2] * y_m4 + x[4] * y_m2,
                    x[2] * y_m5 + x[5] * y_m2,
                    x[3] * y[3],
                    x[3] * y_m4 + x[4] * y_m3,
                    x[3] * y_m5 + x[5] * y_m3,
                    x[4] * y[4],
                    x[4] * y_m5 + x[5] * y_m4,
                    x[5] * y[5]);
}

// *this -= KroneckerProduct(x, y)
void AInertia::SubtractKroneckerProduct(const dse3& x, const dse3& y)
{
    double y_m0 = SCALAR_1_2 * y[0];
    double y_m1 = SCALAR_1_2 * y[1];
    double y_m2 = SCALAR_1_2 * y[2];
    double y_m3 = SCALAR_1_2 * y[3];
    double y_m4 = SCALAR_1_2 * y[4];
    double y_m5 = SCALAR_1_2 * y[5];

    _J[0]  -= x[0] * y[0];
    _J[1]  -= x[0] * y_m1 + x[1] * y_m0;
    _J[2]  -= x[0] * y_m2 + x[2] * y_m0;
    _J[3]  -= x[0] * y_m3 + x[3] * y_m0;
    _J[4]  -= x[0] * y_m4 + x[4] * y_m0;
    _J[5]  -= x[0] * y_m5 + x[5] * y_m0;
    _J[6]  -= x[1] * y[1];
    _J[7]  -= x[1] * y_m2 + x[2] * y_m1;
    _J[8]  -= x[1] * y_m3 + x[3] * y_m1;
    _J[9]  -= x[1] * y_m4 + x[4] * y_m1;
    _J[10] -= x[1] * y_m5 + x[5] * y_m1;
    _J[11] -= x[2] * y[2];
    _J[12] -= x[2] * y_m3 + x[3] * y_m2;
    _J[13] -= x[2] * y_m4 + x[4] * y_m2;
    _J[14] -= x[2] * y_m5 + x[5] * y_m2;
    _J[15] -= x[3] * y[3];
    _J[16] -= x[3] * y_m4 + x[4] * y_m3;
    _J[17] -= x[3] * y_m5 + x[5] * y_m3;
    _J[18] -= x[4] * y[4];
    _J[19] -= x[4] * y_m5 + x[5] * y_m4;
    _J[20] -= x[5] * y[5];
}

se3 AInertia::operator % (const dse3& b) const
{
    double a00 = _J[6] * _J[11] - _J[7] * _J[7];
    double a01 = _J[2] * _J[7] - _J[1] * _J[11];
    double a02 = _J[1] * _J[7] - _J[2] * _J[6];
    double idet = SCALAR_1 / (_J[0] * a00 + _J[1] * a01 + _J[2] * a02);
    double a11 = idet * (_J[0] * _J[11] - _J[2] * _J[2]);
    double a12 = idet * (_J[2] * _J[1] - _J[0] * _J[7]);
    double a22 = idet * (_J[0] * _J[6] - _J[1] * _J[1]);
    a00 *= idet;
    a01 *= idet;
    a02 *= idet;
    double t00 = a00 * _J[3] + a01 * _J[8] + a02 * _J[12];
    double t01 = a00 * _J[4] + a01 * _J[9] + a02 * _J[13];
    double t02 = a00 * _J[5] + a01 * _J[10] + a02 * _J[14];
    double t10 = a01 * _J[3] + a11 * _J[8] + a12 * _J[12];
    double t11 = a01 * _J[4] + a11 * _J[9] + a12 * _J[13];
    double t12 = a01 * _J[5] + a11 * _J[10] + a12 * _J[14];
    double t20 = a02 * _J[3] + a12 * _J[8] + a22 * _J[12];
    double t21 = a02 * _J[4] + a12 * _J[9] + a22 * _J[13];
    double t22 = a02 * _J[5] + a12 * _J[10] + a22 * _J[14];
    double r0 = a00 * b[0] + a01 * b[1] + a02 * b[2];
    double r1 = a01 * b[0] + a11 * b[1] + a12 * b[2];
    double r2 = a02 * b[0] + a12 * b[1] + a22 * b[2];
    a00 = r0;
    a01 = r1;
    a02 = r2;
    double x0 = b[3] - _J[3] * r0 - _J[8] * r1 - _J[12] * r2;
    double x1 = b[4] - _J[4] * r0 - _J[9] * r1 - _J[13] * r2;
    double x2 = b[5] - _J[5] * r0 - _J[10] * r1 - _J[14] * r2;
    double c00 = _J[15] - _J[3] * t00 - _J[8] * t10 - _J[12] * t20;
    double c01 = _J[16] - _J[3] * t01 - _J[8] * t11 - _J[12] * t21;
    double c11 = _J[18] - _J[4] * t01 - _J[9] * t11 - _J[13] * t21;
    double c02 = _J[17] - _J[3] * t02 - _J[8] * t12 - _J[12] * t22;
    double c12 = _J[19] - _J[4] * t02 - _J[9] * t12 - _J[13] * t22;
    double c22 = _J[20] - _J[5] * t02 - _J[10] * t12 - _J[14] * t22;
    double r3 = c02 * c01 - c00 * c12;
    r0 = c11 * c22 - c12 * c12;
    r1 = c02 * c12 - c01 * c22;
    r2 = c01 * c12 - c02 * c11;
    idet = SCALAR_1 / (c00 * r0 + c01 * r1 + c02 * r2);
    a22 = idet * (r0 * x0 + r1 * x1 + r2 * x2);
    a11 = idet * (r1 * x0 + (c00 * c22 - c02 * c02) * x1 + r3 * x2);
    a12 = idet * (r2 * x0 + r3 * x1 + (c00 * c11 - c01 * c01) * x2);
    a00 -= t00 * a22 + t01 * a11 + t02 * a12;
    a01 -= t10 * a22 + t11 * a11 + t12 * a12;
    a02 -= t20 * a22 + t21 * a11 + t22 * a12;

    return se3(a00, a01 ,a02, a22, a11, a12);
}


AInertia Inv(const Inertia& I)
{
    double im = SCALAR_1 / I[9];
    double ims = sqrt(im);
    double r0 = ims * I[6];
    double r1 = ims * I[7];
    double r2 = ims * I[8];
    double a00 = I[0] - (r1 * r1 + r2 * r2);
    double a11 = I[1] - (r0 * r0 + r2 * r2);
    double a22 = I[2] - (r0 * r0 + r1 * r1);
    double a01 = I[3] + r0 * r1;
    double a02 = I[4] + r0 * r2;
    double a12 = I[5] + r1 * r2;
    double j0 = a11 * a22 - a12 * a12;
    double j6 = a02 * a12 - a01 * a22;
    double j12 = a01 * a12 - a02 * a11;
    double idet = SCALAR_1 / (a00 * j0 + a01 * j6 + a02 * j12);
    j0 *= idet;
    j6 *= idet;
    j12 *= idet;
    double j7 = idet * (a00 * a22 - a02 * a02);
    double j13 = idet * (a02 * a01 - a00 * a12);
    double j14 = idet * (a00 * a11 - a01 * a01);
    r0 *= ims;
    r1 *= ims;
    r2 *= ims;
    double j18 = j12 * r1 - j6 * r2;
    double j19 = j13 * r1 - j7 * r2;
    double j20 = j14 * r1 - j13 * r2;
    double j24 = j0 * r2 - j12 * r0;
    double j25 = j6 * r2 - j13 * r0;
    double j26 = j12 * r2 - j14 * r0;
    double j30 = j6 * r0 - j0 * r1;
    double j31 = j7 * r0 - j6 * r1;
    double j32 = j13 * r0 - j12 * r1;

    return AInertia(j0, j6, j12, j18, j24, j30, j7, j13, j19, j25, j31, j14, j20, j26, j32, r1 * j20 - r2 * j19 + im, r1 * j26 - r2 * j25, r1 * j32 - r2 * j31, r2 * j24 - r0 * j26 + im, r2 * j30 - r0 * j32, r0 * j31 - r1 * j30 + im);
}

const AInertia& AInertia::operator = (const AInertia& J)
{
    _J[0] = J[0];	_J[1] = J[1];	_J[2] = J[2];
    _J[3] = J[3];	_J[4] = J[4];	_J[5] = J[5];
    _J[6] = J[6];	_J[7] = J[7];	_J[8] = J[8];
    _J[9] = J[9];	_J[10] = J[10];	_J[11] = J[11];
    _J[12] = J[12];	_J[13] = J[13];	_J[14] = J[14];
    _J[15] = J[15];	_J[16] = J[16];	_J[17] = J[17];
    _J[18] = J[18];	_J[19] = J[19];	_J[20] = J[20];
    return *this;
}

const AInertia& AInertia::operator = (const Inertia& I)
{
    _J[0] = I[0];	_J[1] = I[3];	_J[2] = I[4];	_J[3] = SCALAR_0;	_J[4] = -I[8];			_J[5] = I[7];
                    _J[6] = I[1];	_J[7] = I[5];	_J[8] = I[8];			_J[9] = SCALAR_0;	_J[10] = -I[6];
                                    _J[11] = I[2];	_J[12] = -I[7];			_J[13] = I[6];			_J[14] = SCALAR_0;
                                                    _J[15] = I[9];			_J[16] = SCALAR_0;	_J[17] = SCALAR_0;
                                                                            _J[18] = I[9];			_J[19] = SCALAR_0;
                                                                                                    _J[20] = I[9];
    return *this;
}

Axis::Axis()
{
    _v[0] = _v[1] = _v[2] = 0.0;
}

Axis::Axis(double d)
{
    _v[0] = _v[1] = _v[2] = d;
}

Axis::Axis(const double v[])
{
    _v[0] = v[0];
    _v[1] = v[1];
    _v[2] = v[2];
}

Axis::Axis(double v0, double v1, double v2)
{
    _v[0] = v0;
    _v[1] = v1;
    _v[2] = v2;
}

Axis::Axis(const Vec3& v)
{
    _v[0] = v[0];
    _v[1] = v[1];
    _v[2] = v[2];
}

Axis::~Axis()
{
}

const Axis& Axis::operator+(void) const
{
    return *this;
}

Axis Axis::operator-(void) const
{
    return Axis(-_v[0], -_v[1], -_v[2]);
}

double& Axis::operator[](int i)
{
    return _v[i];
}

const double& Axis::operator[](int i) const
{
    return _v[i];
}

const Axis& Axis::operator = (const Axis& v)
{
    _v[0] = v[0];
    _v[1] = v[1];
    _v[2] = v[2];
    return *this;
}

const Axis& Axis::operator = (const se3& v)
{
    _v[0] = v[0];
    _v[1] = v[1];
    _v[2] = v[2];
    return *this;
}

const Axis& Axis::operator = (double d)
{
    _v[0] = _v[1] = _v[2] = d;
    return *this;
}

const Axis& Axis::operator *= (double d)
{
    _v[0] *= d;
    _v[1] *= d;
    _v[2] *= d;
    return *this;
}

bool Axis::operator==(const Axis& v) const
{
    if ((_v[0] != v._v[0])
            || (_v[1] != v._v[1])
            || (_v[2] != v._v[2]))
        return false;

    return true;
}

bool Axis::operator!=(const Axis& v) const
{
    return !(*this == v);
}

Axis Axis::operator*(double d) const
{
    return Axis(d * _v[0], d * _v[1], d * _v[2]);
}

double Axis::Normalize(void)
{
    double mag = sqrt(_v[0] * _v[0] + _v[1] * _v[1] + _v[2] * _v[2]);
    if ( mag < LIE_EPS )	// make a unit vector in z-direction
    {
        _v[0] = _v[1] = SCALAR_0;
        _v[2] = SCALAR_1;
    } else
    {
        _v[0] /= mag;
        _v[1] /= mag;
        _v[2] /= mag;
    }
    return mag;
}

void Axis::Reparameterize(void)
{
    double theta = sqrt(_v[0] * _v[0] + _v[1] * _v[1] + _v[2] * _v[2]);
    if ( theta > LIE_EPS )
    {
        double eta = 1.0 - (double)((int)(theta / M_PI + 1.0) / 2) * M_2PI / theta;
        _v[0] *= eta;
        _v[1] *= eta;
        _v[2] *= eta;
    }
}

Axis Reparameterize(const Axis& s)
{
    double theta = sqrt(s[0] * s[0] + s[1] * s[1] + s[2] * s[2]);
    double eta = theta < LIE_EPS ? 1.0 : 1.0 - (double)((int)(theta / M_PI + 1.0) / 2) * M_2PI / theta;
    return eta * s;
}

Axis Rotate(const SE3& T, const Axis& v)
{
    return Axis(T[0] * v[0] + T[3] * v[1] + T[6] * v[2],
                T[1] * v[0] + T[4] * v[1] + T[7] * v[2],
                T[2] * v[0] + T[5] * v[1] + T[8] * v[2]);
}

Axis InvRotate(const SE3& T, const Axis& v)
{
    return Axis(T[0] * v[0] + T[1] * v[1] + T[2] * v[2],
                T[3] * v[0] + T[4] * v[1] + T[5] * v[2],
                T[6] * v[0] + T[7] * v[1] + T[8] * v[2]);
}

Axis operator*(double d, const Axis& v)
{
    return Axis(d * v[0], d * v[1], d * v[2]);
}

double Norm(const Axis& v)
{
    return sqrt(v._v[0] * v._v[0] + v._v[1] * v._v[1] + v._v[2] * v._v[2]);
}

Axis Normalize(const Axis& v)
{
    double mag = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if ( mag < LIE_EPS )	// make a unit vector in z-direction
        return Axis(SCALAR_0, SCALAR_0, SCALAR_1);

    mag = SCALAR_1 / mag;
    return Axis(mag * v[0], mag * v[1], mag * v[2]);
}

Axis Cross(const Axis& p, const Axis& q)
{
    return Axis(p[1] * q[2] - p[2] * q[1],
                p[2] * q[0] - p[0] * q[2],
                p[0] * q[1] - p[1] * q[0]);
}

double Inner(const Axis& p, const Axis& q)
{
    return (p[0] * q[0] + p[1] * q[1] + p[2] * q[2]);
}

double Inner(const Vec3& p, const Axis& q)
{
    return (p[0] * q[0] + p[1] * q[1] + p[2] * q[2]);
}

double Inner(const Axis& p, const Vec3& q)
{
    return (p[0] * q[0] + p[1] * q[1] + p[2] * q[2]);
}

double SquareSum(const Axis& p)
{
    return (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
}

Axis Square(const Axis& p)
{
    return Axis(p[0] * p[0], p[1] * p[1], p[2] * p[2]);
}

Axis InvAd(const SE3& T, const Axis& v)
{
    return Axis(T._T[0] * v[0] + T._T[1] * v[1] + T._T[2] * v[2],
                T._T[3] * v[0] + T._T[4] * v[1] + T._T[5] * v[2],
                T._T[6] * v[0] + T._T[7] * v[1] + T._T[8] * v[2]);
}

Axis ad(const Axis& s1, const se3& s2)
{
    return Axis(s2[2] * s1[1] - s2[1] * s1[2],
                s2[0] * s1[2] - s2[2] * s1[0],
                s2[1] * s1[0] - s2[0] * s1[1]);
}

Axis ad(const Axis& s1, const Axis& s2)
{
    return Axis(s2[2] * s1[1] - s2[1] * s1[2],
                s2[0] * s1[2] - s2[2] * s1[0],
                s2[1] * s1[0] - s2[0] * s1[1]);
}

Axis Axis::operator+(const Axis& v) const
{
    return Axis(_v[0] + v[0], _v[1] + v[1], _v[2] + v[2]);
}

se3 Axis::operator+(const Vec3& v) const
{
    return se3(_v[0], _v[1], _v[2], v[0], v[1], v[2]);
}

Axis Axis::operator-(const Axis& v) const
{
    return Axis(_v[0] - v[0], _v[1] - v[1], _v[2] - v[2]);
}

const Axis& Axis::operator += (const Axis& v)
{
    _v[0] += v[0];
    _v[1] += v[1];
    _v[2] += v[2];
    return *this;
}

const Axis& Axis::operator-= (const Axis& v)
{
    _v[0] -= v[0];
    _v[1] -= v[1];
    _v[2] -= v[2];
    return *this;
}

//==============================================================================
ostream& operator<<(ostream& os, const Vec3& v)
{
    ios_base::fmtflags flags = os.setf(ios::left | ios::fixed);
    streamsize sz = os.precision(3);
    os << "[ ";
    for ( int i = 0; i < 3; i++ )
    {
        if ( v[i] >= SCALAR_0 ) os << " " << setw(6) << v[i] << " ";
        else os << setw(7) << v[i] << " ";
    }
    os << "];" << endl;
    os.setf(flags);
    os.precision(sz);
    return os;
}

ostream& operator<<(ostream& os, const Axis& v)
{
    ios_base::fmtflags flags = os.setf(ios::left | ios::fixed);
    streamsize sz = os.precision(3);
    os << "[ ";
    for ( int i = 0; i < 3; i++ )
    {
        if ( v[i] >= SCALAR_0 ) os << " " << setw(6) << v[i] << " ";
        else os << setw(7) << v[i] << " ";
    }
    os << "];" << endl;
    os.setf(flags);
    os.precision(sz);
    return os;
}

ostream& operator<<(ostream& os, const se3& s)
{
    ios_base::fmtflags flags = os.setf(ios::left | ios::fixed);
    streamsize sz = os.precision(3);
    os << "[ ";
    for ( int i = 0; i < 6; i++ )
    {
        if ( s[i] >= SCALAR_0 ) os << " " << setw(6) << s[i] << " ";
        else os << setw(7) << s[i] << " ";
    }
    os << "];" << endl;
    os.setf(flags);
    os.precision(sz);
    return os;
}

ostream& operator<<(ostream& os, const dse3& t)
{
    ios_base::fmtflags flags = os.setf(ios::left | ios::fixed);
    streamsize sz = os.precision(3);
    os << "[ ";
    for ( int i = 0; i < 6; i++ )
    {
        if ( t[i] >= SCALAR_0 ) os << " " << setw(6) << t[i] << " ";
        else os << setw(7) << t[i] << " ";
    }
    os << "];" << endl;
    os.setf(flags);
    os.precision(sz);
    return os;
}

ostream& operator<<(ostream& os, const SE3& T)
{
    ios_base::fmtflags flags = os.setf(ios::left | ios::fixed);
    streamsize sz = os.precision(3);
    os << "[" << endl;
    for ( int i = 0; i < 4; i++ )
    {
        for ( int j = 0; j < 4; j++ )
        {
            if ( T(i,j) >= SCALAR_0 ) os << " " << setw(6) << T(i,j) << " ";
            else os << setw(7) << T(i,j) << " ";
        }
        os << ";" << endl;
    }
    os << "];" << endl;
    os.setf(flags);
    os.precision(sz);
    return os;
}

//==============================================================================
//
//==============================================================================
Jacobian::Jacobian()
{
}

Jacobian::Jacobian(unsigned int _size)
{
    mJ.resize(_size);
}

//Jacobian::Jacobian(const Eigen::MatrixXd& _J)
//{
//    setMatrix(_J);
//}

Jacobian::~Jacobian()
{
}

void Jacobian::setFromEigenMatrix(const Eigen::MatrixXd& _J)
{
    assert(_J.rows() == 6);

    setSize(_J.cols());

    for (int i = 0; i < getSize(); ++i)
        mJ[i].setEigenVector(_J.col(i));
}

Eigen::MatrixXd Jacobian::getEigenMatrix() const
{
    Eigen::MatrixXd J(6, getSize());

    for (int i = 0; i < getSize(); i++)
        J.col(i) = mJ[i].getEigenVector();

    return J;
}

void Jacobian::setZero()
{
    for (int i = 0; i < getSize(); i++)
        mJ[i].setZero();
}

se3 Jacobian::getColumn(int _idx)
{
    return mJ[_idx];
}

Jacobian Jacobian::getColumns(int _idx, int _size)
{
    // TODO: NEED TEST
    assert(0 <= _idx);
    assert(_idx + _size <= mJ.size());

    Jacobian J(_size);

    for (int i = 0; i < _size; ++i)
        J.setColumn(i, getColumn(i));

    return J;
}

Jacobian Jacobian::getAdjointed(const SE3& _T) const
{
    Jacobian ret;

    dterr << "NOT IMPLEMENTED.\n";

    return ret;
}

Jacobian Jacobian::getAdjointedInv(const SE3& _Tinv) const
{
    Jacobian ret;

    dterr << "NOT IMPLEMENTED.\n";

    return ret;
}

se3& Jacobian::operator[](int _i)
{
    assert(0 <= _i && _i < getSize());

    return mJ[_i];
}

const se3& Jacobian::operator[](int _i) const
{
    assert(0 <= _i && _i < getSize());

    return mJ[_i];
}

se3 Jacobian::operator*(const Eigen::VectorXd& _qdot)
{
    assert(_qdot.size() == getSize());

    se3 ret;

    for (int i = 0; i < getSize(); ++i)
        ret += mJ[i] * _qdot(i);

    return ret;
}

bool Jacobian::operator==(const Jacobian& _rhs) const
{
    for (int i = 0; i < getSize(); ++i)
        if (mJ[i] != _rhs.mJ[i])
            return false;

    return true;
}

bool Jacobian::operator!=(const Jacobian& _rhs) const
{
    return !(*this == _rhs);
}

Eigen::VectorXd Jacobian::getInnerProduct(const dse3& _F) const
{
    Eigen::VectorXd ret = Eigen::VectorXd::Zero(getSize());

    assert(ret.size() == getSize());

    int size = getSize();
    for (int i = 0; i < size; ++i)
        ret(i) = Inner(mJ[i], _F);

    return ret;
}

double Inner(const se3& V, const dse3& F)
{
    return (F[0] * V[0] + F[1] * V[1] + F[2] * V[2]
            + F[3] * V[3] + F[4] * V[4] + F[5] * V[5]);
}

double Inner(const dse3& F, const se3& V)
{
    return (F._m[0] * V._w[0] + F._m[1] * V._w[1] + F._m[2] * V._w[2]
            + F._m[3] * V._w[3] + F._m[4] * V._w[4] + F._m[5] * V._w[5]);
}

double Inner(const dse3& F, const Vec3& v)
{
    return (F[3] * v[0] + F[4] * v[1] + F[5] * v[2]);
}

double Inner(const dse3& F, const Axis& w)
{
    return (F._m[0] * w._v[0] + F._m[1] * w._v[1] + F._m[2] * w._v[2]);
}

} // namespace math
} // namespace dart