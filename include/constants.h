#ifndef GLOBALS_H
#define GLOBALS_H

#include <cstdint>
#include <cstdlib>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>

inline const double PI          = 3.1415926535897932384626433832795;
inline const double RAD_TO_DEG  = 180.0 / PI;
inline const double DEG_TO_RAD  = PI / 180.0;
inline const double EPSILON     = 1.0E-6;

inline const int SAMPLE_FACTOR    = 1;

// inline static QMutex gMutex;
typedef unsigned char uchar;
using Vec3        = Eigen::Matrix< double, 3, 1 >;
using Vec2        = Eigen::Matrix< double, 2, 1 >;
using Vec2u       = Eigen::Matrix< size_t, 2, 1 >;
using Vec2i       = Eigen::Matrix< double, 2, 1 >;
using Rot         = Eigen::Quaternion< double >;
using Trans       = Eigen::Translation< double, 3 >;
using Transform3D = Eigen::Transform< double, 3, Eigen::Affine >;

typedef std::vector< Vec3>  CloudBuffer;

#endif  // GLOBALS_H
