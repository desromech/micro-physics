#ifndef UPHYSICS_MATRIX3x3_HPP
#define UPHYSICS_MATRIX3x3_HPP

#include "Vector2.hpp"
#include "Vector3.hpp"
#include <math.h>

namespace UPhysics
{
struct Matrix3x3
{
    static Matrix3x3 identity()
    {
        return Matrix3x3{
            Vector3{1, 0, 0},
            Vector3{0, 1, 0},
            Vector3{0, 0, 1},
        };
    }

    static Matrix3x3 zeros()
    {
        return Matrix3x3({
            Vector3{0, 0, 0},
            Vector3{0, 0, 0},
            Vector3{0, 0, 0}
        });
    }

    static Matrix3x3 scale(Vector3 v)
    {
        return Matrix3x3{
            Vector3{v.x, 0, 0},
            Vector3{0, v.y, 0},
            Vector3{0, 0, v.z}
        };
    }

    static Matrix3x3 skewSymmetric(Vector3 v)
    {
        return Matrix3x3{
            Vector3{0, v.z, -v.y},
            Vector3{-v.z, 0, v.x},
            Vector3{v.y, -v.x, 0}
        };
    }

    static Matrix3x3 XRotation(float angle)
    {
        float c = cos(angle);
        float s = sin(angle);
        return Matrix3x3{
            Vector3{1, 0, 0},
            Vector3{0, c, s},
            Vector3{0, -s, c}
        };
    }

    static Matrix3x3 YRotation(float angle)
    {
        float c = cos(angle);
        float s = sin(angle);
        return Matrix3x3{
            Vector3{c, 0, -s},
            Vector3{0, 1, 0},
            Vector3{s, 0, c}
        };
    }

    static Matrix3x3 ZRotation(float angle)
    {
        float c = cos(angle);
        float s = sin(angle);
        return Matrix3x3{
            Vector3{c, s, 0},
            Vector3{-s, c, 0},
            Vector3{0, 0, 1}
        };
    }

    static Matrix3x3 makeWithRows(Vector3 r1, Vector3 r2, Vector3 r3)
    {
        auto transposed = Matrix3x3{r1, r2, r3};
        return transposed.transposed();
    }

    static Matrix3x3 computeTextureMatrix(Vector2 textureScale, float textureRotation, Vector2 textureOffset)
    {
        auto scale = Matrix3x3{
            Vector3{textureScale.x, 0, 0},
            Vector3{0, textureScale.y, 0},
            Vector3{0, 0, 1}
        };
        float c = cos(textureRotation);
        float s = sin(textureRotation);
        auto rotation = Matrix3x3{
            Vector3{c, s, 0},
            Vector3{-s, c, 0},
            Vector3{0, 0, 1}
        };
        auto offset = Matrix3x3{
            Vector3{1, 0, 0},
            Vector3{0, 1, 0},
            Vector3{textureOffset.x, textureOffset.y, 1}
        };

    return offset *(rotation*scale);
    }

    Matrix3x3()
        : columns{
            Vector3(0, 0, 0), 
            Vector3(0, 0, 0),
            Vector3(0, 0, 0),
        } {}

    Matrix3x3(const Vector3 &c1, const Vector3 &c2, const Vector3 &c3)
        : columns{c1, c2, c3} {}

    Matrix3x3 transposed() const
    {
        return Matrix3x3{
            Vector3{columns[0].x, columns[1].x, columns[2].x},
            Vector3{columns[0].y, columns[1].y, columns[2].y},
            Vector3{columns[0].z, columns[1].z, columns[2].z}
        };
    }

    Matrix3x3 inverse() const
    {
        auto det = determinant();
        auto firstColumn = columns[0];
        auto secondColumn = columns[1];
        auto thirdColumn = columns[2];

        return Matrix3x3{
            secondColumn.cross(thirdColumn) / det,
            thirdColumn.cross(firstColumn) / det,
            firstColumn.cross(secondColumn) / det,
        };
    }

    float determinant() const
    {
        return columns[0].x*columns[1].y*columns[2].z
             + columns[1].x*columns[2].y*columns[0].z
             + columns[2].x*columns[0].y*columns[1].z

             - columns[0].z*columns[1].y*columns[2].x
             - columns[1].z*columns[2].y*columns[0].x
             - columns[2].z*columns[0].y*columns[1].x;
    }

    Vector3 firstRow() const
    {
        return Vector3(columns[0].x, columns[1].x, columns[2].x);
    }
    
    Matrix3x3 operator-() const
    {
        return Matrix3x3{-columns[0], -columns[1], -columns[2]};
    }

    Matrix3x3 operator+(const Matrix3x3 &o) const
    {
        return Matrix3x3{columns[0] + o.columns[0], columns[1] + o.columns[1], columns[2] + o.columns[2]};
    }

    Matrix3x3 operator-(const Matrix3x3 &o) const
    {
        return Matrix3x3{columns[0] - o.columns[0], columns[1] - o.columns[1], columns[2] - o.columns[2]};
    }

    inline friend Vector3 operator*(const Vector3 &v, const Matrix3x3 &m)
    {
        return Vector3(v.dot(m.columns[0]), v.dot(m.columns[1]), v.dot(m.columns[2]));
    }

    Vector3 operator*(const Vector3 &v) const
    {
        return columns[0]*v.x + columns[1]*v.y + columns[2]*v.z;
    }

    Matrix3x3 operator*(const Matrix3x3 &o) const
    {
        return Matrix3x3{
            columns[0]*o.columns[0].x + columns[1]*o.columns[0].y + columns[2]*o.columns[0].z,
            columns[0]*o.columns[1].x + columns[1]*o.columns[1].y + columns[2]*o.columns[1].z,
            columns[0]*o.columns[2].x + columns[1]*o.columns[2].y + columns[2]*o.columns[2].z
        };
    }

    Vector3 columns[3];

    static const Matrix3x3 CubeMapFaceRotations[6];
};

} // End of namespace UPhysics
#endif //UPHYSICS_MATRIX3x3_HPP
