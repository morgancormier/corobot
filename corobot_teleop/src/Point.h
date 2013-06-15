#ifndef QRK_POINT_H
#define QRK_POINT_H

/*!
  \file
  \brief 位置の型定義

  \author Satofumi KAMIMURA

  $Id: Point.h 1811 2010-04-30 16:12:05Z satofumi $
*/

namespace qrk
{
    /*!
      \brief 位置の型定義
    */
    template <typename T>
    class Point
    {
    public:
        T x;
        T y;


        Point(void) : x(0), y(0)
        {
        }


        Point(T x_, T y_) : x(x_), y(y_)
        {
        }


        bool operator == (const Point<T>& rhs) const
        {
            return ((this->x == rhs.x) && (this->y == rhs.y)) ? true : false;
        }


        bool operator < (const Point<T>& rhs) const
        {
            if (y < rhs.y) {
                return true;

            } else if (x < rhs.x) {
                return true;

            } else {
                return false;
            }
        }


        Point<T>& operator += (const Point<T>& rhs)
        {
            this->x += rhs.x;
            this->y += rhs.y;

            return *this;
        }


        const Point<T> operator + (const Point<T>& rhs) const
        {
            return Point<T>(*this) += rhs;
        }


        Point<T>& operator -= (const Point<T>& rhs)
        {
            this->x -= rhs.x;
            this->y -= rhs.y;

            return *this;
        }


        const Point<T> operator - (const Point<T>& rhs) const
        {
            return Point<T>(*this) -= rhs;
        }
    };
}

#endif /* !QRK_POINT_H */
