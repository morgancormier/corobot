#ifndef QRK_COLOR_H
#define QRK_COLOR_H

/*!
  \file
  \brief 色情報の定義

  \author Satofumi KAMIMURA

  $Id: Color.h 1811 2010-04-30 16:12:05Z satofumi $
*/

namespace qrk
{
    /*!
      \brief 色の定義クラス
    */
    class Color {
    public:
        float r;                    //!< 赤 [0.0, 1.0]
        float g;                    //!< 緑 [0.0, 1.0]
        float b;                    //!< 青 [0.0, 1.0]
        float a;                    //!< アルファ値 [0.0, 1.0]


        Color(void) : r(0.0), g(0.0), b(0.0), a(1.0)
        {
        }


        /*!
          \brief コンストラクタ

          \param[in] r_ 赤 [0.0, 1.0]
          \param[in] g_ 緑 [0.0, 1.0]
          \param[in] b_ 青 [0.0, 1.0]
          \param[in] a_ α値 [0.0, 1.0]
        */
        Color(float r_, float g_, float b_, float a_ = 1.0f)
            : r(r_), g(g_), b(b_), a(a_)
        {
        }
    };
}

#endif /* !QRK_COLOR_H */
