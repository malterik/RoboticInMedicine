#ifndef ROTATIONMATRIX2X2_H
#define ROTATIONMATRIX2X2_H

#include <Tools/Math/Matrix2x2.h>

/// Representation of a 2x2 Rotationmatrix
/**
 * This class represents a RotationMatrix
 *
 * @author <a href="mailto:stefan.kaufmann@tu-harburg.de">Stefan Kaufmann</a>
 */
class RotationMatrix2x2 : public Matrix2x2<float>
{
public:
  /** default Constructor */
  RotationMatrix2x2(){}

  /** constructor with columnwise initialization
   * @param c0 The first column of the matrix
   * @param c1 The second column of the matrix
   */
  RotationMatrix2x2(const Vector2<float>& c0, const Vector2<float>& c1):
    Matrix2x2<float>(c0,c1){}

  /**
  * Copy constructor.
  * @param  other  The other matrix that is copied to this one
  */
  RotationMatrix2x2(const Matrix2x2<float>& other):
    Matrix2x2<float>(other){}

  /** constructor with elementwise initialization
   * @param m11 The element (1,1) of the matrix
   * @param m12 The element (1,2) of the matrix
   * @param m21 The element (2,1) of the matrix
   * @param m22 The element (2,2) of the matrix
   */
  RotationMatrix2x2(	const float& m11, const float& m12,
                      const float& m21, const float& m22):
    Matrix2x2<float>(	m11,m12,
                      m21,m22){}

  /** rotation constructor
   * @param alpha The angle of the rotation matrix
   */
  RotationMatrix2x2(const float& alpha):
    Matrix2x2<float>(cos(alpha), -sin(alpha), sin(alpha), cos(alpha)){}

  /** returns the inverted RotationMatrix2x2. Note that inverting RotationMatrices
  * is the same as transposing them
  * @return inverse
  */
  RotationMatrix2x2 invert() const
  {
    return transpose();
  }

};
#endif // ROTATIONMATRIX2X2_H
