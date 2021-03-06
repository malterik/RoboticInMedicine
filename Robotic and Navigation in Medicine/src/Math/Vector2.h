#ifndef __Vector2_h__
#define __Vector2_h__

#include <cmath>
#include <sstream>

/// Vector with 2 elements
/** This class represents a 2-vector 
 * Inspired by <a href="http://www.b-human.de/file_download/32/bhuman10_coderelease.tar.bz2">BHuman Code-Release 2010</a>
 *
 * modified and extended by <a href="mailto:stefan.kaufmann@tu-harburg.de">Stefan Kaufmann</a>
 * 
 * @note This class has to be trivial/plain old data (POD) so that it can be used in unions.
 */
template <class V = float> class Vector2 
{
public:  
  /** The x-value */
  V x;
  /** The y-value */
  V y;

  /** Default constructor*/
  Vector2<V>():x(0),y(0)
  {
  }
//  Vector2() = default;

  /** Default constructor. */
  Vector2<V>(V x, V y):x(x),y(y)
  {}

  /** Assignment operator
   * @param other The other vector that is assigned to this one
   * @return A reference to this object after the assignment.
   */
  /*Vector2<V>& operator=(const Vector2<V>& other) 
  {
    x = other.x;
    y = other.y;
    return *this;
  }*/
//  Vector2& operator = ( Vector2 const& ) = default;

  /** Copy constructor
   * @param other The other vector that is copied to this one
   */
  Vector2<V>(const Vector2<V>& other): x(other.x), y(other.y)
  {
    //*this = other;
  }

  /** Addition of another vector to this one. 
   * @param other The other vector that will be added to this one
   * @return A reference to this object after the calculation.
   */
  Vector2<V>& operator+=(const Vector2<V>& other)
  {
    x += other.x;
    y += other.y;
    return *this;
  }

  /** Substraction of this vector from another one.
   * @param other The other vector this one will be substracted from 
   * @return A reference to this object after the calculation.
   */
  Vector2<V>& operator-=(const Vector2<V>& other)
  {
    x -= other.x;
    y -= other.y;
    return *this;
  }

  /** Multiplication of this vector by a factor.
   * @param factor The factor this vector is multiplied by 
   * @return A reference to this object after the calculation.
   */
  Vector2<V>& operator*=(const V& factor)
  {
    x *= factor;
    y *= factor;
    return *this;
  }

  /** Division of this vector by a factor.
   * @param factor The factor this vector is divided by 
   * @return A reference to this object after the calculation.
   */
  Vector2<V>& operator/=(const V& factor)
  {
    if (factor == V(0)) return *this;
    x /= factor;
    y /= factor;
    return *this;
  }

  /** Addition of another vector to this one.
   * @param other The other vector that will be added to this one
   * @return A new object that contains the result of the calculation.
   */
  Vector2<V> operator+(const Vector2<V>& other) const
    {return Vector2<V>(*this) += other;}

  /** Subtraction of another vector to this one.
   * @param other The other vector that will be added to this one
   * @return A new object that contains the result of the calculation.
   */
  Vector2<V> operator-(const Vector2<V>& other) const
    {return Vector2<V>(*this) -= other;}

  /** Negation of this vector.
   * @return A new object that contains the result of the calculation.
   */
  Vector2<V> operator-() const
    {return Vector2<V>() -= *this;}

  /** Inner product of this vector and another one.
   * @param other The other vector this one will be multiplied by 
   * @return The inner product.
   */
  V operator*(const Vector2<V>& other) const
  {
    return (x*other.x + y*other.y);
  }

 
  /** Multiplication of this vector by a factor.
   * @param factor The factor this vector is multiplied by 
   * @return A new object that contains the result of the calculation.
   */
  Vector2<V> operator*(const V& factor) const
    {return Vector2<V>(*this) *= factor;}

  /** Division of this vector by a factor.
   *
   * @param factor The factor this vector is divided by 
   * @return A new object that contains the result of the calculation.
   */
  Vector2<V> operator/(const V& factor) const
    {return Vector2<V>(*this) /= factor;}

  /** Comparison of another vector with this one.
   * @param other The other vector that will be compared to this one
   * @return Whether the two vectors are equal.
   */
  bool operator==(const Vector2<V>& other) const
  {
    return (x==other.x && y==other.y );
  }

  /** Comparison of another vector with this one.
   * @param other The other vector that will be compared to this one
   * @return Whether the two vectors are unequal.
   */
  bool operator!=(const Vector2<V>& other) const
    {return !(*this == other);}

  
  /**
   * array-like member access.
   * @param i index of coordinate
   * @return reference to x, y or z
   */
  V& operator[](int i)
  {
    switch (i)
    {
    case 0:
      return x;
      break;
    case 1:
      return y;
      break;
    default:
      return (&x)[i];
    }
  }
  
  /**
   * const array-like member access.
   * @param i index of coordinate
   * @return reference to x or y
   */
  const V& operator[](int i) const
  {
    switch (i)
    {
    case 0:
      return x;
      break;
    case 1:
      return y;
      break;
    default:
      return (&x)[i];
    }
  }

  /** Calculation of the length of this vector.
   * @return The length.
   */
  V abs() const 
  {return (V) sqrtf(float((x*x) + (y*y)));}

  /** Calculation of the square length of this vector.
   * @return length*length.
   */
  V squareAbs() const
  {return (x*x) + (y*y);}

  /** normalize this vector.
   * @param len The length, the vector should be normalized to, default=1.
   * @return the normalized vector.
   */
  Vector2<V> normalize(V len)
  {
    V lenghtOfVector = abs();
    if (lenghtOfVector == V(0)) return *this;
    return *this = (*this * len) / lenghtOfVector;
   }

 /** normalize this vector.
  * @return the normalized vector.
  */
  Vector2<V> normalize()
  {
    V lenghtOfVector = abs();
    if (lenghtOfVector == V(0)) return *this;
    return *this /= lenghtOfVector;
  }

  /**
   * changes the vector entries
   * @return The switched vector
   */ 
  Vector2<V> switchEntries() const
  {
	return Vector2<float>(y,x);
  }
  
  /**
   * Vector Information in string form. 
   * Helpful for logging
   * @return The vector Information
   */
  std::string toString() const
  {
	std::ostringstream s;
	s << "x= ";
	s << x;
	s << ", y= ";
	s << y;
	return s.str();
  }

};

#endif // __Vector2_h__

