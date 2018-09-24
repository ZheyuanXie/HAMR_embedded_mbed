// Reminders about C++ standard integer operations on uint32_t and int32_t.
//
// uint32_t u;
// int32_t s;
//
// Signed to Unsigned Conversion: "Wraps (Safe)"
//   static_cast<uint32_t> s is source value + value of 2^32
// Unsigned to Signed Conversion "Wraps (Unsafe)"
//   static_vast<int32_t> u is implementation defined if u is too large
//   however, gcc specifies that in this case the value is reduced by 2^32.
// Unsigned + Unsigned Overflow: "OK"
//   performed modulo 2^32
// Signed + Signed overflow: "BAD"
//   UNDEFINED


#ifndef FIXED_POINT_MATH_HPP
#define FIXED_POINT_MATH_HPP

#include <cstdint>

// Compile time powers.  Template argument is storage/return type.
template <typename T>
const T ipow(T base, unsigned exp, T result = 1) {
  return exp < 1 ? result : ipow(base*base, exp/2, (exp % 2) ? result*base : result);
}

// Compile time powers of 2.  Template argument is storage/return type.
template <typename T>
const T pow2(unsigned exp) {
  return ipow(T(2), exp);
}

#endif // FIXED_POINT_MATH_HPP

