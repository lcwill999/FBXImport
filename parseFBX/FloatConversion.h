#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include "Math.h"
#include "UnionTuple.h"
#ifndef kPI
#define kPI 3.14159265358979323846264338327950288419716939937510F
#endif

#define UNUSED(EXPR_) \
    do { if (false) (void)(EXPR_); } while(0)
#define DebugAssert(test)                    UNUSED(test)
#define DebugAssertMsg(test, msg)            UNUSED(test)
#define DebugAssertFormatMsg(test, msg, ...) UNUSED(test)


typedef uint8_t     UInt8;
typedef uint16_t    UInt16;
typedef uint32_t    UInt32;
typedef uint64_t    UInt64;


typedef int8_t      SInt8;
typedef int16_t     SInt16;
typedef int32_t     SInt32;
typedef int64_t     SInt64;


const float kBiggestFloatSmallerThanOne = 0.99999994f;
const double kBiggestDoubleSmallerThanOne = 0.99999999999999989;

inline float FloatMin(float a, float b)
{
    return std::min<float>(a, b);
}

inline float FloatMax(float a, float b)
{
    return std::max<float>(a, b);
}

static inline float FloatMax3(float a, float b, float c)
{
    return FloatMax(a, FloatMax(b, c));
}

inline float FloatClamp(float v, float mn, float mx)
{
    return FloatMin(FloatMax(v, mn), mx);
}

inline float Abs(float v)
{
    return v < 0.0F ? -v : v;
}

inline double Abs(double v)
{
    return v < 0.0 ? -v : v;
}

inline int Abs(int v)
{
    return v < 0 ? -v : v;
}

//// Use std::nexttoward when C++11 is available
//inline float NextToward(float x, float toward)
//{
//    if (x == toward)
//        return x;
//
//    UInt32 asUInt = bit_cast<UInt32>(x);
//    if (x == 0)
//    {
//        if (x < toward)
//            asUInt = 1;
//        else
//            asUInt = 0x80000001;
//    }
//    else
//    {
//        if ((x < toward) == (x >= 0.0f))
//            asUInt++;
//        else
//            asUInt--;
//    }
//    return bit_cast<float>(asUInt);
//}

// Floor, ceil and round functions.
//
// When changing or implementing these functions, make sure the tests in MathTest.cpp
// still pass.
//
// Floor: rounds to the largest integer smaller than or equal to the input parameter.
// Ceil: rounds to the smallest integer larger than or equal to the input parameter.
// Round: rounds to the nearest integer. Ties (0.5) are rounded up to the smallest integer
// larger than or equal to the input parameter.
// Chop/truncate: use a normal integer cast.
//
// Windows:
// Casts are as fast as a straight fistp on an SSE equipped CPU. This is by far the most common
// scenario and will result in the best code for most users. fistp will use the rounding mode set
// in the control register (round to nearest by default), and needs fiddling to work properly.
// This actually makes code that attempt to use fistp slower than a cast.
// Unless we want round to nearest, in which case fistp should be the best choice, right? But
// it is not. The default rounding mode is round to nearest, but in case of a tie (0.5), round to
// nearest even is used. Thus 0.5 is rounded down to 0, 1.5 is rounded up to 2.
// Conclusion - fistp is useless without stupid fiddling around that actually makes is slower than
// an SSE cast.
//
// OS X Intel:
// Needs investigating
//
// OS X PowerPC:
// Needs investigating
//
//
// iPhone:
// Needs investigating
//
// Android:
// Needs investigating


inline float Sign(float f)
{
    if (f < 0.0F)
        return -1.0F;
    else
        return 1.0;
}

inline int FloorfToInt(float f)
{
    DebugAssert(f >= INT_MIN && f <= INT_MAX);
    return f >= 0 ? (int)f : (int)(f - kBiggestFloatSmallerThanOne);
}

inline UInt32 FloorfToIntPos(float f)
{
    DebugAssert(f >= 0 && f <= UINT_MAX);
    return (UInt32)f;
}

inline float Floorf(float f)
{
    // Use std::floor().
    // We are interested in reliable functions that do not lose precision.
    // Casting to int and back to float would not be helpful.
    return floor(f);
}

inline double Floord(double f)
{
    // Use std::floor().
    // We are interested in reliable functions that do not lose precision.
    // Casting to int and back to float would not be helpful.
    return floor(f);
}

inline int CeilfToInt(float f)
{
    DebugAssert(f >= INT_MIN && f <= INT_MAX);
    return f >= 0 ? (int)(f + kBiggestFloatSmallerThanOne) : (int)(f);
}

inline UInt32 CeilfToIntPos(float f)
{
    DebugAssert(f >= 0 && f <= UINT_MAX);
    return (UInt32)(f + kBiggestFloatSmallerThanOne);
}

inline float Ceilf(float f)
{
    // Use std::ceil().
    // We are interested in reliable functions that do not lose precision.
    // Casting to int and back to float would not be helpful.
    return ceil(f);
}

inline double Ceild(double f)
{
    // Use std::ceil().
    // We are interested in reliable functions that do not lose precision.
    // Casting to int and back to float would not be helpful.
    return ceil(f);
}

inline int RoundfToInt(float f)
{
    return FloorfToInt(f + 0.5F);
}

inline UInt32 RoundfToIntPos(float f)
{
    return FloorfToIntPos(f + 0.5F);
}

inline float Roundf(float f)
{
    return Floorf(f + 0.5F);
}

inline double Roundf(double f)
{
    return Floord(f + 0.5);
}

// Convert float [0...1] to word [0...65535]
inline int NormalizedToWord(float f)
{
    f = FloatClamp(f, 0.0f, 1.0f);
    return RoundfToIntPos(f * 65535.0f);
}

// Convert word [0...65535] to float [0...1]
inline float WordToNormalized(int p)
{
    DebugAssert(p >= 0 && p <= 65535);
    return (float)p / 65535.0F;
}

// Convert float [0...1] to byte [0...255]
inline int NormalizedToByte(float f)
{
    f = FloatClamp(f, 0.0f, 1.0f);
    return RoundfToIntPos(f * 255.0f);
}

// Convert byte [0...255] to float [0...1]
inline float ByteToNormalized(int p)
{
    DebugAssert(p >= 0 && p <= 255);
    return (float)p / 255.0F;
}

// Convert float [-1...1] to signed byte [-127...127]
inline int NormalizedToSignedByte(float f)
{
    f = FloatClamp(f, -1.0f, 1.0f);
    return RoundfToInt(f * 127.0f);
}

// Convert signed byte [-127...127] to float [-1...1]
// Note that -128 is intentionally not clamped to -1
// You must clamp yourself if that's a problem!
inline float SignedByteToNormalized(int p)
{
    DebugAssert(p >= -128 && p <= 127);
    return (float)p * (1.0f / 127.0f);
}

// Returns float remainder for t / length
inline float Repeat(float t, float length)
{
    return t - Floorf(t / length) * length;
}

// Returns double remainder for t / length
inline double RepeatD(double t, double length)
{
    return t - floor(t / length) * length;
}

// Returns relative angle on the interval (-pi, pi]
inline float DeltaAngleRad(float current, float target)
{
    float delta = Repeat((target - current), 2 * kPI);
    if (delta > kPI)
        delta -= 2 * kPI;
    return delta;
}

// Returns true if the distance between f0 and f1 is smaller than epsilon
inline bool CompareApproximately(float f0, float f1, float epsilon = 0.000001F)
{
    float dist = (f0 - f1);
    dist = Abs(dist);
    return dist <= epsilon;
}

inline bool CompareApproximatelyD(double d0, double d1, double epsilon = 0.000001)
{
    double dist = (d0 - d1);
    dist = Abs(dist);
    return dist <= epsilon;
}

/// CopySignf () returns x with its sign changed to y's.
inline float CopySignf(float x, float y)
{
#if PLATFORM_WEBGL
    // Loading floats as integers will not produce correct results in WebGL
    return x * Sign(x) * Sign(y);
#else
    union
    {
        float f;
        UInt32 i;
    } u, u0, u1;
    u0.f = x; u1.f = y;
    UInt32 a = u0.i;
    UInt32 b = u1.i;
    SInt32 mask = 1 << 31;
    UInt32 sign = b & mask;
    a &= ~mask;
    a |= sign;

    u.i = a;
    return u.f;
#endif
}

inline int CompareFloatRobustSignUtility(float A)
{
    // The sign bit of a number is the high bit.
    union
    {
        float f;
        int i;
    } u;
    u.f = A;
    return (u.i) & 0x80000000;
}

// Compare two floats and return true if they do not differ by more than maxUlps.
// A ULP is a unit of least precision, ie the unit value of the least significant
// digit of the mantissa (it is independent of the floating point exponent).
inline bool CompareFloatRobust(float f0, float f1, int maxUlps = 10)
{
    // After adjusting floats so their representations are lexicographically
    // ordered as twos-complement integers a very small positive number
    // will compare as 'close' to a very small negative number. If this is
    // not desireable, and if you are on a platform that supports
    // subnormals (which is the only place the problem can show up) then
    // you need this check.
    // The check for A == B is because zero and negative zero have different
    // signs but are equal to each other.
    if (CompareFloatRobustSignUtility(f0) != CompareFloatRobustSignUtility(f1))
        return f0 == f1;

    union
    {
        float f;
        int i;
    } u0, u1;
    u0.f = f0;
    u1.f = f1;
    int aInt = u0.i;
    // Make aInt lexicographically ordered as a twos-complement int
    if (aInt < 0)
        aInt = 0x80000000 - aInt;
    // Make bInt lexicographically ordered as a twos-complement int
    int bInt = u1.i;
    if (bInt < 0)
        bInt = 0x80000000 - bInt;

    // Now we can compare aInt and bInt to find out how far apart A and B
    // are.
    int intDiff = Abs(aInt - bInt);
    if (intDiff <= maxUlps)
        return true;
    return false;
}

// Returns the t^2
template<class T>
T Sqr(const T& t)
{
    return t * t;
}

#define kDeg2Rad (2.0F * kPI / 360.0F)
#define kRad2Deg (1.F / kDeg2Rad)

inline float Deg2Rad(float deg)
{
    // TODO : should be deg * kDeg2Rad, but can't be changed,
    // because it changes the order of operations and that affects a replay in some RegressionTests
    return deg / 360.0F * 2.0F * kPI;
}

inline float Rad2Deg(float rad)
{
    // TODO : should be rad * kRad2Deg, but can't be changed,
    // because it changes the order of operations and that affects a replay in some RegressionTests
    return rad / 2.0F / kPI * 360.0F;
}

inline float Radians(float deg)
{
    return deg * kDeg2Rad;
}

inline float Degrees(float rad)
{
    return rad * kRad2Deg;
}

inline float Lerp(float from, float to, float t)
{
    return to * t + from * (1.0F - t);
}

inline bool IsNAN(float value)
{
#if defined __APPLE_CC__
    return value != value;
#elif _MSC_VER
    return _isnan(value) != 0;
#elif defined(__GNUC__) && !defined(__ppc__)
    return __builtin_isnan(value);
#else
    return isnan(value);
#endif
}

inline bool IsNAN(double value)
{
#if defined __APPLE_CC__
    return value != value;
#elif _MSC_VER
    return _isnan(value) != 0;
#elif defined(__GNUC__) && !defined(__ppc__)
    return __builtin_isnan(value);
#else
    return isnan(value);
#endif
}

inline bool IsPlusInf(float value) { return value == std::numeric_limits<float>::infinity(); }
inline bool IsMinusInf(float value) { return value == -std::numeric_limits<float>::infinity(); }

inline int SignOrZero(const float& value)
{
    UInt32 bits = bit_cast<UInt32>(value);
    UInt32 sign = bits & 0x80000000;
    UInt32 otherBits = bits & ~0x80000000;
    return (otherBits == 0) ? 0 : (sign == 0) ? 1 : -1;
}

inline bool IsFinite(const float& value)
{
    // Returns false if value is NaN or +/- infinity
    UInt32 intval = bit_cast<UInt32>(value);
    return (intval & 0x7f800000) != 0x7f800000;
}

inline bool IsFinite(const double& value)
{
    // Returns false if value is NaN or +/- infinity
    UInt64 intval = bit_cast<UInt64>(value);
    return (intval & 0x7ff0000000000000LL) != 0x7ff0000000000000LL;
}

inline float InvSqrt(float p) { return 1.0F / sqrt(p); }
inline float Sqrt(float p) { return sqrt(p); }

/// - Almost highest precision sqrt
/// - Returns 0 if value is 0 or -1
/// inline float FastSqrt (float value)

/// - Almost highest precision inv sqrt
/// - if value == 0 or -0 it returns 0.
/// inline float FastInvSqrt (float value)

/// - Low precision inv sqrt approximately
/// - if value == 0 or -0 it returns nan or undefined
/// inline float FastestInvSqrt (float value)

inline float FastSqrt(float value)
{
    return sqrtf(value);
}

inline float FastInvSqrt(float f)
{
    // The Newton iteration trick used in FastestInvSqrt is a bit faster on
    // Pentium4 / Windows, but lower precision. Doing two iterations is precise enough,
    // but actually a bit slower.
    if (fabs(f) == 0.0F)
        return f;
    return 1.0F / sqrtf(f);
}

#if PLATFORM_WEBGL
// loading floats as ints is too slow in WebGL.
#define FastestInvSqrt FastInvSqrt
#else
inline float FastestInvSqrt(float f)
{
    union
    {
        float f;
        int i;
    } u;
    float fhalf = 0.5f * f;
    u.f = f;
    int i = u.i;
    i = 0x5f3759df - (i >> 1);
    u.i = i;
    f = u.f;
    f = f * (1.5f - fhalf * f * f);
    // f = f*(1.5f - fhalf*f*f); // uncommenting this would be two iterations
    return f;
}

#endif


inline float SqrtImpl(float f)
{
    return sqrt(f);
}

inline float Sin(float f)
{
    return sinf(f);
}

inline float Pow(float f, float f2)
{
    return powf(f, f2);
}

inline float Cos(float f)
{
    return cosf(f);
}

// fast fp32/fp16/fp10/fp11 handling (will break on NaN/Inf, will have issues with denorm)
namespace fp
{
    namespace internal
    {
        union FP32Bits
        {
            float   fp32;
            UInt32  u32;
        };
    }
}

#define FP32_TO_FP16_MAGIC(fi)  ((((fi)>>16)&0x8000) | (((((fi)&0x7f800000)-0x38000000)>>13)&0x7c00) | (((fi)>>13)&0x03ff))
#define FP16_TO_FP32_MAGiC(fi)  ((((fi)&0x3ff)<<13) | (((((fi)>>10)&0x1f)+(127-15))<<23) | (((fi)&0x8000)<<16))


inline UInt16 ConvertFP32ToFP16(float val)
{
    fp::internal::FP32Bits fpbits;
    fpbits.fp32 = val;

    UInt32 fi = fpbits.u32;
    return fi != 0 ? FP32_TO_FP16_MAGIC(fi) : 0;
}

inline void ConvertFP32ToFP16(const float* __restrict src, UInt16* __restrict dst, size_t count)
{
    fp::internal::FP32Bits fpbits;
    for (size_t i = 0, n = count; i < n; ++i)
    {
        fpbits.fp32 = src[i];
        const UInt32 fi = fpbits.u32;
        dst[i] = fi != 0 ? FP32_TO_FP16_MAGIC(fi) : 0;
    }
}

inline float ConvertFP16ToFP32(UInt16 val_)
{
    fp::internal::FP32Bits fpbits;
    const UInt32 val = (UInt32)val_;
    fpbits.u32 = (val & 0x777f) != 0 ? FP16_TO_FP32_MAGiC(val) : 0;
    return fpbits.fp32;
}

inline void ConvertFP16ToFP32(const UInt16* __restrict src, float* __restrict dst, size_t count)
{
    fp::internal::FP32Bits fpbits;
    for (size_t i = 0, n = count; i < n; ++i)
    {
        const UInt32 val = src[i];
        fpbits.u32 = (val & 0x777f) != 0 ? FP16_TO_FP32_MAGiC(val) : 0;
        dst[i] = fpbits.fp32;
    }
}

#undef FP16_TO_FP32_MAGiC
#undef FP32_TO_FP16_MAGIC


// proper fp32<->fp16 conversion (will handle all kinds of floats)
class FloatToHalfConverter
{
public:
    FloatToHalfConverter() {}

private:
    struct ExponentData
    {
        UInt32 mask : 16;
        UInt32 shiftMinus1 : 8;
    };

public:
    UInt16 ConvertBits(UInt32 bits)
    {
        UInt8 index = UInt8(bits >> 23);
        UInt32 sign = (bits >> 16) & 0x8000;
        UInt32 mantissa = (bits & 0x007fffff);

        ExponentData exponentData = m_ExponentTable[index];
        UInt16 result;
        result = mantissa >> exponentData.shiftMinus1;
        result |= exponentData.mask;

        bool inputIsNan = index == 0xff && mantissa;
        if (inputIsNan)
        {
            // if input was NaN, make sure we don't clear significand completely,
            // otherwise we would return infinity by error. We set the second most
            // significant bit in the significand to not affect the quiet/signalling
            // bit.
            result = (result | 0x200) >> 1;
        }
        else
        {
            // round the number
            result = (result + 1) >> 1;
        }
        result |= sign;
        return result;
    }

    // Pass the argument by reference so that the value is not loaded into
    // register on some platforms if the function is not inlined.
    //void Convert(const float& src, UInt16& dest)
    //{
    //    dest = ConvertBits(bit_cast<UInt32>(src));
    //}

    //void Convert(size_t count, const float* src, UInt16* dest)
    //{
    //    for (size_t i = 0, n = count; i < n; ++i)
    //        Convert(src[i], dest[i]);
    //}

    static void InitializePrecomputeTables();

private:
    static ExponentData m_ExponentTable[256];
};

extern FloatToHalfConverter g_FloatToHalf;

//inline UInt16 FloatToHalf(const float& src)
//{
//    UInt16 dest;
//    g_FloatToHalf.Convert(src, dest);
//    return dest;
//}

//inline void FloatToHalf(const float* __restrict src, UInt16* __restrict dst, size_t count)
//{
//    g_FloatToHalf.Convert(count, src, dst);
//}

//inline void HalfToFloatImpl(UInt16 src, float& res)
//{
//    // Based on Fabian Giesen's public domain half_to_float_fast3
//    static const UInt32 magic = { 113 << 23 };
//    const float& magicFloat = bit_cast<float>(magic);
//    static const UInt32 shiftedExp = 0x7c00 << 13; // exponent mask after shift
//
//    // Mask out sign bit
//    UnionTuple<float, UInt32> o;
//    o.second = src & 0x7fff;
//    if (o.second)
//    {
//        // Move exponent + mantissa to correct bits
//        o.second <<= 13;
//        UInt32 exponent = o.second & shiftedExp;
//        if (exponent == 0)
//        {
//            // Handle denormal
//            o.second += magic;
//            o.first -= magicFloat;
//        }
//        else if (exponent == shiftedExp) // Inf/NaN
//            o.second += (255 - 31) << 23;
//        else
//            o.second += (127 - 15) << 23;
//    }
//
//    // Copy sign bit
//    o.second |= (src & 0x8000) << 16;
//
//    res = o.first;
//}

//inline float HalfToFloat(UInt16 src)
//{
//    float res;
//    HalfToFloatImpl(src, res);
//    return res;
//}

//inline void HalfToFloat(const UInt16* __restrict src, float* __restrict dst, size_t count)
//{
//    for (size_t i = 0; i < count; ++i)
//        dst[i] = HalfToFloat(src[i]);
//}

extern UInt16 g_ByteToNormalizedHalf[256];
extern UInt16 g_SignedByteToNormalizedHalf[256];

inline UInt16 ByteToNormalizedHalf(int v)
{
    DebugAssert(v >= 0 && v <= 255);
    return g_ByteToNormalizedHalf[static_cast<UInt8>(v)];
}

inline UInt16 SignedByteToNormalizedHalf(int v)
{
    DebugAssert(v >= -128 && v <= 127);
    return g_SignedByteToNormalizedHalf[static_cast<UInt8>(v)];
}

using std::cos;
using std::pow;
using std::atan2;
using std::acos;
using std::sin;
using std::sqrt;
using std::log;
using std::exp;

// On non-C99 platforms log2 is not available, so approximate it.
#if PLATFORM_WIN || PLATFORM_ANDROID
#define kNaturalLogarithm2 0.693147180559945309417f
inline float Log2(float x) { return logf(x) / kNaturalLogarithm2; }
#elif defined(__ARMCC_VERSION)
inline float Log2(float x) { return __builtin_log2f(x); }
#else
inline float Log2(float x) { return log2f(x); }
#endif

// returns logarithmic-nearest power of two
inline int RoundToNearestPowerOfTwo(float v)
{
    return (int)Pow(2, Roundf(std::max<float>(Log2(v), 0.0f)));
}

static float pi() { return 3.14159265358979323846f; }                              // pi
static float pi_over_two() { return 1.57079632679489661923f; }                     // pi / 2
static float pi_over_three() { return 1.0471975511965977461542144610932f; }        // pi / 3
static float pi_over_four() { return 0.785398163397448309615660845819875721f; }    // pi / 4
static float pi_over_six() { return 0.52359877559829887307710723054658f; }         // pi / 6
static float one_over_pi() { return 0.31830988618379067153776752674503f; }         // 1 / pi
static float one_over_two_pi() { return 0.159154943091895335768883763372514362f; } // 1 / 2*pi