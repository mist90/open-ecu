/** @file arm_products.cpp
 * @brief ARM machine-code probes for signed products after inlined saturation
 */
#include "profile_math.hpp"

using namespace libecu;

extern "C" __attribute__((used, externally_visible)) int64_t raw_product(int32_t a, int32_t b) {
    return fixed_detail::wideProduct(a, b);
}
extern "C" __attribute__((used, externally_visible)) int32_t fraction_product(int32_t a, int32_t b) {
    return multiplyTo<FixedPointType<30>>(FixedPointType<30>::fromRaw(a),
                                         FixedPointType<30>::fromRaw(b)).raw();
}
extern "C" __attribute__((used, externally_visible)) int32_t saturated_product(int32_t a, int32_t b) {
    const auto left = FixedPointType<16>::fromRaw(a) - FixedPointType<16>::fromRaw(b);
    const auto right = FixedPointType<14>::fromRaw(a) + FixedPointType<14>::fromRaw(b);
    return multiplyTo<FixedPointType<18>>(left, right).raw();
}
extern "C" __attribute__((used, externally_visible)) int64_t widening_product(int32_t a, int32_t b) {
    return multiplyTo<FixedPointType<32, int64_t>>(FixedPointType<14>::fromRaw(a),
                                                  FixedPointType<14>::fromRaw(b)).raw();
}
extern "C" __attribute__((used, externally_visible)) int32_t reciprocal(int32_t a, int32_t) {
    return FixedMath::inverseBusVoltage(FixedProfile::Voltage::fromRaw(a)).raw();
}
extern "C" __attribute__((used, externally_visible)) uint32_t integer_root(uint32_t low, uint32_t high) {
    return integerSqrt((uint64_t(high) << 32) | low);
}
