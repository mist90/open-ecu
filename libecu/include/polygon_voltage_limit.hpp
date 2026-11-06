/** @file polygon_voltage_limit.hpp
 * @brief Root-free d-priority limiting by a circumscribed regular 32-gon
 */
#ifndef LIBECU_POLYGON_VOLTAGE_LIMIT_HPP
#define LIBECU_POLYGON_VOLTAGE_LIMIT_HPP

#include "numeric_profiles.hpp"
#include "numeric_ops.hpp"

namespace libecu {

/**
 * @brief Axis-aligned tangent faces around a circle of the given inradius.
 *
 * In the first quadrant, q <= min_k(sec(k*pi/16)*L - tan(k*pi/16)*d),
 * k=0..7, and d <= L. Reflection gives 32 faces. The ideal circumradius
 * is L/cos(pi/32), only 1.004838573*L; at d=L, q=L*tan(pi/32), not zero.
 * This boundary rotates with dq: it is not the inverter's stationary hexagon.
 */
template<class Profile>
class PolygonVoltageLimit {
public:
    using Voltage = typename Profile::Voltage;
    using Request = typename Profile::VoltageRequest;
    using Duty = typename Profile::Duty;
    struct Output { Voltage d{}, q{}; bool limited = false; };

    /** @brief Largest inradius modulation whose corners fit the PWM duty span. */
    static Duty modulationCeiling(Duty max_duty, bool low_side_clamped) noexcept {
        const Duty span = low_side_clamped ? max_duty : scaleInteger<Duty>(max_duty, 2) - number_v<Duty, 1>;
        return multiplyTo<Duty>(span, number_v<Duty, 995184726, 1000000000>);
    }

    /** @brief Refresh only when the bus voltage or modulation configuration changes. */
    void prepare(Voltage limit) noexcept {
        limit_ = limit > Voltage{} ? limit : Voltage{};
    }

    /** @brief Radius of the original circle, before the polygon's corner extension. */
    Voltage inradius() const noexcept { return limit_; }

    /** @brief Positive q headroom after d is clamped; no root or division. */
    Voltage qLimit(Voltage d) const noexcept {
        d = numericAbs(numericClamp(d, -limit_, limit_));
        if (d <= crossing(0)) return limit_;
        // Seven sorted crossings select one of eight faces in three comparisons.
        unsigned face = d > crossing(3) ? 4u : 0u;
        if (d > crossing(face + 1)) face += 2;
        if (d > crossing(face)) ++face;
        Voltage room = faceLimit(face, d);
        // Rounded-down crossings can select the next face less than half a
        // raw d count early. With the strict > predicate, the preceding face
        // recovers the EXACT envelope of the quantized fixed coefficients.
        if (face) {
            const Voltage adjacent = faceLimit(face - 1, d);
            if (adjacent < room) room = adjacent;
        }
        if constexpr (!FIXED) {
            // Float multiplication rounds the crossings and line coefficients
            // independently. Include adjacent faces at their shared vertices.
            if (face < 7) {
                const Voltage adjacent = faceLimit(face + 1, d);
                if (adjacent < room) room = adjacent;
            }
        }
        return numericClamp(room, Voltage{}, limit_);
    }

    /** @brief Clamp wide PI requests before narrowing, retaining d priority. */
    Output apply(Request d, Request q) const noexcept {
        Output out;
        // The axis-to-axis diamond lies inside every tangent half-plane. Most
        // unsaturated samples need no face lookup at all; compare BEFORE any
        // narrow conversion so extreme PI requests cannot enter this shortcut.
        if constexpr (FIXED) {
            const uint32_t ad = d.raw() < 0 ? 0u - uint32_t(d.raw()) : uint32_t(d.raw());
            const uint32_t aq = q.raw() < 0 ? 0u - uint32_t(q.raw()) : uint32_t(q.raw());
            constexpr unsigned shift = Voltage::FRACTION_BITS - Request::FRACTION_BITS;
            const uint32_t bound = uint32_t(limit_.raw()) >> shift;
            if (ad <= bound && aq <= bound - ad) {
                out.d = Voltage::fromRaw(d.raw() * (int32_t(1) << shift));
                out.q = Voltage::fromRaw(q.raw() * (int32_t(1) << shift));
                return out;
            }
        } else if (numericAbs(d) + numericAbs(q) <= limit_) {
            out.d = d;
            out.q = q;
            return out;
        }
        out.d = clampRequest(d, limit_, out.limited);
        out.q = clampRequest(q, qLimit(out.d), out.limited);
        return out;
    }

private:
    static constexpr bool FIXED = std::is_same<Profile, FixedProfile>::value;
    static constexpr float SCALE_INVERSE = 1.0f / 268435456.0f;
    // Q28 floor(sec(k*pi/16)), ceil(tan(k*pi/16)), k=0..7. Directed
    // coefficient rounding and the final floor keep each fixed face inward.
    // Total q error versus ideal geometry is <1 + 2*L.raw()/2^28 raw counts.
    inline static constexpr int32_t INTERCEPT[8] = {
        268435456, 273694417, 290552443, 322844577,
        379625062, 483171055, 701455650, 1375954753,
    };
    inline static constexpr int32_t SLOPE[8] = {
        0, 53395133, 111189607, 179362838,
        268435456, 401742051, 648060519, 1349516170,
    };
    // floor(2^32 * (INTERCEPT[k+1]-INTERCEPT[k]) / (SLOPE[k+1]-SLOPE[k])).
    inline static constexpr uint32_t CROSSING[7] = {
        423017309u, 1252795731u, 2034429898u, 2737882096u,
        3336118918u, 3806150648u, 4129914107u,
    };
    static_assert([] {
        for (unsigned i = 0; i < 7; ++i) {
            const uint32_t da = INTERCEPT[i + 1] - INTERCEPT[i];
            const uint32_t db = SLOPE[i + 1] - SLOPE[i];
            if (!da || !db || CROSSING[i] != (uint64_t(da) << 32) / db) return false;
            if (INTERCEPT[i + 1] < INTERCEPT[0] || INTERCEPT[i + 1] < SLOPE[i + 1]) return false;
            // An early crossing must never skip more than one actual face.
            if (i < 6 && uint64_t(CROSSING[i + 1]) * db <= (uint64_t(da) << 32)) return false;
        }
        return uint64_t(INT32_MAX) * INTERCEPT[7] < (uint64_t(1) << 62);
    }(), "Polygon crossings, contained diamond and wide-product bounds must remain valid");

    Voltage crossing(unsigned index) const noexcept {
        if constexpr (FIXED) {
            return Voltage::fromRaw(static_cast<int32_t>((uint64_t(limit_.raw()) * CROSSING[index]) >> 32));
        } else {
            return limit_ * (float(CROSSING[index]) * (1.0f / 4294967296.0f));
        }
    }

    Voltage faceLimit(unsigned face, Voltage d) const noexcept {
        if (!face) return limit_;
        if constexpr (FIXED) {
            // Both products are <2^62. After >>28, the selected/preceding
            // face differences fit nonnegative int32, even at an early seam.
            const int64_t room = fixed_detail::wideProduct(limit_.raw(), INTERCEPT[face]) -
                                 fixed_detail::wideProduct(d.raw(), SLOPE[face]);
            return Voltage::fromRaw(static_cast<int32_t>(room >> 28));
        } else {
            return limit_ * (float(INTERCEPT[face]) * SCALE_INVERSE) -
                   d * (float(SLOPE[face]) * SCALE_INVERSE);
        }
    }

    static Voltage clampRequest(Request request, Voltage bound, bool& limited) noexcept {
        if constexpr (FIXED) {
            static_assert(Voltage::FRACTION_BITS >= Request::FRACTION_BITS,
                          "Wide requests must have at least the applied voltage's range");
            constexpr unsigned shift = Voltage::FRACTION_BITS - Request::FRACTION_BITS;
            const int32_t threshold = bound.raw() >> shift;
            if (request.raw() > threshold) { limited = true; return bound; }
            if (request.raw() < -threshold) { limited = true; return -bound; }
            // The comparisons prove that this signed product fits int32.
            return Voltage::fromRaw(request.raw() * (int32_t(1) << shift));
        } else {
            if (request > bound) { limited = true; return bound; }
            if (request < -bound) { limited = true; return -bound; }
            return request;
        }
    }

    Voltage limit_{};
};

} // namespace libecu
#endif // LIBECU_POLYGON_VOLTAGE_LIMIT_HPP
