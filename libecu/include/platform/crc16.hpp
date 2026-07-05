/**
 * @file crc16.hpp
 * @brief CRC-16 CCITT-FALSE implementation
 */

#ifndef LIBECU_CRC16_HPP
#define LIBECU_CRC16_HPP

#include <cstdint>
#include <cstddef>

namespace libecu {

/**
 * @brief Compute CRC-16 CCITT-FALSE for given data
 * @param data Pointer to data buffer
 * @param length Length of data in bytes
 * @return CRC-16 value
 */
uint16_t crc16_compute(const uint8_t* data, std::size_t length);

/**
 * @brief Verify CRC-16 CCITT-FALSE for given data against expected value
 * @param data Pointer to data buffer
 * @param length Length of data in bytes
 * @param expected_crc Expected CRC-16 value
 * @return true if CRC matches, false otherwise
 */
bool crc16_verify(const uint8_t* data, std::size_t length, uint16_t expected_crc);

} // namespace libecu

#endif // LIBECU_CRC16_HPP
