/**
 *  common_util.h : Common Utilities
 *  Copyright (C) 2019  Appiko
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/**
 * @addtogroup group_util
 * @{
 *
 * @defgroup group_common_util Common utilities
 * @brief Common helpers used in C programs.
 * @{
 */

#ifndef CODEBASE_COMMON_UTIL_H_
#define CODEBASE_COMMON_UTIL_H_

/** The minimum of the two 32-bit arguments */
#define MIN(a, b) ((a) < (b) ? (a) : (b))
/** The maximum of the two 32-bit arguments */
#define MAX(a, b) ((a) < (b) ? (b) : (a))

/**@brief Concatenates or joins two parameters.
 *
 * Two level expansion are needed to make sure that the two
 * parameters are fully expanded before joining them together.
 *
 * @param p1 First parameter for concatenating
 * @param p2 Second parameter for concatenating
 *
 * @return Two parameters joined together.
 *
 * @see CONCAT_3
 */
#define CONCAT_2(p1, p2)      CONCAT_2_(p1, p2)
/** Private macro used by @ref CONCAT_2 */
#define CONCAT_2_(p1, p2)     p1##p2

/**@brief Concatenates or joins three parameters.
 *
 * Two level expansion are needed to make sure that the three
 * parameters are fully expanded before joining them together.
 *
 * @param p1 First parameter for concatenating
 * @param p2 Second parameter for concatenating
 * @param p3 Third parameter for concatenating
 *
 * @return Three parameters joined together.
 *
 * @see CONCAT_2
 */
#define CONCAT_3(p1, p2, p3)  CONCAT_3_(p1, p2, p3)
/** Private macro used by @ref CONCAT_3 */
#define CONCAT_3_(p1, p2, p3) p1##p2##p3

/** The number of elements inside a array  */
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

/**@brief Rounded integer division where the result is the integer closest to the
 *  answer instead of flooring the result
 *
 * @param[in]   A   Numerator for the division.
 * @param[in]   B   Denominator for the division.
 *
 * @return      Rounded integer from dividing A by B.
 */
#define ROUNDED_DIV(A, B) (((A) + ((B) / 2)) / (B))

/**@brief When the result of a division is not an integer, the result is rounded
 *  up to the next integer.
 *
 * @param[in]   A   Numerator for the division.
 * @param[in]   B   Denominator for the division.
 *
 * @return      Rounded up result of dividing A by B.
 */
#define CEIL_DIV(A, B)      \
    (((A) + (B) - 1) / (B))

/**@brief Checks if an integer is a power of two.
 *
 * @param[in]   A   The number to be checked.
 *
 * @return      Bool representing if the value is a power of 2
 */
#define IS_POWER_OF_TWO(A) ( ((A) != 0) && ((((A) - 1) & (A)) == 0) )

enum
{
    UNIT_0_625_MS = 625,  /**< Number of us in 0.625 ms. */
    UNIT_1_25_MS  = 1250, /**< Number of us in 1.25 ms. */
    UNIT_10_MS    = 10000 /**< Number of us in 10 ms. */
};

/**@brief Macro for converting ms to ticks.
 *
 * @param[in] TIME          Number of ms to convert.
 * @param[in] RESOLUTION    Unit to be converted to in [us/ticks].
 */
#define MSEC_TO_UNITS(TIME, RESOLUTION) (((TIME) * 1000) / (RESOLUTION))

/**
 * @brief Macro to set particular bit in a given variable without
 *  affecting other bits
 * @param[in] VAR           Variable from which bit is to be set
 * @param[in] BIT_NO        Bit number which is be set
 */
#define SET_BIT_VAR(VAR, BIT_NO)    (VAR | (1 << BIT_NO))

/**
 * @brief Macro to clear particular bit in a given variable without
 *  affecting other bits
 * @param[in] VAR           Variable from which bit is to be clear
 * @param[in] BIT_NO        Bit number which is be clear
 */
#define CLR_BIT_VAR(VAR, BIT_NO)    (VAR & ~(1 << BIT_NO))

#endif /* CODEBASE_COMMON_UTIL_H_ */

/**
 * @}
 * @}
 */
