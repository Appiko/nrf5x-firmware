/*
 *  common_util.h
 *
 *  Created on: 06-Feb-2017
 *
 *  Copyright (c) 2017, Appiko
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its contributors
 *  may be used to endorse or promote products derived from this software without
 *  specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 *  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 *  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
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

#endif /* CODEBASE_COMMON_UTIL_H_ */

/**
 * @}
 * @}
 */
