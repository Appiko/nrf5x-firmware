/**
 *  boards.h : Board handling
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

#ifndef PLATFORM_BOARDS_H_
#define PLATFORM_BOARDS_H_

/**
 * @addtogroup group_platform
 * @{
 *
 * @brief A @ref boards.h file includes the relevant platform based on
 *  macro from the compile time defines
 */

#if defined(BOARD_PCA10040)
  #include "pca10040.h"
#elif defined(BOARD_PCA10028)
  #include "pca10028.h"
#elif defined(BOARD_DETECT_REV1)
  #include "detect_rev1.h"
#elif defined(BOARD_BLUEY_1V1)
  #include "bluey_1v1.h"
#elif defined(BOARD_SENSEPI_REV2)
  #include "sensepi_rev2.h"
#elif defined(BOARD_SENSEPI_REV3)
  #include "sensepi_rev3.h"
#elif defined(BOARD_SENSEPI_REV4)
  #include "sensepi_rev4.h"
#elif defined(BOARD_HACKABLE)
  #include "hackable.h"
#elif defined(BOARD_SENSEBE_RX_REV1)
  #include "sensebe_rx_rev1.h"
#elif defined(BOARD_SENSEBE_RX_REV2)
  #include "sensebe_rx_rev2.h"
#elif defined(BOARD_SENSEBE_REV1)
  #include "sensebe_rev1.h"
#elif defined(BOARD_SENSEBE_REV3)
  #include "sensebe_rev3.h"
#elif defined(BOARD_SENSEBETX_REV3)
  #include "sensebetx_rev3.h"
#elif defined(BOARD_SENSEBERX_REV3)
  #include "senseberx_rev3.h"
#elif defined(BOARD_SENSEELE_SMA_REV1)
  #include "senseele_sma_rev1.h"
#elif defined(BOARD_SENSEELE_PCB_REV1)
  #include "senseele_pcb_rev1.h"
#elif defined(BOARD_SENSEELE_PCB_REV2)
  #include "senseele_pcb_rev2.h"
#elif defined(BOARD_SENSEBLE_REV1)
  #include "senseble_rev1.h"
#elif defined(BOARD_CUSTOM)
  #include "custom_board.h"
#else
#error "Board is not defined"

#endif

#endif /* PLATFORM_BOARDS_H_ */

/** @} */
