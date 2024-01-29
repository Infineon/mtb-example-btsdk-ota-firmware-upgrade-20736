/*
* Copyright 2016-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*/

/** @file
*
* AIROC Firmware Upgrade
*
* This file provides definitions and function prototypes for the
* Firmware Upgrade process
*
*/
#ifndef WS_UPGRADE_H
#define WS_UPGRADE_H

/*****************************************************************************/
/** @defgroup ws_upgrade   AIROC Firmware Upgrade
 *
 *  AIROC Firmware Upgrade functionality is used to store and retrieve data from
 *  the memory being upgraded.  The 2073x device may be connected to an
 *  EEPROM or Serial Flash.  The functionality in this module allows application
 *  to access the memory without knowing what is used and the connection method.
 */
/*****************************************************************************/

enum
{
    WS_UPGRADE_NV_TYPE_EEPROM = 0,
    WS_UPGRADE_NV_TYPE_SF     = 1
};

extern UINT32   ws_upgrade_active_nv_type;

/**
* \brief Initialize AIROC Firmware Upgrade module
* \ingroup ws_upgrade
*
* \details This function is typically called by the application during initialization
* to initialize upgrade module.  Specifically this function finds out which
* medium (serial flash or EEPROM) is installed and saves in the ws_upgrade_active_nv_type
* variable.
*
*/
void     ws_upgrade_init(void);

/**
* \brief Initialize NV locations
* \ingroup ws_upgrade
*
* \details Application calls this function during the start of the firmware download
* to setup memory locations depending on which partition is being used and which
* medium (serial flash or EEPROM) is installed.
*
*/
UINT32     ws_upgrade_init_nv_locations(void);

/**
* \brief Store memory chunk to memory
* \ingroup ws_upgrade
*
* \details Application can call this function to store the next memory chunk in the
* none volatile memory.  Application does not need to know which type of memory is
* used or which partition is being upgraded.
*
* \param offset Offset in the memory where data need to be stored
* \param data   Pointer to the chunk of data to be stored
* \param len    Size of the memory chunk that need to be stored
*
*/
UINT32   ws_upgrade_store_to_nv(UINT32 offset, UINT8 *data, UINT32 len);

/**
* \brief Retrieve memory chunk from memory
* \ingroup ws_upgrade
*
* \details Application typically calls this function when the upgrade process has
* been completed to verify that the data has been successfully stored.  Application
* does not need to know which type of memory is used or which partition is being upgraded.
*
* \param offset Offset in the memory from where data need to be retrieved
* \param data   Pointer to the location to retrieve the data
* \param len    Size of the memory chunk to be retrieved
*
*/
UINT32   ws_upgrade_retrieve_from_nv(UINT32 offset, UINT8 *data, UINT32 len);

/**
* \brief Retrieve memory chunk from memory
* \ingroup ws_upgrade
*
* \details After download is completed and verified this function is called to
* switch active partitions with the one that has been receiving the new image.
*
*/
void     ws_upgrade_finish(void);

#endif
