/*
* Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
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
* AIROC Firmware Upgrade protocol
*
*/
#ifndef WS_OTA_FU_H
#define WS_OTA_FU_H

#if USE_WS_UPGRADE_DEFS
// Please note that all UUIDs need to be reversed when publishing in the database

// {9E5D1E47-5C13-43A0-8635-82AD38A1386F}
// static const GUID WS_UPGRADE_SERVICE =
// { 0x9e5d1e47, 0x5c13, 0x43a0, { 0x86, 0x35, 0x82, 0xad, 0x38, 0xa1, 0x38, 0x6f } };
#define UUID_WS_UPGRADE_SERVICE                      0x6f, 0x38, 0xa1, 0x38, 0xad, 0x82, 0x35, 0x86, 0xa0, 0x43, 0x13, 0x5c, 0x47, 0x1e, 0x5d, 0x9e

// {E3DD50BF-F7A7-4E99-838E-570A086C666B}
// static const GUID WS_UPGRADE_CHARACTERISTIC_CONTROL_POINT =
// { 0xe3dd50bf, 0xf7a7, 0x4e99, { 0x83, 0x8e, 0x57, 0xa, 0x8, 0x6c, 0x66, 0x6b } };
#define UUID_WS_UPGRADE_CHARACTERISTIC_CONTROL_POINT 0x6b, 0x66, 0x6c, 0x08, 0x0a, 0x57, 0x8e, 0x83, 0x99, 0x4e, 0xa7, 0xf7, 0xbf, 0x50, 0xdd, 0xe3

// {92E86C7A-D961-4091-B74F-2409E72EFE36}
// static const GUID WS_UPGRADE_CHARACTERISTIC_DATA =
// { 0x92e86c7a, 0xd961, 0x4091, { 0xb7, 0x4f, 0x24, 0x9, 0xe7, 0x2e, 0xfe, 0x36 } };
#define UUID_WS_UPGRADE_CHARACTERISTIC_DATA          0x36, 0xfe, 0x2e, 0xe7, 0x09, 0x24, 0x4f, 0xb7, 0x91, 0x40, 0x61, 0xd9, 0x7a, 0x6c, 0xe8, 0x92

// {347F7608-2E2D-47EB-91E9-75D4EDC4DE3B}
// static const GUID WS_UPGRADE_CHARACTERISTIC_APP_INFO =
// { 0x347f7608, 0x2e2d, 0x47eb, { 0x91, 0xe9, 0x75, 0xd4, 0xed, 0xc4, 0xde, 0x3b } };

#define UUID_WS_UPGRADE_CHARACTERISTIC_APP_INFO      0x3b, 0xde, 0xc4, 0xed, 0xd4, 0x75, 0x3b, 0x91, 0xeb, 0x47, 0x2d, 0x2e, 0x08, 0x76, 0x7f, 0x34

#else // USE_WS_UPGRADE_DEFS

// defines from wiced_bt_ota_firmware_upgrade.h

/* {aE5D1E47-5C13-43A0-8635-82AD38A1381F}
   static const GUID WSRU_OTA_SERVICE =
   { 0xae5d1e47, 0x5c13, 0x43a0, { 0x86, 0x35, 0x82, 0xad, 0x38, 0xa1, 0x38, 0x1f } }; */
#define UUID_OTA_FW_UPGRADE_SERVICE                             0x1f, 0x38, 0xa1, 0x38, 0xad, 0x82, 0x35, 0x86, 0xa0, 0x43, 0x13, 0x5c, 0x47, 0x1e, 0x5d, 0xae

/* {C7261110-F425-447A-A1BD-9D7246768BD8}
   static const GUID GUID_OTA_SEC_FW_UPGRADE_SERVICE =
   { 0xc7261110, 0xf425, 0x447a,{ 0xa1, 0xbd, 0x9d, 0x72, 0x46, 0x76, 0x8b, 0xd8 } }; */
#define UUID_OTA_SEC_FW_UPGRADE_SERVICE                         0xd8, 0x8b, 0x76, 0x46, 0x72, 0x9d, 0xbd, 0xa1, 0x7a, 0x44, 0x25, 0xf4, 0x10, 0x11, 0x26, 0xc7

/* {a3DD50BF-F7A7-4E99-838E-570A086C661B}
   static const GUID WSRU_OTA_CHARACTERISTIC_CONTROL_POINT =
   { 0xa3dd50bf, 0xf7a7, 0x4e99, { 0x83, 0x8e, 0x57, 0xa, 0x8, 0x6c, 0x66, 0x1b } }; */
#define UUID_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT        0x1b, 0x66, 0x6c, 0x08, 0x0a, 0x57, 0x8e, 0x83, 0x99, 0x4e, 0xa7, 0xf7, 0xbf, 0x50, 0xdd, 0xa3

/* {a2E86C7A-D961-4091-B74F-2409E72EFE26}
   static const GUID WSRU_OTA_CHARACTERISTIC_DATA =
   { 0xa2e86c7a, 0xd961, 0x4091, { 0xb7, 0x4f, 0x24, 0x9, 0xe7, 0x2e, 0xfe, 0x26 } }; */
#define UUID_OTA_FW_UPGRADE_CHARACTERISTIC_DATA                 0x26, 0xfe, 0x2e, 0xe7, 0x09, 0x24, 0x4f, 0xb7, 0x91, 0x40, 0x61, 0xd9, 0x7a, 0x6c, 0xe8, 0xa2

/* {a47F7608-2E2D-47EB-91E9-75D4EDC4DE4B}
   static const GUID WSRU_OTA_CHARACTERISTIC_APP_INFO =
   { 0xa47f7608, 0x2e2d, 0x47eb, { 0x91, 0xe9, 0x75, 0xd4, 0xed, 0xc4, 0xde, 0x4b } }; */

#define UUID_OTA_FW_UPGRADE_SERVICE_CHARACTERISTIC_APP_INFO     0x4b, 0xde, 0xc4, 0xed, 0xd4, 0x75, 0x3b, 0x91, 0xeb, 0x47, 0x2d, 0x2e, 0x08, 0x76, 0x7f, 0xa4

#define UUID_WS_UPGRADE_SERVICE                       UUID_OTA_FW_UPGRADE_SERVICE
#define UUID_WS_UPGRADE_CHARACTERISTIC_CONTROL_POINT  UUID_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT
#define UUID_WS_UPGRADE_CHARACTERISTIC_DATA           UUID_OTA_FW_UPGRADE_CHARACTERISTIC_DATA
#define UUID_WS_UPGRADE_CHARACTERISTIC_APP_INFO       UUID_OTA_FW_UPGRADE_SERVICE_CHARACTERISTIC_APP_INFO

#endif // USE_WS_UPGRADE_DEFS

// command definitions for the OTA FW upgrade
#define WS_UPGRADE_COMMAND_PREPARE_DOWNLOAD                 1
#define WS_UPGRADE_COMMAND_DOWNLOAD                         2
#define WS_UPGRADE_COMMAND_VERIFY                           3
#define WS_UPGRADE_COMMAND_FINISH                           4 // not currently used
#define WS_UPGRADE_COMMAND_GET_STATUS                       5 // not currently used
#define WS_UPGRADE_COMMAND_CLEAR_STATUS                     6 // not currently used
#define WS_UPGRADE_COMMAND_ABORT                            7

// event definitions for the OTA FW upgrade
#define WS_UPGRADE_STATUS_OK                                0
#define WS_UPGRADE_STATUS_UNSUPPORTED_COMMAND               1
#define WS_UPGRADE_STATUS_ILLEGAL_STATE                     2
#define WS_UPGRADE_STATUS_VERIFICATION_FAILED               3
#define WS_UPGRADE_STATUS_INVALID_IMAGE                     4
#define WS_UPGRADE_STATUS_INVALID_IMAGE_SIZE                5
#define WS_UPGRADE_STATUS_MORE_DATA                         6
#define WS_UPGRADE_STATUS_INVALID_APPID                     7
#define WS_UPGRADE_STATUS_INVALID_VERSION                   8
#define WS_UPGRADE_STATUS_CONTINUE                          9

// following definitions can be shared between client and sensor
// to avoid unnecessary GATT Discovery
//
#define HANDLE_WS_UPGRADE_SERVICE                           0xff00
#define HANDLE_WS_UPGRADE_CHARACTERISTIC_CONTROL_POINT      0xff01
#define HANDLE_WS_UPGRADE_CONTROL_POINT                     0xff02
#define HANDLE_WS_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR   0xff03
#define HANDLE_WS_UPGRADE_CHARACTERISTIC_DATA               0xff04
#define HANDLE_WS_UPGRADE_DATA                              0xff05
#define HANDLE_WS_UPGRADE_CHARACTERISTIC_APP_INFO           0xff06
#define HANDLE_WS_UPGRADE_APP_INFO                          0xff07

// When host is using Write No Response, it should wait for the
// Continue Notifications after NU
#define WS_UPGRADE_NUM_UNACKED_CHUNKS                       8

// Maximum data packet length we can process
#define WS_UPGRADE_MAX_DATA_LEN                             128

// Application ID 2 bytes plus 1 bytes major and minor versions
#define WS_UPGRADE_SUFFIX_LEN                               4

// To save 128 byte chunks to NVRAM, it would be best to receive 128 data packets from the peer
// The ideal MTU is 131 which is 128 + 3 bytes of ATT header.
#define WS_UPGRADE_LOCAL_MTU                (128 + sizeof(LEATT_PDU_WRITE_HDR))
#define WS_UPGRADE_MTU_MAX                  512 // Maximum allowed by BT spec

#pragma pack(1)

// structure to pass application information to upgrade application
typedef struct
{
    UINT16 ID;
    UINT8  Version_Major;
    UINT8  Version_Minor;
} WS_UPGRADE_APP_INFO;
#pragma pack()


// define entry points to upgrade functionality
int ws_upgrade_ota_init(void);
int ws_upgrade_ota_handle_command (UINT8 *data, int len);
int ws_upgrade_ota_handle_configuration (UINT8 *data, int len);
int ws_upgrade_ota_handle_data (UINT8 *data, int len);

#endif
