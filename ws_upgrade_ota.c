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
* This file provides function required to support Over the Air Firmware Upgrade.
*
* To download host sends command to download with length of the patch to be
* transmitted.  After device acks that, host sends fixed chunks of data
* each of which has to be acked.  After all the bytes has been downloaded
* and acknowledged host sends verify command that includes CRC32 of the
* whole patch.  During the download device saves data directly to the EEPROM
* or serial flash.  At the verification stage device reads data back from the
* NVRAM and calculates checksum of the data stored there.  Result of the
* verification is sent back to the host.
*
*/
// #define BLE_TRACE_DISABLE 1

#include "bleprofile.h"
#include "bleapp.h"
#include "puart.h"
#include "gpiodriver.h"
#include "string.h"
#include "stdio.h"
#include "stdint.h"
#include "platform.h"
#include "bleapputils.h"
#include "bleappfwu.h"
#include "ws_upgrade_ota.h"
#include "ws_upgrade.h"
#include "spar_utils.h"

/******************************************************
 *                      Constants
 ******************************************************/

// device states during OTA FW upgrade
#define WS_UPGRADE_STATE_IDLE                   0
#define WS_UPGRADE_STATE_READY_FOR_DOWNLOAD     1
#define WS_UPGRADE_STATE_DATA_TRANSFER          2
#define WS_UPGRADE_STATE_VERIFICATION           3
#define WS_UPGRADE_STATE_VERIFIED               4
#define WS_UPGRADE_STATE_ABORTED                5

// downloader will use 16 byte chunks
#define WS_UPGRADE_READ_CHUNK                   128

// write to eeprom in 64 byte chunks
#define WS_UPGRADE_CHUNK_SIZE_TO_COMMIT         128


/******************************************************
 *               Variables Definitions
 ******************************************************/

UINT8   ws_upgrade_state   = WS_UPGRADE_STATE_IDLE;

UINT16  ws_upgrade_client_configuration;        // characteristic client configuration descriptor
UINT8   ws_upgrade_status;                      // Current status
UINT32  ws_upgrade_current_offset;              // Offset in the image to store the data
UINT32  ws_upgrade_total_len;                   // Total length expected from the host

int     ws_upgrade_current_block_offset;
int     ws_upgrade_total_offset;
UINT32  ws_upgrade_crc32;
uint8_t ws_upgrade_read_buffer[WS_UPGRADE_CHUNK_SIZE_TO_COMMIT + WS_UPGRADE_LOCAL_MTU];
int     ws_upgrade_num_unacked_chunks;

// Initialize peripheral UART upgrade procedure
int ws_upgrade_ota_init(void)
{
    ws_upgrade_state = WS_UPGRADE_STATE_IDLE;

    ws_upgrade_init();
    return TRUE;
}

#define POLYNOMIAL              0x04C11DB7
#define WIDTH                   (8 * sizeof(unsigned long))
#define TOPBIT                  (1 << (WIDTH - 1))
#define INITIAL_REMAINDER       0xFFFFFFFF
#define FINAL_XOR_VALUE         0xFFFFFFFF
#define REFLECT_DATA(X)         ((unsigned char) reflect((X), 8))
#define REFLECT_REMAINDER(X)    ((unsigned long) reflect((X), WIDTH))
#define CHECK_VALUE             0xCBF43926


extern void *__aeabi_memmove(void *dest, const void * src, int size);

/*********************************************************************
 *
 * Function:    reflect()
 *
 * Description: Reorder the bits of a binary sequence, by reflecting
 *              them about the middle position.
 *
 * Notes:       No checking is done that nBits <= 32.
 *
 * Returns:     The reflection of the original data.
 *
 *********************************************************************/
static unsigned long reflect(unsigned long data, unsigned char nBits)
{
    unsigned long  reflection = 0x00000000;
    unsigned char  bit;

    // Reflect the data about the center bit.
    for (bit = 0; bit < nBits; ++bit)
    {
        // If the LSB bit is set, set the reflection of it.
        if (data & 0x01)
        {
            reflection |= (1 << ((nBits - 1) - bit));
        }
        data = (data >> 1);
    }
    return (reflection);

}   /* reflect() */



unsigned long crcSlow(unsigned long  crc32, unsigned char const message[], int nBytes)
{
    int            byte;
    unsigned char  bit;

    // Perform modulo-2 division, a byte at a time.
    for (byte = 0; byte < nBytes; ++byte)
    {
        // Bring the next byte into the crc32.
        crc32 ^= (REFLECT_DATA(message[byte]) << (WIDTH - 8));

        // Perform modulo-2 division, a bit at a time.
        for (bit = 8; bit > 0; --bit)
        {
            // Try to divide the current data bit.
            if (crc32 & TOPBIT)
            {
                crc32 = (crc32 << 1) ^ POLYNOMIAL;
            }
            else
            {
                crc32 = (crc32 << 1);
            }
        }
    }
    return crc32;

}   /* crcSlow() */

unsigned long crcComplete(unsigned long crc32)
{
    // The final crc32 is the CRC result.
    return (REFLECT_REMAINDER(crc32) ^ FINAL_XOR_VALUE);
}

// verify function is called after all the data has been received and stored
// in the NV.  The function reads back data from the NV and calculates the checksum.
// Function returns TRUE if calculated CRC matches the one calculated by the host
int ws_upgrade_verify(void)
{
    unsigned int i;
    unsigned long  crc32 = INITIAL_REMAINDER;

    for (i = 0; i < ws_upgrade_total_len; i += WS_UPGRADE_READ_CHUNK)
    {
        uint8_t memory_chunk[WS_UPGRADE_READ_CHUNK];
        int bytesToRead = i + WS_UPGRADE_READ_CHUNK < ws_upgrade_total_len ? WS_UPGRADE_READ_CHUNK : ws_upgrade_total_len - i;

        ws_upgrade_retrieve_from_nv (i, memory_chunk, bytesToRead);

      //  ble_trace0("!! to crc:\n");
      //  ble_tracen((char *)memory_chunk, bytesToRead);
      //  ble_trace0("\n");

        crc32 = crcSlow(crc32, memory_chunk, bytesToRead);

        // because we can be doing NVRAM read in a tight loop for reasonable time
        // make sure that watch dog is ok
        wdog_restart();
    }
    crc32 = crcComplete(crc32);

    ble_trace2 ("ws_verify rcvd:%x calculated:%x\n", ws_upgrade_crc32, crc32);

    return (crc32 == ws_upgrade_crc32);
}

// the only time we send indication instead of notification is when we are done with upgrade, reboot
void ws_upgrade_IndicationConf(void)
{
    ble_trace0("ws_upgrade_IndicationConf\n");
    // sanity check
    if (ws_upgrade_state == WS_UPGRADE_STATE_VERIFIED)
    {
        ws_upgrade_finish();
        ws_upgrade_state = WS_UPGRADE_STATE_IDLE;
    }
}

//
// Check if client has registered for notification and indication and send message if appropriate
//
int ws_upgrade_send_status(int status, int wait_for_ack)
{
    UINT8 data = (UINT8)status;

    // If client has not registered for indication or notification, do not need to do anything
    if (ws_upgrade_client_configuration == 0)
    {
        ble_trace0("send_status failed\n");
        return 0;
    }
  //  ble_trace2("send_status %d wait ack %d\n", status, wait_for_ack);
    if (wait_for_ack)
    {
        bleprofile_sendIndication(HANDLE_WS_UPGRADE_CONTROL_POINT, &data, 1, ws_upgrade_IndicationConf);
    }
    else
    {
        bleprofile_sendNotification(HANDLE_WS_UPGRADE_CONTROL_POINT, &data, 1);
    }
    return 1;
}

// handle commands received over the control point
int ws_upgrade_ota_handle_command(UINT8 *data, int len)
{
    UINT8 command = data[0];

    ble_trace2("Command:%d State:%d\n", command, ws_upgrade_state);
    if (command == WS_UPGRADE_COMMAND_PREPARE_DOWNLOAD)
    {
        ws_upgrade_state = WS_UPGRADE_STATE_READY_FOR_DOWNLOAD;
        ws_upgrade_send_status(WS_UPGRADE_STATUS_OK, 0);
        return (TRUE);
    }
    if (command == WS_UPGRADE_COMMAND_ABORT)
    {
        ws_upgrade_state = WS_UPGRADE_STATE_ABORTED;
        ws_upgrade_send_status(WS_UPGRADE_STATUS_OK, 0);
        return (FALSE);
    }

    switch (ws_upgrade_state)
    {
    case WS_UPGRADE_STATE_IDLE:
        return (TRUE);

    case WS_UPGRADE_STATE_READY_FOR_DOWNLOAD:
        if (command == WS_UPGRADE_COMMAND_DOWNLOAD)
        {
            // command to start upgrade should be accompanied by 2 bytes with the image size
            if (len < 3)
            {
                ble_trace1("Bad Download len:%d\n", len);
                return (FALSE);
            }

            if (!ws_upgrade_init_nv_locations())
            {
                ble_trace0("bad ws_upgrade_init_nv_locations\n");
        ws_upgrade_send_status(WS_UPGRADE_STATUS_INVALID_IMAGE, 0);
        return (FALSE);
            }

            // to make upgrade faster request host to reduce connection interval to minimum
            lel2cap_sendConnParamUpdateReq(6, 6, 0, 700);

            ws_upgrade_current_offset       = 0;
            ws_upgrade_current_block_offset = 0;
            ws_upgrade_total_offset         = 0;
            ws_upgrade_num_unacked_chunks   = 0;
            ws_upgrade_total_len            = data[1] + (data[2] << 8);
            if (len >= 5)
                ws_upgrade_total_len        += ((data[3] << 16) + (data[4] << 24));

            ws_upgrade_state                = WS_UPGRADE_STATE_DATA_TRANSFER;

            ble_trace2("State:%d total_len:%d\n", ws_upgrade_state, ws_upgrade_total_len);

            ws_upgrade_send_status(WS_UPGRADE_STATUS_OK, 0);
            return (TRUE);
        }
        break;

    case WS_UPGRADE_STATE_DATA_TRANSFER:
        if (command == WS_UPGRADE_COMMAND_VERIFY)
        {
            // command to start upgrade should be accompanied by 2 bytes with the image size
            if (len < 5)
            {
                ble_trace1("Bad Verify len:%d\n", len);
                return (FALSE);
            }
            ws_upgrade_crc32 = data[1] + (data[2] << 8) + (data[3] << 16) + (data[4] << 24);
            if (ws_upgrade_verify())
            {
                ble_trace0("State: verified\n");
                ws_upgrade_state = WS_UPGRADE_STATE_VERIFIED;
                ws_upgrade_send_status(WS_UPGRADE_STATUS_OK, 1);
            }
            else
            {
                ble_trace0("Bad verify result, State: aborted\n");
                ws_upgrade_state = WS_UPGRADE_STATE_ABORTED;
                ws_upgrade_send_status(WS_UPGRADE_STATUS_VERIFICATION_FAILED, 0);
            }
            return (TRUE);
        }
        break;

    case WS_UPGRADE_STATE_ABORTED:
    default:
        break;
    }
    // if we fall through command has not been executed
    ws_upgrade_send_status(WS_UPGRADE_STATUS_ILLEGAL_STATE, 0);
    return (TRUE);
}

// process the next data chunk
int ws_upgrade_ota_handle_configuration (UINT8 *data, int len)
{
    if (len != 2)
    {
        ble_trace0("Bad config length\n");
        return FALSE;
    }
    ws_upgrade_client_configuration = data[0] + (data[1] << 8);
    return (TRUE);
}

int ws_upgrade_ota_handle_data (UINT8 *data, int len)
{
   // ble_trace3("ws_upgrade_ota_handle_data len %d offset %d out of %d\n", len, ws_upgrade_total_offset, ws_upgrade_total_len);
    if ((ws_upgrade_total_offset + ws_upgrade_current_block_offset + len > ws_upgrade_total_len) ||
        (ws_upgrade_current_block_offset + len > (WS_UPGRADE_CHUNK_SIZE_TO_COMMIT + WS_UPGRADE_LOCAL_MTU)))
    {
        ble_trace4("Too much data offsets:%d + %d total:%d len:%d\n",
                ws_upgrade_total_offset, ws_upgrade_current_block_offset, ws_upgrade_total_len, len);
        return (FALSE);
    }
    // send Status Continue to enable client send more chunks
    if (++ws_upgrade_num_unacked_chunks == WS_UPGRADE_NUM_UNACKED_CHUNKS)
    {
        ws_upgrade_num_unacked_chunks = 0;
        ws_upgrade_send_status(WS_UPGRADE_STATUS_CONTINUE, FALSE);
    }

    BT_MEMCPY (&ws_upgrade_read_buffer[ws_upgrade_current_block_offset], data, len);
    ws_upgrade_current_block_offset += len;

    while (ws_upgrade_current_block_offset >= WS_UPGRADE_CHUNK_SIZE_TO_COMMIT)
    {
        ws_upgrade_store_to_nv(ws_upgrade_total_offset, ws_upgrade_read_buffer, WS_UPGRADE_CHUNK_SIZE_TO_COMMIT);
        ws_upgrade_total_offset        += WS_UPGRADE_CHUNK_SIZE_TO_COMMIT;
        ws_upgrade_current_block_offset -= WS_UPGRADE_CHUNK_SIZE_TO_COMMIT;
        __aeabi_memmove(ws_upgrade_read_buffer, &ws_upgrade_read_buffer[WS_UPGRADE_CHUNK_SIZE_TO_COMMIT], ws_upgrade_current_block_offset);
    }
    // Handling end of the file case
    if ((ws_upgrade_current_block_offset != 0) && ((ws_upgrade_total_offset + ws_upgrade_current_block_offset) == ws_upgrade_total_len))
    {
        ws_upgrade_store_to_nv(ws_upgrade_total_offset, ws_upgrade_read_buffer, ws_upgrade_current_block_offset);
        ws_upgrade_total_offset        += ws_upgrade_current_block_offset;
        ws_upgrade_current_block_offset = 0;
    }
    return (TRUE);
}
