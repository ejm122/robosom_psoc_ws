/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * Created 3/26/2021 by ebcohen@andrew.cmu.edu
 * 
 * Eigen_protocol developed by npaiva@andrew.cmu.edu.
 * Code modified from eigenbus_protcol API, adapted for lightweight use on Blaser.
 * ========================================
*/
#include <stdlib.h>
#include <ctype.h>
#include <stdio.h>
#include <math.h>
#include "blaser_messages.h"
#include "error_codes.h"

/** @brief Size of process buffer bytes for reading from data stream */
#define PROCESS_BUFFER_SIZE (256)
/** @brief Size of process buffer bytes for sending from data input */
#define SEND_BUFFER_SIZE (256)

/** @brief Checksum enable define */
#define CHKSUM_ACTIVE (1)

/** @brief Enum to define header indices */
enum header_indices {
    MSG_START_IDX = 0,
    MSG_ID_IDX = 1,
    MSG_LEN_IDX = 2,
    MSG_CRC_IDX = 3,
    MSG_HEADER_SIZE = 4
};

enum packet_end_indicies {
    MSG_DATA_CRC = 0,
    MSG_DATA_ENDCHAR = 1,
    MSG_DATA_NULLTERM = 2,
    MSG_DATA_END_SIZE = 3
};

/** @brief Encode-stages to form state-machine logic */
enum encode_stages {
    MSG_NOT_STARTED = 0,
    READ_MSG_START = 1,
    READ_MSG_ID = 2,
    READ_MSG_LEN = 3,
    READ_HEADER = 4,
    FINISHED_CMD = 5
};

/* Generic Communication struct - Modified from eigenbus protocol */
typedef struct comm_struct {
    //Communication handlers
    uint8_t (*read_char)();
    uint16_t (*num_available)();
    void (*put_data)(uint8_t *buffer, uint16_t count);
    
    //Process buffer
    uint8_t process_buffer[PROCESS_BUFFER_SIZE];
    uint8_t process_header[MSG_HEADER_SIZE];
    uint16_t ptr;
    uint8_t valid_header;
    uint8_t valid_payload;
    uint8_t pkt_stage;
} blaser_comms_t;
static blaser_comms_t comms;

/* Table for CRC-8-CCITT from https://www.3dbrew.org/wiki/CRC-8-CCITT */
static const uint8_t CRC_8_TABLE[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
    0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
    0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
    0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

static void process_data_byte(uint8_t inChar);

static uint8_t crc_8_ccitt(uint8_t *data, uint16_t len)
{
    uint8_t crc = 0xFF; //Seed of 0xFF
    
    for(uint16_t ind = 0; ind < len; ind++){
        uint8_t temp = crc ^ data[ind];
        crc = CRC_8_TABLE[temp];
    }
    
    return crc; //Final of 0x00
}

void blaser_comms_deregister(void) 
{
    comms.read_char = NULL;
    comms.num_available = NULL;
    comms.put_data = NULL;
    comms.ptr = 0;
    comms.valid_header = 0;
    comms.valid_payload = 0;
    comms.pkt_stage = MSG_NOT_STARTED;
}

void blaser_comms_register(uint8_t (*read_char)(), uint16_t (*num_available)(), 
                           void (*put_data)(uint8_t *buffer, uint16_t count))
{
    comms.read_char = read_char;
    comms.num_available = num_available;
    comms.put_data = put_data;
    comms.ptr = 0;
    comms.valid_header = 0;
    comms.valid_payload = 0;
    comms.pkt_stage = MSG_NOT_STARTED;
}

/** @brief Handles decoding a serial stream, and verifies the CRCs on the msg.
    Packet structure: 
    [ M | MSG_ID | PAYLOAD_LEN | HEAD_CRC | PAYLOAD ... | PAYLOAD_CRC | \n ]
    TODO? - COBS encoding would be ideal to properly detect 'end' of packet.
    Current encoding can risk mis-identifying the start of a packet rarely.
    Considered allowable, as 
    1) malformed packets will be detected due to CRC in header
    2) lack of binary-encoding allows improved packet decode/encode speeds.
 */
int8_t read_comms_cmd(void) 
{
    uint8_t inChar;
    int8_t status = NO_ERR;
    
    if (comms.num_available == NULL || comms.read_char == NULL) return ERR_NULL_PTR;
    
    while ((*(comms.num_available))() > 0) 
    {
        inChar = (uint8_t)(*(comms.read_char))();
        
        switch (comms.pkt_stage)
        {
            case MSG_NOT_STARTED:
                if (inChar == ((uint8_t)'m'))
                {
                    comms.pkt_stage = READ_MSG_START;
                    comms.ptr = 0;
                    comms.valid_header = 1;
                    comms.valid_payload = 1;
                    comms.process_header[MSG_START_IDX] = inChar;
                }
                break;
            case READ_MSG_START:
                comms.pkt_stage = READ_MSG_ID;
                comms.process_header[MSG_ID_IDX] = inChar;
                break;
            case READ_MSG_ID:
                comms.pkt_stage = READ_MSG_LEN;
                comms.process_header[MSG_LEN_IDX] = inChar;
                comms.valid_header = (inChar == get_len_MSG_ID(comms.process_header[MSG_ID_IDX]));
                break;
            case READ_MSG_LEN:
                comms.pkt_stage = READ_HEADER;
                comms.process_header[MSG_CRC_IDX] = 0; 
                if (CHKSUM_ACTIVE) 
                {
                    // Designate the end of data used to generate the CRC
                    uint8_t checksum_calc = crc_8_ccitt(comms.process_header, MSG_CRC_IDX);
                    if (inChar != checksum_calc) comms.valid_header = 0;
                }
                // If CRC isn't active, skip over this byte to keep header size constant.
                break;
            case READ_HEADER:
                process_data_byte(inChar);
                break;
        }
        
        if (comms.valid_header == 0 || comms.valid_payload == 0) 
        {
            comms.pkt_stage = MSG_NOT_STARTED;
            status = (comms.valid_header) ? ERR_PARSE_HEADER : ERR_PARSE_PAYLOAD;
        }
        
        if (comms.pkt_stage == FINISHED_CMD) {
            status = read_msg(comms.process_header[MSG_ID_IDX],
                              comms.process_buffer);
            
            comms.pkt_stage = MSG_NOT_STARTED;
        }
    }
    // Returns last packet's status- 
    // Might want better solution for reporting multiple packets at a time.
    return status;
}

static void process_data_byte(uint8_t inChar)
{

    /* Payloads of size 0 don't require CRCs.
       If CRCs are enabled, max payload size = msg_len + 2, else msg_len + 1
       Check for the end designator of the message. */
    if (comms.process_header[MSG_LEN_IDX] == 0 ||
        (comms.ptr == comms.process_header[MSG_LEN_IDX] && !CHKSUM_ACTIVE) ||
        (comms.ptr == (comms.process_header[MSG_LEN_IDX] + 1) && CHKSUM_ACTIVE)) 
    {
        if (inChar != ((uint8_t)'\n'))
        {
            comms.valid_payload = 0;
            return;
        }
        comms.pkt_stage = FINISHED_CMD;
        // Make the char null for adding into the data buffer
        inChar = 0;
    }
    else if (comms.ptr == comms.process_header[MSG_LEN_IDX] && CHKSUM_ACTIVE)
    {
        // Designate the end of data used to generate the CRC
        uint8_t checksum_calc = crc_8_ccitt(comms.process_buffer, comms.ptr);
        if (inChar != checksum_calc) 
        {
            comms.valid_payload = 0;
            return;
        }
        // Make the char null for adding into the data buffer
        inChar = 0;
    }
    // Add char to processing buffer if it's valid
    if (comms.ptr < PROCESS_BUFFER_SIZE)
    {
        comms.process_buffer[comms.ptr] = inChar;
        if (comms.pkt_stage != FINISHED_CMD) 
        {
            comms.ptr++;
        }
    }
    else
    {
        comms.valid_payload = 0; // Ran out of room
    }
}
/** @brief Handles encoding a serial stream, and generates the CRCs on the msg.
    Packet structure: 
    [ M | MSG_ID | PAYLOAD_LEN | HEAD_CRC | PAYLOAD ... | PAYLOAD_CRC | \n ]
 */
int8_t send_comms_cmd(uint8_t cmd_ID) 
{
    // Assert that SEND_BUFFER_SIZE > MSG_HEADER_SIZE + PACKET_END_SIZE in compile-time.
    (BUILD_BUG_ON((SEND_BUFFER_SIZE <= (MSG_HEADER_SIZE + MSG_DATA_END_SIZE))));
    
    uint8_t send_buf[SEND_BUFFER_SIZE];
    int8_t status = NO_ERR;
    uint8_t crc;
    uint16_t payload_bytes = get_len_MSG_ID(cmd_ID);
    
    /* Configure the header */
    send_buf[MSG_START_IDX] = ((uint8_t)'m');
    send_buf[MSG_ID_IDX] = cmd_ID;
    send_buf[MSG_LEN_IDX] = payload_bytes;
    send_buf[MSG_CRC_IDX] = crc_8_ccitt(send_buf, MSG_CRC_IDX);
    
    uint16_t max_payload_bytes = SEND_BUFFER_SIZE - (MSG_HEADER_SIZE + MSG_DATA_END_SIZE);
    // Indexing MSG_HEADER_SIZE grabs first char after header ends
    status = send_msg(cmd_ID, &(send_buf[MSG_HEADER_SIZE]), max_payload_bytes);
    if (status == NO_ERR)
    {
        uint16_t ptr = MSG_HEADER_SIZE + payload_bytes;
        if (CHKSUM_ACTIVE && payload_bytes > 0)
        {
            // Indexing MSG_HEADER_SIZE grabs first char after header ends
            crc = crc_8_ccitt((uint8_t *)(&(send_buf[MSG_HEADER_SIZE])), payload_bytes);
            send_buf[ptr] = crc;
            ptr++;
        }
        send_buf[ptr] = ((uint8_t)'\n');
        ptr++;
        (*(comms.put_data))(send_buf, ptr);
        return NO_ERR;
    }
    return ERR_BUFF_OVERFLOW;
}

/* [] END OF FILE */
