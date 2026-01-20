/**
 * @file dm8x06_driver.c
 * @brief DM8x06 6-port Fast Ethernet switch driver
 *
 * @section License
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Copyright (C) 2010-2023 Oryx Embedded SARL. All rights reserved.
 *
 * This file is part of CycloneTCP Open.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * @author Oryx Embedded SARL (www.oryx-embedded.com)
 * @version 2.3.0
 **/

//Switch to the appropriate trace level
//#define TRACE_LEVEL NIC_TRACE_LEVEL

#define TRACE_LEVEL                            TRACE_LEVEL_INFO

#define TX_PACK_DUMP_ENABLE                    0
#define RX_PACK_DUMP_ENABLE                    0

//Dependencies
#include "core/net.h"
#include "core/ethernet_misc.h"
#include "drivers/switch/dm8x06_driver.h"
#include "debug.h"

/**
 * @brief DM8x06 Ethernet switch driver
 **/
const SwitchDriver dm8x06SwitchDriver = {
    dm8x06Init,                                //
    dm8x06Tick,                                //
        dm8x06EnableIrq,
        dm8x06DisableIrq,
    dm8x06EventHandler,                        //

    dm8x06TagFrame,                            //
    dm8x06UntagFrame,                          //

    dm8x06GetLinkState,                        //
    dm8x06GetLinkSpeed,                        //
    dm8x06GetDuplexMode,                       //

    dm8x06SetPortState,                        //
    dm8x06GetPortState,                        //
    dm8x06SetAgingTime,                        //

    dm8x06EnableIgmpSnooping,                  //
    dm8x06EnableMldSnooping,                   //

    dm8x06EnableRsvdMcastTable,                //

    dm8x06AddStaticFdbEntry,                   //
    dm8x06DeleteStaticFdbEntry,                //
    dm8x06GetStaticFdbEntry,                   //
    dm8x06FlushStaticFdbTable,                 //

    dm8x06GetDynamicFdbEntry,                  //
    dm8x06FlushDynamicFdbTable,                //

    dm8x06SetUnknownMcastFwdPorts              //
};


/**
 * @brief DM8x06 Ethernet switch initialization
 * @param[in] interface Underlying network interface
 * @return Error code
 **/
error_t dm8x06Init(NetInterface *interface) {
    uint_t port;
    uint16_t vid, pid;
    uint16_t reg_value;

    //Debug message
    TRACE_INFO("Initializing DM8x06...\r\n");
#if DEVICE_IS_DM8806
    TRACE_INFO("device is DM8806...\r\n");
#endif

    //Undefined PHY address?
    if(interface->phyAddr >= 32) {
        //Use the default address
        interface->phyAddr = DM8x06_PHY_ADDR;
    }

    //Initialize serial management interface
    if(interface->smiDriver != NULL) {
        interface->smiDriver->init();
    }

    //Wait for the serial interface to be ready
    do {
        //Read switch identifier register
        vid = dm8x06ReadSwitchReg(interface, DM8x06_VENDOR_ID);
        pid = dm8x06ReadSwitchReg(interface, DM8x06_PRODUCT_ID);
        TRACE_DEBUG("device id check 0x%04X - 0x%04X \r\n", vid, pid);
    }while(DM8x06_VENDOR_ID_DEFAULT != vid);

#if (ETH_PORT_TAGGING_SUPPORT == ENABLED)
    TRACE_INFO("ETH_PORT_TAGGING_SUPPORT == ENABLED ....\r\n");
    //set special tag type
    dm8x06WriteSwitchReg(interface, DM8x06_SPECIAL_TAG_ETHER_TYPE, DM8x06_SPECIAL_TAG_ETHER_TYPE_DEFAULT);
    //Enable special tag mode
    dm8x06WriteSwitchReg(
            interface,
            DM8x06_CPU_PORT_AND_MIRROR_CONTROL,
            DM8x06_CPU_PORT_SET(DM8x06_CPU_PORT) | DM8x06_SPECIAL_TAG_TX_ENABLE | DM8x06_SPECIAL_TAG_RX_ENABLE);

    TRACE_DEBUG("DM8x06_SPECIAL_TAG_ETHER_TYPE 0x%04X \r\n", dm8x06ReadSwitchReg(interface, DM8x06_SPECIAL_TAG_ETHER_TYPE));
    TRACE_DEBUG("DM8x06_CPU_PORT_AND_MIRROR_CONTROL 0x%04X \r\n", dm8x06ReadSwitchReg(interface, DM8x06_CPU_PORT_AND_MIRROR_CONTROL));

    TRACE_INFO("== STP / RSTP FUNCTION set enable ==\r\n");
    dm8x06WriteSwitchReg(interface, DM8x06_STP_CONTROL, DM8x06_STP_EN);
#else
    //Disable special tag mode
    dm8x06WriteSwitchReg(interface, DM8x06_CPU_PORT_AND_MIRROR_CONTROL, DM8x06_CPU_PORT_SET(DM8x06_CPU_PORT));

    TRACE_INFO("== STP / RSTP FUNCTION set disable ==\r\n");
    dm8x06WriteSwitchReg(interface, DM8x06_STP_CONTROL, DM8x06_STP_CONTROL_DEFAULT);
#endif
		
    //Loop through the ports
    for (port = DM8x06_PORT0; port <= DM8x06_CPU_PORT; port++) {
        //enable aging function
        reg_value = dm8x06ReadSwitchReg(interface, DM8x06_PORTn_BASIC_CONTROL_0(port));
        reg_value &= ~(DM8x06_PORTn_AGING_DISABLE);
        dm8x06WriteSwitchReg(interface, DM8x06_PORTn_BASIC_CONTROL_0(port), reg_value);

#if (ETH_PORT_TAGGING_SUPPORT == ENABLED)
        //Port separation mode?
        if(interface->port != 0) {
            //Disable packet transmission and address learning
            dm8x06SetPortState(interface, port, SWITCH_PORT_STATE_LISTENING);
        } else
#endif
        {
            //Enable transmission, reception and address learning
            dm8x06SetPortState(interface, port, SWITCH_PORT_STATE_FORWARDING);
        }
    }

    //Flush static MAC table
    dm8x06FlushStaticFdbTable(interface);

    //Restore default age count
    dm8x06SetAgingTime(interface, DM8x06_AGING_CONTROL_512SEC);

    //Loop through the ports
    for (port = DM8x06_PORT0; port < DM8x06_CPU_PORT; port++) {
        //Debug message
        TRACE_DEBUG("Port %u:\r\n", DM8x06_PORTn_PRT_NUM(port));
        //Dump PHY registers for debugging purpose
        dm8x06DumpPhyReg(interface, port);
    }

    //Perform custom configuration
    dm8x06InitHook(interface);

    //Force the TCP/IP stack to poll the link state at startup
    interface->phyEvent = TRUE;

    //Notify the TCP/IP stack of the event
    osSetEvent(&netEvent);

    //Successful initialization
    return NO_ERROR;
}


/**
 * @brief DM8x06 custom configuration
 * @param[in] interface Underlying network interface
 **/
__weak_func void dm8x06InitHook(NetInterface *interface)
{
    uint16_t monitor_reg_31c = dm8x06ReadSwitchReg(interface, DM8x06_MONITOR_REGISTER_1);
    uint16_t monitor_reg_31d = dm8x06ReadSwitchReg(interface, DM8x06_MONITOR_REGISTER_2);
    uint16_t monitor_reg_31e = dm8x06ReadSwitchReg(interface, DM8x06_MONITOR_REGISTER_3);

    //msgout monitor_reg
    TRACE_DEBUG("monitor_reg_1 value 0x%04X \r\n", monitor_reg_31c);
    TRACE_DEBUG("monitor_reg_2 value 0x%04X \r\n", monitor_reg_31d);
    TRACE_DEBUG("monitor_reg_3 value 0x%04X \r\n", monitor_reg_31e);

    TRACE_INFO("==========================================\r\n");

    switch(monitor_reg_31e & ((1 << 11) | (1 << 10) | (1 << 9))) {
        case ((0 << 11) | (0 << 10) | (0 << 9)) : TRACE_INFO("DM8x06 p4 TP/FX mode \r\n"); break;

        case ((1 << 11) | (0 << 10) | (0 << 9)) :
        {

            switch((monitor_reg_31d & (1 << 15)) | (monitor_reg_31c & (1 << 8))) {
                case ((0 << 15) | (0 << 8)) : TRACE_INFO("DM8x06 p4 RevMII Trubo mode \r\n"); break;
                case ((0 << 15) | (1 << 8)) : TRACE_INFO("DM8x06 p4 RMII mode \r\n");break;
                case ((1 << 15) | (0 << 8)) : TRACE_INFO("DM8x06 p4 MII mode \r\n");break;
                case ((1 << 15) | (1 << 8)) : TRACE_INFO("DM8x06 p4 RevMII mode \r\n");break;
            }

            if(monitor_reg_31c & (1 << 1)) {
                TRACE_INFO("DM8x06 p4 link polling mode \r\n");
            } else {
                TRACE_INFO("DM8x06 p4 link force mode \r\n");

                if(monitor_reg_31d & (1 << 14)) {
                    switch(monitor_reg_31c & ((1 << 7) | (1 << 6))) {
                        case ((0 << 7) | (0 << 6)) : TRACE_INFO("DM8x06 p4 10M Half \r\n"); break;
                        case ((0 << 7) | (1 << 6)) : TRACE_INFO("DM8x06 p4 10M Full \r\n");break;
                        case ((1 << 7) | (0 << 6)) : TRACE_INFO("DM8x06 p4 100M Half \r\n");break;
                        case ((1 << 7) | (1 << 6)) : TRACE_INFO("DM8x06 p4 100M Full \r\n");break;
                    }
                } else {
                    TRACE_INFO("DM8x06 p4 link down mode \r\n");
                }
            }
        }
        break;

        default: TRACE_INFO("DM8x06 p4 mode error\r\n"); break;
    }

    TRACE_INFO("DM8x06 p4 status value 0x%04X \r\n", dm8x06ReadSwitchReg(interface, DM8x06_PORT_4_PORT_STATUS));
    TRACE_INFO("DM8x06 p4 mac set value 0x%04X \r\n", dm8x06ReadSwitchReg(interface, DM8x06_PORT_4_MAC_CONTROL));

    TRACE_INFO("==========================================\r\n");

    switch(monitor_reg_31d & ((1 << 11) | (1 << 10))) {
        case ((0 << 11) | (0 << 10)) : TRACE_INFO("DM8x06 p5 mode error \r\n"); break;
        case ((0 << 11) | (1 << 10)) : TRACE_INFO("DM8x06 p5 RMII mode \r\n");break;
        case ((1 << 11) | (0 << 10)) : TRACE_INFO("DM8x06 p5 MII mode \r\n");break;
        case ((1 << 11) | (1 << 10)) : TRACE_INFO("DM8x06 p5 RevMII mode \r\n");break;
    }

    if(monitor_reg_31c & (1 << 2)) {
        TRACE_INFO("DM8x06 p5 link polling mode \r\n");
    } else {
        TRACE_INFO("DM8x06 p5 link force mode \r\n");

        if(monitor_reg_31d & (1 << 12)) {
            switch(monitor_reg_31d & ((1 << 9) | (1 << 8))) {
                case ((0 << 9) | (0 << 8)) : TRACE_INFO("DM8x06 p5 10M Half \r\n");break;
                case ((0 << 9) | (1 << 8)) : TRACE_INFO("DM8x06 p5 10M Full \r\n");break;
                case ((1 << 9) | (0 << 8)) : TRACE_INFO("DM8x06 p5 100M Half \r\n");break;
                case ((1 << 9) | (1 << 8)) : TRACE_INFO("DM8x06 p5 100M Full \r\n");break;
            }
        } else {
            TRACE_INFO("DM8x06 p5 link down mode \r\n");
        }
    }

    TRACE_INFO("DM8x06 p5 status value 0x%04X \r\n", dm8x06ReadSwitchReg(interface, DM8x06_PORT_5_PORT_STATUS));
    TRACE_INFO("DM8x06 p5 mac set value 0x%04X \r\n", dm8x06ReadSwitchReg(interface, DM8x06_PORT_5_MAC_CONTROL));

    TRACE_INFO("==========================================\r\n");

#if 0
    //enable virtual phy enable
    if(monitor_reg_31c & DM8x06_MONITOR_REGISTER_1_VIRTUAL_PHY_EN) {
        TRACE_INFO("== dm8x06 virtual phy enable ==\r\n");
        dm8x06WriteSwitchReg(
                interface,
                DM8x06_VIRTUAL_PHY_CONTROL,
                DM8x06_VIRTUAL_PHY_CONTROL_OR_MODE | DM8x06_VIRTUAL_PHY_CONTROL_MAP);
    }

    //Disable p5
    if(DM8x06_CPU_PORT < DM8x06_PORT5) {
        TRACE_INFO("== dm8x06 p5 disable ==\r\n");
        dm8x06WriteSwitchReg(interface, DM8x06_PORT_5_MAC_CONTROL, DM8x06_PORT_5_MAC_CONTROL_DISABLE);
    }
#endif
}



/**
 * @brief DM8x06 timer handler
 * @param[in] interface Underlying network interface
 **/
__weak_func void dm8x06Tick(NetInterface *interface) {
    uint_t port;
    bool_t linkState;

#if (ETH_PORT_TAGGING_SUPPORT == ENABLED)
     //Port separation mode?
    if(interface->port != 0) {
        uint_t i;
        NetInterface *virtualInterface;
        //Loop through network interfaces
        for (i = 0; i < NET_INTERFACE_COUNT; i++) {
            //Point to the current interface
            virtualInterface = &netInterface[i];
            //Check whether the current virtual interface is attached to the
            //physical interface
            if(virtualInterface == interface || virtualInterface->parent == interface) {
                //Retrieve current link state
                linkState = dm8x06GetLinkState(interface, virtualInterface->port);
                //Link up or link down event?
                if(linkState != virtualInterface->linkState) {
                    //Set event flag
                    interface->phyEvent = TRUE;
                    //Notify the TCP/IP stack of the event
                    osSetEvent(&netEvent);
                }
            }
        }
    } else
#endif
    {
        //Initialize link state
        linkState = FALSE;

        //Loop through the ports
        for (port = DM8x06_PORT0; port < DM8x06_CPU_PORT; port++) {
            //Retrieve current link state
            if(dm8x06GetLinkState(interface, port)) {
                linkState = TRUE;
            }
        }

        //Link up or link down event?
        if(linkState != interface->linkState) {
            //Set event flag
            interface->phyEvent = TRUE;
            //Notify the TCP/IP stack of the event
            osSetEvent(&netEvent);
        }
    }
}


/**
 * @brief Enable interrupts
 * @param[in] interface Underlying network interface
 **/
void dm8x06EnableIrq(NetInterface *interface) {
}


/**
 * @brief Disable interrupts
 * @param[in] interface Underlying network interface
 **/
void dm8x06DisableIrq(NetInterface *interface) {
}


/**
 * @brief DM8x06 event handler
 * @param[in] interface Underlying network interface
 **/
__weak_func void dm8x06EventHandler(NetInterface *interface) {
    uint_t port;
    bool_t linkState;

#if (ETH_PORT_TAGGING_SUPPORT == ENABLED)
   //Port separation mode?
    if(interface->port != 0)
    {
        uint_t i;
        NetInterface *virtualInterface;
        //Loop through network interfaces
        for (i = 0; i < NET_INTERFACE_COUNT; i++) {
            //Point to the current interface
            virtualInterface = &netInterface[i];
            //Check whether the current virtual interface is attached to the
            //physical interface
            if(virtualInterface == interface || virtualInterface->parent == interface) {
                //Get the port number associated with the current interface
                port = virtualInterface->port;
                //Valid port?
                if(port >= DM8x06_PORT0 && port < DM8x06_CPU_PORT) {
                    //Retrieve current link state
                    linkState = dm8x06GetLinkState(interface, port);
                    //Link up event?
                    if(linkState && !virtualInterface->linkState) {
                        //Retrieve host interface speed
                        interface->linkSpeed = dm8x06GetLinkSpeed(interface, DM8x06_CPU_PORT);
                        //Retrieve host interface duplex mode
                        interface->duplexMode = dm8x06GetDuplexMode(interface, DM8x06_CPU_PORT);
                        //Adjust MAC configuration parameters for proper operation
                        interface->nicDriver->updateMacConfig(interface);
                        //Check current speed
                        virtualInterface->linkSpeed = dm8x06GetLinkSpeed(interface, port);
                        //Check current duplex mode
                        virtualInterface->duplexMode = dm8x06GetDuplexMode(interface, port);
                        //Update link state
                        virtualInterface->linkState = TRUE;
                        //Process link state change event
                        nicNotifyLinkChange(virtualInterface);
                    }
                    else if(!linkState && virtualInterface->linkState) { //Link down event
                        //Update link state
                        virtualInterface->linkState = FALSE;
                        //Process link state change event
                        nicNotifyLinkChange(virtualInterface);
                    }
                }								
                else if (port == DM8x06_CPU_PORT) {
                        //Retrieve host interface speed
                        interface->linkSpeed = dm8x06GetLinkSpeed(interface, DM8x06_CPU_PORT);
                        //Retrieve host interface duplex mode
                        interface->duplexMode = dm8x06GetDuplexMode(interface, DM8x06_CPU_PORT);
                        //Adjust MAC configuration parameters for proper operation
                        interface->nicDriver->updateMacConfig(interface);
                        //Check current speed
                        virtualInterface->linkSpeed = interface->linkSpeed;
                        //Check current duplex mode
                        virtualInterface->duplexMode = interface->duplexMode;
                        //Update link state
                        virtualInterface->linkState = TRUE;
                        //Process link state change event
                        nicNotifyLinkChange(virtualInterface);
                }
            }
        }
    } else
#endif
    {
        //Initialize link state
        linkState = FALSE;

        //Loop through the ports
        for (port = DM8x06_PORT0; port < DM8x06_CPU_PORT; port++) {
            //Retrieve current link state
            if(dm8x06GetLinkState(interface, port)) {
                linkState = TRUE;
            }
        }

        //Link up event?
        if(linkState) {
            //Retrieve host interface speed
            interface->linkSpeed = dm8x06GetLinkSpeed(interface, DM8x06_CPU_PORT);
            //Retrieve host interface duplex mode
            interface->duplexMode = dm8x06GetDuplexMode(interface, DM8x06_CPU_PORT);
            //Adjust MAC configuration parameters for proper operation
            interface->nicDriver->updateMacConfig(interface);
            //Update link state
            interface->linkState = TRUE;
        } else {
            //Update link stateS
            interface->linkState = FALSE;
        }
        //Process link state change event
        nicNotifyLinkChange(interface);
    }
}

void netBuffer_dump_prt(uint8_t * prt_head, const NetBuffer *src, size_t srcOffset)
{
   uint_t i;
   uint_t n;
   size_t totalLength;
   uint8_t *p;
   size_t length = netBufferGetLength(src) - srcOffset;

   uint16_t prt_point = 0;
   uint16_t prt_cnt = 0;

   //Total number of bytes copied
   totalLength = 0;

   TRACE_DEBUG(" ===== %s =====", prt_head);

   //Loop through data chunks
   for(i = 0; i < src->chunkCount && totalLength < length; i++)
   {
      //Is there any data to copy from the current chunk?
      if(srcOffset < src->chunk[i].length)
      {
         //Point to the first byte to be read
         p = (uint8_t *) src->chunk[i].address + srcOffset;
         //Compute the number of bytes to copy at a time
         n = MIN(length - totalLength, src->chunk[i].length - srcOffset);

         for(prt_cnt = 0; prt_cnt < n; prt_cnt++) {
             if(!(prt_point % 16)) TRACE_DEBUG("\r\n 0x%04X :", prt_point);
             TRACE_DEBUG(" 0x%02X", p[prt_cnt]);
             prt_point++;
         }

         //Total number of bytes copied
         totalLength += n;
         //Process the next block from the start
         srcOffset = 0;
      }
      else
      {
         //Skip the current chunk
         srcOffset -= src->chunk[i].length;
      }
   }
    TRACE_DEBUG("\r\n\n");
}

void pack_dump_prt(uint8_t * prt_head, uint8_t * out_data, size_t out_len)
{
    uint16_t prt_point;

    TRACE_DEBUG(" ===== %s =====", prt_head);

    for(prt_point = 0; prt_point < out_len; prt_point++) {
        if(!(prt_point % 16)) TRACE_DEBUG("\r\n 0x%04X :", prt_point);
        TRACE_DEBUG(" 0x%02X", out_data[prt_point]);
    }

    TRACE_DEBUG("\r\n\n");
}


/**
 * @brief Add tail tag to Ethernet frame
 * @param[in] interface Underlying network interface
 * @param[in] buffer Multi-part buffer containing the payload
 * @param[in,out] offset Offset to the first payload byte
 * @param[in] ancillary Additional options passed to the stack along with
 *   the packet
 * @return Error code
 **/
typedef __packed_struct {
    uint16_t sp_tag;
    uint8_t pmap;
    uint8_t st_set;
} SpecialTag_Tx;

error_t dm8x06TagFrame(
        NetInterface *interface,
        NetBuffer *buffer,
        size_t *offset,
        NetTxAncillary *ancillary) {
    //Initialize status code
    error_t error;

#if (ETH_PORT_TAGGING_SUPPORT == ENABLED)
    EthHeader *header = netBufferAt(buffer, 0);

    error = NO_ERROR;
    //Valid port?
    TRACE_DEBUG("tag out port %u len %u mac 0x%02X : 0x%02X : 0x%02X : 0x%02X : 0x%02X : 0x%02X\r\n",
            ancillary->port,
            netBufferGetLength(buffer),
            header->srcAddr.b[0],
            header->srcAddr.b[1],
            header->srcAddr.b[2],
            header->srcAddr.b[3],
            header->srcAddr.b[4],
            header->srcAddr.b[5]);

        //Is there enough space for the special tag?
    if(ancillary->port < DM8x06_CPU_PORT) {
        if((*offset >= sizeof(SpecialTag_Tx)) && (0 != ancillary->port)) {
            SpecialTag_Tx *SPTag_tx;

            //Make room for the special tag
            *offset -= sizeof(SpecialTag_Tx);

            //Point to the beginning of the frame
            header = netBufferAt(buffer, *offset);

            //Move the Ethernet header to make room for the special tag
            osMemmove(header, (uint8_t *)header + sizeof(SpecialTag_Tx), sizeof(header->destAddr) + sizeof(header->srcAddr));

            //used as the destination port indicator
            SPTag_tx = (SpecialTag_Tx *) &header->type;
            //add special tag data
            SPTag_tx->sp_tag = ntohs(DM8x06_SPECIAL_TAG_ETHER_TYPE_DEFAULT);
            SPTag_tx->pmap = DM8x06_SPECIAL_TAG_MAP_EN | (DM8x06_SPECIAL_TAG_MAP_TX_PORT(ancillary->port));
            SPTag_tx->st_set = DM8x06_SPECIAL_TAG_UNTAG;
        }

#if TX_PACK_DUMP_ENABLE
		netBuffer_dump_prt((uint8_t *) "tx_out", buffer, *offset);
#endif
    } else {
        //The port number is not valid
        error = ERROR_INVALID_PORT;
    }
#endif

    //Return status code
    return error;
}


/**
 * @brief Decode tail tag from incoming Ethernet frame
 * @param[in] interface Underlying network interface
 * @param[in,out] frame Pointer to the received Ethernet frame
 * @param[in,out] length Length of the frame, in bytes
 * @param[in,out] ancillary Additional options passed to the stack along with
 *   the packet
 * @return Error code
 **/
typedef __packed_struct {
    uint16_t sp_tag;
    uint8_t port;
    uint8_t no_user;
} SpecialTag_Rx;

error_t dm8x06UntagFrame(
        NetInterface *interface,
        uint8_t **frame,
        size_t *length,
        NetRxAncillary *ancillary) {
    //Initialize status code
    error_t error = NO_ERROR;
    EthHeader *header;
#if (ETH_PORT_TAGGING_SUPPORT == ENABLED)
    SpecialTag_Rx *SPTag_rx;
#endif

    //Point to the beginning of the frame
    header = (EthHeader *) *frame;

    TRACE_DEBUG("pack in type 0x%04X len %u mac 0x%02X : 0x%02X : 0x%02X : 0x%02X : 0x%02X : 0x%02X \r\n",
            ntohs(header->type),
            *length,
            header->srcAddr.b[0],
            header->srcAddr.b[1],
            header->srcAddr.b[2],
            header->srcAddr.b[3],
            header->srcAddr.b[4],
            header->srcAddr.b[5]);

#if (ETH_PORT_TAGGING_SUPPORT == ENABLED)
    //Check whether type is used
    if(ntohs(header->type) == DM8x06_SPECIAL_TAG_ETHER_TYPE_DEFAULT) {
        //Valid Ethernet frame received?
        if(*length >= (sizeof(EthHeader) + sizeof(SpecialTag_Rx))) {
            SPTag_rx = (SpecialTag_Rx *) &header->type;
            ancillary->port = DM8x06_SPECIAL_TAG_MAP_RX_PORT(SPTag_rx->port);

            //Strip the special tag from the Ethernet frame
            osMemmove(*frame + sizeof(SpecialTag_Rx), *frame, sizeof(header->destAddr) + sizeof(header->srcAddr));

            //Point to the Ethernet frame header
            *frame += sizeof(SpecialTag_Rx);
            //Retrieve the length of the original frame
            *length -= sizeof(SpecialTag_Rx);
        } else {
            //Drop the received frame
            error = ERROR_INVALID_LENGTH;
        }
    }
#endif

#if RX_PACK_DUMP_ENABLE
    pack_dump_prt((uint8_t *) "rx_rec" , *frame, *length);
#endif

    //Return status code
    return error;
}


/**
 * @brief Get link state
 * @param[in] interface Underlying network interface
 * @param[in] port Port number
 * @return Link state
 **/
bool_t dm8x06GetLinkState(NetInterface *interface, uint8_t port) {
    uint16_t value;
    bool_t linkState;

/*
    //Check port number
    if(port >= DM8x06_PORT0 && port <= DM8x06_CPU_PORT) {
        //Any link failure condition is latched in the BMSR register. Reading
        //the register twice will always return the actual link status
        value = dm8x06ReadPhyReg(interface, port, DM8x06_BMSR);
        value = dm8x06ReadPhyReg(interface, port, DM8x06_BMSR);
        //Retrieve current link state
        linkState = (value & DM8x06_BMSR_LINK_STATUS) ? TRUE : FALSE;
    } else {
*/
    //Check port number
    if(port >= DM8x06_PORT0 && port <= DM8x06_CPU_PORT) {
        value = dm8x06ReadSwitchReg(interface, DM8x06_PORTn_PORT_STATUS(port));
        //Retrieve current link state
        linkState = (value & DM8x06_PORTn_PORT_STATUS_LINK) ? TRUE : FALSE;
    } else {
        //The specified port number is not valid
        linkState = FALSE;
    }

    //Return link status
    return linkState;
}


/**
 * @brief Get link speed
 * @param[in] interface Underlying network interface
 * @param[in] port Port number
 * @return Link speed
 **/
uint32_t dm8x06GetLinkSpeed(NetInterface *interface, uint8_t port) {
    uint16_t value;
    uint32_t linkSpeed;

/*
    //Check port number
    if(port >= DM8x06_PORT0 && port <= DM8x06_CPU_PORT) {
        //Read PHY control register
        value = dm8x06ReadPhyReg(interface, port, DM8x06_PHYCON);
        //Retrieve current link speed
        if((value & DM8x06_PHYCON_SPEED_1000BT) != 0) {
            //1000BASE-T
            linkSpeed = NIC_LINK_SPEED_1GBPS;
        } else if((value & DM8x06_PHYCON_SPEED_100BTX) != 0) {
            //100BASE-TX
            linkSpeed = NIC_LINK_SPEED_100MBPS;
        } else if((value & DM8x06_PHYCON_SPEED_10BT) != 0) {
            //10BASE-T
            linkSpeed = NIC_LINK_SPEED_10MBPS;
        } else {
            //The link speed is not valid
            linkSpeed = NIC_LINK_SPEED_UNKNOWN;
        }
    } else {
*/
    //Check port number
    if(port >= DM8x06_PORT0 && port <= DM8x06_CPU_PORT) {
        value = dm8x06ReadSwitchReg(interface, DM8x06_PORTn_PORT_STATUS(port));
        if(value & DM8x06_PORTn_PORT_STATUS_LINK) {
            //Retrieve current link state
            linkSpeed = (value & DM8x06_PORTn_PORT_STATUS_100M) ? NIC_LINK_SPEED_100MBPS : NIC_LINK_SPEED_10MBPS;
        } else {
            //The specified port number is not valid
            linkSpeed = NIC_LINK_SPEED_UNKNOWN;
        }
    } else {
        //The specified port number is not valid
        linkSpeed = NIC_LINK_SPEED_UNKNOWN;
    }

    //Return link status
    return linkSpeed;
}


/**
 * @brief Get duplex mode
 * @param[in] interface Underlying network interface
 * @param[in] port Port number
 * @return Duplex mode
 **/
NicDuplexMode dm8x06GetDuplexMode(NetInterface *interface, uint8_t port) {
    uint16_t value;
    NicDuplexMode duplexMode;

/*
    //Check port number
    if(port >= DM8x06_PORT0 && port <= DM8x06_CPU_PORT) {
        //Read PHY control register
        value = dm8x06ReadPhyReg(interface, port, DM8x06_PHYCON);
        //Retrieve current duplex mode
        if((value & DM8x06_PHYCON_DUPLEX_STATUS) != 0) {
            duplexMode = NIC_FULL_DUPLEX_MODE;
        } else {
            duplexMode = NIC_HALF_DUPLEX_MODE;
        }
    } else {
*/
    //Check port number
    if(port >= DM8x06_PORT0 && port <= DM8x06_CPU_PORT) {
        value = dm8x06ReadSwitchReg(interface, DM8x06_PORTn_PORT_STATUS(port));
        if(value & DM8x06_PORTn_PORT_STATUS_LINK) {
            //Retrieve current link state
            duplexMode = (value & DM8x06_PORTn_PORT_STATUS_FULL) ? NIC_FULL_DUPLEX_MODE : NIC_HALF_DUPLEX_MODE;
        } else {
            //The specified port number is not valid
            duplexMode = NIC_UNKNOWN_DUPLEX_MODE;
        }
    } else {
        //The specified port number is not valid
        duplexMode = NIC_UNKNOWN_DUPLEX_MODE;
    }

    //Return duplex mode
    return duplexMode;
}


/**
 * @brief Set port state
 * @param[in] interface Underlying network interface
 * @param[in] port Port number
 * @param[in] state Port state
 **/
void dm8x06SetPortState(NetInterface *interface, uint8_t port, SwitchPortState state) {
    uint16_t reg_value;
	
    //Check port number
    if(port >= DM8x06_PORT0 && port < DM8x06_CPU_PORT) {
        //Read STP state register
        reg_value = dm8x06ReadSwitchReg(interface, DM8x06_PORTn_STP_STATE(port));
        reg_value &= ~DM8x06_PORTn_STP_STATE_MASK;
			
        //Update port state
        switch(state) {
            case SWITCH_PORT_STATE_BLOCKING:
                reg_value |= DM8x06_PORTn_STP_STATE_BLOCKING_LISTENING;
                TRACE_DEBUG("dm8x06SetPortState port %u , Set To STATE_%s \r\n", port, "BLOCKING/LISTENING");
                break;
            case SWITCH_PORT_STATE_LISTENING:
                reg_value |= DM8x06_PORTn_STP_STATE_BLOCKING_LISTENING;
                TRACE_DEBUG("dm8x06SetPortState port %u , Set To STATE_%s \r\n", port, "LISTENING/BLOCKING");
                break;
            case SWITCH_PORT_STATE_LEARNING:
                reg_value |= DM8x06_PORTn_STP_STATE_LEARNING;
                TRACE_DEBUG("dm8x06SetPortState port %u , Set To STATE_%s \r\n", port, "LEARNING");
                break;
            case SWITCH_PORT_STATE_FORWARDING:
                reg_value |= DM8x06_PORTn_STP_STATE_FORWARDING;
                TRACE_DEBUG("dm8x06SetPortState port %u , Set To STATE_%s \r\n", port, "FORWARDING");
                break;
            default:
                reg_value |= DM8x06_PORTn_STP_STATE_DISABLED;
                TRACE_DEBUG("dm8x06SetPortState port %u , Set To STATE_%s \r\n", port, "DISABLED");
                break;
        }

        //Write the value back to MSTP state register
        dm8x06WriteSwitchReg(interface, DM8x06_PORTn_STP_STATE(port), reg_value);
    }
}


/**
 * @brief Get port state
 * @param[in] interface Underlying network interface
 * @param[in] port Port number
 * @return Port state
 **/
SwitchPortState dm8x06GetPortState(NetInterface *interface, uint8_t port) {
    uint16_t reg_value;
    SwitchPortState state;

    //Check port number
    if(port >= DM8x06_PORT0 && port < DM8x06_CPU_PORT) {
        //Read STP state register
        reg_value = dm8x06ReadSwitchReg(interface, DM8x06_PORTn_STP_STATE(port));
        reg_value &= DM8x06_PORTn_STP_STATE_MASK;
        switch(reg_value) {
            case DM8x06_PORTn_STP_STATE_BLOCKING_LISTENING:
                state = SWITCH_PORT_STATE_BLOCKING;
                TRACE_DEBUG("dm8x06SetPortState port %u , get value STATE_%s \r\n", port, "BLOCKING/LISTENING");
                break;
            case DM8x06_PORTn_STP_STATE_LEARNING:
                state = SWITCH_PORT_STATE_LEARNING;
                TRACE_DEBUG("dm8x06SetPortState port %u , get value STATE_%s \r\n", port, "LEARNING");
                break;
            case DM8x06_PORTn_STP_STATE_FORWARDING:
                state = SWITCH_PORT_STATE_FORWARDING;
                TRACE_DEBUG("dm8x06SetPortState port %u , get value STATE_%s \r\n", port, "FORWARDING");
                break;
            default:
                state = SWITCH_PORT_STATE_DISABLED;
                TRACE_DEBUG("dm8x06SetPortState port %u , get value STATE_%s \r\n", port, "DISABLED");
                break;
        }
    }
    //Return port state
    return state;
}


/**
 * @brief Set aging time for dynamic filtering entries
 * @param[in] interface Underlying network interface
 * @param[in] agingTime Aging time, in seconds
 **/
void dm8x06SetAgingTime(NetInterface *interface, uint32_t agingTime) {
    uint16_t reg_value;

    reg_value = dm8x06ReadSwitchReg(interface, DM8x06_GLOBAL_LEARNING_AND_AGING_CONTROL);
    reg_value &= ~DM8x06_AGING_CONTROL_MASK;

    //Limit the range of the parameter
    agingTime = MIN(agingTime, 512);
    agingTime = MAX(agingTime, 64);

    //The Age Period in combination with the Age Count field determines the
    //aging time of dynamic entries in the address lookup table
    if(agingTime >= 512) {
        reg_value |= DM8x06_AGING_CONTROL_512SEC;
        TRACE_DEBUG("dm8x06SetAgingTime sec %u \r\n", 512);
    } else if (agingTime >= 256) {
        reg_value |= DM8x06_AGING_CONTROL_256SEC;
        TRACE_DEBUG("dm8x06SetAgingTime sec %u \r\n", 256);
    } else if (agingTime >= 128) {
        reg_value |= DM8x06_AGING_CONTROL_128SEC;
        TRACE_DEBUG("dm8x06SetAgingTime sec %u \r\n", 128);
    } else { //64
        reg_value |= DM8x06_AGING_CONTROL_064SEC;
        TRACE_DEBUG("dm8x06SetAgingTime sec %u \r\n", 64);
    }

    dm8x06WriteSwitchReg(interface, DM8x06_GLOBAL_LEARNING_AND_AGING_CONTROL, reg_value);
}


/**
 * @brief Enable IGMP snooping
 * @param[in] interface Underlying network interface
 * @param[in] enable Enable or disable IGMP snooping
 **/
void dm8x06EnableIgmpSnooping(NetInterface *interface, bool_t enable) {
    uint16_t reg_value;

    //Read the Global Snooping Control register
    reg_value = dm8x06ReadSwitchReg(interface, DM8x06_SNOOPING_CONTROL_0);

    //Enable or disable IGMP snooping
    if(enable) {
      reg_value |= DM8x06_IGMP_SNOOPING_HW_EN;
    } else {
       reg_value &= ~(DM8x06_IGMP_SNOOPING_HW_EN);
    }

    TRACE_INFO("== dm8x06EnableIgmpSnooping set %s ==\r\n", enable ? "enable" : "disable");

    //Write the value back to Global Snooping Control register
    dm8x06WriteSwitchReg(interface, DM8x06_SNOOPING_CONTROL_0, reg_value);
}


/**
 * @brief Enable MLD snooping
 * @param[in] interface Underlying network interface
 * @param[in] enable Enable or disable MLD snooping
 **/
void dm8x06EnableMldSnooping(NetInterface *interface, bool_t enable) {
    uint16_t reg_value;

    //Read the Global Snooping Control register
    reg_value = dm8x06ReadSwitchReg(interface, DM8x06_SNOOPING_CONTROL_0);

    //Enable or disable MLD snooping
    if(enable) {
        reg_value |= DM8x06_MLD_SNOOPING_HW_EN;
    } else {
        reg_value &= ~(DM8x06_MLD_SNOOPING_HW_EN);
    }

    TRACE_INFO("== dm8x06EnableMldSnooping set %s ==\r\n", enable ? "enable" : "disable");

    //Write the value back to Global Snooping Control register
    dm8x06WriteSwitchReg(interface, DM8x06_SNOOPING_CONTROL_0, reg_value);
}


/**
 * @brief Enable reserved multicast table
 * @param[in] interface Underlying network interface
 * @param[in] enable Enable or disable reserved group addresses
 **/
#define SPC_DIS                                                (0)
#define SPC_EN                                                 (1 << 0)
#define SPC_ACT_UNMODIFIED                                     (0 << 5)
#define SPC_ACT_DROP                                           (1 << 5)
#define SPC_ACT_TRAP_TO_CPU                                    (2 << 5)
#define SPC_ACT_FLOOD_EXCLUDING_CPU                            (3 << 5)
void dm8x06EnableRsvdMcastTable(NetInterface *interface, bool_t enable) {
#if DEVICE_IS_DM8806
    //The reserved group addresses are in the range of 01-80-C2-00-00-00 to
    //01-80-C2-00-00-0F
    uint16_t sp_tag_0x00_BPDU  = SPC_DIS;
    uint16_t sp_tag_0x01_PAUSE = SPC_DIS;

    uint16_t sp_tag_0x02_SlowProtocol = SPC_DIS;
    uint16_t sp_tag_0x03_EAP = SPC_DIS;

    uint16_t sp_tag_0x04_0D_0F = SPC_DIS;
    uint16_t sp_tag_0x0E_LLDP = SPC_DIS;

    uint16_t sp_tag_0x10_ABM = SPC_DIS;
    uint16_t sp_tag_0x11_1F = SPC_DIS;

    uint16_t sp_tag_0x20_21_GxRP = SPC_DIS;
    uint16_t sp_tag_0x22_2F = SPC_DIS;

    uint16_t sp_tag_0x30_FF = SPC_DIS;

    uint16_t sp_tag_RARP = SPC_DIS;
    uint16_t sp_tag_ARP = SPC_DIS;

    uint16_t sp_tag_IP_IGMP = SPC_DIS;
    uint16_t sp_tag_IPv6_MLD = SPC_DIS;

    uint16_t sp_tag_IP_ICMP = SPC_DIS;
    uint16_t sp_tag_PPPoE = SPC_DIS;

    TRACE_INFO("== dm8x06EnableRsvdMcastTable set %s ==\r\n", enable ? "enable" : "disable");

    if(enable) {
        sp_tag_0x00_BPDU = SPC_ACT_TRAP_TO_CPU | SPC_EN;
    }

    dm8x06WriteSwitchReg(interface, DM8x06_SPEIAL_PACKET_CONTROL_0, (sp_tag_0x01_PAUSE << 8) | sp_tag_0x00_BPDU);
    dm8x06WriteSwitchReg(interface, DM8x06_SPEIAL_PACKET_CONTROL_1, (sp_tag_0x03_EAP << 8)   | sp_tag_0x02_SlowProtocol);
    dm8x06WriteSwitchReg(interface, DM8x06_SPEIAL_PACKET_CONTROL_2, (sp_tag_0x0E_LLDP << 8)  | sp_tag_0x04_0D_0F);
    dm8x06WriteSwitchReg(interface, DM8x06_SPEIAL_PACKET_CONTROL_3, (sp_tag_0x11_1F << 8)    | sp_tag_0x10_ABM);
    dm8x06WriteSwitchReg(interface, DM8x06_SPEIAL_PACKET_CONTROL_4, (sp_tag_0x22_2F << 8)    | sp_tag_0x20_21_GxRP);
    dm8x06WriteSwitchReg(interface, DM8x06_SPEIAL_PACKET_CONTROL_5,                            sp_tag_0x30_FF);
    dm8x06WriteSwitchReg(interface, DM8x06_SPEIAL_PACKET_CONTROL_6, (sp_tag_ARP << 8)        | sp_tag_RARP);
    dm8x06WriteSwitchReg(interface, DM8x06_SPEIAL_PACKET_CONTROL_7, (sp_tag_IPv6_MLD << 8)   | sp_tag_IP_IGMP);
    dm8x06WriteSwitchReg(interface, DM8x06_SPEIAL_PACKET_CONTROL_8, (sp_tag_PPPoE << 8)      | sp_tag_IP_ICMP);

    TRACE_DEBUG("== DM8x06_SPEIAL_PACKET_CONTROL_0 == 0x%04X \r\n", dm8x06ReadSwitchReg(interface, DM8x06_SPEIAL_PACKET_CONTROL_0));
    TRACE_DEBUG("== DM8x06_SPEIAL_PACKET_CONTROL_1 == 0x%04X \r\n", dm8x06ReadSwitchReg(interface, DM8x06_SPEIAL_PACKET_CONTROL_1));
    TRACE_DEBUG("== DM8x06_SPEIAL_PACKET_CONTROL_2 == 0x%04X \r\n", dm8x06ReadSwitchReg(interface, DM8x06_SPEIAL_PACKET_CONTROL_2));
    TRACE_DEBUG("== DM8x06_SPEIAL_PACKET_CONTROL_3 == 0x%04X \r\n", dm8x06ReadSwitchReg(interface, DM8x06_SPEIAL_PACKET_CONTROL_3));
    TRACE_DEBUG("== DM8x06_SPEIAL_PACKET_CONTROL_4 == 0x%04X \r\n", dm8x06ReadSwitchReg(interface, DM8x06_SPEIAL_PACKET_CONTROL_4));
    TRACE_DEBUG("== DM8x06_SPEIAL_PACKET_CONTROL_5 == 0x%04X \r\n", dm8x06ReadSwitchReg(interface, DM8x06_SPEIAL_PACKET_CONTROL_5));
    TRACE_DEBUG("== DM8x06_SPEIAL_PACKET_CONTROL_6 == 0x%04X \r\n", dm8x06ReadSwitchReg(interface, DM8x06_SPEIAL_PACKET_CONTROL_6));
    TRACE_DEBUG("== DM8x06_SPEIAL_PACKET_CONTROL_7 == 0x%04X \r\n", dm8x06ReadSwitchReg(interface, DM8x06_SPEIAL_PACKET_CONTROL_7));
    TRACE_DEBUG("== DM8x06_SPEIAL_PACKET_CONTROL_8 == 0x%04X \r\n", dm8x06ReadSwitchReg(interface, DM8x06_SPEIAL_PACKET_CONTROL_8));
#endif
}

uint16_t dm8x06_ATP_Wait_busy(NetInterface *interface) {
    uint16_t reg_value;
    uint16_t waite_time_out = 0x7fff;

    //checking Busy bit
    do {
        reg_value = dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_CONTROL_AND_STATUS);
    }while(((reg_value & DM8x06_ATB_BUSY) != 0) & (--waite_time_out));

    if(!waite_time_out) {
        TRACE_DEBUG("dm8x06_ATP_Wait_busy __error__time out__r\n");
    }

    return reg_value;
}


/**
 * @brief Add a new entry to the static MAC table
 * @param[in] interface Underlying network interface
 * @param[in] entry Pointer to the forwarding database entry
 * @return Error code
 **/
error_t dm8x06AddStaticFdbEntry(NetInterface *interface, const SwitchFdbEntry *entry) {
    error_t error;
    uint16_t reg_value;
    uint8_t port;

    //dis
    for (port = DM8x06_PORT0; port <= DM8x06_CPU_PORT; port++) {
        reg_value = dm8x06ReadSwitchReg(interface, DM8x06_PORTn_BASIC_CONTROL_0(port));
        reg_value |= DM8x06_PORTn_BASIC_CONTROL_0_ADLRN_DIS;
        dm8x06WriteSwitchReg(interface, DM8x06_PORTn_BASIC_CONTROL_0(port), reg_value);

        reg_value = dm8x06ReadSwitchReg(interface, DM8x06_PORTn_BASIC_CONTROL_1(port));
        reg_value |= DM8x06_PORTn_BASIC_CONTROL_1_FIRUUSID_EN;
        dm8x06WriteSwitchReg(interface, DM8x06_PORTn_BASIC_CONTROL_1(port), reg_value);
    }

    //checking Busy bit
    dm8x06_ATP_Wait_busy(interface);

    //MAC address register (bytes 0 and 1)
    dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_DATA_3, htons(entry->macAddr.w[0]));
    //MAC address register (bytes 2 and 3)
    dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_DATA_2, htons(entry->macAddr.w[1]));
    //MAC address register (bytes 4 and 5)
    dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_DATA_1, htons(entry->macAddr.w[2]));

    //Multicast address?
    if(macIsMulticastAddr(&entry->macAddr)) {
        uint8_t destPort_map;

        if(entry->destPorts == SWITCH_CPU_PORT_MASK) {
            destPort_map = dm8x06Port_Num_2_Port_Map(DM8x06_CPU_PORT);
        } else {
            destPort_map = (entry->destPorts >> 2) & DM8x06_PORT_MASK;
        }

        //set write map
        dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_PORT_SET, destPort_map);
        //clean static
        dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_STATIC_AND_IGMP, 0x0000);
        //write cmd
        dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_CONTROL_AND_STATUS, DM8x06_ATB_CMD_WRITE | DM8x06_ATB_INDEX_MULTICAST);
    } else {
        uint8_t destPort_num;

        if(entry->destPorts == SWITCH_CPU_PORT_MASK) {
            destPort_num = DM8x06_ADDRESS_TABLE_SET_PORT(DM8x06_CPU_PORT);
        } else {
            destPort_num = dm8x06Port_Map_2_Port_Num(entry->destPorts);
        }

        //set write port
        dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_PORT_SET, destPort_num);
        if(entry->override) {
            reg_value = DM8x06_ADDRESS_TABLE_STATIC_EN | DM8x06_ADDRESS_TABLE_OVERRIDE_EN;
        } else {
            reg_value = DM8x06_ADDRESS_TABLE_STATIC_EN;
        }

        //set static
        dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_STATIC_AND_IGMP, reg_value);
        //write cmd
        dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_CONTROL_AND_STATUS, DM8x06_ATB_CMD_WRITE | DM8x06_ATB_INDEX_UNICAST);
    }

    //checking Busy bit
    reg_value = dm8x06_ATP_Wait_busy(interface);
    if(reg_value & DM8x06_ATB_CR_ERR) {
        //The static MAC table is full
        error = ERROR_TABLE_FULL;
    } else {
        //Successful processing
        error = NO_ERROR;
    }

#if 1
    //checking Busy bit
    dm8x06_ATP_Wait_busy(interface);
    //MAC address register (bytes 0 and 1)
    dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_DATA_3, 0x8606);
    //MAC address register (bytes 2 and 3)
    dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_DATA_2, 0x8606);
    //MAC address register (bytes 4 and 5)
    dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_DATA_1, 0x8606);
    //set write port
    dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_PORT_SET, DM8x06_ADDRESS_TABLE_SET_PORT(DM8x06_CPU_PORT));
    dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_STATIC_AND_IGMP, 0x0000);
    //write cmd
    dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_CONTROL_AND_STATUS, DM8x06_ATB_CMD_WRITE | DM8x06_ATB_INDEX_UNICAST);
#endif

    //EN
    for (port = DM8x06_PORT0; port <= DM8x06_CPU_PORT; port++) {
        reg_value = dm8x06ReadSwitchReg(interface, DM8x06_PORTn_BASIC_CONTROL_0(port));
        reg_value &= ~DM8x06_PORTn_BASIC_CONTROL_0_ADLRN_DIS;
        dm8x06WriteSwitchReg(interface, DM8x06_PORTn_BASIC_CONTROL_0(port), reg_value);

        reg_value = dm8x06ReadSwitchReg(interface, DM8x06_PORTn_BASIC_CONTROL_1(port));
        reg_value &= ~DM8x06_PORTn_BASIC_CONTROL_1_FIRUUSID_EN;
        dm8x06WriteSwitchReg(interface, DM8x06_PORTn_BASIC_CONTROL_1(port), reg_value);
    }

    //Return status code
    return error;
}


/**
 * @brief Remove an entry from the static MAC table
 * @param[in] interface Underlying network interface
 * @param[in] entry Forwarding database entry to remove from the table
 * @return Error code
 **/
error_t dm8x06DeleteStaticFdbEntry(NetInterface *interface, const SwitchFdbEntry *entry) {
    error_t error;
    uint16_t reg_value;

    if(macIsMulticastAddr(&entry->macAddr)) {
        return ERROR_INVALID_ENTRY;
    }

    //checking Busy bit
    dm8x06_ATP_Wait_busy(interface);

    //MAC address register (bytes 0 and 1)
    dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_DATA_3, htons(entry->macAddr.w[0]));
    //MAC address register (bytes 2 and 3)
    dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_DATA_2, htons(entry->macAddr.w[1]));
    //MAC address register (bytes 4 and 5)
    dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_DATA_1, htons(entry->macAddr.w[2]));

    //search cmd
    dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_CONTROL_AND_STATUS, DM8x06_ATB_CMD_SEARCH | DM8x06_ATB_INDEX_MAC);

    //checking Busy bit
    reg_value = dm8x06_ATP_Wait_busy(interface);

    if(reg_value & DM8x06_ATB_CR_ERR) {
        //cmd error
        error = ERROR_NOT_FOUND;
    } else {
        if(reg_value & DM8x06_ATB_CR_ENTRY_DONE) {
            error = ERROR_INVALID_ENTRY;
            if(macIsMulticastAddr(&entry->macAddr)) {
                //delete cmd
                dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_CONTROL_AND_STATUS, DM8x06_ATB_CMD_DELETE | DM8x06_ATB_INDEX_MULTICAST);
                //Successful processing
                error = NO_ERROR;
            } else {
                if(DM8x06_ADDRESS_TABLE_STATIC_EN ==
                        (DM8x06_ADDRESS_TABLE_STATIC_EN & dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_STATIC_AND_IGMP))) {
                    //delete cmd
                    dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_CONTROL_AND_STATUS, DM8x06_ATB_CMD_DELETE | DM8x06_ATB_INDEX_UNICAST);
                    //Successful processing
                    error = NO_ERROR;
                }
            }
        } else {
            //not found
            error = ERROR_NOT_FOUND;
        }
    }

    //Return status code
    return error;
}


/**
 * @brief Read an entry from the static MAC table
 * @param[in] interface Underlying network interface
 * @param[in] index Zero-based index of the entry to read
 * @param[out] entry Pointer to the forwarding database entry
 * @return Error code
 **/
error_t dm8x06GetStaticFdbEntry(NetInterface *interface, uint_t index, SwitchFdbEntry *entry) {
    error_t error;
    uint16_t reg_value;
    uint16_t mac_addr_word_0;

    //Check index parameter
    if(index < DM8x06_MAC_TABLE_SIZE) {
        //checking Busy bit
        dm8x06_ATP_Wait_busy(interface);

        //set index num
        dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_INDEX_SET, index);
        //read cmd
        dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_CONTROL_AND_STATUS, DM8x06_ATB_CMD_READ | DM8x06_ATB_INDEX_MAC);

        //checking Busy bit
        reg_value = dm8x06_ATP_Wait_busy(interface);

        if(reg_value & DM8x06_ATB_CR_ERR) {
            //cmd error
            error = ERROR_NOT_FOUND;
        } else {
            error = ERROR_INVALID_ENTRY;
            if(reg_value & DM8x06_ATB_CR_ENTRY_DONE) {
                //Read MAC address register (bytes 0 and 1)
                mac_addr_word_0 = dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_DATA_3);
                if(mac_addr_word_0 & DM8x06_ADDRESS_TABLE_MULTICAST_BIT) { //multicast
                    //Read MAC address register (bytes 0 and 1)
                    //entry->macAddr.w[0] = ntohs(dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_DATA_3));
                    entry->macAddr.w[0] = ntohs(mac_addr_word_0);
                    //Read MAC address register (bytes 2 and 3)
                    entry->macAddr.w[1] = ntohs(dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_DATA_2));
                    //Read MAC address register (bytes 4 and 5)
                    entry->macAddr.w[2] = ntohs(dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_DATA_1));
                    //Check the value of the OVERRIDE bit
                    entry->override = FALSE;
                    entry->srcPort = 0;
                    reg_value = dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_PORT_SET);
                    entry->destPorts = (reg_value & DM8x06_PORT_MASK) << 2;
                    //Successful processing
                    error = NO_ERROR;
                } else {
                    if(DM8x06_ADDRESS_TABLE_STATIC_EN ==
                            (DM8x06_ADDRESS_TABLE_STATIC_EN & dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_STATIC_AND_IGMP))) {

                        //Read MAC address register (bytes 0 and 1)
                        //entry->macAddr.w[0] = ntohs(dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_DATA_3));
                        entry->macAddr.w[0] = ntohs(mac_addr_word_0);
                        //Read MAC address register (bytes 2 and 3)
                        entry->macAddr.w[1] = ntohs(dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_DATA_2));
                        //Read MAC address register (bytes 4 and 5)
                        entry->macAddr.w[2] = ntohs(dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_DATA_1));

                        //Check the value of the OVERRIDE bit
                        reg_value = dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_STATIC_AND_IGMP);
                        if(reg_value & DM8x06_ADDRESS_TABLE_OVERRIDE_EN) {
                            entry->override = TRUE;
                        } else {
                            entry->override = FALSE;
                        }

                        entry->srcPort = 0;
                        reg_value = (dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_PORT_SET) & 0x0007);
                        reg_value <<= 2;
                        entry->destPorts = reg_value;
                        //Successful processing
                        error = NO_ERROR;
                    }
                }
            }
        }
    } else {
        //The end of the table has been reached
        error = ERROR_END_OF_TABLE;
    }

    return error;
}


/**
 * @brief Flush static MAC table
 * @param[in] interface Underlying network interface
 **/
void dm8x06FlushStaticFdbTable(NetInterface *interface) {
    uint16_t reg_value;
    uint8_t port;

    //disable rx
    for (port = DM8x06_PORT0; port <= DM8x06_CPU_PORT; port++) {
        reg_value = dm8x06ReadSwitchReg(interface, DM8x06_PORTn_BASIC_CONTROL_0(port));
        reg_value |= DM8x06_PORTn_BASIC_CONTROL_0_RX_DIS;
        dm8x06WriteSwitchReg(interface, DM8x06_PORTn_BASIC_CONTROL_0(port), reg_value);
    }

/*
    for (port = DM8x06_PORT0; port <= DM8x06_CPU_PORT; port++) {
        //checking Busy bit
        dm8x06_ATP_Wait_busy(interface);
        dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_PORT_SET, DM8x06_ADDRESS_TABLE_SET_PORT(port));

        //clean cmd
        dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_CONTROL_AND_STATUS,
                 DM8x06_ATB_CLEAN_PORT_EN | DM8x06_ATB_CMD_CLEAN | DM8x06_ATB_INDEX_UNICAST);

        //checking Busy bit
        dm8x06_ATP_Wait_busy(interface);
        dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_PORT_SET, DM8x06_ADDRESS_TABLE_SET_MAP(port));

        //clean cmd
        dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_CONTROL_AND_STATUS,
                 DM8x06_ATB_CLEAN_PORT_EN | DM8x06_ATB_CMD_CLEAN | DM8x06_ATB_INDEX_MULTICAST);
    }
*/
    //checking Busy bit
    dm8x06_ATP_Wait_busy(interface);
    //clean cmd
    dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_CONTROL_AND_STATUS, DM8x06_ATB_CMD_CLEAN | DM8x06_ATB_INDEX_UNICAST);
    //checking Busy bit
    dm8x06_ATP_Wait_busy(interface);
    //clean cmd
    dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_CONTROL_AND_STATUS, DM8x06_ATB_CMD_CLEAN | DM8x06_ATB_INDEX_MULTICAST);

    //enable rx
    for (port = DM8x06_PORT0; port <= DM8x06_CPU_PORT; port++) {
        reg_value = dm8x06ReadSwitchReg(interface, DM8x06_PORTn_BASIC_CONTROL_0(port));
        reg_value &= ~(DM8x06_PORTn_BASIC_CONTROL_0_RX_DIS);
        dm8x06WriteSwitchReg(interface, DM8x06_PORTn_BASIC_CONTROL_0(port), reg_value);
    }
}


/**
 * @brief Read an entry from the dynamic MAC table
 * @param[in] interface Underlying network interface
 * @param[in] index Zero-based index of the entry to read
 * @param[out] entry Pointer to the forwarding database entry
 * @return Error code
 **/
error_t dm8x06GetDynamicFdbEntry(NetInterface *interface, uint_t index, SwitchFdbEntry *entry) {
    error_t error;
    uint16_t reg_value;
    uint16_t mac_addr_word_0;

    //Check index parameter
    if(index < DM8x06_MAC_TABLE_SIZE) {
        //checking Busy bit
        dm8x06_ATP_Wait_busy(interface);

        //set index num
        dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_INDEX_SET, index);
        //read cmd
        dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_CONTROL_AND_STATUS, DM8x06_ATB_CMD_READ | DM8x06_ATB_INDEX_MAC);

        //checking Busy bit
        reg_value = dm8x06_ATP_Wait_busy(interface);

        if(reg_value & DM8x06_ATB_CR_ERR) {
            //cmd error
            error = ERROR_NOT_FOUND;
        } else {
            error = ERROR_INVALID_ENTRY;
            if(reg_value & DM8x06_ATB_CR_ENTRY_DONE) {
                mac_addr_word_0 = dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_DATA_3);
                if(!(mac_addr_word_0 & DM8x06_ADDRESS_TABLE_MULTICAST_BIT)) {
                    if(DM8x06_ADDRESS_TABLE_STATIC_EN !=
                            (DM8x06_ADDRESS_TABLE_STATIC_EN & dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_STATIC_AND_IGMP))) {

                        //Read MAC address register (bytes 0 and 1)
                        //entry->macAddr.w[0] = ntohs(dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_DATA_3));
                        entry->macAddr.w[0] = ntohs(mac_addr_word_0);
                        //Read MAC address register (bytes 2 and 3)
                        entry->macAddr.w[1] = ntohs(dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_DATA_2));
                        //Read MAC address register (bytes 4 and 5)
                        entry->macAddr.w[2] = ntohs(dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_DATA_1));

                        //Check the value of the OVERRIDE bit
                        reg_value = dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_STATIC_AND_IGMP);
                        if(reg_value & DM8x06_ADDRESS_TABLE_OVERRIDE_EN) {
                            entry->override = TRUE;
                        } else {
                            entry->override = FALSE;
                        }

                        entry->srcPort = 0;
                        reg_value = (dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_PORT_SET) & 0x0007);
                        reg_value <<= 2;
                        entry->destPorts = reg_value;
                        //Successful processing
                        error = NO_ERROR;
                    }
                }
            }
        }
    } else {
        //The end of the table has been reached
        error = ERROR_END_OF_TABLE;
    }

    return error;
}


/**
 * @brief Flush dynamic MAC table
 * @param[in] interface Underlying network interface
 * @param[in] port Port number
 **/
void dm8x06FlushDynamicFdbTable(NetInterface *interface, uint8_t port) {
    uint16_t reg_value;
    uint16_t entry_index;

    //disable rx
    reg_value = dm8x06ReadSwitchReg(interface, DM8x06_PORTn_BASIC_CONTROL_0(port));
    reg_value |= DM8x06_PORTn_BASIC_CONTROL_0_RX_DIS;
    dm8x06WriteSwitchReg(interface, DM8x06_PORTn_BASIC_CONTROL_0(port), reg_value);

    for (entry_index = 0; entry_index < DM8x06_MAC_TABLE_SIZE; entry_index++) {
        //checking Busy bit
        dm8x06_ATP_Wait_busy(interface);

        //set index num
        dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_INDEX_SET, entry_index);
        //read cmd
        dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_CONTROL_AND_STATUS, DM8x06_ATB_CMD_READ | DM8x06_ATB_INDEX_MAC);

        //checking Busy bit
        reg_value = dm8x06_ATP_Wait_busy(interface);

        if(!(reg_value & DM8x06_ATB_CR_ERR)) {
            if(reg_value & DM8x06_ATB_CR_ENTRY_DONE) {
                TRACE_DEBUG("port %u mac 0x%02X : 0x%02X : 0x%02X : 0x%02X : 0x%02X : 0x%02X set 0x%04X \r\n",
                        dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_PORT_SET) & 0x07,
                        (dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_DATA_3) >> 8) & 0xff,
                        dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_DATA_3) & 0xff,
                        (dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_DATA_2) >> 8) & 0xff,
                        dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_DATA_2) & 0xff,
                        (dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_DATA_1) >> 8) & 0xff,
                        dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_DATA_1) & 0xff,
                        dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_STATIC_AND_IGMP));

                //Read MAC address register (bytes 0 and 1)
                //unicast only
                if(!(DM8x06_ADDRESS_TABLE_MULTICAST_BIT & dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_DATA_3))) {
                    reg_value = dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_PORT_SET);

                    if(port == DM8x06_ADDRESS_TABLE_GET_PORT(reg_value)) {
                        if(DM8x06_ADDRESS_TABLE_STATIC_EN !=
                                (DM8x06_ADDRESS_TABLE_STATIC_EN & dm8x06ReadSwitchReg(interface, DM8x06_ADDRESS_TABLE_STATIC_AND_IGMP))) {

                            //delete cmd
                            dm8x06WriteSwitchReg(interface, DM8x06_ADDRESS_TABLE_CONTROL_AND_STATUS,
                                    DM8x06_ATB_CMD_DELETE | DM8x06_ATB_INDEX_UNICAST);
                         }
                    }
                }
            }
        }
    }

    //enable rx
    reg_value = dm8x06ReadSwitchReg(interface, DM8x06_PORTn_BASIC_CONTROL_0(port));
    reg_value &= ~(DM8x06_PORTn_BASIC_CONTROL_0_RX_DIS);
    dm8x06WriteSwitchReg(interface, DM8x06_PORTn_BASIC_CONTROL_0(port), reg_value);
}


/**
 * @brief Set forward ports for unknown multicast packets
 * @param[in] interface Underlying network interface
 * @param[in] enable Enable or disable forwarding of unknown multicast packets
 * @param[in] forwardPorts Port map
 **/
void dm8x06SetUnknownMcastFwdPorts(NetInterface *interface, bool_t enable, uint32_t forwardPorts) {
    uint16_t reg_value;
    uint8_t port;

    for (port = DM8x06_PORT0; port <= DM8x06_CPU_PORT; port++) {
        if(forwardPorts & 1 << port) {
            reg_value = dm8x06ReadSwitchReg(interface, DM8x06_PORTn_BASIC_CONTROL_1(port));

            if(enable) {
                reg_value &= ~(DM8x06_PORTn_PORT_UNKNOWN_MULTICAST_DIS);
            } else {
                reg_value |= DM8x06_PORTn_PORT_UNKNOWN_MULTICAST_DIS;
            }

            dm8x06WriteSwitchReg(interface, DM8x06_PORTn_BASIC_CONTROL_1(port), reg_value);
        }
    }
}


/**
 * @brief Write PHY register
 * @param[in] interface Underlying network interface
 * @param[in] port Port number
 * @param[in] address PHY register address
 * @param[in] data Register value
 **/
void dm8x06WritePhyReg(NetInterface *interface, uint8_t port, uint8_t address, uint16_t data) {
    if(interface->smiDriver != NULL) {
        //Write the specified PHY register
        interface->smiDriver->writePhyReg(SMI_OPCODE_WRITE, port, address, data);
    } else {
        //Write the specified PHY register
        interface->nicDriver->writePhyReg(SMI_OPCODE_WRITE, port, address, data);
    }
}


/**
 * @brief Read PHY register
 * @param[in] interface Underlying network interface
 * @param[in] port Port number
 * @param[in] address PHY register address
 * @return Register value
 **/
uint16_t dm8x06ReadPhyReg(NetInterface *interface, uint8_t port, uint8_t address) {
    uint16_t data;

    if(interface->smiDriver != NULL) {
        //Read the specified PHY register
        data = interface->smiDriver->readPhyReg(SMI_OPCODE_READ, port, address);
    } else {
        //Read the specified PHY register
        data = interface->nicDriver->readPhyReg(SMI_OPCODE_READ, port, address);
    }

    //Return register value
    return data;
}


/**
 * @brief Dump PHY registers for debugging purpose
 * @param[in] interface Underlying network interface
 * @param[in] port Port number
 **/
void dm8x06DumpPhyReg(NetInterface *interface, uint8_t port) {
    uint8_t i;

    //Loop through PHY registers
    for (i = 0; i < DM8x06_GENERAL_REGISTERS_RANGE; i++) {
        //Display current PHY register
        TRACE_DEBUG("%u : 0x%04X \r\n", i, dm8x06ReadPhyReg(interface, port, i));
    }

    //Terminate with a line feed
    TRACE_DEBUG("\r\n");
}


/**
 * @brief Write switch absolute register (16 bits)
 * @param[in] interface Underlying network interface
 * @param[in] address Switch register address
 * @param[in] data Register value
 **/
void dm8x06WriteSwitchReg(NetInterface *interface, uint16_t address, uint16_t data) {
    uint16_t phy_addr, reg_addr;

    reg_addr = address & 0x1f;
    phy_addr = (address >> 5) & 0x1f ;

    if(interface->smiDriver != NULL) {
        //Write the specified PHY register
        interface->smiDriver->writePhyReg(SMI_OPCODE_WRITE, phy_addr, reg_addr, data);
    } else {
        //Write the specified PHY register
        interface->nicDriver->writePhyReg(SMI_OPCODE_WRITE, phy_addr, reg_addr, data);
    }
}


/**
 * @brief Read switch register (16 bits)
 * @param[in] interface Underlying network interface
 * @param[in] address Switch register address
 * @return Register value
 **/
uint16_t dm8x06ReadSwitchReg(NetInterface *interface, uint16_t address) {
    uint16_t data;
    uint16_t phy_addr, reg_addr;

    reg_addr = address & 0x1f;
    phy_addr = (address >> 5) & 0x1f ;

    if(interface->smiDriver != NULL) {
        //Read the specified PHY register
        data = interface->smiDriver->readPhyReg(SMI_OPCODE_READ, phy_addr, reg_addr);
    } else {
        //Read the specified PHY register
        data = interface->nicDriver->readPhyReg(SMI_OPCODE_READ, phy_addr, reg_addr);
    }

    //Return register value
    return data;
}

uint8_t dm8x06Port_Map_2_Port_Num(uint8_t port_map) {
    uint8_t port_num = 0;
    uint8_t new_port_map = port_map >> 2;

    switch(new_port_map & DM8x06_PORT_MASK) {
        case DM8x06_PORT0_MASK :
            port_num = DM8x06_ADDRESS_TABLE_SET_PORT(DM8x06_PORT0);
            break;
        case DM8x06_PORT1_MASK :
            port_num = DM8x06_ADDRESS_TABLE_SET_PORT(DM8x06_PORT1);
            break;
        case DM8x06_PORT2_MASK :
            port_num = DM8x06_ADDRESS_TABLE_SET_PORT(DM8x06_PORT2);
            break;
        case DM8x06_PORT3_MASK :
            port_num = DM8x06_ADDRESS_TABLE_SET_PORT(DM8x06_PORT3);
            break;
        case DM8x06_PORT4_MASK :
            port_num = DM8x06_ADDRESS_TABLE_SET_PORT(DM8x06_PORT4);
            break;
        case DM8x06_PORT5_MASK :
            port_num = DM8x06_ADDRESS_TABLE_SET_PORT(DM8x06_PORT5);
            break;
    }

    return port_num;
}

uint8_t dm8x06Port_Num_2_Port_Map(uint8_t port_num) {
    uint8_t port_map = 0;

    port_map = (1 << (port_num - 2));

    return port_map;
}



