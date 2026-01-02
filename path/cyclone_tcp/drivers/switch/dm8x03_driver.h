/**
 * @file dm8x03_driver.h
 * @brief DM8x03 3-port Fast Ethernet switch driver
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

#ifndef _DM8x03_DRIVER_H
#define _DM8x03_DRIVER_H

//Dependencies
#include "core/nic.h"

//Size of of the MAC address lookup table
#define DM8x03_MAC_TABLE_SIZE                  2048

//Port identifiers
#define DM8x03_PORT0                           2
#define DM8x03_PORT1                           3
#define DM8x03_PORT2                           4

//PHY address
#define DM8x03_PHY_ADDR                        0
#define DM8x03_CPU_PORT                        DM8x03_PORT2
#define DM_SWITCH_NUM_PORTS                    (DM8x03_CPU_PORT - 1)
#define DM8x03_CPU_PORT_SET(x)                 (x - 2)

//Port masks
#define DM8x03_PORT_MASK                       0x3F
#define DM8x03_PORT0_MASK                      0x01
#define DM8x03_PORT1_MASK                      0x02
#define DM8x03_PORT2_MASK                      0x04

//DM8x03 PHY registers
#define DM8x03_BMCR                            0x00
#define DM8x03_BMSR                            0x01
#define DM8x03_PHYID1                          0x02
#define DM8x03_PHYID2                          0x03
#define DM8x03_ANAR                            0x04
#define DM8x03_ANLPAR                          0x05
#define DM8x03_ANER                            0x06
#define DM8x03_GENERAL_REGISTERS_RANGE             0x06

//PHY Control register
#define DM8x03_BMCR_SW_RESET                   0x8000
#define DM8x03_BMCR_LOOPBACK                   0x4000
#define DM8x03_BMCR_SPEED_LSB                  0x2000
#define DM8x03_BMCR_ANEG_EN                    0x1000
#define DM8x03_BMCR_PWR_DWN                    0x0800
#define DM8x03_BMCR_ISOLATE                    0x0400
#define DM8x03_BMCR_RESTART_ANEG               0x0200
#define DM8x03_BMCR_DUPLEX                     0x0100
#define DM8x03_BMCR_COL_TEST                   0x0080
#define DM8x03_BMCR_SPEED_MSB                  0x0040

//PHY Status register
#define DM8x03_BMSR_100T4                      0x8000
#define DM8x03_BMSR_100FDX                     0x4000
#define DM8x03_BMSR_100HDX                     0x2000
#define DM8x03_BMSR_10FDX                      0x1000
#define DM8x03_BMSR_10HPX                      0x0800
#define DM8x03_BMSR_100T2FDX                   0x0400
#define DM8x03_BMSR_100T2HDX                   0x0200
#define DM8x03_BMSR_EXTD_STATUS                0x0100
#define DM8x03_BMSR_MF_PRE_SUP                 0x0040
#define DM8x03_BMSR_ANEG_DONE                  0x0020
#define DM8x03_BMSR_REMOTE_FAULT               0x0010
#define DM8x03_BMSR_ANEG_ABLE                  0x0008
#define DM8x03_BMSR_LINK                       0x0004
#define DM8x03_BMSR_JABBER_DET                 0x0002
#define DM8x03_BMSR_EXTD_REG                   0x0001

//PHY Identifier 1 register
#define DM8x03_PHYID1_OUI_MSB                  0xFFFF
#define DM8x03_PHYID1_OUI_MSB_DEFAULT              0x0181

//PHY Identifier 2 register
#define DM8x03_PHYID2_OUI_LSB                  0xFC00
#define DM8x03_PHYID2_OUI_LSB_DEFAULT              0xb800
#define DM8x03_PHYID2_MODEL_NUM                0x03F0
#define DM8x03_PHYID2_MODEL_NUM_DEFAULT            0x00b0
#define DM8x03_PHYID2_REV_NUM                      0x0001

//Auto-Negotiation Advertisement register
#define DM8x03_ANAR_ANEG_AD_NXT_PAGE           0x8000
#define DM8x03_ANAR_ACK                        0x4000
#define DM8x03_ANAR_ANEG_AD_RE_FAULT           0x2000
#define DM8x03_ANAR_ANEG_AD_PAUSE              0x0400
#define DM8x03_ANAR_ANEG_AD_100T4              0x0200
#define DM8x03_ANAR_ANEG_AD_100FDX             0x0100
#define DM8x03_ANAR_ANEG_AD_100HDX             0x0080
#define DM8x03_ANAR_ANEG_AD_10FDX              0x0040
#define DM8x03_ANAR_ANEG_AD_10HDX              0x0020
#define DM8x03_ANAR_ANEG_AD_SELECTOR           0x001F
#define DM8x03_ANAR_ANEG_AD_SELECTOR_DEFAULT   0x0001

//Link Partner Ability register
#define DM8x03_ANLPAR_LP_NXT_PAGE              0x8000
#define DM8x03_ANLPAR_LP_ACK                   0x4000
#define DM8x03_ANLPAR_LP_REMOTE_FAULT          0x2000
#define DM8x03_ANLPAR_LP_TECH_ABLE             0x1FE0
#define DM8x03_ANLPAR_LP_SELECTOR              0x001F

//Auto-Negotiation Expansion register
#define DM8x03_ANER_PAR_FAULT_DET              0x0010
#define DM8x03_ANER_LP_NXT_PG_ABLE             0x0008
#define DM8x03_ANER_LOCAL_NXT_PG_ABLE          0x0004
#define DM8x03_ANER_RX_NEW_PAGE                0x0002
#define DM8x03_ANER_LP_ANEG_ABLE               0x0001


//dm8x03 Switch Per-Port Registers
#define DM8x03_PORT_0_PORT_STATUS                              0x110
#define DM8x03_PORT_0_BASIC_CONTROL_0                          0x111
#define DM8x03_PORT_0_BASIC_CONTROL_1                          0x112
#define DM8x03_PORT_0_BLOCK_CONTROL_0                          0x113
#define DM8x03_PORT_0_BLOCK_CONTROL_1                          0x114
#define DM8x03_PORT_0_BANDWIDTH_CONTROL                        0x115
#define DM8x03_PORT_0_VLAN_TAG_INFOMATION                      0x116
#define DM8x03_PORT_0_PRIORITY_AND_VLAN_CONTROL                0x117
#define DM8x03_PORT_0_SPANNING_TREE_STATE_CONTROL              0x119

#define DM8x03_PORTn_PORT_STATUS(port)                            (DM8x03_PORT_0_PORT_STATUS + ((port - 2) * 0x20))
#define DM8x03_PORTn_BASIC_CONTROL_0(port)                        (DM8x03_PORT_0_BASIC_CONTROL_0 + ((port - 2) * 0x20))
#define DM8x03_PORTn_BASIC_CONTROL_0_RX_DIS                       (1 << 1)
#define DM8x03_PORTn_BASIC_CONTROL_0_TX_DIS                       (1 << 0)
#define DM8x03_PORTn_BASIC_CONTROL_0_ADLRN_DIS                    (1 << 12)

#define DM8x03_PORTn_BASIC_CONTROL_1(port)                        (DM8x03_PORT_0_BASIC_CONTROL_1 + ((port - 2) * 0x20))
#define DM8x03_PORTn_BASIC_CONTROL_1_FIRUUSID_EN                  (1 << 9)

#define DM8x03_PORTn_BLOCK_CONTROL_0(port)                        (DM8x03_PORT_0_BLOCK_CONTROL_0 + ((port - 2) * 0x20))
#define DM8x03_PORTn_BLOCK_CONTROL_1(port)                        (DM8x03_PORT_0_BLOCK_CONTROL_1 + ((port - 2) * 0x20))
#define DM8x03_PORTn_BANDWIDTH_CONTROL(port)                      (DM8x03_PORT_0_BANDWIDTH_CONTROL + ((port - 2) * 0x20))
#define DM8x03_PORTn_VLAN_TAG_INFOMATION(port)                    (DM8x03_PORT_0_VLAN_TAG_INFOMATION + ((port - 2) * 0x20))
#define DM8x03_PORTn_PRIORITY_AND_VLAN_CONTROL(port)              (DM8x03_PORT_0_PRIORITY_AND_VLAN_CONTROL + ((port - 2) * 0x20))
#define DM8x03_PORTn_SPANNING_TREE_STATE_CONTROL(port)            (DM8x03_PORT_0_SPANNING_TREE_STATE_CONTROL + ((port - 2) * 0x20))

#define DM8x03_PORT_1_PORT_STATUS                              0x130
#define DM8x03_PORT_1_BASIC_CONTROL_0                          0x131
#define DM8x03_PORT_1_BASIC_CONTROL_1                          0x132
#define DM8x03_PORT_1_BLOCK_CONTROL_0                          0x133
#define DM8x03_PORT_1_BLOCK_CONTROL_1                          0x134
#define DM8x03_PORT_1_BANDWIDTH_CONTROL                        0x135
#define DM8x03_PORT_1_VLAN_TAG_INFOMATION                      0x136
#define DM8x03_PORT_1_PRIORITY_AND_VLAN_CONTROL                0x137
#define DM8x03_PORT_1_SPANNING_TREE_STATE_CONTROL              0x139

#define DM8x03_PORT_2_PORT_STATUS                              0x150
#define DM8x03_PORT_2_BASIC_CONTROL_0                          0x151
#define DM8x03_PORT_2_BASIC_CONTROL_1                          0x152
#define DM8x03_PORT_2_BLOCK_CONTROL_0                          0x153
#define DM8x03_PORT_2_BLOCK_CONTROL_1                          0x154
#define DM8x03_PORT_2_BANDWIDTH_CONTROL                        0x155
#define DM8x03_PORT_2_VLAN_TAG_INFOMATION                      0x156
#define DM8x03_PORT_2_PRIORITY_AND_VLAN_CONTROL                0x157
#define DM8x03_PORT_2_SPANNING_TREE_STATE_CONTROL              0x159


//dm8x03 Switch Engine Registers
#define DM8x03_SWITCH_STATUS                                   0x210
#define DM8x03 SWITCH_RESET                                    0x211
#define DM8x03_SWITCH_CONTROL                                  0x212
#define DM8x03_CPU_PORT_AND_MIRROR_CONTROL                     0x213
#define DM8x03_CPU_PORT_AND_MIRROR_CONTROL_DEFAULT                 0x0000
#define DM8x03_SPECIAL_TAG_TX_ENABLE                               (1 << 7) //to cpu
#define DM8x03_SPECIAL_TAG_RX_ENABLE                               (1 << 6) //from cpu
#define DM8x03_SPECIAL_TAG_ETHER_TYPE                          0x214
#define DM8x03_SPECIAL_TAG_ETHER_TYPE_DEFAULT                      0x8606

#define DM8x03_GLOBAL_LEARNING_AND_AGING_CONTROL               0x215
#define DM8x03_AGING_CONTROL_MASK                                  0x03
#define DM8x03_AGING_CONTROL_512SEC                                0x03
#define DM8x03_AGING_CONTROL_256SEC                                0x02
#define DM8x03_AGING_CONTROL_128SEC                                0x01
#define DM8x03_AGING_CONTROL_064SEC                                0x00
#define DM8x03_ADDRESS_TABLE_SEPARATED_MODE                        (1 << 2)

//0x217 ~ 0x21f

//0x230 ~ 0x233  , 0x23E

//0x270 ~ 0x27f

//0x292
#define DM8x03_STP_CONTROL                                     0x292
#define DM8x03_STP_CONTROL_DEFAULT                                 0x0000
#define DM8x03_STP_EN                                              (1 << 0)


#define DM8x03_SNOOPING_CONTROL_0                              0x29B
#define DM8x03_IGMP_SNOOPING_HW_EN                                 (1 << 0)
#define DM8x03_MLD_SNOOPING_HW_EN                                  (1 << 1)
#define DM8x03_SNOOPING_CONTROL_1                              0x29C

#define DM8x03_ADDRESS_TABLE_CONTROL_AND_STATUS                0x2B0
#define DM8x03_ATB_BUSY                                            (1 << 15)
#define DM8x03_ATB_CR_ERR                                          (1 << 14)
#define DM8x03_ATB_CR_ENTRY_DONE                                   (1 << 13)

#define DM8x03_ATB_CMD_READ                                        (0 << 2)
#define DM8x03_ATB_CMD_WRITE                                       (1 << 2)
#define DM8x03_ATB_CMD_DELETE                                      (2 << 2)
#define DM8x03_ATB_CMD_SEARCH                                      (3 << 2)

#define DM8x03_ATB_INDEX_UNICAST                                   (0)
#define DM8x03_ATB_INDEX_MULTICAST                                 (1)
#define DM8x03_ATB_INDEX_IGMP                                      (2)
#define DM8x03_ATB_INDEX_MAC                                       (0)

#define DM8x03_ADDRESS_TABLE_DATA_0                            0x2B1
#define DM8x03_ADDRESS_TABLE_PORT_SET                              DM8x03_ADDRESS_TABLE_DATA_0
#define DM8x03_ADDRESS_TABLE_SET_PORT(port)                        (port - 2)
#define DM8x03_ADDRESS_TABLE_GET_PORT(port)                        ((port & 0x07) + 2)
#define DM8x03_ADDRESS_TABLE_SET_MAP(port)                         (1 << (port - 2))

#define DM8x03_ADDRESS_TABLE_DATA_1                            0x2B2
#define DM8x03_ADDRESS_TABLE_INDEX_SET                             DM8x03_ADDRESS_TABLE_DATA_1
#define DM8x03_ADDRESS_TABLE_DATA_2                            0x2B3
#define DM8x03_ADDRESS_TABLE_DATA_3                            0x2B4
#define DM8x03_ADDRESS_TABLE_MULTICAST_BIT                         (0x0100)
#define DM8x03_ADDRESS_TABLE_DATA_4                            0x2B5
#define DM8x03_ADDRESS_TABLE_STATIC_AND_IGMP                       DM8x03_ADDRESS_TABLE_DATA_4
#define DM8x03_ADDRESS_TABLE_STATIC_EN                             (1 << 0)
//#define DM8x03_ADDRESS_TABLE_STATIC_EN                             ((1 << 0) | (1 << 2))
#define DM8x03_ADDRESS_TABLE_OVERRIDE_EN                           (1 << 1)
#define DM8x03_ADDRESS_TABLE_AUTH_EN                               (1 << 2)
#define DM8x03_ADDRESS_TABLE_IGMP_EN                               (1 << 12)

/*
      ATB_STATIC  = Reg2B5h.[00]     // Static Entry
      ATB_OVERRIDE= Reg2B5h.[01]     // Overriding Entry
      ATB_AUTH    = Reg2B5h.[02]     // Authorization Entry
      ATB_CVLAN   = Reg2B5h.[03]     // Cross VLAN
      ATB_MIRR    = Reg2B5h.[04]     // Mirror
      ATB_TXTAG   = Reg2B5h.[06:05]  // TX Tagging Control
      ATB_PRIEN   = Reg2B5h.[07]     // ATB_PRI Enable
      ATB_PRI     = Reg2B5h.[09:08]  // Priority Queue ID
      ATB_FILTER  = Reg2B5h.[11:10]  // Filter Control
*/


#define DM8x03_VENDOR_ID                                       0x310
#define DM8x03_VENDOR_ID_DEFAULT                                   0x0a46

#define DM8x03_PRODUCT_ID                                      0x311
#define DM8x03_PRODUCT_ID_DEFAULT                                  0x8603

#define DM8x03_PORT_2_MAC_CONTROL                              0x315
#define DM8x03_PORT_5_MAC_CONTROL_DISABLE                          0x000f

#define DM8x03_EEPROM_CONTROL_AND_ADDRESS                      0x31A
#define DM8x03_EEPROM_DATA                                     0x31B
#define DM8x03_MONITOR_REGISTER_1                              0x31C

#define DM8x03_SERIAL_BUS_ERROR_CHECK REGISTER                 0x339

#define DM8x03_PHY_CONTROL_TEST                                0x33E


#define DM8x03_PORTn_STP_STATE(port)                               DM8x03_PORTn_SPANNING_TREE_STATE_CONTROL(port)
#define DM8x03_PORTn_STP_STATE_MASK                                0x03

#define DM8x03_PORTn_STP_STATE_FORWARDING                          0x00
#define DM8x03_PORTn_STP_STATE_DISABLED                            0x01
#define DM8x03_PORTn_STP_STATE_LEARNING                            0x02
#define DM8x03_PORTn_STP_STATE_BLOCKING                            0x03
#define DM8x03_PORTn_STP_STATE_LISTENING                           0x03
#define DM8x03_PORTn_STP_STATE_BLOCKING_LISTENING                  0x03

#define DM8x03_PORTn_RSTP_STATE_FORWARDING                         0x00
#define DM8x03_PORTn_RSTP_STATE_DISCARDING                         0x01
#define DM8x03_PORTn_RSTP_STATE_LEARNING                           0x02

#define DM8x03_PORTn_AGING_DISABLE                                 (1 << 13)
#define DM8x03_PORTn_PRT_NUM(port)                                 (port -2)
#define DM8x03_PORTn_PORT_STATUS_LINK                              (1 << 0)
#define DM8x03_PORTn_PORT_STATUS_FULL                              (1 << 1)
#define DM8x03_PORTn_PORT_STATUS_100M                              (1 << 2)
#define DM8x03_PORTn_PORT_UNKNOWN_MULTICAST_DIS                    (1 << 7)

#define DM8x03_SPECIAL_TAG_MAP_EN                                  (1 << 6)
#define DM8x03_SPECIAL_TAG_MAP_TX_PORT(port)                       (1 << (port - 2))
#define DM8x03_SPECIAL_TAG_MAP_RX_PORT(port)                       (port + 2)
#define DM8x03_SPECIAL_TAG_UNTAG                                   (2 << 0)

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//DM8x03 Ethernet switch driver
extern const SwitchDriver dm8x03SwitchDriver;

//DM8x03 related functions
error_t dm8x03Init(NetInterface *interface);
void dm8x03InitHook(NetInterface *interface);

void dm8x03Tick(NetInterface *interface);

void dm8x03EnableIrq(NetInterface *interface);
void dm8x03DisableIrq(NetInterface *interface);

void dm8x03EventHandler(NetInterface *interface);

error_t dm8x03TagFrame(NetInterface *interface, NetBuffer *buffer, size_t *offset, NetTxAncillary *ancillary);

error_t dm8x03UntagFrame(NetInterface *interface, uint8_t **frame, size_t *length, NetRxAncillary *ancillary);

bool_t dm8x03GetLinkState(NetInterface *interface, uint8_t port);
uint32_t dm8x03GetLinkSpeed(NetInterface *interface, uint8_t port);
NicDuplexMode dm8x03GetDuplexMode(NetInterface *interface, uint8_t port);

void dm8x03SetPortState(NetInterface *interface, uint8_t port, SwitchPortState state);

SwitchPortState dm8x03GetPortState(NetInterface *interface, uint8_t port);

void dm8x03SetAgingTime(NetInterface *interface, uint32_t agingTime);

void dm8x03EnableIgmpSnooping(NetInterface *interface, bool_t enable);
void dm8x03EnableMldSnooping(NetInterface *interface, bool_t enable);
void dm8x03EnableRsvdMcastTable(NetInterface *interface, bool_t enable);

error_t dm8x03AddStaticFdbEntry(NetInterface *interface, const SwitchFdbEntry *entry);

error_t dm8x03DeleteStaticFdbEntry(NetInterface *interface, const SwitchFdbEntry *entry);

error_t dm8x03GetStaticFdbEntry(NetInterface *interface, uint_t index, SwitchFdbEntry *entry);

void dm8x03FlushStaticFdbTable(NetInterface *interface);

error_t dm8x03GetDynamicFdbEntry(NetInterface *interface, uint_t index, SwitchFdbEntry *entry);

void dm8x03FlushDynamicFdbTable(NetInterface *interface, uint8_t port);

void dm8x03SetUnknownMcastFwdPorts(NetInterface *interface, bool_t enable, uint32_t forwardPorts);


void dm8x03WritePhyReg(NetInterface *interface, uint8_t port, uint8_t address, uint16_t data);

uint16_t dm8x03ReadPhyReg(NetInterface *interface, uint8_t port, uint8_t address);

void dm8x03DumpPhyReg(NetInterface *interface, uint8_t port);

void dm8x03WriteSwitchReg(NetInterface *interface, uint16_t address, uint16_t data);
uint16_t dm8x03ReadSwitchReg(NetInterface *interface, uint16_t address);

uint16_t dm8x03_ATP_Wait_busy(NetInterface *interface);
uint8_t dm8x03Port_Map_2_Port_Num(uint8_t port_map);
uint8_t dm8x03Port_Num_2_Port_Map(uint8_t port_num);


//C++ guard
#ifdef __cplusplus
}
#endif

#endif
