/**
 * @file dm8x06_driver.h
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

#ifndef _DM8x06_DRIVER_H
#define _DM8x06_DRIVER_H

//Dependencies
#include "core/nic.h"

#define DEVICE_IS_DM8806                       0

//Size of of the MAC address lookup table
#define DM8x06_MAC_TABLE_SIZE                  2048

//Port identifiers
#define DM8x06_PORT_VIRTUAL                    1
#define DM8x06_PORT0                           2
#define DM8x06_PORT1                           3
#define DM8x06_PORT2                           4
#define DM8x06_PORT3                           5
#define DM8x06_PORT4                           6
#define DM8x06_PORT5                           7

//PHY address
#define DM8x06_PHY_ADDR                        0
#define DM8x06_CPU_PORT                        DM8x06_PORT5 // DM8x06_PORT4
#define DM_SWITCH_NUM_PORTS                    (DM8x06_CPU_PORT - 1)
#define DM8x06_CPU_PORT_SET(x)                 (x - 2)

//Port masks
#define DM8x06_PORT_MASK                       0x3F
#define DM8x06_PORT0_MASK                      0x01
#define DM8x06_PORT1_MASK                      0x02
#define DM8x06_PORT2_MASK                      0x04
#define DM8x06_PORT3_MASK                      0x08
#define DM8x06_PORT4_MASK                      0x10
#define DM8x06_PORT5_MASK                      0x20

//DM8x06 PHY registers
#define DM8x06_BMCR                            0x00
#define DM8x06_BMSR                            0x01
#define DM8x06_PHYID1                          0x02
#define DM8x06_PHYID2                          0x03
#define DM8x06_ANAR                            0x04
#define DM8x06_ANLPAR                          0x05
#define DM8x06_ANER                            0x06
#define DM8x06_GENERAL_REGISTERS_RANGE             0x03

//PHY Control register
#define DM8x06_BMCR_SW_RESET                   0x8000
#define DM8x06_BMCR_LOOPBACK                   0x4000
#define DM8x06_BMCR_SPEED_LSB                  0x2000
#define DM8x06_BMCR_ANEG_EN                    0x1000
#define DM8x06_BMCR_PWR_DWN                    0x0800
#define DM8x06_BMCR_ISOLATE                    0x0400
#define DM8x06_BMCR_RESTART_ANEG               0x0200
#define DM8x06_BMCR_DUPLEX                     0x0100
#define DM8x06_BMCR_COL_TEST                   0x0080
#define DM8x06_BMCR_SPEED_MSB                  0x0040

//PHY Status register
#define DM8x06_BMSR_100T4                      0x8000
#define DM8x06_BMSR_100FDX                     0x4000
#define DM8x06_BMSR_100HDX                     0x2000
#define DM8x06_BMSR_10FDX                      0x1000
#define DM8x06_BMSR_10HPX                      0x0800
#define DM8x06_BMSR_100T2FDX                   0x0400
#define DM8x06_BMSR_100T2HDX                   0x0200
#define DM8x06_BMSR_EXTD_STATUS                0x0100
#define DM8x06_BMSR_MF_PRE_SUP                 0x0040
#define DM8x06_BMSR_ANEG_DONE                  0x0020
#define DM8x06_BMSR_REMOTE_FAULT               0x0010
#define DM8x06_BMSR_ANEG_ABLE                  0x0008
#define DM8x06_BMSR_LINK                       0x0004
#define DM8x06_BMSR_JABBER_DET                 0x0002
#define DM8x06_BMSR_EXTD_REG                   0x0001

//PHY Identifier 1 register
#define DM8x06_PHYID1_OUI_MSB                  0xFFFF
#define DM8x06_PHYID1_OUI_MSB_DEFAULT              0x0181

//PHY Identifier 2 register
#define DM8x06_PHYID2_OUI_LSB                  0xFC00
#define DM8x06_PHYID2_OUI_LSB_DEFAULT              0xb800
#define DM8x06_PHYID2_MODEL_NUM                0x03F0
#define DM8x06_PHYID2_MODEL_NUM_DEFAULT            0x00b0
#define DM8x06_PHYID2_REV_NUM                      0x0001

//Auto-Negotiation Advertisement register
#define DM8x06_ANAR_ANEG_AD_NXT_PAGE           0x8000
#define DM8x06_ANAR_ACK                        0x4000
#define DM8x06_ANAR_ANEG_AD_RE_FAULT           0x2000
#define DM8x06_ANAR_ANEG_AD_PAUSE              0x0400
#define DM8x06_ANAR_ANEG_AD_100T4              0x0200
#define DM8x06_ANAR_ANEG_AD_100FDX             0x0100
#define DM8x06_ANAR_ANEG_AD_100HDX             0x0080
#define DM8x06_ANAR_ANEG_AD_10FDX              0x0040
#define DM8x06_ANAR_ANEG_AD_10HDX              0x0020
#define DM8x06_ANAR_ANEG_AD_SELECTOR           0x001F
#define DM8x06_ANAR_ANEG_AD_SELECTOR_DEFAULT   0x0001

//Link Partner Ability register
#define DM8x06_ANLPAR_LP_NXT_PAGE              0x8000
#define DM8x06_ANLPAR_LP_ACK                   0x4000
#define DM8x06_ANLPAR_LP_REMOTE_FAULT          0x2000
#define DM8x06_ANLPAR_LP_TECH_ABLE             0x1FE0
#define DM8x06_ANLPAR_LP_SELECTOR              0x001F

//Auto-Negotiation Expansion register
#define DM8x06_ANER_PAR_FAULT_DET              0x0010
#define DM8x06_ANER_LP_NXT_PG_ABLE             0x0008
#define DM8x06_ANER_LOCAL_NXT_PG_ABLE          0x0004
#define DM8x06_ANER_RX_NEW_PAGE                0x0002
#define DM8x06_ANER_LP_ANEG_ABLE               0x0001


//dm8x06 Switch Per-Port Registers
#define DM8x06_PORT_0_PORT_STATUS                              0x110
#define DM8x06_PORT_0_BASIC_CONTROL_0                          0x111
#define DM8x06_PORT_0_BASIC_CONTROL_1                          0x112
#define DM8x06_PORT_0_BLOCK_CONTROL_0                          0x113
#define DM8x06_PORT_0_BLOCK_CONTROL_1                          0x114
#define DM8x06_PORT_0_BANDWIDTH_CONTROL                        0x115
#define DM8x06_PORT_0_VLAN_TAG_INFOMATION                      0x116
#define DM8x06_PORT_0_PRIORITY_AND_VLAN_CONTROL                0x117
#define DM8x06_PORT_0_SECURITY_CONTROL                         0x118
#define DM8x06_PORT_0_SPANNING_TREE_STATE_CONTROL              0x119
#define DM8x06_PORT_0_MEMORY_CONFIGURATION                     0x11A
#define DM8x06_PORT_0_DISCARD_PACKET_LIMITATION                0x11B
#define DM8x06_PORT_0_ENERGY_EFFICIENT_ETHERNET_CONTROL        0x11E

#define DM8x06_PORTn_PORT_STATUS(port)                             (DM8x06_PORT_0_PORT_STATUS + ((port - 2) * 0x20))
#define DM8x06_PORTn_BASIC_CONTROL_0(port)                         (DM8x06_PORT_0_BASIC_CONTROL_0 + ((port - 2) * 0x20))
#define DM8x06_PORTn_BASIC_CONTROL_0_RX_DIS                        (1 << 1)
#define DM8x06_PORTn_BASIC_CONTROL_0_TX_DIS                        (1 << 0)
#define DM8x06_PORTn_BASIC_CONTROL_0_ADLRN_DIS                     (1 << 12)

#define DM8x06_PORTn_BASIC_CONTROL_1(port)                         (DM8x06_PORT_0_BASIC_CONTROL_1 + ((port - 2) * 0x20))
#define DM8x06_PORTn_BASIC_CONTROL_1_FIRUUSID_EN                   (1 << 9)

#define DM8x06_PORTn_BLOCK_CONTROL_0(port)                         (DM8x06_PORT_0_BLOCK_CONTROL_0 + ((port - 2) * 0x20))
#define DM8x06_PORTn_BLOCK_CONTROL_1(port)                         (DM8x06_PORT_0_BLOCK_CONTROL_1 + ((port - 2) * 0x20))
#define DM8x06_PORTn_BANDWIDTH_CONTROL(port)                       (DM8x06_PORT_0_BANDWIDTH_CONTROL + ((port - 2) * 0x20))
#define DM8x06_PORTn_VLAN_TAG_INFOMATION(port)                     (DM8x06_PORT_0_VLAN_TAG_INFOMATION + ((port - 2) * 0x20))
#define DM8x06_PORTn_PRIORITY_AND_VLAN_CONTROL(port)               (DM8x06_PORT_0_PRIORITY_AND_VLAN_CONTROL + ((port - 2) * 0x20))
#define DM8x06_PORTn_SECURITY_CONTROL(port)                        (DM8x06_PORT_0_SECURITY_CONTROL + ((port - 2) * 0x20))
#define DM8x06_PORTn_SPANNING_TREE_STATE_CONTROL(port)             (DM8x06_PORT_0_SPANNING_TREE_STATE_CONTROL + ((port - 2) * 0x20))
#define DM8x06_PORTn_MEMORY_CONFIGURATION(port)                    (DM8x06_PORT_0_MEMORY_CONFIGURATION + ((port - 2) * 0x20))
#define DM8x06_PORTn_DISCARD_PACKET_LIMITATION(port)               (DM8x06_PORT_0_DISCARD_PACKET_LIMITATION + ((port - 2) * 0x20))
#define DM8x06_PORTn_ENERGY_EFFICIENT_ETHERNET_CONTROL(port)       (DM8x06_PORT_0_ENERGY_EFFICIENT_ETHERNET_CONTROL + ((port - 2) * 0x20))

#define DM8x06_PORT_1_PORT_STATUS                              0x130
#define DM8x06_PORT_1_BASIC_CONTROL_0                          0x131
#define DM8x06_PORT_1_BASIC_CONTROL_1                          0x132
#define DM8x06_PORT_1_BLOCK_CONTROL_0                          0x133
#define DM8x06_PORT_1_BLOCK_CONTROL_1                          0x134
#define DM8x06_PORT_1_BANDWIDTH_CONTROL                        0x135
#define DM8x06_PORT_1_VLAN_TAG_INFOMATION                      0x136
#define DM8x06_PORT_1_PRIORITY_AND_VLAN_CONTROL                0x137
#define DM8x06_PORT_1_SECURITY_CONTROL                         0x138
#define DM8x06_PORT_1_SPANNING_TREE_STATE_CONTROL              0x139
#define DM8x06_PORT_1_MEMORY_CONFIGURATION                     0x13A
#define DM8x06_PORT_1_DISCARD_PACKET_LIMITATION                0x13B
#define DM8x06_PORT_1_ENERGY_EFFICIENT_ETHERNET_CONTROL        0x13E

#define DM8x06_PORT_2_PORT_STATUS                              0x150
#define DM8x06_PORT_2_BASIC_CONTROL_0                          0x151
#define DM8x06_PORT_2_BASIC_CONTROL_1                          0x152
#define DM8x06_PORT_2_BLOCK_CONTROL_0                          0x153
#define DM8x06_PORT_2_BLOCK_CONTROL_1                          0x154
#define DM8x06_PORT_2_BANDWIDTH_CONTROL                        0x155
#define DM8x06_PORT_2_VLAN_TAG_INFOMATION                      0x156
#define DM8x06_PORT_2_PRIORITY_AND_VLAN_CONTROL                0x157
#define DM8x06_PORT_2_SECURITY_CONTROL                         0x158
#define DM8x06_PORT_2_SPANNING_TREE_STATE_CONTROL              0x159
#define DM8x06_PORT_2_MEMORY_CONFIGURATION                     0x15A
#define DM8x06_PORT_2_DISCARD_PACKET_LIMITATION                0x15B
#define DM8x06_PORT_2_ENERGY_EFFICIENT_ETHERNET_CONTROL        0x15E

#define DM8x06_PORT_3_PORT_STATUS                              0x170
#define DM8x06_PORT_3_BASIC_CONTROL_0                          0x171
#define DM8x06_PORT_3_BASIC_CONTROL_1                          0x172
#define DM8x06_PORT_3_BLOCK_CONTROL_0                          0x173
#define DM8x06_PORT_3_BLOCK_CONTROL_1                          0x174
#define DM8x06_PORT_3_BANDWIDTH_CONTROL                        0x175
#define DM8x06_PORT_3_VLAN_TAG_INFOMATION                      0x176
#define DM8x06_PORT_3_PRIORITY_AND_VLAN_CONTROL                0x177
#define DM8x06_PORT_3_SECURITY_CONTROL                         0x178
#define DM8x06_PORT_3_SPANNING_TREE_STATE_CONTROL              0x179
#define DM8x06_PORT_3_MEMORY_CONFIGURATION                     0x17A
#define DM8x06_PORT_3_DISCARD_PACKET_LIMITATION                0x17B
#define DM8x06_PORT_3_ENERGY_EFFICIENT_ETHERNET_CONTROL        0x17E

#define DM8x06_PORT_4_PORT_STATUS                              0x190
#define DM8x06_PORT_4_BASIC_CONTROL_0                          0x191
#define DM8x06_PORT_4_BASIC_CONTROL_1                          0x192
#define DM8x06_PORT_4_BLOCK_CONTROL_0                          0x193
#define DM8x06_PORT_4_BLOCK_CONTROL_1                          0x194
#define DM8x06_PORT_4_BANDWIDTH_CONTROL                        0x195
#define DM8x06_PORT_4_VLAN_TAG_INFOMATION                      0x196
#define DM8x06_PORT_4_PRIORITY_AND_VLAN_CONTROL                0x197
#define DM8x06_PORT_4_SECURITY_CONTROL                         0x198
#define DM8x06_PORT_4_SPANNING_TREE_STATE_CONTROL              0x199
#define DM8x06_PORT_4_MEMORY_CONFIGURATION                     0x19A
#define DM8x06_PORT_4_DISCARD_PACKET_LIMITATION                0x19B
#define DM8x06_PORT_4_ENERGY_EFFICIENT_ETHERNET_CONTROL        0x19E

#define DM8x06_PORT_5_PORT_STATUS                              0x1B0
#define DM8x06_PORT_5_BASIC_CONTROL_0                          0x1B1
#define DM8x06_PORT_5_BASIC_CONTROL_1                          0x1B2
#define DM8x06_PORT_5_BLOCK_CONTROL_0                          0x1B3
#define DM8x06_PORT_5_BLOCK_CONTROL_1                          0x1B4
#define DM8x06_PORT_5_BANDWIDTH_CONTROL                        0x1B5
#define DM8x06_PORT_5_VLAN_TAG_INFOMATION                      0x1B6
#define DM8x06_PORT_5_PRIORITY_AND_VLAN_CONTROL                0x1B7
#define DM8x06_PORT_5_SECURITY_CONTROL                         0x1B8
#define DM8x06_PORT_5_SPANNING_TREE_STATE_CONTROL              0x1B9
#define DM8x06_PORT_5_MEMORY_CONFIGURATION                     0x1BA
#define DM8x06_PORT_5_DISCARD_PACKET_LIMITATION                0x1BB
#define DM8x06_PORT_5_ENERGY_EFFICIENT_ETHERNET_CONTROL        0x1BE



//dm8x06 Switch Engine Registers
#define DM8x06_SWITCH_STATUS                                   0x210
#define DM8x06 SWITCH_RESET                                    0x211
#define DM8x06_SWITCH_CONTROL                                  0x212
#define DM8x06_CPU_PORT_AND_MIRROR_CONTROL                     0x213
#define DM8x06_SPECIAL_TAG_TX_ENABLE                               (1 << 7) //to cpu
#define DM8x06_SPECIAL_TAG_RX_ENABLE                               (1 << 6) //from cpu
#define DM8x06_SPECIAL_TAG_ETHER_TYPE                          0x214
#define DM8x06_SPECIAL_TAG_ETHER_TYPE_DEFAULT                      0x8606

#define DM8x06_GLOBAL_LEARNING_AND_AGING_CONTROL               0x215
#define DM8x06_AGING_CONTROL_MASK                                  0x03
#define DM8x06_AGING_CONTROL_512SEC                                0x03
#define DM8x06_AGING_CONTROL_256SEC                                0x02
#define DM8x06_AGING_CONTROL_128SEC                                0x01
#define DM8x06_AGING_CONTROL_064SEC                                0x00
#define DM8x06_ADDRESS_TABLE_SEPARATED_MODE                        (1 << 2)

#define DM8x06_SPEIAL_PACKET_CONTROL_0                         0x234
#define DM8x06_SPEIAL_PACKET_CONTROL_1                         0x235
#define DM8x06_SPEIAL_PACKET_CONTROL_2                         0x236
#define DM8x06_SPEIAL_PACKET_CONTROL_3                         0x237
#define DM8x06_SPEIAL_PACKET_CONTROL_4                         0x238
#define DM8x06_SPEIAL_PACKET_CONTROL_5                         0x239
#define DM8x06_SPEIAL_PACKET_CONTROL_6                         0x23A
#define DM8x06_SPEIAL_PACKET_CONTROL_7                         0x23B
#define DM8x06_SPEIAL_PACKET_CONTROL_8                         0x23C

#define DM8x06_STP_CONTROL                                     0x292
#define DM8x06_STP_EN                                              (1 << 0)
#define DM8x06_STP_CONTROL_DEFAULT                                 0x00


#define DM8x06_SNOOPING_CONTROL_0                              0x29B
#define DM8x06_IGMP_SNOOPING_HW_EN                                 (1 << 0)
#define DM8x06_MLD_SNOOPING_HW_EN                                  (1 << 1)
#define DM8x06_SNOOPING_CONTROL_1                              0x29C

#define DM8x06_ADDRESS_TABLE_CONTROL_AND_STATUS                0x2B0
#define DM8x06_ATB_BUSY                                            (1 << 15)
#define DM8x06_ATB_CR_ERR                                          (1 << 14)
#define DM8x06_ATB_CR_ENTRY_DONE                                   (1 << 13)
#define DM8x06_ATB_CLEAN_PORT_EN                                   (1 << 5)
#define DM8x06_ATB_CMD_READ                                        (0 << 2)
#define DM8x06_ATB_CMD_WRITE                                       (1 << 2)
#define DM8x06_ATB_CMD_DELETE                                      (2 << 2)
#define DM8x06_ATB_CMD_SEARCH                                      (3 << 2)
#define DM8x06_ATB_CMD_CLEAN                                       (4 << 2)
#define DM8x06_ATB_INDEX_UNICAST                                   (0)
#define DM8x06_ATB_INDEX_MULTICAST                                 (1)
#define DM8x06_ATB_INDEX_IGMP                                      (2)
#define DM8x06_ATB_INDEX_MAC                                       (0)

#define DM8x06_ADDRESS_TABLE_DATA_0                            0x2B1
#define DM8x06_ADDRESS_TABLE_PORT_SET                              DM8x06_ADDRESS_TABLE_DATA_0
#define DM8x06_ADDRESS_TABLE_SET_PORT(port)                        (port - 2)
#define DM8x06_ADDRESS_TABLE_GET_PORT(port)                        ((port & 0x07) + 2)
#define DM8x06_ADDRESS_TABLE_SET_MAP(port)                         (1 << (port - 2))

#define DM8x06_ADDRESS_TABLE_DATA_1                            0x2B2
#define DM8x06_ADDRESS_TABLE_INDEX_SET                             DM8x06_ADDRESS_TABLE_DATA_1
#define DM8x06_ADDRESS_TABLE_DATA_2                            0x2B3
#define DM8x06_ADDRESS_TABLE_DATA_3                            0x2B4
#define DM8x06_ADDRESS_TABLE_MULTICAST_BIT                         (0x0100)
#define DM8x06_ADDRESS_TABLE_DATA_4                            0x2B5
#define DM8x06_ADDRESS_TABLE_STATIC_AND_IGMP                       DM8x06_ADDRESS_TABLE_DATA_4
#define DM8x06_ADDRESS_TABLE_STATIC_EN                             (1 << 0)
//#define DM8x06_ADDRESS_TABLE_STATIC_EN                             ((1 << 0) | (1 << 2))
#define DM8x06_ADDRESS_TABLE_OVERRIDE_EN                           (1 << 1)
#define DM8x06_ADDRESS_TABLE_AUTH_EN                               (1 << 2)
#define DM8x06_ADDRESS_TABLE_IGMP_EN                               (1 << 12)

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


#define DM8x06_VENDOR_ID                                       0x310
#define DM8x06_VENDOR_ID_DEFAULT                                   0x0a46

#define DM8x06_PRODUCT_ID                                      0x311
#define DM8x06_PRODUCT_ID_DEFAULT                                  0x8606

#define DM8x06_PORT_4_MAC_CONTROL                              0x314
#define DM8x06_PORT_5_MAC_CONTROL                              0x315
#define DM8x06_PORT_5_MAC_CONTROL_DISABLE                          0x000f
#define DM8x06_FIBER_CONTROL                                   0x316
#define DM8x06_IRQ_AND_LED_CONTROL                             0x317
#define DM8x06_INTERRUPT_STATUS                                0x318
#define DM8x06_INTERRUPT_MASK_AND_CONTROL                      0x319
#define DM8x06_EEPROM_CONTROL_AND_ADDRESS                      0x31A
#define DM8x06_EEPROM_DATA                                     0x31B
#define DM8x06_MONITOR_REGISTER_1                              0x31C
#define DM8x06_MONITOR_REGISTER_2                              0x31D
#define DM8x06_MONITOR_REGISTER_3                              0x31E
//#define DM8x06_MONITOR_REGISTER_4                             0x31F

#define DM8x06_SYSTEM_CLOCK_SELECT                             0x338
#define DM8x06_SERIAL_BUS_ERROR_CHECK REGISTER                 0x339

#define DM8x06_VIRTUAL_PHY_CONTROL                             0x33D
#define DM8x06_PHY_CONTROL_TEST                                0x33E


#define DM8x06_PORTn_STP_STATE(port)                               DM8x06_PORTn_SPANNING_TREE_STATE_CONTROL(port)
#define DM8x06_PORTn_STP_STATE_MASK                                0x03

#define DM8x06_PORTn_STP_STATE_FORWARDING                          0x00
#define DM8x06_PORTn_STP_STATE_DISABLED                            0x01
#define DM8x06_PORTn_STP_STATE_LEARNING                            0x02
#define DM8x06_PORTn_STP_STATE_BLOCKING                            0x03
#define DM8x06_PORTn_STP_STATE_LISTENING                           0x03
#define DM8x06_PORTn_STP_STATE_BLOCKING_LISTENING                  0x03

#define DM8x06_PORTn_RSTP_STATE_FORWARDING                         0x00
#define DM8x06_PORTn_RSTP_STATE_DISCARDING                         0x01
#define DM8x06_PORTn_RSTP_STATE_LEARNING                           0x02

#define DM8x06_PORTn_AGING_DISABLE                                 (1 << 13)
#define DM8x06_PORTn_PRT_NUM(port)                                 (port -2)
#define DM8x06_PORTn_PORT_STATUS_LINK                              (1 << 0)
#define DM8x06_PORTn_PORT_STATUS_FULL                              (1 << 1)
#define DM8x06_PORTn_PORT_STATUS_100M                              (1 << 2)
#define DM8x06_PORTn_PORT_UNKNOWN_MULTICAST_DIS                    (1 << 7)

#define DM8x06_MONITOR_REGISTER_1_VIRTUAL_PHY_EN                   (1 << 9)
#define DM8x06_VIRTUAL_PHY_CONTROL_OR_MODE                         (1 << 8)
#define DM8x06_VIRTUAL_PHY_CONTROL_MAP                             (0x0f)

#define DM8x06_SPECIAL_TAG_MAP_EN                                  (1 << 6)
#define DM8x06_SPECIAL_TAG_MAP_TX_PORT(port)                       (1 << (port - 2))
#define DM8x06_SPECIAL_TAG_MAP_RX_PORT(port)                       (port + 2)
#define DM8x06_SPECIAL_TAG_UNTAG                                   (2 << 0)

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//DM8x06 Ethernet switch driver
extern const SwitchDriver dm8x06SwitchDriver;

//DM8x06 related functions
error_t dm8x06Init(NetInterface *interface);
void dm8x06InitHook(NetInterface *interface);

void dm8x06Tick(NetInterface *interface);

void dm8x06EnableIrq(NetInterface *interface);
void dm8x06DisableIrq(NetInterface *interface);

void dm8x06EventHandler(NetInterface *interface);

error_t dm8x06TagFrame(NetInterface *interface, NetBuffer *buffer, size_t *offset, NetTxAncillary *ancillary);

error_t dm8x06UntagFrame(NetInterface *interface, uint8_t **frame, size_t *length, NetRxAncillary *ancillary);

bool_t dm8x06GetLinkState(NetInterface *interface, uint8_t port);
uint32_t dm8x06GetLinkSpeed(NetInterface *interface, uint8_t port);
NicDuplexMode dm8x06GetDuplexMode(NetInterface *interface, uint8_t port);

void dm8x06SetPortState(NetInterface *interface, uint8_t port, SwitchPortState state);

SwitchPortState dm8x06GetPortState(NetInterface *interface, uint8_t port);

void dm8x06SetAgingTime(NetInterface *interface, uint32_t agingTime);

void dm8x06EnableIgmpSnooping(NetInterface *interface, bool_t enable);
void dm8x06EnableMldSnooping(NetInterface *interface, bool_t enable);
void dm8x06EnableRsvdMcastTable(NetInterface *interface, bool_t enable);

error_t dm8x06AddStaticFdbEntry(NetInterface *interface, const SwitchFdbEntry *entry);

error_t dm8x06DeleteStaticFdbEntry(NetInterface *interface, const SwitchFdbEntry *entry);

error_t dm8x06GetStaticFdbEntry(NetInterface *interface, uint_t index, SwitchFdbEntry *entry);

void dm8x06FlushStaticFdbTable(NetInterface *interface);

error_t dm8x06GetDynamicFdbEntry(NetInterface *interface, uint_t index, SwitchFdbEntry *entry);

void dm8x06FlushDynamicFdbTable(NetInterface *interface, uint8_t port);

void dm8x06SetUnknownMcastFwdPorts(NetInterface *interface, bool_t enable, uint32_t forwardPorts);


void dm8x06WritePhyReg(NetInterface *interface, uint8_t port, uint8_t address, uint16_t data);

uint16_t dm8x06ReadPhyReg(NetInterface *interface, uint8_t port, uint8_t address);

void dm8x06DumpPhyReg(NetInterface *interface, uint8_t port);

void dm8x06WriteSwitchReg(NetInterface *interface, uint16_t address, uint16_t data);
uint16_t dm8x06ReadSwitchReg(NetInterface *interface, uint16_t address);

uint16_t dm8x06_ATP_Wait_busy(NetInterface *interface);
uint8_t dm8x06Port_Map_2_Port_Num(uint8_t port_map);
uint8_t dm8x06Port_Num_2_Port_Map(uint8_t port_num);


//C++ guard
#ifdef __cplusplus
}
#endif

#endif
