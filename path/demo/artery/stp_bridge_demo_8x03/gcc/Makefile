RESULT ?= http_server_demo

DEFINES = \
	-DAT32F437ZMT7 \
	-DAT_START_F437_V1 \
	-D_WINSOCK_H \
	-D__error_t_defined

INCLUDES = \
	-I../src \
	-I../../../../../third_party/cmsis/include \
	-I../../../../../third_party/artery/devices/at32f435_437 \
	-I../../../../../third_party/artery/devices/at32f435_437/drivers/inc \
	-I../../../../../third_party/artery/boards/at32f435_437_board \
	-I../../../../../third_party/freertos/include \
	-I../../../../../third_party/freertos/portable/gcc/arm_cm4f \
	-I../../../../../common \
	-I../../../../../cyclone_tcp

ASM_SOURCES = \
	./startup_at32f435_437.S

C_SOURCES = \
	../src/at32f435_437_clock.c \
	../src/at32f435_437_int.c \
	../src/system_at32f435_437.c \
	./syscalls.c \
	../src/main.c \
	../src/debug.c \
	../src/res.c \
	../../../../../common/cpu_endian.c \
	../../../../../common/os_port_freertos.c \
	../../../../../common/date_time.c \
	../../../../../common/str.c \
	../../../../../common/path.c \
	../../../../../common/resource_manager.c \
	../../../../../cyclone_tcp/core/net.c \
	../../../../../cyclone_tcp/core/net_mem.c \
	../../../../../cyclone_tcp/core/net_misc.c \
	../../../../../cyclone_tcp/drivers/mac/at32f4xx_eth_driver.c \
	../../../../../cyclone_tcp/drivers/phy/dm9162_driver.c \
	../../../../../cyclone_tcp/core/nic.c \
	../../../../../cyclone_tcp/core/ethernet.c \
	../../../../../cyclone_tcp/core/ethernet_misc.c \
	../../../../../cyclone_tcp/ipv4/arp.c \
	../../../../../cyclone_tcp/ipv4/arp_cache.c \
	../../../../../cyclone_tcp/ipv4/ipv4.c \
	../../../../../cyclone_tcp/ipv4/ipv4_frag.c \
	../../../../../cyclone_tcp/ipv4/ipv4_misc.c \
	../../../../../cyclone_tcp/ipv4/icmp.c \
	../../../../../cyclone_tcp/igmp/igmp_host.c \
	../../../../../cyclone_tcp/igmp/igmp_host_misc.c \
	../../../../../cyclone_tcp/igmp/igmp_common.c \
	../../../../../cyclone_tcp/ipv6/ipv6.c \
	../../../../../cyclone_tcp/ipv6/ipv6_frag.c \
	../../../../../cyclone_tcp/ipv6/ipv6_misc.c \
	../../../../../cyclone_tcp/ipv6/ipv6_pmtu.c \
	../../../../../cyclone_tcp/ipv6/icmpv6.c \
	../../../../../cyclone_tcp/ipv6/mld.c \
	../../../../../cyclone_tcp/ipv6/ndp.c \
	../../../../../cyclone_tcp/ipv6/ndp_cache.c \
	../../../../../cyclone_tcp/ipv6/ndp_misc.c \
	../../../../../cyclone_tcp/ipv6/slaac.c \
	../../../../../cyclone_tcp/ipv6/slaac_misc.c \
	../../../../../cyclone_tcp/core/ip.c \
	../../../../../cyclone_tcp/core/tcp.c \
	../../../../../cyclone_tcp/core/tcp_fsm.c \
	../../../../../cyclone_tcp/core/tcp_misc.c \
	../../../../../cyclone_tcp/core/tcp_timer.c \
	../../../../../cyclone_tcp/core/udp.c \
	../../../../../cyclone_tcp/core/socket.c \
	../../../../../cyclone_tcp/core/socket_misc.c \
	../../../../../cyclone_tcp/core/bsd_socket.c \
	../../../../../cyclone_tcp/core/bsd_socket_options.c \
	../../../../../cyclone_tcp/core/bsd_socket_misc.c \
	../../../../../cyclone_tcp/core/raw_socket.c \
	../../../../../cyclone_tcp/dns/dns_cache.c \
	../../../../../cyclone_tcp/dns/dns_client.c \
	../../../../../cyclone_tcp/dns/dns_common.c \
	../../../../../cyclone_tcp/dns/dns_debug.c \
	../../../../../cyclone_tcp/mdns/mdns_client.c \
	../../../../../cyclone_tcp/mdns/mdns_responder.c \
	../../../../../cyclone_tcp/mdns/mdns_responder_misc.c \
	../../../../../cyclone_tcp/mdns/mdns_common.c \
	../../../../../cyclone_tcp/netbios/nbns_client.c \
	../../../../../cyclone_tcp/netbios/nbns_responder.c \
	../../../../../cyclone_tcp/netbios/nbns_common.c \
	../../../../../cyclone_tcp/llmnr/llmnr_client.c \
	../../../../../cyclone_tcp/llmnr/llmnr_responder.c \
	../../../../../cyclone_tcp/llmnr/llmnr_common.c \
	../../../../../cyclone_tcp/dhcp/dhcp_client.c \
	../../../../../cyclone_tcp/dhcp/dhcp_client_fsm.c \
	../../../../../cyclone_tcp/dhcp/dhcp_client_misc.c \
	../../../../../cyclone_tcp/dhcp/dhcp_common.c \
	../../../../../cyclone_tcp/dhcp/dhcp_debug.c \
	../../../../../cyclone_tcp/http/http_server.c \
	../../../../../cyclone_tcp/http/http_server_auth.c \
	../../../../../cyclone_tcp/http/http_server_misc.c \
	../../../../../cyclone_tcp/http/mime.c \
	../../../../../cyclone_tcp/http/ssi.c \
	../../../../../third_party/freertos/portable/gcc/arm_cm4f/port.c \
	../../../../../third_party/freertos/croutine.c \
	../../../../../third_party/freertos/list.c \
	../../../../../third_party/freertos/queue.c \
	../../../../../third_party/freertos/tasks.c \
	../../../../../third_party/freertos/timers.c \
	../../../../../third_party/freertos/portable/memmang/heap_3.c \
	../../../../../third_party/artery/boards/at32f435_437_board/at32f435_437_board.c \
	../../../../../third_party/artery/devices/at32f435_437/drivers/src/at32f435_437_acc.c \
	../../../../../third_party/artery/devices/at32f435_437/drivers/src/at32f435_437_adc.c \
	../../../../../third_party/artery/devices/at32f435_437/drivers/src/at32f435_437_can.c \
	../../../../../third_party/artery/devices/at32f435_437/drivers/src/at32f435_437_crc.c \
	../../../../../third_party/artery/devices/at32f435_437/drivers/src/at32f435_437_crm.c \
	../../../../../third_party/artery/devices/at32f435_437/drivers/src/at32f435_437_dac.c \
	../../../../../third_party/artery/devices/at32f435_437/drivers/src/at32f435_437_debug.c \
	../../../../../third_party/artery/devices/at32f435_437/drivers/src/at32f435_437_dma.c \
	../../../../../third_party/artery/devices/at32f435_437/drivers/src/at32f435_437_dvp.c \
	../../../../../third_party/artery/devices/at32f435_437/drivers/src/at32f435_437_edma.c \
	../../../../../third_party/artery/devices/at32f435_437/drivers/src/at32f435_437_emac.c \
	../../../../../third_party/artery/devices/at32f435_437/drivers/src/at32f435_437_ertc.c \
	../../../../../third_party/artery/devices/at32f435_437/drivers/src/at32f435_437_exint.c \
	../../../../../third_party/artery/devices/at32f435_437/drivers/src/at32f435_437_flash.c \
	../../../../../third_party/artery/devices/at32f435_437/drivers/src/at32f435_437_gpio.c \
	../../../../../third_party/artery/devices/at32f435_437/drivers/src/at32f435_437_i2c.c \
	../../../../../third_party/artery/devices/at32f435_437/drivers/src/at32f435_437_misc.c \
	../../../../../third_party/artery/devices/at32f435_437/drivers/src/at32f435_437_pwc.c \
	../../../../../third_party/artery/devices/at32f435_437/drivers/src/at32f435_437_qspi.c \
	../../../../../third_party/artery/devices/at32f435_437/drivers/src/at32f435_437_scfg.c \
	../../../../../third_party/artery/devices/at32f435_437/drivers/src/at32f435_437_sdio.c \
	../../../../../third_party/artery/devices/at32f435_437/drivers/src/at32f435_437_spi.c \
	../../../../../third_party/artery/devices/at32f435_437/drivers/src/at32f435_437_tmr.c \
	../../../../../third_party/artery/devices/at32f435_437/drivers/src/at32f435_437_usart.c \
	../../../../../third_party/artery/devices/at32f435_437/drivers/src/at32f435_437_usb.c \
	../../../../../third_party/artery/devices/at32f435_437/drivers/src/at32f435_437_wdt.c \
	../../../../../third_party/artery/devices/at32f435_437/drivers/src/at32f435_437_wwdt.c \
	../../../../../third_party/artery/devices/at32f435_437/drivers/src/at32f435_437_xmc.c

HEADERS = \
	../src/os_port_config.h \
	../src/net_config.h \
	../src/FreeRTOSConfig.h \
	../src/at32f435_437_conf.h \
	../src/at32f435_437_clock.h \
	../src/at32f435_437_int.h \
	../../../../../common/cpu_endian.h \
	../../../../../common/os_port.h \
	../../../../../common/os_port_freertos.h \
	../../../../../common/date_time.h \
	../../../../../common/str.h \
	../../../../../common/path.h \
	../../../../../common/resource_manager.h \
	../../../../../common/error.h \
	../../../../../common/debug.h \
	../../../../../cyclone_tcp/core/net.h \
	../../../../../cyclone_tcp/core/net_mem.h \
	../../../../../cyclone_tcp/core/net_misc.h \
	../../../../../cyclone_tcp/drivers/mac/at32f4xx_eth_driver.h \
	../../../../../cyclone_tcp/drivers/phy/dm9162_driver.h \
	../../../../../cyclone_tcp/core/nic.h \
	../../../../../cyclone_tcp/core/ethernet.h \
	../../../../../cyclone_tcp/core/ethernet_misc.h \
	../../../../../cyclone_tcp/ipv4/arp.h \
	../../../../../cyclone_tcp/ipv4/arp_cache.h \
	../../../../../cyclone_tcp/ipv4/ipv4.h \
	../../../../../cyclone_tcp/ipv4/ipv4_frag.h \
	../../../../../cyclone_tcp/ipv4/ipv4_misc.h \
	../../../../../cyclone_tcp/ipv4/icmp.h \
	../../../../../cyclone_tcp/igmp/igmp_host.h \
	../../../../../cyclone_tcp/igmp/igmp_host_misc.h \
	../../../../../cyclone_tcp/igmp/igmp_common.h \
	../../../../../cyclone_tcp/ipv6/ipv6.h \
	../../../../../cyclone_tcp/ipv6/ipv6_frag.h \
	../../../../../cyclone_tcp/ipv6/ipv6_misc.h \
	../../../../../cyclone_tcp/ipv6/ipv6_pmtu.h \
	../../../../../cyclone_tcp/ipv6/icmpv6.h \
	../../../../../cyclone_tcp/ipv6/mld.h \
	../../../../../cyclone_tcp/ipv6/ndp.h \
	../../../../../cyclone_tcp/ipv6/ndp_cache.h \
	../../../../../cyclone_tcp/ipv6/ndp_misc.h \
	../../../../../cyclone_tcp/ipv6/slaac.h \
	../../../../../cyclone_tcp/ipv6/slaac_misc.h \
	../../../../../cyclone_tcp/core/ip.h \
	../../../../../cyclone_tcp/core/tcp.h \
	../../../../../cyclone_tcp/core/tcp_fsm.h \
	../../../../../cyclone_tcp/core/tcp_misc.h \
	../../../../../cyclone_tcp/core/tcp_timer.h \
	../../../../../cyclone_tcp/core/udp.h \
	../../../../../cyclone_tcp/core/socket.h \
	../../../../../cyclone_tcp/core/socket_misc.h \
	../../../../../cyclone_tcp/core/bsd_socket.h \
	../../../../../cyclone_tcp/core/bsd_socket_options.h \
	../../../../../cyclone_tcp/core/bsd_socket_misc.h \
	../../../../../cyclone_tcp/core/raw_socket.h \
	../../../../../cyclone_tcp/dns/dns_cache.h \
	../../../../../cyclone_tcp/dns/dns_client.h \
	../../../../../cyclone_tcp/dns/dns_common.h \
	../../../../../cyclone_tcp/dns/dns_debug.h \
	../../../../../cyclone_tcp/mdns/mdns_client.h \
	../../../../../cyclone_tcp/mdns/mdns_responder.h \
	../../../../../cyclone_tcp/mdns/mdns_responder_misc.h \
	../../../../../cyclone_tcp/mdns/mdns_common.h \
	../../../../../cyclone_tcp/netbios/nbns_client.h \
	../../../../../cyclone_tcp/netbios/nbns_responder.h \
	../../../../../cyclone_tcp/netbios/nbns_common.h \
	../../../../../cyclone_tcp/llmnr/llmnr_client.h \
	../../../../../cyclone_tcp/llmnr/llmnr_responder.h \
	../../../../../cyclone_tcp/llmnr/llmnr_common.h \
	../../../../../cyclone_tcp/dhcp/dhcp_client.h \
	../../../../../cyclone_tcp/dhcp/dhcp_client_fsm.h \
	../../../../../cyclone_tcp/dhcp/dhcp_client_misc.h \
	../../../../../cyclone_tcp/dhcp/dhcp_common.h \
	../../../../../cyclone_tcp/dhcp/dhcp_debug.h \
	../../../../../cyclone_tcp/http/http_server.h \
	../../../../../cyclone_tcp/http/http_server_auth.h \
	../../../../../cyclone_tcp/http/http_server_misc.h \
	../../../../../cyclone_tcp/http/mime.h \
	../../../../../cyclone_tcp/http/ssi.h \
	../../../../../third_party/freertos/portable/gcc/arm_cm4f/portmacro.h \
	../../../../../third_party/freertos/include/croutine.h \
	../../../../../third_party/freertos/include/FreeRTOS.h \
	../../../../../third_party/freertos/include/list.h \
	../../../../../third_party/freertos/include/mpu_wrappers.h \
	../../../../../third_party/freertos/include/portable.h \
	../../../../../third_party/freertos/include/projdefs.h \
	../../../../../third_party/freertos/include/queue.h \
	../../../../../third_party/freertos/include/semphr.h \
	../../../../../third_party/freertos/include/stack_macros.h \
	../../../../../third_party/freertos/include/task.h \
	../../../../../third_party/freertos/include/timers.h \
	../../../../../third_party/artery/boards/at32f435_437_board/at32f435_437_board.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_acc.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_adc.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_can.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_crc.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_crm.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_dac.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_debug.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_def.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_dma.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_dvp.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_edma.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_emac.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_ertc.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_exint.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_flash.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_gpio.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_i2c.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_misc.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_pwc.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_qspi.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_scfg.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_sdio.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_spi.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_tmr.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_usart.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_usb.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_wdt.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_wwdt.h \
	../../../../../third_party/artery/devices/at32f435_437/drivers/inc/at32f435_437_xmc.h

ASM_OBJECTS = $(patsubst %.S, %.o, $(ASM_SOURCES))

C_OBJECTS = $(patsubst %.c, %.o, $(C_SOURCES))

OBJ_DIR = obj

LINKER_SCRIPT = at32f437_flash.ld

CFLAGS += -fno-common -Wall -Os -g3
CFLAGS += -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=softfp
CFLAGS += -ffunction-sections -fdata-sections -Wl,--gc-sections
CFLAGS += $(DEFINES)
CFLAGS += $(INCLUDES)

CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
OBJDUMP = $(CROSS_COMPILE)objdump
OBJCOPY = $(CROSS_COMPILE)objcopy
SIZE = $(CROSS_COMPILE)size

THIS_MAKEFILE := $(lastword $(MAKEFILE_LIST))

all: build size

install: build size program

build: $(RESULT).elf $(RESULT).lst $(RESULT).bin $(RESULT).hex
	
$(RESULT).elf: $(ASM_OBJECTS) $(C_OBJECTS) $(HEADERS) $(LINKER_SCRIPT) $(THIS_MAKEFILE)
	$(CC) -Wl,-M=$(RESULT).map -Wl,-T$(LINKER_SCRIPT) $(CFLAGS) $(addprefix $(OBJ_DIR)/, $(notdir $(ASM_OBJECTS))) $(addprefix $(OBJ_DIR)/, $(notdir $(C_OBJECTS))) -o $@

$(ASM_OBJECTS): | $(OBJ_DIR)

$(C_OBJECTS): | $(OBJ_DIR)

$(OBJ_DIR):
	mkdir -p $@

../src/res.c: ../../../../../utils/ResourceCompiler/bin/rc
	$< ../resources $@

../../../../../utils/ResourceCompiler/bin/rc: ../../../../../utils/ResourceCompiler/main.c
	gcc $< -o $@

%.o: %.c $(HEADERS) $(THIS_MAKEFILE)
	$(CC) $(CFLAGS) -c $< -o $(addprefix $(OBJ_DIR)/, $(notdir $@))

%.o: %.S $(HEADERS) $(THIS_MAKEFILE)
	$(CC) $(CFLAGS) -c $< -o $(addprefix $(OBJ_DIR)/, $(notdir $@))

%.lst: %.elf
	$(OBJDUMP) -x -S $(RESULT).elf > $@

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

%.hex: %.elf
	$(OBJCOPY) -O ihex $< $@

size: $(RESULT).elf
	$(SIZE) $(RESULT).elf

clean:
	rm -f $(RESULT).elf
	rm -f $(RESULT).bin
	rm -f $(RESULT).map
	rm -f $(RESULT).hex
	rm -f $(RESULT).lst
	rm -f $(OBJ_DIR)/*.o
