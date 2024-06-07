#include "gpio.h"

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <linux/gpio.h>

#include "terminal.h"

static bool GPIO_MAPPED = false;
uint32_t* BCM2712_PERI_BASE; 


static GPIO_Function GPIO_PIN_FUNCTIONS[28][9] = {
    {GPIO_SPI0_SIO,  GPIO_DPI_PCLK,  GPIO_UART1_TX,  	GPIO_I2C0_SDA, 	    GPIO_NOFUNC, 	  GPIO_SYS_RIO, GPIO_PROC_RIO, GPIO_PIO, GPIO_SPI2_CSn },
    {GPIO_SPI0_SIO,  GPIO_DPI_DE, 	 GPIO_UART1_RX,  	GPIO_NOFUNC, 	    GPIO_I2C0_SCL, 	  GPIO_SYS_RIO, GPIO_PROC_RIO, GPIO_PIO, GPIO_SPI2_SIO },
    {GPIO_SPI0_CSn,  GPIO_DPI_VSYNC, GPIO_UART1_CTS, 	GPIO_I2C1_SDA, 	    GPIO_UART0_IR_RX, GPIO_SYS_RIO, GPIO_PROC_RIO, GPIO_PIO, GPIO_SPI2_SIO },
    {GPIO_SPI0_CSn,  GPIO_DPI_HSYNC, GPIO_UART1_RTS, 	GPIO_I2C1_SCL, 	    GPIO_UART0_IR_TX, GPIO_SYS_RIO, GPIO_PROC_RIO, GPIO_PIO, GPIO_SPI2_SCLK },
    {GPIO_GPCLK_0, 	 GPIO_DPI_D, 	 GPIO_UART2_TX,  	GPIO_I2C2_SDA, 	    GPIO_UART0_RI, 	  GPIO_SYS_RIO, GPIO_PROC_RIO, GPIO_PIO, GPIO_SPI3_CSn },
    {GPIO_GPCLK_0, 	 GPIO_DPI_D, 	 GPIO_UART2_RX,  	GPIO_I2C2_SCL, 	    GPIO_UART0_DTR,   GPIO_SYS_RIO, GPIO_PROC_RIO, GPIO_PIO, GPIO_SPI3_SIO },
    {GPIO_GPCLK_0, 	 GPIO_DPI_D, 	 GPIO_UART2_CTS, 	GPIO_I2C3_SDA, 	    GPIO_UART0_DCD,   GPIO_SYS_RIO, GPIO_PROC_RIO, GPIO_PIO, GPIO_SPI3_SIO },
    {GPIO_SPI0_CSn,  GPIO_DPI_D, 	 GPIO_UART2_RTS, 	GPIO_I2C3_SCL, 	    GPIO_UART0_DSR,   GPIO_SYS_RIO, GPIO_PROC_RIO, GPIO_PIO, GPIO_SPI3_SCLK },
    {GPIO_SPI0_CSn,  GPIO_DPI_D, 	 GPIO_UART3_TX,  	GPIO_I2C0_SDA, 	    GPIO_NOFUNC,	  GPIO_SYS_RIO, GPIO_PROC_RIO, GPIO_PIO, GPIO_SPI4_CSn },
    {GPIO_SPI0_SIO,  GPIO_DPI_D, 	 GPIO_UART3_RX,  	GPIO_I2C0_SCL, 	    GPIO_NOFUNC,	  GPIO_SYS_RIO, GPIO_PROC_RIO, GPIO_PIO, GPIO_SPI4_SIO },
    {GPIO_SPI0_SIO,  GPIO_DPI_D, 	 GPIO_UART3_CTS, 	GPIO_I2C1_SDA, 	    GPIO_NOFUNC,	  GPIO_SYS_RIO, GPIO_PROC_RIO, GPIO_PIO, GPIO_SPI4_SIO },
    {GPIO_SPI0_SCLK, GPIO_DPI_D, 	 GPIO_UART3_RTS, 	GPIO_I2C1_SCL, 	    GPIO_NOFUNC,	  GPIO_SYS_RIO, GPIO_PROC_RIO, GPIO_PIO, GPIO_SPI4_SCLK },
    {GPIO_PWM0_0,	 GPIO_DPI_D, 	 GPIO_UART4_TX,  	GPIO_I2C2_SDA, 	    GPIO_AUDIO_OUT_L, GPIO_SYS_RIO, GPIO_PROC_RIO, GPIO_PIO, GPIO_SPI5_CSn},
    {GPIO_PWM0_0, 	 GPIO_DPI_D, 	 GPIO_UART4_RX,  	GPIO_I2C2_SCL, 	    GPIO_AUDIO_OUT_R, GPIO_SYS_RIO, GPIO_PROC_RIO, GPIO_PIO, GPIO_SPI5_SIO},
    {GPIO_PWM0_0, 	 GPIO_DPI_D, 	 GPIO_UART4_CTS, 	GPIO_I2C3_SDA, 	    GPIO_UART0_TX, 	  GPIO_SYS_RIO, GPIO_PROC_RIO, GPIO_PIO, GPIO_SPI5_SIO},
    {GPIO_PWM0_0, 	 GPIO_DPI_D, 	 GPIO_UART4_RTS, 	GPIO_I2C3_SCL, 	    GPIO_UART0_RX, 	  GPIO_SYS_RIO, GPIO_PROC_RIO, GPIO_PIO, GPIO_SPI5_SCLK},
    {GPIO_SPI1_CSn,  GPIO_DPI_D, 	 GPIO_MIPI0_DSI_TE, GPIO_NOFUNC, 	    GPIO_UART0_CTS,   GPIO_SYS_RIO, GPIO_PROC_RIO, GPIO_PIO, GPIO_NOFUNC },
    {GPIO_SPI1_CSn,  GPIO_DPI_D, 	 GPIO_MIPI1_DSI_TE, GPIO_NOFUNC, 	    GPIO_UART0_RTS,   GPIO_SYS_RIO, GPIO_PROC_RIO, GPIO_PIO, GPIO_NOFUNC },
    {GPIO_SPI1_CSn,  GPIO_DPI_D, 	 GPIO_I2S0_SCLK, 	GPIO_PWM0_0, 	    GPIO_I2S1_SCLK,   GPIO_SYS_RIO, GPIO_PROC_RIO, GPIO_PIO, GPIO_GPCLK_2},
    {GPIO_SPI1_SIO,  GPIO_DPI_D, 	 GPIO_I2S0_WS, 		GPIO_PWM0_0, 	    GPIO_I2S1_WS, 	  GPIO_SYS_RIO, GPIO_PROC_RIO, GPIO_PIO, GPIO_NOFUNC},
    {GPIO_SPI1_SIO,  GPIO_DPI_D, 	 GPIO_I2S0_SDI, 	GPIO_GPCLK_1, 	    GPIO_I2S1_SDI, 	  GPIO_SYS_RIO, GPIO_PROC_RIO, GPIO_PIO, GPIO_NOFUNC},
    {GPIO_SPI1_SCLK, GPIO_DPI_D, 	 GPIO_I2S0_SDO, 	GPIO_GPCLK_1, 	    GPIO_I2S1_SDO, 	  GPIO_SYS_RIO, GPIO_PROC_RIO, GPIO_PIO, GPIO_NOFUNC},
    {GPIO_SDIO0_CLK, GPIO_DPI_D,	 GPIO_I2S0_SDI, 	GPIO_I2C3_SDA,	    GPIO_I2S1_SDI,	  GPIO_SYS_RIO, GPIO_PROC_RIO, GPIO_PIO, GPIO_NOFUNC},
    {GPIO_SDIO0_CMD, GPIO_DPI_D,	 GPIO_I2S0_SDO, 	GPIO_I2C3_SCL,	    GPIO_I2S1_SDO,	  GPIO_SYS_RIO, GPIO_PROC_RIO, GPIO_PIO, GPIO_NOFUNC},
    {GPIO_SDIO0_DAT,  GPIO_DPI_D, 	 GPIO_I2S0_SDI, 	GPIO_NOFUNC,	    GPIO_I2S1_SDI, 	  GPIO_SYS_RIO, GPIO_PROC_RIO, GPIO_PIO, GPIO_SPI2_CSn},
    {GPIO_SDIO0_DAT,  GPIO_DPI_D, 	 GPIO_I2S0_SDO, 	GPIO_AUDIO_IN_CLK,  GPIO_I2S1_SDO, 	  GPIO_SYS_RIO, GPIO_PROC_RIO, GPIO_PIO, GPIO_SPI3_CSn},
    {GPIO_SDIO0_DAT,  GPIO_DPI_D, 	 GPIO_I2S0_SDI, 	GPIO_AUDIO_IN_DAT0, GPIO_I2S1_SDI, 	  GPIO_SYS_RIO, GPIO_PROC_RIO, GPIO_PIO, GPIO_SPI5_CSn},
    {GPIO_SDIO0_DAT,  GPIO_DPI_D, 	 GPIO_I2S0_SDO, 	GPIO_AUDIO_IN_DAT1, GPIO_I2S1_SDO, 	  GPIO_SYS_RIO, GPIO_PROC_RIO, GPIO_PIO, GPIO_NOFUNC}
};

bool GPIO_MAP() {
	int mem_fd;
	unsigned int *map;

	if(GPIO_MAPPED)
		return false;

	/* if we run as root */
	if(geteuid() == 0) {
		if((mem_fd = open("/dev/mem", O_RDWR | O_SYNC) ) < 0) {
			printf(ESC_RED_BOLD "Failed to open /dev/mem\n" ESC_WHITE);
			return false;
		}

		map = mmap(
			NULL,
			64 * 1024 * 1024,
			PROT_READ | PROT_WRITE,
			MAP_SHARED,
			mem_fd,
			0x1f00000000	
		);

		close(mem_fd);
		if(map == MAP_FAILED){
			printf(ESC_RED_BOLD "Error mapping GPIO: %lld\n" ESC_WHITE, (long long)map);
			return false;
		}

		BCM2712_PERI_BASE = (uint32_t*)map;
	} else { // If we run in user mode
		if((mem_fd = open("/dev/gpiomem0", O_RDWR | O_SYNC) ) < 0) {
			printf(ESC_RED_BOLD "Failed to open /dev/gpiomem0\n" ESC_WHITE);
			return false;
		}

		map = mmap(
			NULL,
			0x30000,
			PROT_READ | PROT_WRITE,
			MAP_SHARED,
			mem_fd,
			0x0
		);

		close(mem_fd);
		if(map == MAP_FAILED){
			printf(ESC_RED_BOLD "Error mapping GPIO: %lld\n" ESC_WHITE, (long long)map);
			return false;
		}

		/* Since we don't want to mess up our macros, we offset the 
		 * base, but that also means that we should never use the 
		 * base directly as an address in user mode */
		BCM2712_PERI_BASE = PTROFFSET(map, -0xD0000);
	}

	GPIO_MAPPED = true;

	return true;
}


GPIO_ERROR GPIO_FUNCTION(const uint8_t pin, GPIO_Function function) {
	if(pin > GPIO_MAX_PIN)
		return GPIO_INVALID_PIN;

	if(GPIO_PIN_FUNCTIONS[pin][function & 0xF] != function)
		return GPIO_INVALID_FUNCTION;

	GPIO[pin].ctrl.FUNCSEL = function & 0xF;

	return GPIO_SUCCESS;
}

void GPIO_RESET_PIN(const uint8_t pin) {
	if(pin > GPIO_MAX_PIN)
		return;

	if(GPIO[pin].ctrl.FUNCSEL == GPIO_SYS_RIO) {
		RIO_CLR->OE = PIN(pin);
		RIO_CLR->OUT = PIN(pin);
	}

	GPIO_ctrl pin_reset = {0};
	pin_reset.FUNCSEL = 0x1f;
	pin_reset.F_M = 0x4;
	GPIO[pin].ctrl = pin_reset;

	PAD[pin] = (PAD_register){0, 1, 0, 0, 1, 0, 1, 0};
}

void GPIO_CLEANUP() {
	/* Pins 0 and 1 are "reserved for advanced use". Don't reset them. */
	for(int i = 2; i < GPIO_MAX_PIN; i++)
		GPIO_RESET_PIN(i);
}
