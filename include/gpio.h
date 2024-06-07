#ifndef _GPIO_H
#define _GPIO_H

// ----------------------------------------------------------------------------------------------------------------------------------
// REFERENCE: https://www.i-programmer.info/programming/148-hardware/16887-raspberry-pi-iot-in-c-pi-5-memory-mapped-gpio.html?start=2
//
// ALSO SEE RP1-PERIPHERALS.PDF IN DOCS FOLDER!!!
// ----------------------------------------------------------------------------------------------------------------------------------

#include <stdint.h>

#define PTROFFSET(ptr, offset) 	((typeof(ptr))(((uintptr_t) ptr) + ((intptr_t)offset)))

#define GPIO_BASE        PTROFFSET(BCM2712_PERI_BASE, 0xD0000)
#define RIO_BASE 		 PTROFFSET(BCM2712_PERI_BASE, 0xE0000)
#define PAD_BASE		 PTROFFSET(BCM2712_PERI_BASE, 0xF0000) 

/* For registers which support atomic aliases, see the docs */
#define ATOMIC_XOR(register, pin) PTROFFSET(register, 0x1000) = (1 << pin)
#define ATOMIC_SET(register, pin) PTROFFSET(register, 0x2000) = (1 << pin)
#define ATOMIC_CLR(register, pin) PTROFFSET(register, 0x3000) = (1 << pin)



/* GPIO */
#define GPIO ((GPIO_register*) GPIO_BASE)
#define GPIO_MAX_PIN 27
#define PIN(pin)	(1 << pin)


/* Errors */
typedef uint8_t GPIO_ERROR;

#define GPIO_SUCCESS			0x00
#define GPIO_NOT_MAPPED 		0x01
#define GPIO_INVALID_PIN		0x02
#define GPIO_PIN_WRONG_MODE		0x03
#define GPIO_INVALID_FUNCTION	0x04
#define GPIO_NOT_IMPLEMENTED	0xFF


/* GPIO Structs */
#include <stdint.h>
typedef struct {
	/* All values are Read-Only */
	const volatile uint32_t _reserved_1	        : 0x8;
	const volatile uint32_t OUTFROMPERI 	    : 0x1;
	const volatile uint32_t OUTTOPAD	 	    : 0x1;
	const volatile uint32_t _reserved_2 	    : 0x2;
	const volatile uint32_t OEFROMPERI  	    : 0x1;
	const volatile uint32_t OETOPAD	 	        : 0x1;
	const volatile uint32_t _reserved_3 	    : 0x2;
	const volatile uint32_t INISDIRECT  	    : 0x1;
	const volatile uint32_t INFROMPAD   	    : 0x1;
	const volatile uint32_t INFILTERED  	    : 0x1;
	const volatile uint32_t INTOPERI    	    : 0x1;
	const volatile uint32_t EVENT_EDGE_LOW      : 0x1;
	const volatile uint32_t EVENT_EDGE_HIGH     : 0x1;
	const volatile uint32_t EVENT_LEVEL_LOW     : 0x1;
	const volatile uint32_t EVENT_LEVEL_HIGH    : 0x1; 
	const volatile uint32_t EVENT_F_EDGE_LOW    : 0x1;
	const volatile uint32_t EVENT_F_EDGE_HIGH   : 0x1;
	const volatile uint32_t EVENT_DB_LEVEL_LOW  : 0x1;
	const volatile uint32_t EVENT_DB_LEVEL_HIGH : 0x1;
	const volatile uint32_t IRQCOMBINED		    : 0x1;
	const volatile uint32_t IRQTOPROC		    : 0x1;
	const volatile uint32_t _reserved_4		    : 0x2;
} GPIO_status;


typedef struct {
	/* Set bits to change GPIO behaviour */
	volatile uint32_t FUNCSEL  	 		    : 0x5;
	volatile uint32_t F_M	  	 		    : 0x7;
	volatile uint32_t OUTOVER  	 		    : 0x2;
	volatile uint32_t OEOVER	  	 	    : 0x2;
	volatile uint32_t INOVER   	 		    : 0x2;
	volatile uint32_t _reserved_1 		    : 0x2;
	volatile uint32_t IRQMASK_EDGE_LOW      : 0x1;
	volatile uint32_t IRQMASK_EDGE_HIGH     : 0x1;
	volatile uint32_t IRQMASK_LEVEL_LOW     : 0x1;
	volatile uint32_t IRQMASK_LEVEL_HIGH    : 0x1;
	volatile uint32_t IRQMASK_F_EDGE_LOW    : 0x1;
	volatile uint32_t IRQMASK_F_EDGE_HIGH   : 0x1;
	volatile uint32_t IRQMASK_DB_LEVEL_LOW  : 0x1;
	volatile uint32_t IRQMASK_DB_LEVEL_HIGH : 0x1;
	volatile uint32_t IRQRESET			    : 0x1;
	volatile uint32_t _reserved_2		    : 0x1;
	volatile uint32_t IRQOVER			    : 0x1;
} GPIO_ctrl;


/* There are in total 28 GPIO registers */ 
/* all in an array at the GPIO_BASE 	*/
typedef struct {
	GPIO_status status;
	GPIO_ctrl ctrl;
} GPIO_register;


/* GPIO Enums */
typedef enum {
/* If a certain function shares index, with other
 * functions (see reference), then the higher 4
 * bits are used to differentiate between them */
	GPIO_SPI0_SIO  = 0x0 | (0  << 4),
	GPIO_SPI0_CSn  = 0x0 | (1  << 4),
	GPIO_SPI0_SCLK = 0x0 | (2  << 4),
	GPIO_SPI1_SIO  = 0x0 | (3  << 4),
	GPIO_SPI1_CSn  = 0x0 | (4  << 4),
	GPIO_SPI1_SCLK = 0x0 | (5  << 4),
	GPIO_SDIO0_CLK = 0x0 | (6  << 4),
	GPIO_SDIO0_CMD = 0x0 | (7  << 4),
	GPIO_SDIO0_DAT = 0x0 | (8  << 4),
	GPIO_GPCLK_0   = 0x0 | (9  << 4), /* For pins 4 - 6 */
	GPIO_PWM0_0	   = 0x0 | (10 << 4), /* For pins 12 - 15 */

	GPIO_DPI_PCLK  = 0x1 | (0 << 4),
	GPIO_DPI_DE	   = 0x1 | (1 << 4),
	GPIO_DPI_VSYNC = 0x1 | (2 << 4),
	GPIO_DPI_HSYNC = 0x1 | (3 << 4),
	GPIO_DPI_D	   = 0x1 | (4 << 4),

	GPIO_UART1_TX 	  = 0x2 | (0  << 4),
	GPIO_UART1_RX     = 0x2 | (1  << 4),
	GPIO_UART1_CTS    = 0x2 | (2  << 4),
	GPIO_UART1_RTS    = 0x2 | (3  << 4),
	GPIO_UART2_TX     = 0x2 | (4  << 4),
	GPIO_UART2_RX     = 0x2 | (5  << 4),
	GPIO_UART2_CTS    = 0x2 | (6  << 4),
	GPIO_UART2_RTS    = 0x2 | (7  << 4),
	GPIO_UART3_TX     = 0x2 | (8  << 4),
	GPIO_UART3_RX     = 0x2 | (9  << 4),
	GPIO_UART3_CTS    = 0x2 | (10 << 4),
	GPIO_UART3_RTS    = 0x2 | (11 << 4),
	GPIO_UART4_TX     = 0x2 | (12 << 4),
	GPIO_UART4_RX     = 0x2 | (13 << 4),
	GPIO_UART4_CTS    = 0x2 | (14 << 4),
	GPIO_UART4_RTS    = 0x2 | (15 << 4),
	GPIO_MIPI0_DSI_TE = 0x2 | (16 << 4),
	GPIO_MIPI1_DSI_TE = 0x2 | (17 << 4),
	GPIO_I2S0_SCLK	  = 0x2 | (18 << 4),
	GPIO_I2S0_WS	  = 0x2 | (19 << 4),
	GPIO_I2S0_SDI	  = 0x2 | (20 << 4),
	GPIO_I2S0_SDO 	  = 0x2 | (21 << 4),

	GPIO_I2C0_SDA      = 0x3 | (0  << 4),
	GPIO_I2C0_SCL      = 0x3 | (1  << 4),
	GPIO_I2C1_SDA      = 0x3 | (2  << 4),
	GPIO_I2C1_SCL      = 0x3 | (3  << 4),
	GPIO_I2C2_SDA      = 0x3 | (4  << 4),
	GPIO_I2C2_SCL      = 0x3 | (5  << 4),
	GPIO_I2C3_SDA      = 0x3 | (6  << 4),
	GPIO_I2C3_SCL      = 0x3 | (7  << 4),
	GPIO_AUDIO_IN_CLK  = 0x3 | (8  << 4),
	GPIO_AUDIO_IN_DAT0 = 0x3 | (9  << 4),
	GPIO_AUDIO_IN_DAT1 = 0x3 | (10 << 4),
	GPIO_PWM0_1        = 0x3 | (11 << 4), /* For pins 18 and 19 */
	GPIO_GPCLK_1       = 0x3 | (12 << 4), /* For pins 20 and 21 */

	GPIO_UART0_TX    = 0x4 | (1  << 4),
	GPIO_UART0_RX    = 0x4 | (2  << 4),
	GPIO_UART0_CTS   = 0x4 | (3  << 4),
	GPIO_UART0_RTS   = 0x4 | (4  << 4),
	GPIO_UART0_IR_RX = 0x4 | (5  << 4),
	GPIO_UART0_IR_TX = 0x4 | (6  << 4),
	GPIO_UART0_RI    = 0x4 | (7  << 4),
	GPIO_UART0_DTR   = 0x4 | (8  << 4),
	GPIO_UART0_DCD   = 0x4 | (9  << 4),
	GPIO_UART0_DSR   = 0x4 | (10 << 4),
	GPIO_AUDIO_OUT_L = 0x4 | (11 << 4),
	GPIO_AUDIO_OUT_R = 0x4 | (12 << 4),
	GPIO_I2S1_SCLK   = 0x4 | (13 << 4),
	GPIO_I2S1_WS     = 0x4 | (14 << 4),
	GPIO_I2S1_SDI    = 0x4 | (15 << 4),
	GPIO_I2S1_SDO    = 0x4 | (16 << 4),

	GPIO_SYS_RIO   = 0x5,
	GPIO_PROC_RIO  = 0x6,
	GPIO_PIO	   = 0x7,

	GPIO_SPI2_CSn  = 0x8 | (0  << 4),
	GPIO_SPI2_SIO  = 0x8 | (1  << 4),
	GPIO_SPI2_SCLK = 0x8 | (2  << 4),
	GPIO_SPI3_CSn  = 0x8 | (3  << 4),
	GPIO_SPI3_SIO  = 0x8 | (4  << 4),
	GPIO_SPI3_SCLK = 0x8 | (5  << 4),
	GPIO_SPI4_CSn  = 0x8 | (6  << 4),
	GPIO_SPI4_SIO  = 0x8 | (7  << 4),
	GPIO_SPI4_SCLK = 0x8 | (8  << 4),
	GPIO_SPI5_CSn  = 0x8 | (9  << 4),
	GPIO_SPI5_SIO  = 0x8 | (10 << 4),
	GPIO_SPI5_SCLK = 0x8 | (11 << 4),
	GPIO_GPCLK_2   = 0x8 | (12 << 4), /* For pin 18 */

	GPIO_NOFUNC    = 0x1f
} GPIO_Function;


typedef enum {
	GPIO_IN = 0x1,
	GPIO_OUT = 0x2
} GPIO_Mode;


/* GPIO functions */
GPIO_ERROR GPIO_MODE(const uint8_t pin, GPIO_Mode mode);
GPIO_ERROR GPIO_FUNCTION(const uint8_t pin, GPIO_Function function);

/* Bit values for GPIO register statuses */
#define IRQCOMBINED(pin) 			(GPIO[pin] & (1 << 0x1C))
#define EVENT_DB_LEVEL_HIGH(pin) 	(GPIO[pin] & (1 << 0x1B))
#define EVENT_DB_LEVEL_LOW(pin) 	(GPIO[pin] & (1 << 0x1A))
#define EVENT_F_EDGE_HIGH(pin) 		(GPIO[pin] & (1 << 0x19))
#define EVENT_F_EDGE_LOW(pin) 		(GPIO[pin] & (1 << 0x18))
#define EVENT_LEVEL_HIGH(pin) 		(GPIO[pin] & (1 << 0x17))
#define EVENT_LEVEL_LOW(pin) 		(GPIO[pin] & (1 << 0x16))
#define EVENT_EDGE_HIGH(pin) 		(GPIO[pin] & (1 << 0x15))
#define EVENT_EDGE_LOW(pin) 		(GPIO[pin] & (1 << 0x14))
#define INTOPERI(pin) 				(GPIO[pin] & (1 << 0x13))
#define INFILTERED(pin) 			(GPIO[pin] & (1 << 0x12))
#define INFROMPAD(pin) 				(GPIO[pin] & (1 << 0x11))
#define INISDIRECT(pin) 			(GPIO[pin] & (1 << 0x10))
#define OETOPAD(pin)				(GPIO[pin] & (1 << 0x0D))
#define OEFROMPERI(pin)				(GPIO[pin] & (1 << 0x0C))
#define OUTTOPAD(pin)				(GPIO[pin] & (1 << 0x09))
#define OUTFROMPERI(pin)			(GPIO[pin] & (1 << 0x08))


/* RIO */
typedef struct {
	volatile uint32_t OUT;
	volatile uint32_t OE;
	volatile uint32_t IN;
	volatile uint32_t IN_SYNC;
} RIO_register;

/* RIO macros */
#define RIO 	 ((RIO_register*) RIO_BASE)

#define RIO_XOR  ((RIO_register*) PTROFFSET(RIO_BASE, 0x1000)) // Atomic
#define RIO_SET  ((RIO_register*) PTROFFSET(RIO_BASE, 0x2000)) // Atomic
#define RIO_CLR  ((RIO_register*) PTROFFSET(RIO_BASE, 0x3000)) // Atomic


/* PAD */
typedef struct {
	/* Acts as additional settings for the pin */
	/* Must be configured before using the pin */
	uint32_t SLEWFAST	: 0x01;
	uint32_t SCHMITT	: 0x01;
	uint32_t PDE		: 0x01;
	uint32_t PUE		: 0x01;
	uint32_t DRIVE		: 0x02;
	uint32_t IE			: 0x01;
	uint32_t OD			: 0x01;
	uint32_t _reserved  : 0x18;
} PAD_register;


/* PAD macros */

#define VOLTAGE_SELECT		        	((PAD_register*) PAD_BASE)
#define PAD			  		        	(((PAD_register*) PAD_BASE) + 1)

#define PAD_2mA  		0x0
#define PAD_4mA  		0x1
#define PAD_8mA  		0x2
#define PAD_12mA 		0x3

#define PAD_DEFAULT_OUT ((PAD_register){0, 0, 0, 0, 1, 0, 0, 0}) // 0x10: 4mA, slow slew rate, no schmitt trigger/pull-up/pull-down
#define PAD_DEFAULT_IN  ((PAD_register){0, 0, 0, 0, 1, 1, 0, 0}) // 0x30: 4mA, slow slew rate, no schmitt trigger/pull-up/pull-down

/* Interrupts */
typedef uint32_t IRQ_register;
typedef void (*IRQ_CALLBACK)(void);

/* NOTE: Interrupts are handled on a per-component basis (PROC0, PROC1, PCIe), but masks */
/*		 are still done in the actual GPIO register, not in any global IRQ register.     */
#define GPIO_INTR	((IRQ_register) PTROFFSET(GPIO_BASE, 0x100)) // Raw interrupts

#define PROC0_INTE  ((IRQ_register) PTROFFSET(GPIO_BASE, 0x104)) // Enable interrupts for PROC0
#define PROC0_INTF	((IRQ_register) PTROFFSET(GPIO_BASE, 0x108)) // Force interrupts for PROC0
#define PROC0_INTS  ((IRQ_register) PTROFFSET(GPIO_BASE, 0x10C)) // Interrupts status after force and mask for PROC0

#define PROC1_INTE  ((IRQ_register) PTROFFSET(GPIO_BASE, 0x110)) // Enable interrupts for PROC1
#define PROC1_INTF	((IRQ_register) PTROFFSET(GPIO_BASE, 0x114)) // Force interrupts for PROC1
#define PROC1_INTS  ((IRQ_register) PTROFFSET(GPIO_BASE, 0x118)) // Interrupts status after force and mask for PROC1

#define PCIE_INTE	((IRQ_register) PTROFFSET(GPIO_BASE, 0x11C)) // Enable interrupts for PCIe
#define PCIE_INTF	((IRQ_register) PTROFFSET(GPIO_BASE, 0x120)) // Force interrupts for PCIe
#define PCIE_INTS 	((IRQ_register) PTROFFSET(GPIO_BASE, 0x124)) // Interrupts status after force and mask for PCIe

#define PROC0 	0x1
#define PROC1	0x2
#define PCIe	0x4

#define IRQ_NONE 			0x0
#define IRQ_EDGE_RISING 	0x1
#define IRQ_EDGE_FALLING	0x2
#define IRQ_EDGE_BOTH		IRQ_EDGE_RISING | IRQ_EDGE_FALLING
#define IRQ_LEVEL_HIGH 		0x4
#define IRQ_LEVEL_LOW 		0x8


/* Utility */
#include <stdbool.h>

extern uint32_t* BCM2712_PERI_BASE;
uint32_t* GPIO_GET_BCM2712_PERI_BASE(); 
bool GPIO_MAP();
void GPIO_CLEANUP();
void GPIO_RESET_PIN(const uint8_t pin);

#endif
