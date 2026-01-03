#include "lpc111x.h"
#include "printf.h"


void SystemInit(void) {
}


void Clock_Init(void) {
  // 12 MHz internal RC oscillator
  //SCB_SYSPLLCLKSEL
  //SYSPLLCLKSEL // This register is used to trim the on-chip 12 MHz oscillator
  
  //
  
  //SCB_IRCCTRL |= SCB_CLKSEL_SOURCE_INTERNALOSC;//
  //SCB_PLLCTRL |= ;//
}

uint32_t counter = '0';

void UART_Init(void) {

  // the UART pins must be configured in the IOCONFIG register block (Section 7.4) before the UART clocks can be enabled in the SYSAHBCLKCTRL register
  IOCON_PIO1_6 |= IOCON_PIO1_6_FUNC_UART_RXD;
  IOCON_PIO1_7 |= IOCON_PIO1_7_FUNC_UART_TXD | IOCON_PIO1_7_MODE_PULLUP;

  // UART clock enable
  SCB_SYSAHBCLKCTRL |= SCB_SYSAHBCLKCTRL_UART;
  // UART clock divider
  SCB_UARTCLKDIV |= SCB_UARTCLKDIV_DIV1;

  if (1) {
    UART_U0LCR |= UART_U0LCR_Divisor_Latch_Access_Enabled; //DLAB_Enabled;
    // DLM = 0, DLL = 4, DIVADDVAL = 5, and MULVAL = 8.
    UART_U0DLM = 0;
    UART_U0DLL = 4;
    UART_U0FDR &= ~(UART_U0FDR_DIVADDVAL_MASK | UART_U0FDR_MULVAL_MASK);
    UART_U0FDR |= (8 << 4) | (5 << 0);
    
    UART_U0LCR &= ~(UART_U0LCR_Word_Length_Select_MASK | UART_U0LCR_Parity_Select_MASK |UART_U0LCR_Parity_Enable_MASK | UART_U0LCR_Stop_Bit_Select_MASK);
    UART_U0LCR |= UART_U0LCR_Word_Length_Select_8Chars | UART_U0LCR_Stop_Bit_Select_1Bits;
    
    UART_U0LCR &= ~UART_U0LCR_Divisor_Latch_Access_Enabled;
  }

  // Transmitter Enable
  UART_U0TER |= UART_U0TER_TXEN_Enabled;

  // Enable Interrupt
  UART_U0IER |= UART_U0IER_THRE_Interrupt_Enabled;
  NVIC_EnableIRQ(UART_IRQn);
  __enable_irq();
}

void I2C_Init(void) {
  IOCON_PIO0_4 |= IOCON_PIO0_4_FUNC_I2CSCL;
  IOCON_PIO0_5 |= IOCON_PIO0_5_FUNC_I2CSDA;
  
  IOCON_PIO0_4 |= IOCON_PIO0_4_I2CMODE_STANDARDI2C;
  IOCON_PIO0_5 |= IOCON_PIO0_5_I2CMODE_STANDARDI2C;

  // I2C clock enable
  SCB_SYSAHBCLKCTRL |= SCB_SYSAHBCLKCTRL_I2C;
  SCB_PRESETCTRL = SCB_PRESETCTRL_I2C_RESETDISABLED;

  I2C_I2CCONCLR = I2C_I2CCONCLR_I2ENC;
  {
    const uint32_t pclk = 12000000;
    const uint32_t div = (pclk / (2 * 100000)) - 1;
    I2C_I2CSCLH = div;
    I2C_I2CSCLL = div;
  }
  I2C_I2CCONSET = I2C_I2CCONSET_I2EN;

  I2C_I2CCONSET = I2C_I2CCONSET_I2EN;

  // Enable Interrupt
  //NVIC_EnableIRQ(I2C_IRQn);
  //__enable_irq();
}

void I2C_Start(void) {
  I2C_I2CCONSET = I2C_I2CCONSET_STA; // (1 << 5);  // Set STA bit
}

void I2C_Restart(void) {
  // FIXME: !!!!
  {
    I2C_I2CCONCLR = I2C_I2CCONCLR_SIC;
    I2C_I2CCONSET = I2C_I2CCONSET_STA; // (1 << 5);  // Set STA bit

    while (
        !(I2C_I2CSTAT & 0x08)) { // 0x08 A START condition has been transmitted.
    }
  }
}

void I2C_Stop(void) {
  I2C_I2CCONSET = I2C_I2CCONSET_STO;
  I2C_I2CCONCLR = I2C_I2CCONCLR_SIC;

  while(I2C_I2CCONSET & I2C_I2CCONSET_STO) {
  }
}


uint8_t I2C_SendAddress(uint8_t addr, uint8_t rw) { // rw=1 indicates a read
  uint32_t status1, status2, status3, status4, status5;
  uint32_t conset1, conset2, conset3, conset4, conset5;
  conset1 = I2C_I2CCONSET; status1 = I2C_I2CSTAT;
  // Wait for start condition sent
  while (!(I2C_I2CSTAT & 0x08)) { // 0x08 A START condition has been transmitted.
  }
  conset2 = I2C_I2CCONSET; status2 = I2C_I2CSTAT;
  I2C_I2CDAT = (addr << 1) | rw; // 7-bit address + R/W bit
  conset3 = I2C_I2CCONSET; status3 = I2C_I2CSTAT;

  // IMPORTANT! "The STA bit should be cleared after writing the slave address"
  // "load the slave address ... to the DAT register, and then clear the SI bit"
  //
  // "While SI is set, the low period of the serial clock on the SCL line is stretched, and the
  // serial transfer is suspended. When SCL is HIGH, it is unaffected by the state of the SI flag"
  // D.Sh ^^^ means while SI is 1, SCL is being stretched? and we load to DAT during this condition
  I2C_I2CCONCLR = I2C_I2CCONCLR_STAC | I2C_I2CCONCLR_SIC;
  conset4 = I2C_I2CCONSET; status4 = I2C_I2CSTAT;

  // When the slave address and R/W bit have been transmitted and an acknowledgment bit
  // has been received, the SI bit is set again, and the possible status codes
  // now are 0x18, 0x20, or 0x38 for the master mode (Write)
  //         0x40, 0x48, or 0x38 for the master mode (Read)
  while (!(I2C_I2CCONSET & I2C_I2CCONSET_SI)) {
  }
  conset5 = I2C_I2CCONSET; status5 = I2C_I2CSTAT;

  // Check status
  uint8_t status = I2C_I2CSTAT;
  if (status == 0x20 || status == 0x48) { // NACK bit recevied
    return 0;
  }

  if (status != 0x18 && status != 0x40) { // 0x18: ACK, 0x40: ACK in read mode
    return 0;                             // NACK received
  }

  return 1; // ACK received
}

//__attribute__((interrupt))
void UART_IRQHandler(void) {
  GPIO_GPIO1DATA = GPIO_IO_P0 | GPIO_IO_P2; // Green
  //asm("bkpt");
  
  //U0IIR; // INTID[3:1] : 0x1 "3 - THRE Interrupt"
  unsigned int u0iir = UART_U0IIR;
  (void)u0iir;
  
  if (counter == '9' + 1) {
    UART_U0THR = '\n';
  }
  else if (counter == '9' + 2) {
    UART_U0THR = '\r';
    counter = '0';
  } else {
    UART_U0THR = counter;
  }
  ++counter;
}

void _putchar(char character)
{
  UART_U0THR = character;
}

__attribute__((noreturn,naked))
void _start(void) {
    // Turn on clock for GPIO, IOCON
  SCB_SYSAHBCLKCTRL |= SCB_SYSAHBCLKCTRL_GPIO | SCB_SYSAHBCLKCTRL_IOCON;

  Clock_Init();
  I2C_Init();
  I2C_Start();
  I2C_SendAddress(0x11, 1);
  I2C_Restart();
  I2C_Stop();

  counter = '0';
  UART_Init();

  IOCON_JTAG_TMS_PIO1_0 |= IOCON_JTAG_TMS_PIO1_0_FUNC_GPIO;
  IOCON_JTAG_TDO_PIO1_1 |= IOCON_JTAG_TDO_PIO1_1_FUNC_GPIO;
  IOCON_JTAG_nTRST_PIO1_2 |= IOCON_JTAG_nTRST_PIO1_2_FUNC_GPIO;

  IOCON_JTAG_TMS_PIO1_0 |= IOCON_JTAG_TMS_PIO1_0_MODE_PULLDOWN;
  IOCON_JTAG_TDO_PIO1_1 |= IOCON_JTAG_TDO_PIO1_1_MODE_PULLDOWN;
  IOCON_JTAG_nTRST_PIO1_2 |= IOCON_JTAG_nTRST_PIO1_2_MODE_PULLDOWN;

  GPIO_GPIO1DIR |= GPIO_IO_P0 | GPIO_IO_P1 | GPIO_IO_P2;

  GPIO_GPIO1DATA = GPIO_IO_P0 | GPIO_IO_P1; // Blue

  printf("%s", "Hello\n");
  while(1) {
  }
  
  if (0) {
    const int delay = 1000000;
    int n;
    GPIO_GPIO1DATA = GPIO_IO_P0 | GPIO_IO_P1; // Blue
    n=delay; while(--n);
    GPIO_GPIO1DATA = GPIO_IO_P0 | GPIO_IO_P2; // Green
    n=delay; while(--n);
    GPIO_GPIO1DATA = GPIO_IO_P1 | GPIO_IO_P2; // Red
    n=delay; while(--n);
    GPIO_GPIO1DATA = GPIO_IO_P0; // Green + Blue // Cyan?
    n=delay; while(--n);
    GPIO_GPIO1DATA = GPIO_IO_P1; // Red + Blue   // Magenta
    n=delay; while(--n);
    GPIO_GPIO1DATA = GPIO_IO_P2; // Red + Green  // Orange?
    n=delay; while(--n);
    GPIO_GPIO1DATA = 0;    // White
    n=delay; while(--n);
    GPIO_GPIO1DATA = GPIO_IO_P0 | GPIO_IO_P1 | GPIO_IO_P2; // No light
    n=delay; while(--n);
  }
}
