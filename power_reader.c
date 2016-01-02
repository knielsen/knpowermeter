#include <inttypes.h>
#include <string.h>

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "inc/hw_ssi.h"
#include "inc/hw_uart.h"
#include "inc/hw_timer.h"
#include "inc/hw_adc.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/ssi.h"
#include "driverlib/systick.h"
#include "driverlib/adc.h"

#include "nrf24l01p.h"


/*
  nRF24L01 pinout:

  Tx:
    PF2  SCK        GND *1 2. VCC
    PF3  CSN        PB3 .3 4. PF3
    PF0  MISO       PF2 .5 6. PF1
    PF1  MOSI       PF0 .7 8. PB0
    PB0  IRQ
    PB3  CE
*/


/*
  Note that to change these, may require additional changes in
  config_ssi_gpio() and in IRQ handler setup.
*/
#define NRF_SSI_BASE SSI1_BASE
#define NRF_CSN_BASE GPIO_PORTF_BASE
#define NRF_CSN_PIN GPIO_PIN_3
#define NRF_CE_BASE GPIO_PORTB_BASE
#define NRF_CE_PIN GPIO_PIN_3
#define NRF_IRQ_BASE GPIO_PORTB_BASE
#define NRF_IRQ_PIN GPIO_PIN_0
#define NRF_DMA_CH_RX UDMA_CHANNEL_SSI1RX
#define NRF_DMA_CH_TX UDMA_CHANNEL_SSI1TX

#define NRF_PACKET_SIZE 32


/* To change this, must fix clock setup in the code. */
#define MCU_HZ 80000000


static void
serial_output_hexdig(uint32_t dig)
{
  ROM_UARTCharPut(UART0_BASE, (dig >= 10 ? 'A' - 10 + dig : '0' + dig));
}


static void
serial_output_hexbyte(uint8_t byte)
{
  serial_output_hexdig(byte >> 4);
  serial_output_hexdig(byte & 0xf);
}


static void
serial_output_str(const char *str)
{
  char c;

  while ((c = *str++))
    ROM_UARTCharPut(UART0_BASE, c);
}


 __attribute__ ((unused))
static void
println_uint32(uint32_t val)
{
  char buf[12];
  char *p = buf;
  uint32_t l, d;

  l = 1000000000UL;
  while (l > val && l > 1)
    l /= 10;

  do
  {
    d = val / l;
    *p++ = '0' + d;
    val -= d*l;
    l /= 10;
  } while (l > 0);

  *p++ = '\n';
  *p = '\0';
  serial_output_str(buf);
}


static void
config_led(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);
}


__attribute__ ((unused))
static void
led_on(void)
{
  GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
}


__attribute__ ((unused))
static void
led_off(void)
{
  GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);
}


static void
config_ssi_gpio(void)
{
  /* Config Tx on SSI1, PF0-PF3 + PB0/PB3. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

  /* PF0 is special (NMI), needs unlock to be re-assigned to SSI1. */
  HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY_DD;
  HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
  HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

  ROM_GPIOPinConfigure(GPIO_PF2_SSI1CLK);
  ROM_GPIOPinConfigure(GPIO_PF0_SSI1RX);
  ROM_GPIOPinConfigure(GPIO_PF1_SSI1TX);
  ROM_GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
  /* CSN pin, high initially */
  ROM_GPIOPinTypeGPIOOutput(NRF_CSN_BASE, NRF_CSN_PIN);
  ROM_GPIOPinWrite(NRF_CSN_BASE, NRF_CSN_PIN, NRF_CSN_PIN);
  /* CE pin, low initially */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  ROM_GPIOPinTypeGPIOOutput(NRF_CE_BASE, NRF_CE_PIN);
  ROM_GPIOPinWrite(NRF_CE_BASE, NRF_CE_PIN, 0);
  /* IRQ pin as input. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  ROM_GPIOPinTypeGPIOInput(NRF_IRQ_BASE, NRF_IRQ_PIN);
}


static void
config_spi(uint32_t base)
{
  /*
    Configure the SPI for correct mode to read from nRF24L01+.

    We need CLK inactive low, so SPO=0.
    We need to setup and sample on the leading, rising CLK edge, so SPH=0.

    The datasheet says up to 10MHz SPI is possible, depending on load
    capacitance. Let's go with a slightly cautious 8MHz, which should be
    aplenty.
  */

  ROM_SSIDisable(base);
  ROM_SSIConfigSetExpClk(base, ROM_SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                         SSI_MODE_MASTER, 8000000, 8);
  ROM_SSIEnable(base);
}


static inline void
hw_set_bit(uint32_t addr, uint32_t bitnum)
{
  uint32_t bitband_addr = 0x42000000 + (addr-0x40000000)*32 + bitnum*4;
  HWREG(bitband_addr) = 1;
}


static inline void
hw_clear_bit(uint32_t addr, uint32_t bitnum)
{
  uint32_t bitband_addr = 0x42000000 + (addr-0x40000000)*32 + bitnum*4;
  HWREG(bitband_addr) = 0;
}


static inline uint32_t
my_gpio_read(unsigned long gpio_base, uint32_t bits)
{
  return HWREG(gpio_base + GPIO_O_DATA + (bits << 2));
}


static inline void
my_gpio_write(unsigned long gpio_base, uint32_t bits, uint32_t val)
{
  HWREG(gpio_base + GPIO_O_DATA + (bits << 2)) = val;
}


static inline void
csn_low(uint32_t csn_base, uint32_t csn_pin)
{
  my_gpio_write(csn_base, csn_pin, 0);
}


static inline void
csn_high(uint32_t csn_base, uint32_t csn_pin)
{
  my_gpio_write(csn_base, csn_pin, csn_pin);
}


static inline void
ce_low(uint32_t ce_base, uint32_t ce_pin)
{
  my_gpio_write(ce_base, ce_pin, 0);
}


static inline void
ce_high(uint32_t ce_base, uint32_t ce_pin)
{
  my_gpio_write(ce_base, ce_pin, ce_pin);
}


static void
ssi_cmd(uint8_t *recvbuf, const uint8_t *sendbuf, uint32_t len,
        uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint32_t i;
  uint32_t data;

  /* Take CSN low to initiate transfer. */
  csn_low(csn_base, csn_pin);

  for (i = 0; i < len; ++i)
  {
    ROM_SSIDataPut(ssi_base, sendbuf[i]);
    while (ROM_SSIBusy(ssi_base))
      ;
    ROM_SSIDataGet(ssi_base, &data);
    recvbuf[i] = data;
  }

  /* Take CSN high to complete transfer. */
  csn_high(csn_base, csn_pin);
}


static void
nrf_rx(uint8_t *data, uint32_t len,
       uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t sendbuf[33], recvbuf[33];

  if (len > NRF_PACKET_SIZE)
    len = NRF_PACKET_SIZE;
  sendbuf[0] = nRF_R_RX_PAYLOAD;
  memset(&sendbuf[1], 0, len);
  ssi_cmd(recvbuf, sendbuf, len+1, ssi_base, csn_base, csn_pin);
  memcpy(data, &recvbuf[1], len);
}


static void
nrf_tx(uint8_t *data, uint32_t len,
       uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t sendbuf[33], recvbuf[33];

  if (len > NRF_PACKET_SIZE)
    len = NRF_PACKET_SIZE;
  sendbuf[0] = nRF_W_TX_PAYLOAD;
  memcpy(&sendbuf[1], data, len);
  ssi_cmd(recvbuf, sendbuf, len+1, ssi_base, csn_base, csn_pin);
}


static void
nrf_flush_tx(uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t cmd = nRF_FLUSH_TX;
  uint8_t status;
  ssi_cmd(&status, &cmd, 1, ssi_base, csn_base, csn_pin);
}


static void
nrf_flush_rx(uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t cmd = nRF_FLUSH_RX;
  uint8_t status;
  ssi_cmd(&status, &cmd, 1, ssi_base, csn_base, csn_pin);
}


static void
nrf_write_reg_n(uint8_t reg, const uint8_t *data, uint32_t len,
                uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t sendbuf[6], recvbuf[6];
  if (len > 5)
    len = 5;
  sendbuf[0] = nRF_W_REGISTER | reg;
  memcpy(&sendbuf[1], data, len);
  ssi_cmd(recvbuf, sendbuf, len+1, ssi_base, csn_base, csn_pin);
}


static void
nrf_write_reg(uint8_t reg, uint8_t val,
              uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  nrf_write_reg_n(reg, &val, 1, ssi_base, csn_base, csn_pin);
}


static void
nrf_read_reg_n(uint8_t reg, uint8_t *out, uint32_t len,
               uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t sendbuf[6];
  if (len > 5)
    len = 5;
  sendbuf[0] = nRF_R_REGISTER | reg;
  memset(&sendbuf[1], 0, len);
  ssi_cmd(out, sendbuf, len+1, ssi_base, csn_base, csn_pin);
}


static uint8_t
nrf_read_reg(uint8_t reg, uint8_t *status_ptr,
             uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t recvbuf[2];
  nrf_read_reg_n(reg, recvbuf, 2, ssi_base, csn_base, csn_pin);
  if (status_ptr)
    *status_ptr = recvbuf[0];
  return recvbuf[1];
}


static const uint8_t nrf_addr[3] = { 0xe7, 0x8c, 0x5e };

/*
  Configure nRF24L01+ as Rx or Tx.
    channel - radio frequency channel to use, 0 <= channel <= 127.
    power - nRF_RF_PWR_<X>DBM, <X> is 0, 6, 12, 18 dBm.
*/
static void
nrf_init_config(uint8_t is_rx, uint32_t channel, uint32_t power,
                uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  if (is_rx)
    nrf_write_reg(nRF_CONFIG, nRF_PRIM_RX | nRF_MASK_TX_DS |
                  nRF_MASK_MAX_RT|nRF_EN_CRC|nRF_CRCO|nRF_PWR_UP,
                  ssi_base, csn_base, csn_pin);
  else
    nrf_write_reg(nRF_CONFIG, nRF_MASK_RX_DR |
                  nRF_MASK_MAX_RT|nRF_EN_CRC|nRF_CRCO|nRF_PWR_UP,
                  ssi_base, csn_base, csn_pin);
  /* Disable auto-ack, saving 9 bits/packet. Else 0x3f. */
  nrf_write_reg(nRF_EN_AA, 0, ssi_base, csn_base, csn_pin);
  /* Enable only pipe 0. */
  nrf_write_reg(nRF_EN_RXADDR, nRF_ERX_P0, ssi_base, csn_base, csn_pin);
  /* 3 byte adresses. */
  nrf_write_reg(nRF_SETUP_AW, nRF_AW_3BYTES, ssi_base, csn_base, csn_pin);
  /* Disable auto retransmit. */
  nrf_write_reg(nRF_SETUP_RETR, 0, ssi_base, csn_base, csn_pin);
  nrf_write_reg(nRF_RF_CH, channel, ssi_base, csn_base, csn_pin);
  /* Use 2Mbps, and set transmit power. */
  nrf_write_reg(nRF_RF_SETUP, nRF_RF_DR_LOW | power,
                ssi_base, csn_base, csn_pin);
  nrf_write_reg_n(nRF_RX_ADDR_P0, nrf_addr, 3, ssi_base, csn_base, csn_pin);
  nrf_write_reg_n(nRF_TX_ADDR, nrf_addr, 3, ssi_base, csn_base, csn_pin);
  /* Set payload size for pipe 0. */
  nrf_write_reg(nRF_RX_PW_P0, NRF_PACKET_SIZE, ssi_base, csn_base, csn_pin);
  /* Disable pipe 1-5. */
  nrf_write_reg(nRF_RX_PW_P1, 0, ssi_base, csn_base, csn_pin);
  /* Disable dynamic payload length. */
  nrf_write_reg(nRF_DYNDP, 0, ssi_base, csn_base, csn_pin);
  /* Allow disabling acks. */
  nrf_write_reg(nRF_FEATURE, nRF_EN_DYN_ACK, ssi_base, csn_base, csn_pin);

  /* Clear out all FIFOs. */
  nrf_flush_tx(ssi_base, csn_base, csn_pin);
  nrf_flush_rx(ssi_base, csn_base, csn_pin);
  /* Clear the IRQ bits in STATUS register. */
  nrf_write_reg(nRF_STATUS, nRF_RX_DR|nRF_TX_DS|nRF_MAX_RT,
                ssi_base, csn_base, csn_pin);
}


/*
  Configure nRF24L01+ as Tx.

  The nRF24L01+ is configured as transmitter, with auto-ack and
  retransmission enabled.
*/
static void
nrf_config_tx(uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  nrf_write_reg(nRF_CONFIG, nRF_MASK_RX_DR |
                  nRF_EN_CRC|nRF_CRCO|nRF_PWR_UP,
                  ssi_base, csn_base, csn_pin);
  /* Enable auto-ack. */
  nrf_write_reg(nRF_EN_AA, nRF_ENAA_P0|nRF_ENAA_P1|nRF_ENAA_P2|
                           nRF_ENAA_P3|nRF_ENAA_P4|nRF_ENAA_P5,
                ssi_base, csn_base, csn_pin);
  /* Enable auto retransmit. */
  nrf_write_reg(nRF_SETUP_RETR, (1 << nRF_ARD_SHIFT) | 15,
                ssi_base, csn_base, csn_pin);

  /* Clear out all FIFOs. */
  nrf_flush_tx(ssi_base, csn_base, csn_pin);
  nrf_flush_rx(ssi_base, csn_base, csn_pin);
  /* Clear the IRQ bits in STATUS register. */
  nrf_write_reg(nRF_STATUS, nRF_RX_DR|nRF_TX_DS|nRF_MAX_RT,
                ssi_base, csn_base, csn_pin);
}


/*
  Configure nRF24L01+ as Rx.

  The nRF24L01+ is configured as receiver, with auto-ack and
  retransmission enabled.
*/
static void
nrf_config_rx(uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  nrf_write_reg(nRF_CONFIG, nRF_PRIM_RX | nRF_MASK_TX_DS |
                  nRF_MASK_MAX_RT|nRF_EN_CRC|nRF_CRCO|nRF_PWR_UP,
                  ssi_base, csn_base, csn_pin);
  /* Enable auto-ack. */
  nrf_write_reg(nRF_EN_AA, nRF_ENAA_P0|nRF_ENAA_P1|nRF_ENAA_P2|
                           nRF_ENAA_P3|nRF_ENAA_P4|nRF_ENAA_P5,
                ssi_base, csn_base, csn_pin);
  /* Enable auto retransmit. */
  nrf_write_reg(nRF_SETUP_RETR, (1 << nRF_ARD_SHIFT) | 15,
                ssi_base, csn_base, csn_pin);

  /* Clear out all FIFOs. */
  nrf_flush_tx(ssi_base, csn_base, csn_pin);
  nrf_flush_rx(ssi_base, csn_base, csn_pin);
  /* Clear the IRQ bits in STATUS register. */
  nrf_write_reg(nRF_STATUS, nRF_RX_DR|nRF_TX_DS|nRF_MAX_RT,
                ssi_base, csn_base, csn_pin);
}


static void
setup_systick(void)
{
  /* Interrupt every millisecond. */
  ROM_SysTickPeriodSet(MCU_HZ/1000);
  /* Force reload. */
  HWREG(NVIC_ST_CURRENT) = 0;
  ROM_SysTickEnable();
}


static volatile uint32_t millisecond_counter = 0;

void
SysTickHandler(void)
{
  ++millisecond_counter;
}


/*
  Read both the normal and FIFO status registers.
  Returns normal status or'ed with (fifo status left-shifted 8).
*/
static uint32_t
nrf_get_status(uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t status;
  uint32_t fifo_status;

  fifo_status =
    nrf_read_reg(nRF_FIFO_STATUS, &status, ssi_base, csn_base, csn_pin);
  return (fifo_status << 8) | status;
}


/* ADC */
static void
config_adc_single(void)
{
  /* Enable ADC0 on PD2 (AIN5). */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  ROM_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2);
  ROM_ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
  ROM_ADCSequenceStepConfigure(ADC0_BASE, 3, 0,
                               ADC_CTL_CH5 | ADC_CTL_IE | ADC_CTL_END);
  ROM_ADCSequenceEnable(ADC0_BASE, 3);
  ROM_ADCIntClear(ADC0_BASE, 3);
}


static unsigned long
read_adc(void)
{
  unsigned long val;

  ROM_ADCProcessorTrigger(ADC0_BASE, 3);
  while(!ROM_ADCIntStatus(ADC0_BASE, 3, false))
    ;
  ROM_ADCIntClear(ADC0_BASE, 3);
  ROM_ADCSequenceDataGet(ADC0_BASE, 3, &val);
  return val;
}


static void
config_adc_comparator(void)
{
  /* Enable ADC1 on PD2 (AIN5). */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  ROM_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2);
  ROM_ADCComparatorConfigure(ADC1_BASE, 0,
                             ADC_COMP_TRIG_NONE | ADC_COMP_INT_HIGH_HONCE);
  ROM_ADCComparatorRegionSet(ADC1_BASE, 0, 30, 420);
  ROM_ADCComparatorReset(ADC1_BASE, 0, 1, 1);
  ROM_ADCSequenceConfigure(ADC1_BASE, 3, ADC_TRIGGER_ALWAYS, 0);
  ROM_ADCSequenceStepConfigure(ADC1_BASE, 3, 0,
                               ADC_CTL_CH5 | ADC_CTL_CMP0 | ADC_CTL_END);
  ROM_ADCSequenceEnable(ADC1_BASE, 3);
  ROM_ADCIntClear(ADC1_BASE, 3);
}


static volatile uint32_t last_blip_millis = 0;

void
ADC1Seq3Handler(void)
{
  uint32_t now = millisecond_counter;
  unsigned long adc_status;
  static uint8_t got_blip = 0;
  static uint32_t last_blip_time;

  adc_status = ROM_ADCIntStatus(ADC1_BASE, 3, true);
  if (adc_status)
  {
    if (ROM_ADCComparatorIntStatus(ADC1_BASE) & (1 << 0))
    {
      if (got_blip)
      {
        last_blip_millis = now - last_blip_time;
        last_blip_time = now;
      }
      got_blip = 1;
      ROM_ADCComparatorIntClear(ADC1_BASE, (1 << 0));
    }
    ROM_ADCIntClear(ADC1_BASE, 3);
  }
}


int main()
{
  uint8_t status;
  uint8_t val;
  uint32_t counter;

  /* Use the full 80MHz system clock. */
  ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL |
                     SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
  ROM_FPULazyStackingEnable();
  setup_systick();

  /* Configure serial. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
  ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
  ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  ROM_UARTConfigSetExpClk(UART0_BASE, (ROM_SysCtlClockGet()), 500000,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_PAR_NONE));

  config_ssi_gpio();
  config_spi(NRF_SSI_BASE);
  config_led();

  /* nRF24L01+ datasheet says to wait 100msec for bootup. */
  ROM_SysCtlDelay(MCU_HZ/3/10);

  serial_output_str("Tx: Setting up...\r\n");
  nrf_init_config(0 /* Tx */, 81, nRF_RF_PWR_0DBM,
                  NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
  serial_output_str("Tx: Read CONFIG=0x");
  val = nrf_read_reg(nRF_CONFIG, &status,
                     NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
  serial_output_hexbyte(val);
  serial_output_str(" status=0x");
  serial_output_hexbyte(status);
  serial_output_str("\r\n");
  serial_output_str("Done!\r\n");
  config_adc_single();
  config_adc_comparator();
  serial_output_str("ADC config done\r\n");

  ROM_IntMasterEnable();
  ROM_SysTickIntEnable();
  ROM_ADCComparatorIntEnable(ADC1_BASE, 3);
  ROM_ADCIntEnable(ADC1_BASE, 3);
  ROM_IntEnable(INT_ADC1SS3);

  counter = 0;
  for (;;)
  {
    uint32_t blip_millis;

    ++counter;
    if (counter > 500000)
    {
      counter = 0;
//      println_uint32(read_adc());
    }

    blip_millis = last_blip_millis;
    if (blip_millis)
    {
      last_blip_millis = 0;
      serial_output_str("Blip: ");
      println_uint32(blip_millis);
      ROM_ADCComparatorIntClear(ADC1_BASE, (1 << 0));
    }
  }
}
