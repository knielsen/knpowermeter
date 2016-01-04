#include <inttypes.h>
#include <string.h>
#include <math.h>

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "inc/hw_ssi.h"
#include "inc/hw_uart.h"
#include "inc/hw_timer.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/ssi.h"
#include "driverlib/systick.h"
#include "driverlib/usb.h"
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"

#include "nrf24l01p.h"
#include "usb_serial_structs.h"


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


/* Communications protocol. */
#define NRF_PACKET_SIZE 32


/* To change this, must fix clock setup in the code. */
#define MCU_HZ 80000000


#define likely(x) __builtin_expect((x)!=0, 1)
#define unlikely(x) __builtin_expect((x)!=0, 0)


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


/*
  USB stuff.
  Mostly copied and simplified from the Stellaris/Tiva USB virtual COM port
  example.
*/

/*
  Define my own memcpy(). The usblib gets to access gcc builtin memcpy() for
  struct assignment, and somehow it seems to get linked with a version that
  uses not thumb2 instructions but basic full-length ARM instructions, which
  does not work ...
*/
void *
memcpy(void *dest, const void *src, unsigned n)
{
  unsigned char *d = dest;
  const unsigned char *s = src;
  while(n--)
    *d++ = *s++;
  return dest;
}


__attribute__ ((unused))
static void
usb_data_put(const unsigned char *buf, uint32_t size)
{
  while (size > 0)
  {
    uint32_t actual = USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, buf, size);
    if (size >= actual)
      size -= actual;
    else
      size = 0;
    buf += actual;
  }
}
#define USB_DBG(x) usb_data_put((const unsigned char *)("!" x), sizeof(x))


/* USB stuff. */

#define RECV_BUF_SIZE 8192
static volatile
struct recv_buf_struct {
  uint32_t head;
  uint32_t tail;
  unsigned char buf[RECV_BUF_SIZE];
} recvbuf;

static void
usb_data_get(void)
{
  unsigned long h, t, size, actual;

  h = recvbuf.head;
  t = recvbuf.tail;
  if (h >= t)
  {
    size = RECV_BUF_SIZE - h;
    /* Do not overrun the FIFO. */
    if (t == 0)
      --size;
  }
  else
    size = t - h - 1;
  if (size == 0)
    return;

  actual = USBBufferRead((tUSBBuffer *)&g_sRxBuffer,
                         (unsigned char *)&recvbuf.buf[h], size);
  h += actual;
  if (h >= RECV_BUF_SIZE)
    h = 0;
  if (actual >= size && h + 1 < t)
  {
    size = t - h - 1;
    actual = USBBufferRead((tUSBBuffer *)&g_sRxBuffer,
                           (unsigned char *)&recvbuf.buf[h], size);
    h += actual;
  }
  recvbuf.head = h;
}


//*****************************************************************************
//
// Get the communication parameters in use on the UART.
//
//*****************************************************************************
static void
GetLineCoding(tLineCoding *psLineCoding)
{
    psLineCoding->ulRate = 2000000;
    psLineCoding->ucDatabits = 8;
    psLineCoding->ucParity = USB_CDC_PARITY_NONE;
    psLineCoding->ucStop = USB_CDC_STOP_BITS_1;
}

//*****************************************************************************
//
// Handles CDC driver notifications related to control and setup of the device.
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ulEvent identifies the event we are being notified about.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to perform control-related
// operations on behalf of the USB host.  These functions include setting
// and querying the serial communication parameters, setting handshake line
// states and sending break conditions.
//
// \return The return value is event-specific.
//
//*****************************************************************************
unsigned long
ControlHandler(void *pvCBData, unsigned long ulEvent,
               unsigned long ulMsgValue, void *pvMsgData)
{
    //
    // Which event are we being asked to process?
    //
    switch(ulEvent)
    {
        //
        // We are connected to a host and communication is now possible.
        //
        case USB_EVENT_CONNECTED:
            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);
            break;

        //
        // The host has disconnected.
        //
        case USB_EVENT_DISCONNECTED:
            break;

        //
        // Return the current serial communication parameters.
        //
        case USBD_CDC_EVENT_GET_LINE_CODING:
            GetLineCoding(pvMsgData);
            break;

        /* Ignored stuff. */
        case USBD_CDC_EVENT_SET_LINE_CODING:
        case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:
        case USBD_CDC_EVENT_SEND_BREAK:
        case USBD_CDC_EVENT_CLEAR_BREAK:
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
            break;

        //
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        //
        default:
            break;
    }

    return(0);
}

//*****************************************************************************
//
// Handles CDC driver notifications related to the transmit channel (data to
// the USB host).
//
// \param ulCBData is the client-supplied callback pointer for this channel.
// \param ulEvent identifies the event we are being notified about.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
unsigned long
TxHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue,
          void *pvMsgData)
{
    //
    // Which event have we been sent?
    //
    switch(ulEvent)
    {
        case USB_EVENT_TX_COMPLETE:
            //
            // Since we are using the USBBuffer, we don't need to do anything
            // here.
            //
            break;

        //
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        //
        default:
            break;
    }
    return(0);
}

//*****************************************************************************
//
// Handles CDC driver notifications related to the receive channel (data from
// the USB host).
//
// \param ulCBData is the client-supplied callback data value for this channel.
// \param ulEvent identifies the event we are being notified about.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
unsigned long
RxHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue,
          void *pvMsgData)
{
    //
    // Which event are we being sent?
    //
    switch(ulEvent)
    {
        //
        // A new packet has been received.
        //
        case USB_EVENT_RX_AVAILABLE:
        {
            usb_data_get();
            break;
        }

        case USB_EVENT_DATA_REMAINING:
        {
          uint32_t h, t;

          h = recvbuf.head;
          t = recvbuf.tail;
          if (t > h)
            return (RECV_BUF_SIZE - t) + h;
          else
            return h - t;
        }

        //
        // We are being asked to provide a buffer into which the next packet
        // can be read. We do not support this mode of receiving data so let
        // the driver know by returning 0. The CDC driver should not be sending
        // this message but this is included just for illustration and
        // completeness.
        //
        case USB_EVENT_REQUEST_BUFFER:
        {
            return(0);
        }

        //
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        //
        default:
            break;
    }

    return(0);
}


static void
config_usb(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  ROM_GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_5 | GPIO_PIN_4);

  USBBufferInit((tUSBBuffer *)&g_sTxBuffer);
  USBBufferInit((tUSBBuffer *)&g_sRxBuffer);
  USBStackModeSet(0, USB_MODE_FORCE_DEVICE, 0);
  USBDCDCInit(0, (tUSBDCDCDevice *)&g_sCDCDevice);
}

/* End of USB stuff. */


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


static void
bzero(uint8_t *buf, uint32_t len)
{
  while (len > 0)
  {
    *buf++ = 0;
    --len;
  }
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
  bzero(&sendbuf[1], len);
  ssi_cmd(recvbuf, sendbuf, len+1, ssi_base, csn_base, csn_pin);
  memcpy(data, &recvbuf[1], len);
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
  bzero(&sendbuf[1], len);
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


static const uint8_t nrf_addr[3] = { 0x39, 0x56, 0x5e };

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
    nrf_write_reg(nRF_CONFIG, nRF_MASK_RX_DR|nRF_EN_CRC|nRF_CRCO|nRF_PWR_UP,
                  ssi_base, csn_base, csn_pin);
  /* Enable auto-ack (0x3f sets enable bit for all 6 data pipes). */
  nrf_write_reg(nRF_EN_AA, 0x3f, ssi_base, csn_base, csn_pin);
  /* Enable only pipe 0. */
  nrf_write_reg(nRF_EN_RXADDR, nRF_ERX_P0, ssi_base, csn_base, csn_pin);
  /* 3 byte adresses. */
  nrf_write_reg(nRF_SETUP_AW, nRF_AW_3BYTES, ssi_base, csn_base, csn_pin);
  /* Enable auto retransmit, up to 15 (max) retransmits, fair pause in-between . */
  nrf_write_reg(nRF_SETUP_RETR, 15 | (0xb << nRF_ARD_SHIFT),
                ssi_base, csn_base, csn_pin);
  nrf_write_reg(nRF_RF_CH, channel, ssi_base, csn_base, csn_pin);
  /* Use lowest 250kbps speed for best range, and set transmit power. */
  nrf_write_reg(nRF_RF_SETUP, nRF_RF_DR_LOW | power,
                ssi_base, csn_base, csn_pin);
  nrf_write_reg_n(nRF_RX_ADDR_P0, nrf_addr, 3, ssi_base, csn_base, csn_pin);
  nrf_write_reg_n(nRF_TX_ADDR, nrf_addr, 3, ssi_base, csn_base, csn_pin);
  /* Set payload size for pipe 0. */
  nrf_write_reg(nRF_RX_PW_P0, 4, ssi_base, csn_base, csn_pin);
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


static void
delay_us(uint32_t microseconds)
{
  /* This isn't ideal, but it should work for now. */
  ROM_SysCtlDelay((MCU_HZ/1000000)*microseconds/3);
}


static uint32_t
num2str(char *buf, uint32_t val)
{
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

  *p = '\0';
  return p - buf;
}


/* Clear the RX_DR (data sent) flag. */
static void
nrf_clear_dr_status(uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  nrf_write_reg(nRF_STATUS, nRF_RX_DR, ssi_base, csn_base, csn_pin);
}


int main()
{
  uint8_t status;
  uint8_t val;

  /* Use the full 80MHz system clock. */
  ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL |
                     SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
  ROM_FPULazyStackingEnable();

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

  config_usb();

  /* Enable interrrupts. */
  ROM_IntMasterEnable();

  /* nRF24L01+ datasheet says to wait 100msec for bootup. */
  ROM_SysCtlDelay(MCU_HZ/3/10);

  serial_output_str("Tx: Setting up...\r\n");
  nrf_init_config(1 /* Rx */, 115, nRF_RF_PWR_0DBM,
                  NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
  serial_output_str("Rx: Read CONFIG=0x");
  val = nrf_read_reg(nRF_CONFIG, &status,
                     NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
  serial_output_hexbyte(val);
  serial_output_str(" status=0x");
  serial_output_hexbyte(status);
  serial_output_str("\r\n");
  serial_output_str("Done!\r\n");

  /* Small delay to allow USB to get up ... */
  ROM_SysCtlDelay(MCU_HZ/3);

  /* Now take CE high to start receiving blips on the nRF24L01+. */
  ce_high(NRF_CE_BASE, NRF_CE_PIN);
  /*
    nRF24L01+ datasheet says that there must be at least 4 microseconds
    from a positive edge on CE to CSN being taken low.
  */
  delay_us(4);

  for (;;)
  {
    uint32_t irq;

    irq = my_gpio_read(NRF_IRQ_BASE, NRF_IRQ_PIN);
    if (irq)
    {
      /* Nothing received from the nRF24L01+ yet. */
      continue;
    }

    /*
      First clear the IRQ flag, then pull all received packets.
      This way, we are sure to see IRQ active again if new packets arrive
      while we are pulling out old ones (we may get spurious IRQ though).
    */
    nrf_clear_dr_status(NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
    for (;;)
    {
      uint32_t status;
      uint8_t buf[4];
      uint32_t millis;
      char strbuf[20];
      uint32_t len;

      status = nrf_get_status(NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
      if ((status >> 8) & nRF_RX_EMPTY)
        break;
      nrf_rx(buf, 4, NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
      memcpy(&millis, buf, 4);
      len = num2str(strbuf, millis);
      strbuf[len] = '\n';
      serial_output_str("Send to usb...");
      usb_data_put(strbuf, len+1);
      serial_output_str(" done!\r\n");
    }
  }
}
