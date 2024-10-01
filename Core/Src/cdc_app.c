#include "bsp/board_api.h"
#include "tusb.h"
#include "rxtx_if.h"

// Invoked when DTR and RTS line states changed
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
	(void) itf;

	PTT_DTR_TX(dtr); /* DTR is used for external telegraph keyer only */
    PTT_RTS_TX(rts); /* RTS is used for external telegraph keyer only */
}

static uint8_t cdc_itf = 0xFF;

// Invoked when CDC interface transmites data to host
void CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
    if (cdc_itf != 0xFF)
	{
		tud_cdc_n_write(cdc_itf, Buf, Len);
		tud_cdc_n_write_flush(cdc_itf);
	}
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
	uint8_t buf[64];
	uint32_t count;

	if (tud_cdc_available()) // data is available
	{
		cdc_itf = itf;
		count = tud_cdc_n_read(itf, buf, sizeof(buf));
		CAT_Buff_Write (buf, count);
	}
}

/*
void set_dsr(bool value) {

    if (cdc_itf != 0xFF)
	{
		cdcd_interface_t* p_cdc = &_cdcd_itf[cdc_itf];

		uint8_t packet[10];

		packet[0] = 0xA1;	//   bmRequestType
		packet[1] = CDC_NOTIF_SERIAL_STATE;	//   bNotification
		packet[2] = 0x00;	//   wValue
		packet[3] = 0x00;
		packet[4] = 0x00;                        //   wIndex
		packet[5] = 0x00;
		packet[6] = 0x02;                        //   wLength
		packet[7] = 0x00;
		packet[8] = value ?  0x02 : 0x00; 
		packet[9] = 0x00;

		usbd_edpt_xfer(TUD_OPT_RHPORT, p_cdc->ep_notif, packet, sizeof(packet));
	}
}
*/
