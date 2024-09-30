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
