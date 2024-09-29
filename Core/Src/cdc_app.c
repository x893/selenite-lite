#include "bsp/board_api.h"
#include "tusb.h"

// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
	(void) itf;
	(void) rts;

	if (dtr)
	{
		// Terminal connected
	}
	else
	{
		// Terminal disconnected
	}
}

static uint8_t cdc_itf = 0xFF;

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
	if (tud_cdc_connected() && cdc_itf != 0xFF)
	{
		tud_cdc_n_write(cdc_itf, Buf, Len);
		tud_cdc_n_write_flush(cdc_itf);
	}
	return 0;
}

void CAT_Buff_Write (uint8_t* pbuf, uint32_t len);

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
	uint8_t buf[64];
	uint32_t count;

	// connected() check for DTR bit
	// Most but not all terminal client set this when making connection
	if (tud_cdc_connected())
	{
		if (tud_cdc_available()) // data is available
		{
			cdc_itf = itf;
			count = tud_cdc_n_read(itf, buf, sizeof(buf));
			CAT_Buff_Write (buf, count);
		}
	}
}
