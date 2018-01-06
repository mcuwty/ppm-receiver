#include "sbus.h"
#include "usart.h"

uint16_t sbus_channel[16] = {0};

void sbus_buffer_init(struct sbus_buffer *sbus_b)
{
	sbus_b->frame_available = FALSE;
	sbus_b->status = SBUS_STATUS_UNINIT;
	sbus_b->frame_capture_faile = 0;
	sbus_b->frame_decode_faile = 0;
	sbus_b->frame_count = 0;
	sbus_b->rssi = 0;
}

void encode_sbus_frame(uint16_t *values, uint16_t num_values, uint8_t *oframe)
{
	uint8_t byteindex = 1; /*Data starts one byte into the sbus frame. */
	uint8_t offset = 0;
	uint16_t value;
	int i;

  	osal_memset(oframe, 0, SBUS_FRAME_SIZE);
	oframe[0]=SBUS_START_BYTE;

	/* 16 is sbus number of servos/channels minus 2 single bit channels.
	* currently ignoring single bit channels.  */
	for (i = 0; (i < num_values) && (i < SBUS_MAX_RC_COUNT); ++i) 
	{
		value = (uint16_t)values[i];
    	/*protect from out of bounds values and limit to 11 bits*/
    	if (value > 0x07ff) 
    	{
			value = 0x07ff;
    	}
		while (offset >= 8) 
		{
			++byteindex;
			offset -= 8;
      	}
  		oframe[byteindex] |= (value << (offset)) & 0xff;
  		oframe[byteindex + 1] |= (value >> (8 - offset)) & 0xff;
  		oframe[byteindex + 2] |= (value >> (16 - offset)) & 0xff;
  		offset += 11;
	}

	oframe[SBUS_FLAGS_BYTE_IDX]= 0;
	oframe[SBUS_END_BYTE_IDX]= SBUS_END_BYTE;
}

static uint8_t oframe[SBUS_FRAME_SIZE] = { 0x0f };
void send_sbus_out (uint16_t *rc_chans, uint16_t rc_count, void (*cb)(uint8_t * data, int len))
{
	encode_sbus_frame(rc_chans,rc_count, oframe);
	cb( oframe, SBUS_FRAME_SIZE);
}
