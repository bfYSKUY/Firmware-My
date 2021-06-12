

#include "tfminiplus_parser.h"
#include <string.h>
#include <stdlib.h>

int tfminiplus_parse(char c, char *parserbuf, unsigned *parserbuf_index, enum TFMINIPLUS_PARSE_STATE *state, float *dist)
{
	int ret = -1;
	//char *end;

	switch (*state) {
	case TFMINIPLUS_PARSE_STATE6_GOT_CHECKSUM:
		if (c == 'Y') {
			*state = TFMINIPLUS_PARSE_STATE1_SYNC_1;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;

		} else {
			*state = TFMINIPLUS_PARSE_STATE0_UNSYNC;
		}

		break;

	case TFMINIPLUS_PARSE_STATE0_UNSYNC:
		if (c == 'Y') {
			*state = TFMINIPLUS_PARSE_STATE1_SYNC_1;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;
		}

		break;

	case TFMINIPLUS_PARSE_STATE1_SYNC_1:
		if (c == 'Y') {
			*state = TFMINIPLUS_PARSE_STATE1_SYNC_2;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;

		} else {
			*state = TFMINIPLUS_PARSE_STATE0_UNSYNC;
			*parserbuf_index = 0;
		}

		break;

	case TFMINIPLUS_PARSE_STATE1_SYNC_2:
		*state = TFMINIPLUS_PARSE_STATE2_GOT_DIST_L;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case TFMINIPLUS_PARSE_STATE2_GOT_DIST_L:
		*state = TFMINIPLUS_PARSE_STATE2_GOT_DIST_H;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case TFMINIPLUS_PARSE_STATE2_GOT_DIST_H:
		*state = TFMINIPLUS_PARSE_STATE3_GOT_STRENGTH_L;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case TFMINIPLUS_PARSE_STATE3_GOT_STRENGTH_L:
		*state = TFMINIPLUS_PARSE_STATE3_GOT_STRENGTH_H;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case TFMINIPLUS_PARSE_STATE3_GOT_STRENGTH_H:
		*state = TFMINIPLUS_PARSE_STATE4_GOT_RESERVED;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case TFMINIPLUS_PARSE_STATE4_GOT_RESERVED:
		*state = TFMINIPLUS_PARSE_STATE5_GOT_QUALITY;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case TFMINIPLUS_PARSE_STATE5_GOT_QUALITY:
		// Find the checksum
		unsigned char cksm = 0;

		for (int i = 0; i < 8; i++) {
			cksm += parserbuf[i];
		}

		if (c == cksm) {
			parserbuf[*parserbuf_index] = '\0';
			unsigned int t1 = parserbuf[2];
			unsigned int t2 = parserbuf[3];
			t2 <<= 8;
			t2 += t1;
			*dist = ((float)t2) / 100;
			*state = TFMINIPLUS_PARSE_STATE6_GOT_CHECKSUM;
			*parserbuf_index = 0;
			ret = 0;

		} else {
			*state = TFMINIPLUS_PARSE_STATE0_UNSYNC;
			*parserbuf_index = 0;
		}

		break;
	}

#ifdef TFMINIPLUS_DEBUG
	printf("state: TFMINIPLUS_PARSE_STATE%s\n", parser_state[*state]);
#endif

	return ret;
}


int tfminiplus_reading(char c, char *parserbuf, unsigned *parserbuf_index, float *dist)
{
	int ret = -1;
	//char *end;
	float sum_range = 0;
    	uint16_t count = 0;
	uint16_t count_out_of_range = 0;
	uint16_t reading_cm = 0;

	if (c == BENEWAKE_FRAME_HEADER){
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;
	}else if ((*parserbuf_index) == 2){
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;
	}else if ((*parserbuf_index) == 3){
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

	}else {
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;
	}

	if(sizeof(parserbuf) == BENEWAKE_FRAME_LENGTH){
		(*parserbuf_index) = 0;  // clear buffer

		// calculate checksum
                uint8_t checksum = 0;
                for (uint8_t j=0; j<BENEWAKE_FRAME_LENGTH-1; j++) {
                    checksum += parserbuf[j];
                }
                // if checksum matches extract contents
                if (checksum == parserbuf[BENEWAKE_FRAME_LENGTH-1]) {
                    // calculate distance
                    uint16_t distl = ((uint16_t)parserbuf[3] << 8) | parserbuf[2];
                    if (distl >= BENEWAKE_TFMINI_OUT_OF_RANGE_CM) {
                        // this reading is out of range
                        count_out_of_range++;
                    } else {
                        sum_range += distl;
                        count++;
                    }
		    ret = 0;
                }

	}

	if (count > 0){
		reading_cm = (sum_range / count);
		*dist = ((float)reading_cm) / 100;
	}

	return ret;
}
