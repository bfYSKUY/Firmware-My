

#pragma once

// Data Format for Benewake TFmini
// ===============================
// 9 bytes total per message:
// 1) 0x59
// 2) 0x59
// 3) Dist_L (low 8bit)
// 4) Dist_H (high 8bit)
// 5) Strength_L (low 8bit)
// 6) Strength_H (high 8bit)
// 7) Reserved bytes
// 8) Original signal quality degree
// 9) Checksum parity bit (low 8bit), Checksum = Byte1 + Byte2 +...+Byte8. This is only a low 8bit though




// format of serial packets received from benewake TFminiPlus lidar
//
// Data Bit             Definition      Description
// ------------------------------------------------
// byte 0               Frame header    0x59
// byte 1               Frame header    0x59
// byte 2               DIST_L          Distance (in cm) low 8 bits
// byte 3               DIST_H          Distance (in cm) high 8 bits
// byte 4               STRENGTH_L      Strength low 8 bits
// byte 5               STRENGTH_H      Strength high 8 bits
// byte 6               Temp_L
// byte 7		Temp_H          Exposure time in two levels 0x03 and 0x06
// byte 8               Checksum        Checksum byte, sum of bytes 0 to bytes 7



enum TFMINIPLUS_PARSE_STATE {
	TFMINIPLUS_PARSE_STATE0_UNSYNC = 0,
	TFMINIPLUS_PARSE_STATE1_SYNC_1,
	TFMINIPLUS_PARSE_STATE1_SYNC_2,
	TFMINIPLUS_PARSE_STATE2_GOT_DIST_L,
	TFMINIPLUS_PARSE_STATE2_GOT_DIST_H,
	TFMINIPLUS_PARSE_STATE3_GOT_STRENGTH_L,
	TFMINIPLUS_PARSE_STATE3_GOT_STRENGTH_H,
	TFMINIPLUS_PARSE_STATE4_GOT_RESERVED,
	TFMINIPLUS_PARSE_STATE5_GOT_QUALITY,
	TFMINIPLUS_PARSE_STATE6_GOT_CHECKSUM
};

#define BENEWAKE_FRAME_HEADER 0x59
#define BENEWAKE_FRAME_LENGTH 9
#define BENEWAKE_TFMINI_OUT_OF_RANGE_CM 1200

int tfminiplus_parse(char c, char *parserbuf, unsigned *parserbuf_index, enum TFMINIPLUS_PARSE_STATE *state, float *dist);

int tfminiplus_reading(char c, char *parserbuf, unsigned *parserbuf_index, float *dist);   ///自定义修改

