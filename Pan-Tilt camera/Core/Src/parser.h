/*
 * parser.h
 *
 *  Created on: 17-Jun-2023
 *      Author: HP
 */

#ifndef SRC_PARSER_H_
#define SRC_PARSER_H_
#define DATASIZE 10U
typedef union{

	struct DataFormat{
		uint8_t header;
		uint8_t data[DATASIZE];
		uint8_t crc;
	};
	uint32_t X;
} DataPacket;



#endif /* SRC_PARSER_H_ */
