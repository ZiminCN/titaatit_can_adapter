#pragma once
#include "struct.h"

bool decodec_pack(struct ddt_protocol *decodec_data, unsigned char data);
void decodec_init(struct ddt_protocol *decodec_data);
uint32_t encodec_pack(struct adapter_data_t *adapter, uint16_t cmd, uint8_t *data, int len);
