#include "struct.h"
#include "can.h"
#include "acm.h"

LOG_MODULE_REGISTER(main, CONFIG_TITA_ADAPTER_LOG_LEVEL);

static struct adapter_data_t m_adapter_data = {.master_canfd.router_tb[0] =
						       {
							       .is_enable = true,
							       .rang_canid_min = 0x124,
							       .rang_canid_max = 0x124 + 4,
							       .is_tx2master = false,
							       .is_tx2slave = true,
							       .is_tx2peripheral = false,
							       .offset_master = 0,
							       .offset_slave = -4,
							       .offset_peripheral = 0,
						       },
					       .slave_canfd.router_tb[0] = {
						       .is_enable = true,
						       .rang_canid_min = 0x108,
						       .rang_canid_max = 0x108 + 2,
						       .is_tx2master = true,
						       .is_tx2slave = false,
						       .is_tx2peripheral = false,
						       .offset_master = 2,
						       .offset_slave = 0,
						       .offset_peripheral = 0,
					       }};

void main(void)
{
	can_init(&m_adapter_data);
	acm_init(&m_adapter_data);

	while (1) {
		k_sleep(K_MSEC(1000));
	}
}
