// Copyright (c) Direct Drive Technology Co., Ltd. All rights reserved.
// Author: Zi Min <jianming.zeng@directdrivetech.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "can.hpp"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(can, LOG_LEVEL_INF);

static const struct device *canfd_1_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canfd1));
static const struct device *canfd_2_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canfd2));
static const struct device *canfd_3_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canfd3));

std::unique_ptr<CAN> CAN::Instance = std::make_unique<CAN>();

std::unique_ptr<CAN> CAN::getInstance()
{
        return std::move(Instance);
}

bool CAN::init()
{
       int ret = 0;
       
       if(!device_is_ready(canfd_1_dev)){
                LOG_INF("CANFD device 1 is not ready");
       }
   
       if(!device_is_ready(canfd_2_dev)){
                LOG_INF("CANFD device 2 is not ready");
       }

       if(!device_is_ready(canfd_3_dev)){
                LOG_INF("CANFD device 3 is not ready");
       }

       can_set_mode(canfd_1_dev, CAN_MODE_FD);
       can_set_mode(canfd_2_dev, CAN_MODE_FD);
       can_set_mode(canfd_3_dev, CAN_MODE_FD);

       ret = can_start(canfd_1_dev);
       if (ret < 0) {
                LOG_ERR("Failed to start CANFD device 1: %d", ret);
                return false;
       }

      ret = can_start(canfd_2_dev);
       if (ret < 0) {
                LOG_ERR("Failed to start CANFD device 2: %d", ret);
                return false;
       }
       
      ret = can_start(canfd_3_dev);
       if (ret < 0) {
                LOG_ERR("Failed to start CANFD device 3: %d", ret);
                return false;
       }

       return true;
}
