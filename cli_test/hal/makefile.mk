# Copyright 2020 ETH Zurich
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0
# Author: Robert Balas (balasr@iis.ee.ethz.ch)

SRCS += $(dir)/source/hal_apb_i2cs.c
SRCS += $(dir)/source/hal_fc_event.c
SRCS += $(dir)/source/hal_fll_pi.c
SRCS += $(dir)/source/hal_fll.c
SRCS += $(dir)/source/hal_gpio_pulp.c
SRCS += $(dir)/source/hal_gpio.c
SRCS += $(dir)/source/hal_irq.c
SRCS += $(dir)/source/hal_pinmux.c
SRCS += $(dir)/source/hal_pinmux1.c
SRCS += $(dir)/source/hal_soc_eu.c
SRCS += $(dir)/source/hal_timer_irq.c

#CV_CPPFLAGS += -I$(dir)/include