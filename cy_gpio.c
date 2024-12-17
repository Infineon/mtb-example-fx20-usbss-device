/***************************************************************************//**
* \file cy_gpio.c
* \version 1.0
*
* Provides GPIO update functions used in the FX20 middleware library.
*
*******************************************************************************
* \copyright
* (c) (2021-2023), Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <stdint.h>
#include <stddef.h>

void Cy_GPIO_Set(uint32_t* base, uint32_t pinNum)
{
	if ((base != NULL) && (pinNum < 8)) {
		base[2] = (1UL << pinNum);
	}
}

void Cy_GPIO_Clr(uint32_t* base, uint32_t pinNum)
{
	if ((base != NULL) && (pinNum < 8)) {
		base[1] = (1UL << pinNum);
	}
}


