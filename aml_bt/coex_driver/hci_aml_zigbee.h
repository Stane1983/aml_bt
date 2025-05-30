/******************************************************************************
*
*  Copyright (C) 2019-2025 Amlogic Corporation
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at:
*
*  http://www.apache.org/licenses/LICENSE-2.0
*
*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS,
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*  See the License for the specific language governing permissions and
*  limitations under the License.
*
******************************************************************************/

#ifndef __AML_ZIGBEE_H__
#define __AML_ZIGBEE_H__

#define AML_CHAR_ZIGBEE_DEVICE_NAME "aml_zigbee"

#define ZIGBEE_MAX_FRAME_SIZE 1024

int aml_zigbee_init(void);
int aml_zigbee_deinit(void);
int aml_zigbee_recv_frame(struct hci_dev *hdev, struct sk_buff *skb);

#endif

