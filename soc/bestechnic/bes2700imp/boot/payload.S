/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "image_config.h"
.section .rodata.h01_payload,"a",%progbits
.balign 4
.global payload_start
.global payload_end
payload_start:
.incbin PAYLOAD_FILE
payload_end:
