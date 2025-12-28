/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 - 2020 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * kveStorage.h - Low level storage functions
 *
 */

/**
 * 这些函数用于嵌入式键值模块的内部使用，
 * 不建议被其他模块直接调用。
 */

#pragma once

#include "kve/kve_common.h"

#include <stddef.h>
#include <stdint.h>


#define KVE_STORAGE_IS_VALID(a) (a != SIZE_MAX)

#define KVE_STORAGE_INVALID_ADDRESS (SIZE_MAX)

#define END_TAG_LENDTH 2

typedef struct itemHeader_s {
  uint16_t full_length;
  uint8_t key_length;
} __attribute((packed)) kveItemHeader_t;

/** 在地址 "address" 处添加条目
 *
 * 这是一个工具函数，不做任何检查，调用方需确保不会越界访问
 *
 * 返回该条目在内存中的完整长度
 */
int kveStorageWriteItem(kveMemory_t *kve, size_t address, const char* key, const void* buffer, size_t length);

/** 在指定地址写入长度为 full_length 的空洞
 *
 * 空洞长度必须至少为 3 字节，本函数不做检查！
 *
 * 返回该条目在内存中的完整长度
 */
uint16_t kveStorageWriteHole(kveMemory_t *kve, size_t address, size_t full_length);

/** 在指定地址写入表结束标记
 *
 * 本函数不做检查，指定地址处应至少有 2 字节可用
 *
 * 返回该条目在内存中的完整长度
 */
uint16_t kveStorageWriteEnd(kveMemory_t *kve, size_t address);

/** 将一块内存从一个地址移动到另一个地址
 *
 * 源地址必须大于目标地址
 */
void kveStorageMoveMemory(kveMemory_t *kve, size_t sourceAddress, size_t destinationAddress, size_t length);

size_t kveStorageFindItemByKey(kveMemory_t *kve, size_t address, const char * key);

/** 查找并返回表结束的地址
 *
 * 地址可设置为某个条目的起始位置，以便从表中部开始搜索。
 *
 * 若在找到结束标记前已到达内存末尾，将返回无效地址。
 * 若发生此情况，表已损坏！（可用于检测 kve 表健康状况）
 */
size_t kveStorageFindEnd(kveMemory_t *kve, size_t address);

size_t kveStorageFindHole(kveMemory_t *kve, size_t address);

size_t kveStorageFindNextItem(kveMemory_t *kve, size_t address);

kveItemHeader_t kveStorageGetItemInfo(kveMemory_t *kve, size_t address);

size_t kveStorageGetKeyLength(kveItemHeader_t header);

size_t kveStorageGetBufferLength(kveItemHeader_t header);

size_t kveStorageGetKey(kveMemory_t *kve, size_t address, kveItemHeader_t header, char* key, size_t maxLength);

size_t kveStorageGetBuffer(kveMemory_t *kve, size_t address, kveItemHeader_t header, void* buffer, size_t maxLength);
