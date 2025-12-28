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
 * storage.h: Key/Buffer persistant storage
 *
 */


#pragma once

#include <stdbool.h>
#include <stddef.h>

/**
 * 初始化存储子系统。
 *
 * 该函数不可重入。
 */
void storageInit();

/**
 * 测试存储子系统
 *
 * 检查键/缓冲区表健康状况，若不健康则格式化。
 */
bool storageTest();

/**
 * 将缓冲区存入指定 key。若表中已存在该 key，将被替换。
 *
 * 该函数可能耗时较长：若新缓冲区无可用空间，
 * 会在写入前对存储进行碎片整理。
 *
 * 若内存不足或内存损坏，该函数会失败。
 *
 * @param[key] 以空字符结尾的 key 字符串，长度需在 1 到 255 之间
 * @param[buffer] 需要存储的缓冲区指针
 * @param[length] 需要存储的缓冲区长度
 *
 * @return 成功返回 true，否则返回 false。
 */
bool storageStore(char* key, const void* buffer, size_t length);

/**
 * 根据 key 从存储中读取缓冲区。
 *
 * @param[key] 以空字符结尾的 key 字符串，长度需在 1 到 255 之间
 * @param[buffer] 用于接收数据的缓冲区指针
 * @param[length] 允许拷贝的最大长度
 *
 * @return 实际读取的数据长度。该值为接收缓冲区长度与存储数据长度中的较小者。
 *         若未找到 key，返回 0。
 */
size_t storageFetch(char *key, void* buffer, size_t length);

/**
 * 从存储中删除一个条目。
 *
 * @param[key] 以空字符结尾的 key 字符串，长度需在 1 到 255 之间
 *
 * @return 成功返回 true；若未找到 key 或发生错误返回 false。
 */
bool storageDelete(char* key);
