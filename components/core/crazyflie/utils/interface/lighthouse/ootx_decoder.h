/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
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
 * ootx_decoder.h - lighthouse positioning ootx (slow) data receiver
 *
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>


#define OOTX_MAX_FRAME_LENGTH 43

// 内容来自 https://github.com/nairol/LighthouseRedox/blob/master/docs/Base%20Station.md#base-station-info-block
struct ootxDataFrame_s {
  uint16_t protocolVersion:6;
  uint16_t firmwareVersion:10;
  uint32_t id;
  __fp16 phase0;
  __fp16 phase1;
  __fp16 tilt0;
  __fp16 tilt1;
  uint8_t unlockCount;
  uint8_t hwVersion;
  __fp16 curve0;
  __fp16 curve1;
  int8_t accelX;
  int8_t accelY;
  int8_t accelZ;
  __fp16 gibphase0;
  __fp16 gibphase1;
  __fp16 gibmag0;
  __fp16 gibmag1;
  uint8_t mode;
  uint8_t faults;

  // 仅用于使用更长数据块的 LH 2
  __fp16 ogeephase0;
  __fp16 ogeephase1;
  __fp16 ogeemag0;
  __fp16 ogeemag1;
} __attribute__((packed));

typedef struct ootxDecoderState_s {
  int frameLength;
  int bytesReceived;

  uint16_t currentWord;

  uint32_t crc32;

  int bitInWord;
  int wordReceived;
  bool synchronized;
  int nZeros;
  enum {rxLength, rxData, rxCrc0, rxCrc1, rxDone} rxState;

  union {
    uint16_t data[(OOTX_MAX_FRAME_LENGTH+1) / 2];
    struct ootxDataFrame_s frame;
  };
} ootxDecoderState_t;

/**
 * @brief 处理下一个 OOTX 位并指示是否已解码出一帧
 *
 * 若已解码出一帧，该函数返回 true，帧数据可从 state->frame 获取。
 *
 * 当前未检查帧的 CRC32 校验和！
 *
 * @param state 解码器状态指针
 * @param data OOTX 数据位，应为 0 或 1
 * @return true 已解码出一帧（即到达该帧最后一位）
 * @return false 还未解码出一帧
 */
bool ootxDecoderProcessBit(ootxDecoderState_t * state, int data);
