/**
 * ATK-MS901M USB console commands (USB-CDC/USB-Serial-JTAG).
 */
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "platform.h"
#include "static_mem.h"
#include "sensors_atk_ms901m.h"
#include "param.h"
#include "motors.h"

#define ATK_CONSOLE_LINE_MAX 128
#define ATK_CONSOLE_MAX_ARGS 16
#define ATK_CONSOLE_RAW_MAX  64
#define ATK_CONSOLE_TIMEOUT_MS 100
#define ATK_CONSOLE_FRAME_HEAD_L 0x55
#define ATK_CONSOLE_FRAME_HEAD_ACK_H 0xAF

#if USE_ATK_MS901M && defined(SENSOR_INCLUDED_ATK_MS901M) && ENABLE_ATK_USB_CONSOLE && \
    (defined(CONFIG_ESP_CONSOLE_USB_CDC) || defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG))
#define ATK_CONSOLE_ENABLED 1
#else
#define ATK_CONSOLE_ENABLED 0
#endif

#if ATK_CONSOLE_ENABLED
STATIC_MEM_TASK_ALLOC(atkConsoleTask, ATK_CONSOLE_TASK_STACKSIZE);

static void atkConsoleTask(void *param);

static int splitArgs(char *line, char **argv, int max_args)
{
  int argc = 0;
  char *p = line;

  while (*p != '\0' && argc < max_args)
  {
    while (isspace((unsigned char)*p))
    {
      p++;
    }
    if (*p == '\0')
    {
      break;
    }
    argv[argc++] = p;
    while (*p != '\0' && !isspace((unsigned char)*p))
    {
      p++;
    }
    if (*p != '\0')
    {
      *p = '\0';
      p++;
    }
  }

  return argc;
}

static bool parseU8(const char *s, uint8_t *out)
{
  char *end = NULL;
  long val = strtol(s, &end, 0);
  if ((s == end) || (*end != '\0') || (val < 0) || (val > 0xFF))
  {
    return false;
  }
  *out = (uint8_t)val;
  return true;
}

static bool parsePercentU16(const char *s, uint16_t *ratio_out)
{
  char *end = NULL;
  long val = strtol(s, &end, 0);
  if ((s == end) || (*end != '\0') || (val < 0) || (val > 100))
  {
    return false;
  }
  *ratio_out = (uint16_t)((val * 65535L + 50) / 100);
  return true;
}

static void toLowerStr(char *s)
{
  while (*s != '\0')
  {
    *s = (char)tolower((unsigned char)*s);
    s++;
  }
}

static bool initMotorParams(paramVarId_t *enableId,
                            paramVarId_t *m1Id,
                            paramVarId_t *m2Id,
                            paramVarId_t *m3Id,
                            paramVarId_t *m4Id,
                            paramVarId_t *forceArmId)
{
  static bool inited = false;
  static paramVarId_t cachedEnable;
  static paramVarId_t cachedM1;
  static paramVarId_t cachedM2;
  static paramVarId_t cachedM3;
  static paramVarId_t cachedM4;
  static paramVarId_t cachedForceArm;

  if (!inited)
  {
    cachedEnable = paramGetVarId("motorPowerSet", "enable");
    cachedM1 = paramGetVarId("motorPowerSet", "m1");
    cachedM2 = paramGetVarId("motorPowerSet", "m2");
    cachedM3 = paramGetVarId("motorPowerSet", "m3");
    cachedM4 = paramGetVarId("motorPowerSet", "m4");
    cachedForceArm = paramGetVarId("system", "forceArm");
    inited = true;
  }

  *enableId = cachedEnable;
  *m1Id = cachedM1;
  *m2Id = cachedM2;
  *m3Id = cachedM3;
  *m4Id = cachedM4;
  *forceArmId = cachedForceArm;

  return PARAM_VARID_IS_VALID(cachedEnable) &&
         PARAM_VARID_IS_VALID(cachedM1) &&
         PARAM_VARID_IS_VALID(cachedM2) &&
         PARAM_VARID_IS_VALID(cachedM3) &&
         PARAM_VARID_IS_VALID(cachedM4);
}

static bool readAxis3WithTimeout(bool (*reader)(Axis3f *), Axis3f *out, uint32_t timeout_ms)
{
  TickType_t start = xTaskGetTickCount();
  TickType_t timeout = pdMS_TO_TICKS(timeout_ms);

  while ((xTaskGetTickCount() - start) < timeout)
  {
    if (reader(out))
    {
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  return false;
}

static bool readBaroWithTimeout(baro_t *baro, uint32_t timeout_ms)
{
  TickType_t start = xTaskGetTickCount();
  TickType_t timeout = pdMS_TO_TICKS(timeout_ms);

  while ((xTaskGetTickCount() - start) < timeout)
  {
    if (sensorsAtkMs901mReadBaro(baro))
    {
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  return false;
}

static bool readAttitudeWithTimeout(attitude_t *attitude, uint32_t timeout_ms)
{
  TickType_t start = xTaskGetTickCount();
  TickType_t timeout = pdMS_TO_TICKS(timeout_ms);

  while ((xTaskGetTickCount() - start) < timeout)
  {
    if (sensorsAtkMs901mGetAttitude(attitude))
    {
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  return false;
}

static bool readQuaternionWithTimeout(quaternion_t *quat, uint32_t timeout_ms)
{
  TickType_t start = xTaskGetTickCount();
  TickType_t timeout = pdMS_TO_TICKS(timeout_ms);

  while ((xTaskGetTickCount() - start) < timeout)
  {
    if (sensorsAtkMs901mGetQuaternion(quat))
    {
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  return false;
}

static void printAxis3(const char *label, const Axis3f *v, const char *unit)
{
  printf("%s: x=%.3f y=%.3f z=%.3f %s\n", label, v->x, v->y, v->z, unit);
}

static void printAtt(const attitude_t *att)
{
  printf("att[deg]: roll=%.2f pitch=%.2f yaw=%.2f\n", att->roll, att->pitch, att->yaw);
}

static void printQuat(const quaternion_t *quat)
{
  printf("quat: q0=%.4f q1=%.4f q2=%.4f q3=%.4f\n", quat->q0, quat->q1, quat->q2, quat->q3);
}

#if ATK_MS901M_ECHO_PC_TX
static void printTxBytes(const uint8_t *buf, uint8_t len)
{
  if ((buf == NULL) || (len == 0))
  {
    return;
  }

  printf("atk tx:");
  for (uint8_t i = 0; i < len; i++)
  {
    printf(" %02X", buf[i]);
  }
  printf("\n");
}
#endif

static void printHelp(void)
{
  printf("ATK commands:\n");
  printf("  atk help\n");
  printf("  atk acc | gyro | mag | baro | imu\n");
  printf("  atk att | quat\n");
  printf("  atk motor <duty%%> | <m1%%> <m2%%> <m3%%> <m4%%>\n");
  printf("  atk motor off | arm | disarm\n");
  printf("  atk read <reg>\n");
  printf("  atk write <reg> <byte0> [byte1]\n");
  printf("  atk raw <byte0> [byte1 ...]\n");
  printf("  atk passthru <byte0> [byte1 ...]\n");
  printf("Notes: raw/passthru prints RX bytes in hex (100ms window).\n");
  printf("Tip: enable att/quat with 'atk write 0x08 <mask>' (bit0/bit1).\n");
}

static void handleAcc(void)
{
  Axis3f acc;
  quaternion_t quat;

  if (!readAxis3WithTimeout(sensorsAtkMs901mReadAcc, &acc, ATK_CONSOLE_TIMEOUT_MS))
  {
    printf("acc: no data\n");
    return;
  }

  printAxis3("acc[g]", &acc, "");

  if (readQuaternionWithTimeout(&quat, ATK_CONSOLE_TIMEOUT_MS))
  {
    printQuat(&quat);
  }
  else
  {
    printf("quat: n/a (enable returnset bit1)\n");
  }
}

static void handleGyro(void)
{
  Axis3f gyro;

  if (!readAxis3WithTimeout(sensorsAtkMs901mReadGyro, &gyro, ATK_CONSOLE_TIMEOUT_MS))
  {
    printf("gyro: no data\n");
    return;
  }

  printAxis3("gyro[dps]", &gyro, "");
}

static void handleMag(void)
{
  Axis3f mag;

  if (!readAxis3WithTimeout(sensorsAtkMs901mReadMag, &mag, ATK_CONSOLE_TIMEOUT_MS))
  {
    printf("mag: no data\n");
    return;
  }

  printAxis3("mag[gauss]", &mag, "");
}

static void handleBaro(void)
{
  baro_t baro;

  if (!readBaroWithTimeout(&baro, ATK_CONSOLE_TIMEOUT_MS))
  {
    printf("baro: no data\n");
    return;
  }

  printf("baro: p=%.2f mbar t=%.2f C alt=%.2f m\n", baro.pressure, baro.temperature, baro.asl);
}

static void handleAtt(void)
{
  attitude_t att;

  if (!readAttitudeWithTimeout(&att, ATK_CONSOLE_TIMEOUT_MS))
  {
    printf("att: no data (enable returnset bit0)\n");
    return;
  }

  printAtt(&att);
}

static void handleQuat(void)
{
  quaternion_t quat;

  if (!readQuaternionWithTimeout(&quat, ATK_CONSOLE_TIMEOUT_MS))
  {
    printf("quat: no data (enable returnset bit1)\n");
    return;
  }

  printQuat(&quat);
}

static void handleImu(void)
{
  handleGyro();
  handleAcc();
  handleMag();
  handleBaro();
}

static void handleRead(int argc, char **argv)
{
  uint8_t reg = 0;
  uint8_t data[32];
  uint8_t len = 0;

  if (argc < 3 || !parseU8(argv[2], &reg))
  {
    printf("usage: atk read <reg>\n");
    return;
  }

#if ATK_MS901M_ECHO_PC_TX
  uint8_t frame[6];
  frame[0] = ATK_CONSOLE_FRAME_HEAD_L;
  frame[1] = ATK_CONSOLE_FRAME_HEAD_ACK_H;
  frame[2] = (uint8_t)(reg | 0x80);
  frame[3] = 0x01;
  frame[4] = 0x00;
  frame[5] = (uint8_t)(frame[0] + frame[1] + frame[2] + frame[3] + frame[4]);
  printTxBytes(frame, sizeof(frame));
#endif

  if (!sensorsAtkMs901mReadReg(reg, data, &len, ATK_CONSOLE_TIMEOUT_MS))
  {
    printf("read 0x%02X failed\n", reg);
    return;
  }

  printf("read 0x%02X len=%u:", reg, (unsigned)len);
  for (uint8_t i = 0; i < len; i++)
  {
    printf(" %02X", data[i]);
  }
  printf("\n");
}

static void handleWrite(int argc, char **argv)
{
  uint8_t reg = 0;
  uint8_t data[2];
  uint8_t len = 0;

  if (argc < 4 || !parseU8(argv[2], &reg))
  {
    printf("usage: atk write <reg> <byte0> [byte1]\n");
    return;
  }

  if (!parseU8(argv[3], &data[0]))
  {
    printf("invalid byte0\n");
    return;
  }
  len = 1;

  if (argc >= 5)
  {
    if (!parseU8(argv[4], &data[1]))
    {
      printf("invalid byte1\n");
      return;
    }
    len = 2;
  }

#if ATK_MS901M_ECHO_PC_TX
  uint8_t frame[7];
  uint8_t frame_len = (len == 2) ? 7 : 6;
  frame[0] = ATK_CONSOLE_FRAME_HEAD_L;
  frame[1] = ATK_CONSOLE_FRAME_HEAD_ACK_H;
  frame[2] = reg;
  frame[3] = len;
  frame[4] = data[0];
  if (len == 2)
  {
    frame[5] = data[1];
    frame[6] = (uint8_t)(frame[0] + frame[1] + frame[2] + frame[3] + frame[4] + frame[5]);
  }
  else
  {
    frame[5] = (uint8_t)(frame[0] + frame[1] + frame[2] + frame[3] + frame[4]);
  }
  printTxBytes(frame, frame_len);
#endif

  if (!sensorsAtkMs901mWriteReg(reg, data, len))
  {
    printf("write 0x%02X failed\n", reg);
    return;
  }

  printf("write 0x%02X ok\n", reg);
}

static void handleRaw(int argc, char **argv)
{
  uint8_t tx[ATK_CONSOLE_RAW_MAX];
  uint8_t rx[ATK_CONSOLE_RAW_MAX];
  uint8_t tx_len = 0;

  if (argc < 3)
  {
    printf("usage: atk raw <byte0> [byte1 ...]\n");
    return;
  }

  for (int i = 2; i < argc; i++)
  {
    if (tx_len >= ATK_CONSOLE_RAW_MAX)
    {
      printf("tx too long (max %u bytes)\n", (unsigned)ATK_CONSOLE_RAW_MAX);
      return;
    }
    if (!parseU8(argv[i], &tx[tx_len]))
    {
      printf("invalid byte: %s\n", argv[i]);
      return;
    }
    tx_len++;
  }

#if ATK_MS901M_ECHO_PC_TX
  printTxBytes(tx, tx_len);
#endif

  int rx_len = sensorsAtkMs901mRawTransfer(tx, tx_len, rx, sizeof(rx), ATK_CONSOLE_TIMEOUT_MS);
  if (rx_len <= 0)
  {
    printf("rx empty\n");
    return;
  }

  printf("rx:");
  for (int i = 0; i < rx_len; i++)
  {
    printf(" %02X", rx[i]);
  }
  printf("\n");
}

static void handleMotor(int argc, char **argv)
{
  paramVarId_t enableId;
  paramVarId_t m1Id;
  paramVarId_t m2Id;
  paramVarId_t m3Id;
  paramVarId_t m4Id;
  paramVarId_t forceArmId;

  if (!initMotorParams(&enableId, &m1Id, &m2Id, &m3Id, &m4Id, &forceArmId))
  {
    printf("motor params not ready\n");
    return;
  }

  if (argc < 3)
  {
    printf("usage: atk motor <duty%%> | <m1%%> <m2%%> <m3%%> <m4%%>\n");
    printf("       atk motor off | arm | disarm\n");
    return;
  }

  if (strcmp(argv[2], "off") == 0)
  {
    paramSetInt(m1Id, 0);
    paramSetInt(m2Id, 0);
    paramSetInt(m3Id, 0);
    paramSetInt(m4Id, 0);
    paramSetInt(enableId, 0);
    printf("motor off\n");
    return;
  }

  if (strcmp(argv[2], "arm") == 0)
  {
    if (PARAM_VARID_IS_VALID(forceArmId))
    {
      paramSetInt(forceArmId, 1);
      printf("system.forceArm=1\n");
    }
    else
    {
      printf("system.forceArm not available\n");
    }
    return;
  }

  if (strcmp(argv[2], "disarm") == 0)
  {
    if (PARAM_VARID_IS_VALID(forceArmId))
    {
      paramSetInt(forceArmId, 0);
    }
    paramSetInt(m1Id, 0);
    paramSetInt(m2Id, 0);
    paramSetInt(m3Id, 0);
    paramSetInt(m4Id, 0);
    paramSetInt(enableId, 0);
    printf("motor disarm\n");
    return;
  }

  uint16_t ratio[4] = {0};
  if (argc == 3)
  {
    if (!parsePercentU16(argv[2], &ratio[0]))
    {
      printf("invalid duty (0-100)\n");
      return;
    }
    ratio[1] = ratio[2] = ratio[3] = ratio[0];
  }
  else if (argc == 6)
  {
    for (int i = 0; i < 4; i++)
    {
      if (!parsePercentU16(argv[2 + i], &ratio[i]))
      {
        printf("invalid duty (0-100)\n");
        return;
      }
    }
  }
  else
  {
    printf("usage: atk motor <duty%%> | <m1%%> <m2%%> <m3%%> <m4%%>\n");
    return;
  }

  if (PARAM_VARID_IS_VALID(forceArmId) && paramGetInt(forceArmId) == 0)
  {
    printf("note: system.forceArm=0, motors may be stopped\n");
  }

  paramSetInt(m1Id, ratio[0]);
  paramSetInt(m2Id, ratio[1]);
  paramSetInt(m3Id, ratio[2]);
  paramSetInt(m4Id, ratio[3]);
  paramSetInt(enableId, 1);
  if (PARAM_VARID_IS_VALID(forceArmId) && paramGetInt(forceArmId) == 0)
  {
    printf("motor duty set (armed=0, pwm not forced)\n");
    return;
  }

  motorsSetRatio(MOTOR_M1, ratio[0]);
  motorsSetRatio(MOTOR_M2, ratio[1]);
  motorsSetRatio(MOTOR_M3, ratio[2]);
  motorsSetRatio(MOTOR_M4, ratio[3]);
  printf("motor duty set (direct pwm)\n");
}

static void handleLine(char *line)
{
  char *argv[ATK_CONSOLE_MAX_ARGS];
  int argc = splitArgs(line, argv, ATK_CONSOLE_MAX_ARGS);

  if (argc == 0)
  {
    return;
  }

  for (int i = 0; i < argc; i++)
  {
    toLowerStr(argv[i]);
  }

  if (strcmp(argv[0], "atk") != 0)
  {
    printf("unknown command: %s (try 'atk help')\n", argv[0]);
    return;
  }

  if (argc == 1 || strcmp(argv[1], "help") == 0)
  {
    printHelp();
    return;
  }

  bool ready = sensorsAtkMs901mTest();
  if (!ready)
  {
    if ((strcmp(argv[1], "read") != 0) &&
        (strcmp(argv[1], "write") != 0) &&
        (strcmp(argv[1], "raw") != 0) &&
        (strcmp(argv[1], "passthru") != 0))
    {
      printf("ATK-MS901M not ready\n");
      return;
    }
    printf("ATK-MS901M not ready (raw/read/write only)\n");
  }

  if (strcmp(argv[1], "read") == 0)
  {
    handleRead(argc, argv);
  }
  else if (strcmp(argv[1], "acc") == 0)
  {
    handleAcc();
  }
  else if (strcmp(argv[1], "gyro") == 0)
  {
    handleGyro();
  }
  else if (strcmp(argv[1], "mag") == 0)
  {
    handleMag();
  }
  else if (strcmp(argv[1], "baro") == 0)
  {
    handleBaro();
  }
  else if (strcmp(argv[1], "att") == 0)
  {
    handleAtt();
  }
  else if (strcmp(argv[1], "quat") == 0)
  {
    handleQuat();
  }
  else if (strcmp(argv[1], "imu") == 0 || strcmp(argv[1], "all") == 0)
  {
    handleImu();
  }
  else if (strcmp(argv[1], "write") == 0)
  {
    handleWrite(argc, argv);
  }
  else if (strcmp(argv[1], "raw") == 0 || strcmp(argv[1], "passthru") == 0)
  {
    handleRaw(argc, argv);
  }
  else if (strcmp(argv[1], "motor") == 0)
  {
    handleMotor(argc, argv);
  }
  else
  {
    printf("unknown atk subcommand: %s\n", argv[1]);
  }
}

static void atkConsoleTask(void *param)
{
  char line[ATK_CONSOLE_LINE_MAX];
  size_t len = 0;

  (void)param;
  setvbuf(stdin, NULL, _IONBF, 0);
  setvbuf(stdout, NULL, _IONBF, 0);

  printf("\n[ATK] USB console ready. Type 'atk help'.\n");

  while (1)
  {
    int ch = getchar();
    if (ch < 0)
    {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if (ch == '\r')
    {
      continue;
    }
    if (ch == '\n')
    {
      line[len] = '\0';
      handleLine(line);
      len = 0;
      continue;
    }
    if (ch == 0x08 || ch == 0x7F)
    {
      if (len > 0)
      {
        len--;
      }
      continue;
    }

    if (len < (sizeof(line) - 1))
    {
      line[len++] = (char)ch;
    }
  }
}
#endif

void atkConsoleInit(void)
{
#if ATK_CONSOLE_ENABLED
  STATIC_MEM_TASK_CREATE(atkConsoleTask, atkConsoleTask, ATK_CONSOLE_TASK_NAME, NULL, ATK_CONSOLE_TASK_PRI);
#else
  (void)0;
#endif
}
