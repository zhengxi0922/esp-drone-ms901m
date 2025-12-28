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
#include "static_mem.h"
#include "sensors_atk_ms901m.h"

#define ATK_CONSOLE_LINE_MAX 128
#define ATK_CONSOLE_MAX_ARGS 16
#define ATK_CONSOLE_RAW_MAX  64
#define ATK_CONSOLE_TIMEOUT_MS 100

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

static void printHelp(void)
{
  printf("ATK commands:\n");
  printf("  atk help\n");
  printf("  atk read <reg>\n");
  printf("  atk write <reg> <byte0> [byte1]\n");
  printf("  atk raw <byte0> [byte1 ...]\n");
  printf("  atk passthru <byte0> [byte1 ...]\n");
  printf("Notes: raw/passthru prints RX bytes in hex (100ms window).\n");
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

static void handleLine(char *line)
{
  char *argv[ATK_CONSOLE_MAX_ARGS];
  int argc = splitArgs(line, argv, ATK_CONSOLE_MAX_ARGS);

  if (argc == 0)
  {
    return;
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

  if (!sensorsAtkMs901mTest())
  {
    printf("ATK-MS901M not ready\n");
    return;
  }

  if (strcmp(argv[1], "read") == 0)
  {
    handleRead(argc, argv);
  }
  else if (strcmp(argv[1], "write") == 0)
  {
    handleWrite(argc, argv);
  }
  else if (strcmp(argv[1], "raw") == 0 || strcmp(argv[1], "passthru") == 0)
  {
    handleRaw(argc, argv);
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
