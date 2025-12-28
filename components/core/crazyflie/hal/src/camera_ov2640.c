/**
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * camera_ov2640.c - OV2640 camera streaming over Wi-Fi
 */

#include "camera_ov2640.h"
#include "config.h"
#define DEBUG_MODULE "CAM"
#include "debug_cf.h"

#if ENABLE_OV2640_STREAM

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_camera.h"
#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_psram.h"

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static httpd_handle_t stream_httpd;
static bool isInit;
static bool camOk;

static esp_err_t camera_capture_handler(httpd_req_t *req)
{
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

static esp_err_t camera_stream_handler(httpd_req_t *req)
{
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      return ESP_FAIL;
    }

    uint8_t *jpg_buf = fb->buf;
    size_t jpg_buf_len = fb->len;

    if (fb->format != PIXFORMAT_JPEG) {
      bool converted = frame2jpg(fb, OV2640_JPEG_QUALITY, &jpg_buf, &jpg_buf_len);
      esp_camera_fb_return(fb);
      if (!converted) {
        return ESP_FAIL;
      }
    }

    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) {
      char part_buf[64];
      int hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, (unsigned int)jpg_buf_len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)jpg_buf, jpg_buf_len);
    }

    if (fb->format == PIXFORMAT_JPEG) {
      esp_camera_fb_return(fb);
    } else {
      free(jpg_buf);
    }

    if (res != ESP_OK) {
      break;
    }
  }

  return res;
}

static void camera_start_httpd(void)
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = OV2640_HTTP_PORT;

  if (httpd_start(&stream_httpd, &config) != ESP_OK) {
    DEBUG_PRINTW("camera http server start failed");
    return;
  }

  httpd_uri_t capture_uri = {
    .uri = "/capture",
    .method = HTTP_GET,
    .handler = camera_capture_handler,
    .user_ctx = NULL,
  };

  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = camera_stream_handler,
    .user_ctx = NULL,
  };

  httpd_register_uri_handler(stream_httpd, &capture_uri);
  httpd_register_uri_handler(stream_httpd, &stream_uri);
}

void cameraOv2640Init(void)
{
  if (isInit) {
    return;
  }

  camera_config_t config = {
    .pin_pwdn = OV2640_PIN_PWDN,
    .pin_reset = OV2640_PIN_RESET,
    .pin_xclk = OV2640_PIN_XCLK,
    .pin_sccb_sda = OV2640_PIN_SDA,
    .pin_sccb_scl = OV2640_PIN_SCL,

    .pin_d7 = OV2640_PIN_Y9,
    .pin_d6 = OV2640_PIN_Y8,
    .pin_d5 = OV2640_PIN_Y7,
    .pin_d4 = OV2640_PIN_Y6,
    .pin_d3 = OV2640_PIN_Y5,
    .pin_d2 = OV2640_PIN_Y4,
    .pin_d1 = OV2640_PIN_Y3,
    .pin_d0 = OV2640_PIN_Y2,
    .pin_vsync = OV2640_PIN_VSYNC,
    .pin_href = OV2640_PIN_HREF,
    .pin_pclk = OV2640_PIN_PCLK,

    .xclk_freq_hz = OV2640_XCLK_FREQ_HZ,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = OV2640_JPEG_QUALITY,
    .fb_count = 2,
    .grab_mode = CAMERA_GRAB_LATEST,
  };

  if (!psramFound()) {
    config.frame_size = FRAMESIZE_QQVGA;
    config.fb_count = 1;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    DEBUG_PRINTW("camera init failed, err=0x%x", err);
    camOk = false;
    isInit = true;
    return;
  }

  camera_start_httpd();
  camOk = true;
  isInit = true;
}

bool cameraOv2640Test(void)
{
  return camOk;
}

#else

void cameraOv2640Init(void)
{
}

bool cameraOv2640Test(void)
{
  return false;
}

#endif
