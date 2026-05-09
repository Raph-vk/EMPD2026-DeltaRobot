/*
 * Thin wrapper for the only u8g2 display setup used by this project.
 * This keeps the build from compiling the full generated u8g2_d_setup.c file.
 */

#include "u8g2.h"

void u8g2_Setup_ssd1309_i2c_128x64_noname0_1(
    u8g2_t *u8g2,
    const u8g2_cb_t *rotation,
    u8x8_msg_cb byte_cb,
    u8x8_msg_cb gpio_and_delay_cb)
{
  uint8_t tile_buf_height;
  uint8_t *buf;

  u8g2_SetupDisplay(u8g2, u8x8_d_ssd1309_128x64_noname0, u8x8_cad_ssd13xx_i2c, byte_cb, gpio_and_delay_cb);
  buf = u8g2_m_16_8_1(&tile_buf_height);
  u8g2_SetupBuffer(u8g2, buf, tile_buf_height, u8g2_ll_hvline_vertical_top_lsb, rotation);
}
