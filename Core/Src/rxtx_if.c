/**
 *******************************************************************************
 *
 * @file    ptt_if.c
 * @brief   PTT driver
 * @version v2.0
 * @date    18.09.2024
 * @author  Dmitrii Rudnev
 *
 *******************************************************************************
 * Copyrigh &copy; 2024 Selenite Project. All rights reserved.
 *
 * This software component is licensed under [BSD 3-Clause license]
 * (http://opensource.org/licenses/BSD-3-Clause/), the "License".<br>
 * You may not use this file except in compliance with the License.
 *******************************************************************************
 */

#include "main.h"
#include "rxtx_if.h"
#include "i2c_if.h"
#include "dsp_if.h"
#include "user_if.h"
#include "si5351a.h"
#include <string.h>

void CDC_Transmit_FS (uint8_t *Buf, uint16_t Len);

#define CAT_BUFF_SIZE 20U

typedef struct
{
    uint16_t wr_ptr;
    uint16_t rd_ptr;
    uint8_t read_tx_state;
    uint8_t get_freq [5];
    uint8_t buff [CAT_BUFF_SIZE];
} CAT_TypeDef;

TRX_TypeDef trx;
PTT_TypeDef ptt;
CAT_TypeDef cat = { .wr_ptr = 0, .rd_ptr = 0, .read_tx_state = 0 };

extern RTC_HandleTypeDef hrtc;

/**
 * @brief This function sets si5351a frequency
 *
 */

void vfo_set_freq (uint32_t freq)
{
    Si5351a_Set_Freq (freq, 0, 90);
}

/**
 * @brief This function converts HEX to BCD format
 *
 */

uint32_t HEX_to_BCD (uint32_t input)
{
    int8_t i;
    uint32_t output = 0U;

    if (input < 100000000U)
    {
        for (i = 26; i >= 0; i--)
        {
            if ((output & 0xF) >= 5)
                output += 3;

            if (((output & 0xF0) >> 4) >= 5)
                output += (3 << 4);

            if (((output & 0xF00) >> 8) >= 5)
                output += (3 << 8);

            if (((output & 0xF000) >> 12) >= 5)
                output += (3 << 12);

            if (((output & 0xF0000) >> 16) >= 5)
                output += (3 << 16);

            if (((output & 0xF00000) >> 20) >= 5)
                output += (3 << 20);

            if (((output & 0xF000000) >> 24) >= 5)
                output += (3 << 24);

            output = (output << 1) | ((input >> i) & 1);
        }
    }
    return output;
}

/**
 * @brief This function changes order of bytes from 0-1-2-3 to 3-2-1-0
 *
 */

uint32_t BCD_to_CAT (uint32_t input)
{
    uint8_t i;
    uint32_t output = 0U;

    for (i = 0; i < 3; i++)
    {
        output |= (input & 0xFF);
        output = output << 8;
        input = input >> 8;
    }
    output |= (input & 0xFF);

    return output;
}

/**
 * @brief This function converts frequency from CAT to HEX format
 *
 */

uint32_t CAT_to_HEX (void)
{
    uint8_t i;
    uint32_t output = 0U;

    for (i = 0; i < 4; i++)
    {
        output *= 100U;
        output += (cat.get_freq [i] >> 4U) * 10U + (cat.get_freq [i] & 0x0F);
    }
    output *= 10U;

    return output;
}

/**
 * @brief This function controls external power amplifier
 *
 */

void pca9554_write (uint8_t addr, uint8_t data)
{
    return; /* There is no PA yet ;) */
    I2C_Transmit (&hi2c_tx, (uint16_t) PCA9554_BUS_BASE_ADDR, addr, data);
}

/**
 * @brief This function sets BPF band and mode
 *
 * The function sets BPF band according to VFO frequency and sets RX/TX mode
 *
 */

void ptt_set_bpf (uint32_t tune_new)
{
    uint8_t bpf = 0x06;
    uint8_t lpf = 0x01;

    if (tune_new > 2000000U)
        lpf = 0x02;

    if (tune_new > 4000000U)
    {
      bpf = 0x04;
      lpf = 0x04;
    }
    if (tune_new > 8000000U)
    {
      bpf = 0x02;
      lpf = 0x08;
    }
    if (tune_new > 16000000U)
    {
        bpf = 0x00;
        lpf = 0x10;
    }
    if (tune_new > 24000000U)
        lpf = 0x20;

    if (bpf & 0x02)
    {
        HAL_GPIO_WritePin (S1_GPIO_Port, S1_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin (S1_GPIO_Port, S1_Pin, GPIO_PIN_RESET);
    }

    if (bpf & 0x04)
    {
        HAL_GPIO_WritePin (S2_GPIO_Port, S2_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin (S2_GPIO_Port, S2_Pin, GPIO_PIN_RESET);
    }

    if (trx.is_tx)
    {
        HAL_GPIO_WritePin (TX_GPIO_Port, TX_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin (QSE_EN_GPIO_Port, QSE_EN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin (QSD_EN_GPIO_Port, QSD_EN_Pin, GPIO_PIN_SET);

        lpf |= 0x40;
    }
    else
    {
        HAL_GPIO_WritePin (TX_GPIO_Port, TX_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin (QSE_EN_GPIO_Port, QSE_EN_Pin, GPIO_PIN_SET);

        lpf &= 0xBF;

        if (trx.mode == MODE_PKT)
        {
          HAL_GPIO_WritePin (QSD_EN_GPIO_Port, QSD_EN_Pin, GPIO_PIN_SET);
        }
        else
        {
          HAL_GPIO_WritePin (QSD_EN_GPIO_Port, QSD_EN_Pin, GPIO_PIN_RESET);
        }
    }

    pca9554_write (1, lpf);
}

/**
 * @brief This function sets TX mode
 *
 * The function also sets TX mode for VFO and DSP
 *
 */

void ptt_set_tx (void)
{
    ptt.key_off_time = 0U;

    if (!trx.is_tx)
    {
        if (trx.split)
            VFO_Toggle_VFO ();

        trx.is_tx = 1U;
        cat.read_tx_state &= 0x7F;

        DSP_Set_TX ();

        if (trx.vfo)
            ptt_set_bpf (trx.vfob);
        else
            ptt_set_bpf (trx.vfoa);
    }
}

/**
 * @brief This function sets RX mode
 *
 * The function sets RX mode if the PTT button is not pressed,
 * the telegraph key is not used and CAT status is not TX.
 * The function also sets RX mode for VFO and DSP
 *
 */

void ptt_set_rx (void)
{
    if (ptt.cat_is_on ||
        ptt.key_dah_is_on ||
        ptt.key_dit_is_on ||
        ptt.dtr_is_on ||
        ptt.rts_is_on)
    {
        return;
    }

    if (trx.is_tx)
    {
        trx.is_tx = 0U;
        cat.read_tx_state |= 0x80;

        if (trx.split)
            VFO_Toggle_VFO ();

        DSP_Set_RX ();

        if (trx.vfo)
            ptt_set_bpf (trx.vfob);
        else
            ptt_set_bpf (trx.vfoa);
    }
}

/**
 * @brief  This function sets frequency from CAT
 *
 */

void vfo_set_tune_cat (void)
{
    uint32_t tune_new = CAT_to_HEX ();

    if (trx.vfo)
    {
        trx.vfob_bcd = *(uint32_t*) cat.get_freq;
        trx.vfob = tune_new;
    }
    else
    {
        trx.vfoa_bcd = *(uint32_t*) cat.get_freq;
        trx.vfoa = tune_new;
    }

    vfo_set_freq (tune_new);
    ptt_set_bpf (tune_new);
}

/**
 * @brief This function sets TX mode from CAT command
 *
 * @param  0 - sets PTT OFF
 * @param  1 - sets PTT ON
 *
 * @retval 0x00 - OK
 * @retval 0xF0 - PTT's already On/Off
 *
 */

uint8_t ptt_cat_tx (uint8_t cat)
{
    uint8_t retval = 0xF0;

    if (ptt.cat_is_on != cat)
    {
        ptt.cat_is_on = cat;

        if (cat)
            ptt_set_tx ();
        else
            ptt_set_rx ();

        retval = 0x00;
    }
    return retval;
}

/**
 * @brief This function decodes and handles FT817 CAT commands
 *
 * The function reads 5th byte from CAT buffer to and decodes CAT command *
 * If CAT command is decoded successfully, the function sends reply.
 *
 * **List of FT817 CAT commands according to User Manual:**
 * | Command          | req[0] | req[1] | req[2] | req[3] | req[4] | Macros              |
 * |:-----------------|:------:|:------:|:------:|:------:|:------:|---------------------|
 * | LOCK ON          |   --   |   --   |   --   |   --   |  0x00  |                     |
 * | LOCK OFF         |   --   |   --   |   --   |   --   |  0x80  |                     |
 * | PTT ON           |   --   |   --   |   --   |   --   |  0x08  | FT817_PTT_ON        |
 * | PTT OFF          |   --   |   --   |   --   |   --   |  0x88  | FT817_PTT_OFF       |
 * | Set Frequency    |   P1   |   P2   |   P3   |   P4   |  0x01  | FT817_SET_FREQ      |
 * | Operating Mode   |   P1   |   --   |   --   |   --   |  0x07  | FT817_MODE_SET      |
 * | CLAR ON          |   --   |   --   |   --   |   --   |  0x05  |                     |
 * | CLAR OFF         |   --   |   --   |   --   |   --   |  0x85  |                     |
 * | CLAR Frequency   |   P1   |   --   |   P3   |   P4   |  0xF5  |                     |
 * | VFO-A/B          |   --   |   --   |   --   |   --   |  0x81  | FT817_TOGGLE_VFO    |
 * | SPLIT ON         |   --   |   --   |   --   |   --   |  0x02  | FT817_SPLIT_ON      |
 * | SPLIT OFF        |   --   |   --   |   --   |   --   |  0x82  | FT817_SPLIT_OFF     |
 * | Repeater Offset  |   P1   |   --   |   --   |   --   |  0x09  |                     |
 * | Repeater Offset  |   P1   |   P2   |   P3   |   P4   |  0xF9  |                     |
 * | CTCSS/DCS Mode   |   P1   |   --   |   --   |   --   |  0x0A  |                     |
 * | CTCSS Tone       |   P1   |   P2   |   --   |   --   |  0x0A  |                     |
 * | DCS Code         |   P1   |   P2   |   --   |   --   |  0x0A  |                     |
 * | Read RX Status   |   --   |   --   |   --   |   --   |  0xE7  |                     |
 * | Read TX Status   |   --   |   --   |   --   |   --   |  0xF7  | FT817_READ_TX_STATE |
 * | Read Freq & Mode |   --   |   --   |   --   |   --   |  0x03  | FT817_GET_FREQ      |
 * | POWER ON         |   --   |   --   |   --   |   --   |  0x0F  |                     |
 * | POWER OFF        |   --   |   --   |   --   |   --   |  0x8F  |                     |
 *
 *
 * <p> </p>
 * **List of FT817 Operating Modes according to User Manual:**
 *
 * <p>0x00 = LSB; 0x01 = USB; 0x02 = CW;  0x03 = CW-R;</p>
 * <p>0x04 = AM;  0x08 = FM;  0x0A = DIG; 0x0C = PKT</p>
 *
 */
typedef enum
{
    FT817_SET_FREQ = 0x01,
    FT817_TOGGLE_VFO = 0x81,
    FT817_SPLIT_ON = 0x02,
    FT817_SPLIT_OFF = 0x82,
    FT817_GET_FREQ = 0x03,
    FT817_MODE_SET = 0x07,
    FT817_PTT_ON = 0x08,
    FT817_PTT_OFF = 0x88,
    FT817_READ_TX_STATE = 0xF7,
} FT817_COMMAND;

#define PACKAGE_P1_IDX  0
#define PACKAGE_CMD_IDX 4
#define PACKAGE_SIZE    5

void cat_cmd_handler (void)
{
    static uint8_t package[8];
    static uint16_t package_idx = 0;
    uint8_t cmd, reply;

    while (cat.wr_ptr != cat.rd_ptr)
    {
        package[package_idx++] = cat.buff[cat.rd_ptr++];
        if (cat.rd_ptr >= CAT_BUFF_SIZE)
            cat.rd_ptr = 0;
        if (package_idx < PACKAGE_SIZE)
            continue;
        package_idx = 0;

        cmd = package[PACKAGE_CMD_IDX];

        if (cmd == FT817_GET_FREQ)
        {
            CDC_Transmit_FS (cat.get_freq, 5);
            continue;
        }

        reply = 0U;

        switch (cmd)
        {
        case FT817_READ_TX_STATE:
            reply = cat.read_tx_state;
            break;
        case FT817_SET_FREQ:
            *(uint32_t*) cat.get_freq = *(uint32_t*) &package [PACKAGE_P1_IDX];
            vfo_set_tune_cat ();
            break;
        case FT817_MODE_SET:
            cat.get_freq [4] = package [PACKAGE_P1_IDX];
            PTT_Set_Mode (package [PACKAGE_P1_IDX]);
            break;
        case FT817_TOGGLE_VFO:
            VFO_Toggle_VFO ();
            break;
        case FT817_SPLIT_ON:
            VFO_Set_Split (1);
            break;
        case FT817_SPLIT_OFF:
            VFO_Set_Split (0);
            break;
        case FT817_PTT_ON:
            reply = ptt_cat_tx (1);
            break;
        case FT817_PTT_OFF:
            reply = ptt_cat_tx (0);
            break;
        default:
            break;
        }
        CDC_Transmit_FS (&reply, 1);
    }
}

/**
 * @brief  This function sets TRX main frequency according to toggled VFO and sets VFO and BPF
 *
 * @param  Main frequency in Hz
 */

void VFO_Set_Tune (uint32_t tune_new)
{
  switch (trx.vfo)
  {
    case 0:
      if (trx.vfoa != tune_new)
      {
        HAL_RTCEx_BKUPWrite (&hrtc, RTC_BKP_DR2, tune_new);

        trx.vfoa = tune_new;
        trx.vfoa_bcd = HEX_to_BCD (trx.vfoa / 10U);
        trx.vfoa_bcd = BCD_to_CAT (trx.vfoa_bcd);
        *(uint32_t*) cat.get_freq = trx.vfoa_bcd;
      }
      break;
    case 1:
      if (trx.vfob != tune_new)
      {
        HAL_RTCEx_BKUPWrite (&hrtc, RTC_BKP_DR3, tune_new);

        trx.vfob = tune_new;
        trx.vfob_bcd = HEX_to_BCD (trx.vfob / 10U);
        trx.vfob_bcd = BCD_to_CAT (trx.vfob_bcd);
        *(uint32_t*) cat.get_freq = trx.vfob_bcd;
      }
      break;
  }

  vfo_set_freq (tune_new);
  ptt_set_bpf (tune_new);
}

/**
 * @brief  This function returns TRX main frequency according to toggled VFO
 *
 * @retval Main frequency in Hz
 */

uint32_t VFO_Get_Tune (void)
{
  uint32_t retval;

  if (trx.vfo)
  {
    retval = trx.vfob;
  }
  else
  {
    retval = trx.vfoa;
  }

  return retval;
}

/**
 * @brief  This function returns TRX main frequency in BCD according to toggled VFO
 *
 * @retval Main frequency in Hz
 */

uint32_t VFO_Get_Tune_BCD (void)
{
  uint32_t retval;

  if (trx.vfo)
  {
    retval = trx.vfob_bcd;
  }
  else
  {
    retval = trx.vfoa_bcd;
  }

  return retval;
}

/**
 * @brief  This function switches VFO from one to another
 *
 */

void VFO_Toggle_VFO (void)
{
  if (trx.is_tx)
  {
    return;
  }

  uint32_t mode_bkup;
  mode_bkup = HAL_RTCEx_BKUPRead (&hrtc, RTC_BKP_DR1);

  if (trx.vfo)
  {
    trx.vfo = 0U;
    mode_bkup = mode_bkup & 0xFFFFFFEFU;
    *(uint32_t*) cat.get_freq = trx.vfoa_bcd;
    VFO_Set_Tune (trx.vfoa);
  }
  else
  {
    trx.vfo = 1U;
    mode_bkup = mode_bkup | 0x00000010U;
    *(uint32_t*) cat.get_freq = trx.vfob_bcd;
    VFO_Set_Tune (trx.vfob);
  }

  HAL_RTCEx_BKUPWrite (&hrtc, RTC_BKP_DR1, mode_bkup);
}

/**
 * @brief This function sets VFO Split Mode On/Off
 *
 */

void VFO_Set_Split (uint8_t split)
{
  if (trx.is_tx)
  {
    return;
  }

  uint32_t mode_bkup;
  mode_bkup = HAL_RTCEx_BKUPRead (&hrtc, RTC_BKP_DR1);

  trx.split = split;

  if (trx.split)
  {
    mode_bkup = mode_bkup | 0x00000020U;
    cat.read_tx_state &= 0xDF;
  }
  else
  {
    mode_bkup = mode_bkup & 0xFFFFFFDFU;
    cat.read_tx_state |= 0x20;
  }

  HAL_RTCEx_BKUPWrite (&hrtc, RTC_BKP_DR1, mode_bkup);
}

/**
 * @brief This function sets TRX operating mode from CAT command
 *
 * @param TRX operating mode
 */

void PTT_Set_Mode (uint8_t trx_mode)
{
  if (trx.is_tx)
  {
    return;
  }

  uint32_t mode_bkup;

  switch (trx_mode)
  {
    case 0x00:
      trx.mode = MODE_LSB;
      break;
    case 0x01:
      trx.mode = MODE_USB;
      break;
    case 0x02:
      trx.mode = MODE_CW; /* CW-USB */
      break;
    case 0x03:
      trx.mode = MODE_CW; /* CW-USB */
      break;
    case 0x04:
      trx.mode = MODE_AM;
      break;
    case 0x08:
      trx.mode = MODE_FM;
      break;
    case 0x0A:
      trx.mode = MODE_USB;
      break;
    case 0x0C:
      trx.mode = MODE_PKT; /* Used for tests only */
      break;
    default:
      trx.mode = MODE_USB;
      break;
  }

  cat.get_freq [4] = trx.mode;

  if (trx.mode != MODE_PKT)
  {
    mode_bkup = HAL_RTCEx_BKUPRead (&hrtc, RTC_BKP_DR1);

    mode_bkup = mode_bkup & 0xFFFFFFF0U;
    mode_bkup = mode_bkup + trx.mode;

    HAL_RTCEx_BKUPWrite (&hrtc, RTC_BKP_DR1, mode_bkup);
  }

  //DSP_Set_Mode (trx_mode);
}

/**
 * @brief This function sets TX mode from DTR line
 *
 * DTR line works like external telegraph key
 *
 */

void PTT_DTR_TX (uint8_t dtr)
{
  if (ptt.dtr_is_on != dtr)
  {
    ptt.dtr_is_on = dtr;

    if (dtr)
    {
      ptt_set_tx ();
    }
    else
    {
      ptt.key_off_time = trx.sysclock;
    }
  }
}

/**
 * @brief This function sets TX mode from RTS line
 *
 * RTS line works like external PTT
 *
 */

void PTT_RTS_TX (uint8_t rts)
{
  if (ptt.rts_is_on != rts)
  {
    ptt.rts_is_on = rts;

    if (rts)
    {
      ptt_set_tx ();
    }
    else
    {
      ptt_set_rx ();
    }
  }
}

/**
 * @brief This function calls VFO_Init () to sets HSE = 24.576 MHz
 *
 */

void VFO_Init (void)
{
  Si5351a_Init ();
}

__weak void UI_Init (void)
{
}

__weak void UI_Handler(void)
{
}

/**
 * @brief This function initialize PTT, VFO and DSP
 *
 */

void RXTX_Init (void)
{
    uint32_t vfo_tune;
    uint32_t mode_bkup;
    uint8_t bkup = 0U;

    /* PTT and CAT mode init */
    trx.vfo = 0U;
    trx.split = 0U;
    trx.is_tx = 0U;
    cat.read_tx_state = 0xFF;

    /* VFOA backup checking and restoring */
    vfo_tune = HAL_RTCEx_BKUPRead (&hrtc, RTC_BKP_DR2);

    if (vfo_tune)
    {
        trx.vfoa = HAL_RTCEx_BKUPRead (&hrtc, RTC_BKP_DR2);
        bkup = 1U;
    }
    else
    {
        trx.vfoa = 7050000U;
        HAL_RTCEx_BKUPWrite (&hrtc, RTC_BKP_DR2, trx.vfoa);
    }

    /* VFOB backup checking and restoring */
    vfo_tune = HAL_RTCEx_BKUPRead (&hrtc, RTC_BKP_DR3);

    if (vfo_tune)
    {
        trx.vfob = HAL_RTCEx_BKUPRead (&hrtc, RTC_BKP_DR3);
        bkup = 1U;
    }
    else
    {
        trx.vfob = 7010000U;
        HAL_RTCEx_BKUPWrite (&hrtc, RTC_BKP_DR3, trx.vfob);
    }

    /* VFO frequency format converting HEX to CAT */
    trx.vfoa_bcd = HEX_to_BCD (trx.vfoa / 10U);
    trx.vfob_bcd = HEX_to_BCD (trx.vfob / 10U);

    trx.vfoa_bcd = BCD_to_CAT (trx.vfoa_bcd);
    trx.vfob_bcd = BCD_to_CAT (trx.vfob_bcd);

    /* TRX mode backup checking and restoring */
    mode_bkup = HAL_RTCEx_BKUPRead (&hrtc, RTC_BKP_DR1);

    if (bkup)
    {
        trx.mode = (uint8_t) (mode_bkup & 0x0000000FU);

        if (mode_bkup & 0x00000010U)
            trx.vfo = 1U;
        if (mode_bkup & 0x00000020U)
            trx.split = 1U;
    }
    else
    {
        trx.mode = MODE_LSB;
        mode_bkup = 0x80000000U + trx.mode;
        HAL_RTCEx_BKUPWrite (&hrtc, RTC_BKP_DR1, mode_bkup);
    }

    cat.get_freq [4] = trx.mode;

    /* PA mode init */
    pca9554_write (2, 0x00);
    pca9554_write (3, 0x00);

    if (trx.vfo)
    {
        VFO_Set_Tune (trx.vfob);
        *(uint32_t*) cat.get_freq = trx.vfob_bcd;
    }
    else
    {
        VFO_Set_Tune (trx.vfoa);
        *(uint32_t*) cat.get_freq = trx.vfoa_bcd;
    }

    /* CAT buffer init */
    cat.rd_ptr = 0U;
    cat.wr_ptr = 0U;

    /* TRX timeouts init */
    trx.systicks = 10U;
    trx.sysclock = 0U;
    trx.displayed = trx.sysclock;

    DSP_Init ();
    UI_Init ();
}

/**
 * @brief This function writes to CAT buffer
 *
 */

void CAT_Buff_Write (uint8_t *pbuf, uint32_t len)
{
    uint16_t next;
    while (len != 0)
    {
        --len;
        next = cat.wr_ptr + 1;
        if (next >= CAT_BUFF_SIZE)
            next = 0;
        if (next == cat.rd_ptr)
            break; // No space in buffer
        cat.buff[cat.wr_ptr] = *pbuf++;
        cat.wr_ptr = next;
    }
}

/**
 * @brief This function sets telegraph key off time
 *
 */

void PTT_Key_RX (void)
{
    ptt.key_off_time = trx.sysclock;
}

/**
 * @brief This function processes the timeout in switching to RX mode
 * after releasing the telegraph key
 *
 */

void RXTX_Handler (void)
{
    if (cat.wr_ptr != cat.rd_ptr)
    {
        cat_cmd_handler ();
    }

    if (ptt.key_off_time != 0U)
    {
        if ((trx.sysclock - ptt.key_off_time) > KEY_TIMEOUT)
        {
            ptt.key_off_time = 0U;
            ptt_set_rx ();
        }
    }

    if ((trx.sysclock - trx.displayed) > 19)
    {
        trx.displayed = trx.sysclock;
        UI_Handler ();
    }
}

/**
 * @brief This function is GPIO_EXTI handler
 *
 * @param KEY_DAH_Pin - Telegraph key DAH paddle
 * @param KEY_DIT_Pin - Telegraph key DIT paddle
 *
 */

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
    case KEY_DAH_Pin:
        ptt.key_dah_is_on = !HAL_GPIO_ReadPin (KEY_DAH_GPIO_Port, KEY_DAH_Pin);
        if (ptt.key_dah_is_on)
            ptt_set_tx ();
        else
            ptt.key_off_time = trx.sysclock;
        break;
    case KEY_DIT_Pin:
        ptt.key_dit_is_on = !HAL_GPIO_ReadPin (KEY_DIT_GPIO_Port, KEY_DIT_Pin);
        if (ptt.key_dit_is_on)
            ptt_set_tx ();
        else
            ptt.key_off_time = trx.sysclock;
        break;
    }
}

/****END OF FILE****/
