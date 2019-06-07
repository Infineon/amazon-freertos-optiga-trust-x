/**
* \copyright
* MIT License
*
* Copyright (c) 2018 Infineon Technologies AG
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE
*
* \endcopyright
*
* \author Infineon Technologies AG
*/

#include "dps310.h"

/* Meaningful Default Configuration */
#define     IFX_DPS310_TEMPERATURE_OSR                  OSR_2
#define     IFX_DPS310_PRESSURE_OSR                     OSR_64
#define     IFX_DPS310_TEMPERATURE_MR                   TMP_MR_4
#define     IFX_DPS310_PRESSURE_MR                      PM_MR_8

/**/

static dps310_scaling_coeffs_e
dps310_get_scaling_coef (dps310_osr_e osr)
{
        dps310_scaling_coeffs_e scaling_coeff;

        switch (osr){

              case OSR_1:
                    scaling_coeff = OSR_SF_1;
                    break;
              case OSR_2:
                    scaling_coeff = OSR_SF_2;
                    break;
              case OSR_4:
                    scaling_coeff = OSR_SF_4;
                    break;
              case OSR_8:
                    scaling_coeff = OSR_SF_8;
                    break;
              case OSR_16:
                    scaling_coeff = OSR_SF_16;
                    break;
              case OSR_32:
                    scaling_coeff = OSR_SF_32;
                    break;
              case OSR_64:
                    scaling_coeff = OSR_SF_64;
                    break;
              case OSR_128:
                    scaling_coeff = OSR_SF_128;
                    break;
              default:
                     scaling_coeff = OSR_SF_1;
                     break;
        }
        return scaling_coeff;
}

static int dps310_read_calib_coeffs(struct dps310_state *drv_state)

{
        s32 ret;
        u8 read_buffer[IFX_DPS310_COEF_LEN] = {0};

        if (drv_state == NULL)
            return -EINVAL;

        ret =  drv_state->io->read_block((u8)IFX_DPS310_COEF_REG_ADDR,
                                 (u8)IFX_DPS310_COEF_LEN, read_buffer);

        if ( ret != IFX_DPS310_COEF_LEN )
            return ret;

        drv_state->calib_coeffs.C0 = (read_buffer[0] << 4) + ((read_buffer[1] >>4) & 0x0F);

        if(drv_state->calib_coeffs.C0 > POW_2_11_MINUS_1)
            drv_state->calib_coeffs.C0 = drv_state->calib_coeffs.C0 - POW_2_12;

        drv_state->calib_coeffs.C1 = (read_buffer[2] + ((read_buffer[1] & 0x0F)<<8));

        if(drv_state->calib_coeffs.C1 > POW_2_11_MINUS_1)
            drv_state->calib_coeffs.C1 = drv_state->calib_coeffs.C1 - POW_2_12;

        drv_state->calib_coeffs.C00 = ((read_buffer[4]<<4) + (read_buffer[3]<<12)) + ((read_buffer[5]>>4) & 0x0F);

        if(drv_state->calib_coeffs.C00 > POW_2_19_MINUS_1)
            drv_state->calib_coeffs.C00 = drv_state->calib_coeffs.C00 -POW_2_20;

        drv_state->calib_coeffs.C10 = ((read_buffer[5] & 0x0F)<<16) + read_buffer[7] + (read_buffer[6]<<8);

        if(drv_state->calib_coeffs.C10 > POW_2_19_MINUS_1)
            drv_state->calib_coeffs.C10 = drv_state->calib_coeffs.C10 - POW_2_20;

        drv_state->calib_coeffs.C01 = (read_buffer[9] + (read_buffer[8]<<8));

        if(drv_state->calib_coeffs.C01 > POW_2_15_MINUS_1)
            drv_state->calib_coeffs.C01 = drv_state->calib_coeffs.C01 - POW_2_16;

        drv_state->calib_coeffs.C11 = (read_buffer[11] + (read_buffer[10]<<8));

        if(drv_state->calib_coeffs.C11 > POW_2_15_MINUS_1)
            drv_state->calib_coeffs.C11 = drv_state->calib_coeffs.C11 - POW_2_16;

        drv_state->calib_coeffs.C20 = (read_buffer[13] + (read_buffer[12]<<8));

        if(drv_state->calib_coeffs.C20 > POW_2_15_MINUS_1)
            drv_state->calib_coeffs.C20 = drv_state->calib_coeffs.C20 - POW_2_16;

        drv_state->calib_coeffs.C21 = (read_buffer[15] + (read_buffer[14]<<8));

        if(drv_state->calib_coeffs.C21 > POW_2_15_MINUS_1)
            drv_state->calib_coeffs.C21 = drv_state->calib_coeffs.C21 - POW_2_16;

        drv_state->calib_coeffs.C30 = (read_buffer[17] + (read_buffer[16]<<8));

        if(drv_state->calib_coeffs.C30 > POW_2_15_MINUS_1)
            drv_state->calib_coeffs.C30 = drv_state->calib_coeffs.C30 - POW_2_16;

        /* lets see which temperature diode is used for calibration and update state accordingly*/
        ret = drv_state->io->read_byte(IFX_DPS310_TMP_COEF_SRCE_REG_ADDR);

        if (ret < 0){
                return -EIO;
        }
        if ((ret >> IFX_DPS310_TMP_COEF_SRCE_REG_POS_MASK) & 1 ){
				drv_state->tmp_ext = TMP_EXT_MEMS;
		}
		else{
				drv_state->tmp_ext = TMP_EXT_ASIC;
		}
        return 0;
}


int dps310_resume(struct dps310_state *drv_state)
{
        s32 ret;
        if (drv_state == NULL)
            return -EINVAL;

        ret = drv_state->io->write_byte(IFX_DPS310_MEAS_CFG_REG_ADDR,
                                        (u8)DPS310_MODE_BACKGROUND_ALL);
        if (ret < 0)
                return -EIO;

        drv_state->dev_mode = DPS310_MODE_BACKGROUND_ALL;

        return 0;

}


int dps310_standby(struct dps310_state *drv_state)
{
        s32 ret;
        if (drv_state == NULL)
          return -EINVAL;

        ret = drv_state->io->write_byte(IFX_DPS310_MEAS_CFG_REG_ADDR,
                                       (u8)DPS310_MODE_IDLE);
        if (ret < 0)
                return -EIO;

        drv_state->dev_mode = DPS310_MODE_IDLE;

        return 0;
}

int dps310_config(struct dps310_state *drv_state,
                            dps310_osr_e osr_temp,
                            dps310_tmp_rate_e mr_temp,
                            dps310_osr_e osr_press,
                            dps310_pm_rate_e mr_press,
                            dps310_temperature_src_e temp_src)
{

        s32 ret;
        u8  config;

        if (drv_state == NULL)
			return -EINVAL;

       /* configure temperature measurements first*/
       /*Prepare a configuration word for TMP_CFG register*/
        config = (u8) temp_src;

        /*First Set the TMP_RATE[2:0] -> 6:4 */
        config |= ((u8)mr_temp);

       /*Set the TMP_PRC[3:0] -> 2:0 */
        config |= ((u8)osr_temp);

        ret = drv_state->io->write_byte(IFX_DPS310_TMP_CFG_REG_ADDR,
                                  config);
        if (ret < 0)
            return -EIO;

        /*Prepare a configuration word for PRS_CFG register*/
        /*First Set the PM_RATE[2:0] -> 6:4 */
        config = (u8) ( 0x00 ) | ((u8)mr_press);

        /*Set the PM_PRC[3:0] -> 3:0 */
        config |= ((u8)osr_press);

        ret = drv_state->io->write_byte(IFX_DPS310_PRS_CFG_REG_ADDR,
                                  config);
        if (ret < 0)
            return -EIO;

        /* always take configuration word from state*/
        config = drv_state->cfg_word;

        /*If oversampling rate for temperature is greater than 8 times, then set TMP_SHIFT bit in CFG_REG */
        if ((u8)osr_temp > (u8) OSR_8){
                config |= (u8) IFX_DPS310_CFG_TMP_SHIFT_EN_SET_VAL;
        }

        /*If oversampling rate for pressure is greater than 8 times, then set P_SHIFT bit in CFG_REG */
        if ((u8)osr_press > (u8) OSR_8){
                config |= (u8) IFX_DPS310_CFG_PRS_SHIFT_EN_SET_VAL;
        }

        /* write CFG_REG */
        ret =  drv_state->io->write_byte(IFX_DPS310_CFG_REG_ADDR,
                                  config);
        if (ret < 0)
            return -EIO;

       /*Update state accordingly with proper scaling factors based on oversampling rates*/
        drv_state->tmp_osr_scale_coeff = dps310_get_scaling_coef(osr_temp);

        drv_state->prs_osr_scale_coeff = dps310_get_scaling_coef(osr_press);

        drv_state->press_mr = mr_press;

        drv_state->temp_mr  = mr_temp;

        drv_state->temp_osr = osr_temp;

        drv_state->press_osr = osr_press;

        drv_state->tmp_ext  = temp_src;

        return 0;

}

int dps310_get_processed_data (struct dps310_state *drv_state,
                                      f64 *pressure,
                                      f64 *temperature)
{
        s32    ret;
        u8     read_buffer[IFX_DPS310_PSR_TMP_READ_LEN] = {0};
        f64    press_raw;
        f64    temp_raw;

        f64 temp_scaled;
        f64 temp_final;
        f64 press_scaled;
        f64 press_final;

        if (drv_state == NULL)
            return -EINVAL;

        ret = drv_state->io->read_block(IFX_DPS310_PSR_TMP_READ_REG_ADDR,
                              IFX_DPS310_PSR_TMP_READ_LEN,
                              read_buffer);

        if (ret < IFX_DPS310_PSR_TMP_READ_LEN)
            return -EINVAL;

        press_raw = (read_buffer[2]) + (read_buffer[1]<<8) + (read_buffer[0] <<16);
        temp_raw  = (read_buffer[5]) + (read_buffer[4]<<8) + (read_buffer[3] <<16);

//        printf("data raw 0x%02X:0x%02X:0x%02X:0x%02X:0x%02X:0x%02X\r\n",read_buffer[0],read_buffer[1],read_buffer[2],read_buffer[3],read_buffer[4],read_buffer[5]);

        if(temp_raw > POW_2_23_MINUS_1){
            temp_raw = temp_raw - POW_2_24;
        }

        if(press_raw > POW_2_23_MINUS_1){
            press_raw = press_raw - POW_2_24;
        }

        temp_scaled = (double)temp_raw / (double) (drv_state->tmp_osr_scale_coeff);

        temp_final =  (drv_state->calib_coeffs.C0 /2.0f) + drv_state->calib_coeffs.C1 * temp_scaled ;

        press_scaled = (double) press_raw / drv_state->prs_osr_scale_coeff;

        press_final = drv_state->calib_coeffs.C00 +
                      press_scaled *  (  drv_state->calib_coeffs.C10 + press_scaled *
                      ( drv_state->calib_coeffs.C20 + press_scaled * drv_state->calib_coeffs.C30 )  ) +
                      temp_scaled * drv_state->calib_coeffs.C01 +
                      temp_scaled * press_scaled * ( drv_state->calib_coeffs.C11 +
                                                      press_scaled * drv_state->calib_coeffs.C21 );

        press_final = press_final * 0.01f;	//to convert it into mBar

        *temperature = temp_final;
        *pressure    = press_final;  //press_final;

//        printf("datad %lf %lf\r\n",temp_final, press_final);

        return 0;
}

int dps310_init(struct dps310_state *drv_state, dps310_bus_connection *io)
{
        s32 ret;

        if (!drv_state){
            return -EINVAL;
        }

        if (!io){
            return -EINVAL;
        }

        drv_state->cfg_word = 0;
        drv_state->enable = 0;
       
        /*first verify chip by reading product and rev id*/
        ret = io->read_byte(IFX_DPS310_PROD_REV_ID_REG_ADDR);

        if (ret < 0){
            ret = -EIO;
				goto err_handler_iio;
        }

        if (ret != IFX_DSPS310_PROD_REV_ID_VAL){
            ret = -EINVAL;
			goto err_handler_iio;
        }

        io->delayms(40);
        /*first verify chip by reading product and rev id*/
        ret = io->read_byte(IFX_DPS310_PROD_REV_ID_REG_ADDR);

        if (ret < 0){
            ret = -EIO;
				goto err_handler_iio;
        }

        /* attach bus connection instance to state*/
        drv_state->io = io;

        /* from here wait for about 40ms till calibration coefficients become available*/
        if (drv_state->io->delayms != NULL)
            drv_state->io->delayms(40);

        /* read now the calibration coeffs, temperature coef source and store in driver state*/
        ret = dps310_read_calib_coeffs(drv_state);

        if (ret < 0){
				goto err_handler_iio;
        }

        /* Now apply ADC Temperature gain settings*/
        /* First write valid signature on 0x0e and 0x0f
         * to unlock address 0x62 */
        drv_state->io->write_byte((u8)0x0e,(u8)0xa5);
        drv_state->io->write_byte((u8)0x0f,(u8)0x96);
        /*Then update high gain value for Temperature*/
        drv_state->io->write_byte((u8)0x62,(u8)0x02);

        /*Finally lock back the location 0x62*/
        drv_state->io->write_byte((u8)0x0e,(u8)0x00);
        drv_state->io->write_byte((u8)0x0f,(u8)0x00);


        /* configure sensor for default ODR settings*/
        ret = dps310_config(drv_state,
                            IFX_DPS310_TEMPERATURE_OSR,
                            IFX_DPS310_TEMPERATURE_MR,
                            IFX_DPS310_PRESSURE_OSR,
                            IFX_DPS310_PRESSURE_MR,
                            drv_state->tmp_ext);
        if (ret < 0){
            goto err_handler_iio;
        }

        /* activate sensor*/
        ret = dps310_resume(drv_state);
        if (ret < 0){
            goto err_handler_iio;
        }

        return 0;

err_handler_iio:
		return ret;

}
