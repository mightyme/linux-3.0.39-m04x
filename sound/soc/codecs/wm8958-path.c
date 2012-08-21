/*
 * linux/sound/soc/codecs/wm8958_path.c
 *
 *
 * Copyright (C) 2011 Meizu, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 */

/***************************** INCLUDES ************************************/
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <linux/delay.h>
#include <asm/mach-types.h>
#ifdef CONFIG_SND_SOC_MX_WM8958
#include "wm8958_path.h"

#ifdef CONFIG_AUDIENCE_ES305B
#include <linux/es305b_soc.h>
extern int es305b_setmode(int mode);
#endif
#ifdef CONFIG_AUDIENCE_A1028
#include <linux/a1028_soc.h>
extern int (*a1028_setmode)(struct a1028_soc *, int);
#endif

/* enable debug prints in the driver */
//#define DEBUG
#undef DEBUG

#ifdef DEBUG
#define dprintk(x...) 	printk(x)
#else
#define dprintk(x...)
#endif

#define	SWTICH_TO_BB	(0)
#define	SWTICH_TO_AP	(1)

#define	EQ_HP_NORMAL	0
#define	EQ_SPK_NORMAL	1
#define	EQ_REC_INCALL	2
static int aid1dca1_eq_gain1 =0x6318;// 0db
static int aid1dca1_eq_gain2 =0x6300;// 0db
extern void audio_switch(int High);

void SetEQBase(struct snd_soc_codec *codec,int mMode)
{
	static int mode = -1;
	dprintk ("%s(%d) \n", __func__,mMode);

	if( mode == mMode)
		return;

	switch( mMode )
	{
		case EQ_HP_NORMAL:
			if(mode != EQ_HP_NORMAL)
			{
				snd_soc_update_bits(codec,WM8994_AIF1_DAC1_EQ_GAINS_1, 0xFFFE,aid1dca1_eq_gain1);
				snd_soc_update_bits(codec,WM8994_AIF1_DAC1_EQ_GAINS_2, 0xFFFF,aid1dca1_eq_gain2);
			}
			mode = EQ_HP_NORMAL;

			snd_soc_update_bits(codec,WM8994_AIF1_DAC1_EQ_GAINS_1, WM8994_AIF1DAC1_EQ_ENA_MASK,0);
			snd_soc_update_bits(codec,WM8994_AIF1_DRC1_1, WM8994_AIF1DAC1_DRC_ENA_MASK,0);

			// 100 
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_1_A, 0x0FD2);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_1_B, 0x0409);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_1_PG, 0x00B8);

			// 500 - 2
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_2_A, 0x1F75);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_2_B, 0xF08A);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_2_C, 0x0408);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_2_PG, 0x0223);

			// 1000 - 2
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_3_A, 0x1F9E);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_3_B, 0xF061);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_3_C, 0x0409);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_3_PG, 0x0180);

			// 4000 - 2 
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_4_A, 0x168E);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_4_B, 0xF829);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_4_C, 0x07AD);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_4_PG, 0x1103);

			// 16000
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_5_A, 0x0564);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_5_B, 0x0559);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_5_PG, 0x4000);
			break;
			
		case EQ_SPK_NORMAL:
			if(mode == EQ_HP_NORMAL)
			{
				aid1dca1_eq_gain1 = snd_soc_read(codec,WM8994_AIF1_DAC1_EQ_GAINS_1);
				aid1dca1_eq_gain2 = snd_soc_read(codec,WM8994_AIF1_DAC1_EQ_GAINS_2);
			}
			mode = EQ_SPK_NORMAL;
			snd_soc_update_bits(codec,WM8994_AIF1_DAC1_EQ_GAINS_1, WM8994_AIF1DAC1_EQ_ENA_MASK,0);
			snd_soc_update_bits(codec,WM8994_AIF1_DRC1_1, WM8994_AIF1DAC1_DRC_ENA_MASK,0);

			// EQ
#if 0
			snd_soc_update_bits(codec,WM8994_AIF1_DAC1_EQ_GAINS_1, WM8994_AIF1DAC1_EQ_B1_GAIN_MASK,0<<WM8994_AIF1DAC1_EQ_B1_GAIN_SHIFT);// -12db
			snd_soc_update_bits(codec,WM8994_AIF1_DAC1_EQ_GAINS_1, WM8994_AIF1DAC1_EQ_B2_GAIN_MASK,2<<WM8994_AIF1DAC1_EQ_B2_GAIN_SHIFT);// -10dB
			snd_soc_update_bits(codec,WM8994_AIF1_DAC1_EQ_GAINS_1, WM8994_AIF1DAC1_EQ_B3_GAIN_MASK,13<<WM8994_AIF1DAC1_EQ_B3_GAIN_SHIFT);// 1dB
			snd_soc_update_bits(codec,WM8994_AIF1_DAC1_EQ_GAINS_2, WM8994_AIF1DAC1_EQ_B4_GAIN_MASK,13<<WM8994_AIF1DAC1_EQ_B4_GAIN_SHIFT);// 1dB
			snd_soc_update_bits(codec,WM8994_AIF1_DAC1_EQ_GAINS_2, WM8994_AIF1DAC1_EQ_B5_GAIN_MASK,6<<WM8994_AIF1DAC1_EQ_B5_GAIN_SHIFT);// -6dB

			// 105 
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_1_A, 0x0FD2);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_1_B, 0x0400);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_1_PG, 0x00D8);

			// 300 - 2
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_2_A, 0x1E0B);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_2_B, 0xF1EE);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_2_C, 0x040B);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_2_PG, 0x07A4);

			// 850 - 2
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_3_A, 0x1B8F);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_3_B, 0xF390);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_3_C, 0x040B);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_3_PG, 0x0E1C);

			// 2400 -2
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_4_A, 0x1666);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_4_B, 0xF67F);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_4_C, 0x040A);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_4_PG, 0x19BA);

			// 6900
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_5_A, 0x08C0);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_5_B, 0x0630);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_5_PG, 0x4000);
			
#else
			snd_soc_update_bits(codec,WM8994_AIF1_DAC1_EQ_GAINS_1, WM8994_AIF1DAC1_EQ_B1_GAIN_MASK,0<<WM8994_AIF1DAC1_EQ_B1_GAIN_SHIFT);// -12db
			snd_soc_update_bits(codec,WM8994_AIF1_DAC1_EQ_GAINS_1, WM8994_AIF1DAC1_EQ_B2_GAIN_MASK,3<<WM8994_AIF1DAC1_EQ_B2_GAIN_SHIFT);// -9dB
			snd_soc_update_bits(codec,WM8994_AIF1_DAC1_EQ_GAINS_1, WM8994_AIF1DAC1_EQ_B3_GAIN_MASK,13<<WM8994_AIF1DAC1_EQ_B3_GAIN_SHIFT);// 1dB
			snd_soc_update_bits(codec,WM8994_AIF1_DAC1_EQ_GAINS_2, WM8994_AIF1DAC1_EQ_B4_GAIN_MASK,13<<WM8994_AIF1DAC1_EQ_B4_GAIN_SHIFT);// 1dB
			snd_soc_update_bits(codec,WM8994_AIF1_DAC1_EQ_GAINS_2, WM8994_AIF1DAC1_EQ_B5_GAIN_MASK,12<<WM8994_AIF1DAC1_EQ_B5_GAIN_SHIFT);// 0dB

			// 200 
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_1_A, 0x0FD2);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_1_B, 0x0400);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_1_PG, 0x00D8);

			// 400 - 2
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_2_A, 0x1E08);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_2_B, 0xF1EE);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_2_C, 0x040B);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_2_PG, 0x07A4);

			// 800 - 2
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_3_A, 0x1B8f);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_3_B, 0xF390);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_3_C, 0x040B);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_3_PG, 0x0E1C);

			// 2400 - 2
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_4_A, 0x1666);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_4_B, 0xF67F);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_4_C, 0x040A);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_4_PG, 0x19BA);

			// 6900
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_5_A, 0x08C0);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_5_B, 0x0630);
			snd_soc_write(codec,WM8994_AIF1_DAC1_EQ_BAND_5_PG, 0x4000);
#endif
			// DRC	
			snd_soc_write(codec,WM8994_AIF1_DRC1_2, 0x0845);
			snd_soc_write(codec,WM8994_AIF1_DRC1_3, 0x0011);
			snd_soc_write(codec,WM8994_AIF1_DRC1_4, 0x00E0);
			snd_soc_write(codec,WM8994_AIF1_DRC1_5, 0x0000);
			
			snd_soc_update_bits(codec,WM8994_AIF1_DAC1_EQ_GAINS_1, WM8994_AIF1DAC1_EQ_ENA_MASK,WM8994_AIF1DAC1_EQ_ENA);
			snd_soc_update_bits(codec,WM8994_AIF1_DRC1_1, WM8994_AIF1DAC1_DRC_ENA_MASK,WM8994_AIF1DAC1_DRC_ENA);
			break;
			
		case EQ_REC_INCALL:
			if(mode == EQ_HP_NORMAL)
			{
				aid1dca1_eq_gain1 = snd_soc_read(codec,WM8994_AIF1_DAC1_EQ_GAINS_1);
				aid1dca1_eq_gain2 = snd_soc_read(codec,WM8994_AIF1_DAC1_EQ_GAINS_2);
			}
			mode = EQ_REC_INCALL;
			break;
			
		default:
			break;
			
	}	
}

#if 1

void SetSpkMute(struct snd_soc_codec *codec,bool bmute)
{
	dprintk ("%s(%d) \n", __func__,bmute);
	if(bmute)
	{		
		snd_soc_update_bits(codec, WM8994_SPEAKER_VOLUME_RIGHT,WM8994_SPKOUTR_MUTE_N_MASK,0x00);	
		snd_soc_update_bits(codec, WM8994_SPEAKER_VOLUME_LEFT,WM8994_SPKOUTL_MUTE_N_MASK,0x00);
		
		snd_soc_update_bits(codec,WM8994_POWER_MANAGEMENT_3,WM8994_SPKLVOL_ENA_MASK|WM8994_SPKRVOL_ENA_MASK,0x00);
		snd_soc_update_bits(codec,WM8994_POWER_MANAGEMENT_1,WM8994_SPKOUTL_ENA_MASK|WM8994_SPKOUTR_ENA_MASK,0x00);
	}
	else
	{
		snd_soc_update_bits(codec,WM8994_POWER_MANAGEMENT_3,WM8994_SPKLVOL_ENA_MASK|WM8994_SPKRVOL_ENA_MASK,WM8994_SPKLVOL_ENA|WM8994_SPKRVOL_ENA);
		snd_soc_update_bits(codec,WM8994_POWER_MANAGEMENT_1,WM8994_SPKOUTL_ENA_MASK|WM8994_SPKOUTR_ENA_MASK,WM8994_SPKOUTL_ENA);
		
		snd_soc_update_bits(codec, WM8994_SPEAKER_VOLUME_RIGHT,WM8994_SPKOUTR_MUTE_N_MASK,WM8994_SPKOUTR_MUTE_N);
		snd_soc_update_bits(codec, WM8994_SPEAKER_VOLUME_LEFT,WM8994_SPKOUTL_MUTE_N_MASK,WM8994_SPKOUTL_MUTE_N);
	}
}

void SetRecMute(struct snd_soc_codec *codec,bool bmute)
{
	dprintk ("%s(%d) \n", __func__,bmute);
	if(bmute)
	{		
		snd_soc_update_bits(codec, WM8994_HPOUT2_VOLUME,WM8994_HPOUT2_MUTE_MASK,WM8994_HPOUT2_MUTE);
		
		snd_soc_update_bits(codec, WM8994_ANTIPOP_1, WM8994_HPOUT2_IN_ENA_MASK,0);
		snd_soc_update_bits(codec, WM8994_POWER_MANAGEMENT_1,WM8994_HPOUT2_ENA_MASK,0x00);	
	}
	else
	{
		snd_soc_update_bits(codec, WM8994_ANTIPOP_1, WM8994_HPOUT2_IN_ENA_MASK,WM8994_HPOUT2_IN_ENA);
		udelay(50);
		
		snd_soc_update_bits(codec, WM8994_POWER_MANAGEMENT_1,WM8994_HPOUT2_ENA_MASK,WM8994_HPOUT2_ENA);	
		snd_soc_update_bits(codec, WM8994_HPOUT2_VOLUME,WM8994_HPOUT2_MUTE_MASK,0x00);	
	}
}

#define DEFAULT_VAL_CHARGE_PUMP_1 0x1F25
void SetHpMute(struct snd_soc_codec *codec,bool bmute)
{
	dprintk ("%s(%d) \n", __func__,bmute);
	
	if(bmute)
	{
		snd_soc_update_bits(codec, WM8994_RIGHT_OUTPUT_VOLUME,WM8994_HPOUT1R_MUTE_N_MASK,0x00);	
		snd_soc_update_bits(codec, WM8994_LEFT_OUTPUT_VOLUME,WM8994_HPOUT1L_MUTE_N_MASK,0x00);
				
		snd_soc_write(codec,WM8994_CHARGE_PUMP_1 ,DEFAULT_VAL_CHARGE_PUMP_1);
		snd_soc_update_bits(codec, WM8994_POWER_MANAGEMENT_1,(WM8994_HPOUT1L_ENA_MASK |WM8994_HPOUT1R_ENA_MASK ),0x00);	
	}
	else
	{						
		snd_soc_write(codec, WM8994_CHARGE_PUMP_1, WM8994_CP_ENA | DEFAULT_VAL_CHARGE_PUMP_1);
		snd_soc_update_bits(codec, WM8994_POWER_MANAGEMENT_1,(WM8994_HPOUT1L_ENA_MASK |WM8994_HPOUT1R_ENA_MASK ),WM8994_HPOUT1R_ENA	 |WM8994_HPOUT1L_ENA);	
		
		snd_soc_update_bits(codec, WM8994_RIGHT_OUTPUT_VOLUME,WM8994_HPOUT1R_MUTE_N_MASK,WM8994_HPOUT1R_MUTE_N);	
		snd_soc_update_bits(codec, WM8994_LEFT_OUTPUT_VOLUME,WM8994_HPOUT1L_MUTE_N_MASK,WM8994_HPOUT1L_MUTE_N);
	}
}

#else

void SetSpkMute(struct snd_soc_codec *codec,bool bmute)
{
	dprintk ("%s() \n", __func__);
	if(bmute)
	{
		snd_soc_update_bits(codec, WM8994_SPEAKER_VOLUME_RIGHT,WM8994_SPKOUTR_MUTE_N_MASK,0x00);	
		snd_soc_update_bits(codec, WM8994_SPEAKER_VOLUME_LEFT,WM8994_SPKOUTL_MUTE_N_MASK,0x00);
	}
	else
	{
		snd_soc_update_bits(codec, WM8994_SPEAKER_VOLUME_RIGHT,WM8994_SPKOUTR_MUTE_N_MASK,WM8994_SPKOUTR_MUTE_N);
		snd_soc_update_bits(codec, WM8994_SPEAKER_VOLUME_LEFT,WM8994_SPKOUTL_MUTE_N_MASK,WM8994_SPKOUTL_MUTE_N);
	}
}

void SetRecMute(struct snd_soc_codec *codec,bool bmute)
{
	dprintk ("%s() \n", __func__);
	if(bmute)
	{
		snd_soc_update_bits(codec, WM8994_HPOUT2_VOLUME,WM8994_HPOUT2_MUTE_MASK,WM8994_HPOUT2_MUTE);
	}
	else
	{
		snd_soc_update_bits(codec, WM8994_HPOUT2_VOLUME,WM8994_HPOUT2_MUTE_MASK,0x00);	
	}
}

void SetHpMute(struct snd_soc_codec *codec,bool bmute)
{
	dprintk ("%s() \n", __func__);
	if(bmute)
	{
		snd_soc_update_bits(codec, WM8994_RIGHT_OUTPUT_VOLUME,WM8994_HPOUT1R_MUTE_N_MASK,0x00);	
		snd_soc_update_bits(codec, WM8994_LEFT_OUTPUT_VOLUME,WM8994_HPOUT1L_MUTE_N_MASK,0x00);
	}
	else
	{
		snd_soc_update_bits(codec, WM8994_RIGHT_OUTPUT_VOLUME,WM8994_HPOUT1R_MUTE_N_MASK,WM8994_HPOUT1R_MUTE_N);	
		snd_soc_update_bits(codec, WM8994_LEFT_OUTPUT_VOLUME,WM8994_HPOUT1L_MUTE_N_MASK,WM8994_HPOUT1L_MUTE_N);
	}
}

#endif

void SetVolume_Normal(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8958 = snd_soc_codec_get_drvdata(codec);
	dprintk ("%s() nrec_switch = %d\n", __func__,wm8958->music_headset_nrec_switch);
	// Speaker Boost
	snd_soc_update_bits(codec, WM8994_CLASSD,WM8994_SPKOUTL_BOOST_MASK,VOL_NORMAL_SPK_BOOST);
	
	// Speaker
	snd_soc_update_bits(codec, WM8994_SPEAKER_VOLUME_LEFT,WM8994_SPKOUTL_VOL_MASK|WM8994_SPKOUT_VU_MASK,VOL_NORMAL_SPK);
	snd_soc_update_bits(codec, WM8994_SPEAKER_VOLUME_RIGHT,WM8994_SPKOUTR_VOL_MASK|WM8994_SPKOUT_VU_MASK,VOL_NORMAL_SPK);
	snd_soc_update_bits(codec, WM8994_SPEAKER_VOLUME_LEFT,WM8994_SPKOUT_VU_MASK,WM8994_SPKOUT_VU);
	snd_soc_update_bits(codec, WM8994_SPEAKER_VOLUME_RIGHT,WM8994_SPKOUT_VU_MASK,WM8994_SPKOUT_VU);


	if(wm8958->music_headset_nrec_switch == 1)
	{
		// HP
		snd_soc_update_bits(codec, WM8994_LEFT_OUTPUT_VOLUME,WM8994_HPOUT1L_VOL_MASK|WM8994_HPOUT1_VU_MASK,VOL_NORMAL_HP_NR);
		snd_soc_update_bits(codec, WM8994_RIGHT_OUTPUT_VOLUME,WM8994_HPOUT1R_VOL_MASK|WM8994_HPOUT1_VU_MASK,VOL_NORMAL_HP_NR);
		snd_soc_update_bits(codec, WM8994_LEFT_OUTPUT_VOLUME,WM8994_HPOUT1_VU_MASK,WM8994_HPOUT1_VU);
		snd_soc_update_bits(codec, WM8994_RIGHT_OUTPUT_VOLUME,WM8994_HPOUT1_VU_MASK,WM8994_HPOUT1_VU);
	}
	 else
	{
		// HP
		snd_soc_update_bits(codec, WM8994_LEFT_OUTPUT_VOLUME,WM8994_HPOUT1L_VOL_MASK|WM8994_HPOUT1_VU_MASK,VOL_NORMAL_HP);
		snd_soc_update_bits(codec, WM8994_RIGHT_OUTPUT_VOLUME,WM8994_HPOUT1R_VOL_MASK|WM8994_HPOUT1_VU_MASK,VOL_NORMAL_HP);
		snd_soc_update_bits(codec, WM8994_LEFT_OUTPUT_VOLUME,WM8994_HPOUT1_VU_MASK,WM8994_HPOUT1_VU);
		snd_soc_update_bits(codec, WM8994_RIGHT_OUTPUT_VOLUME,WM8994_HPOUT1_VU_MASK,WM8994_HPOUT1_VU);
	}

	// REC
	snd_soc_update_bits(codec, WM8994_LEFT_OPGA_VOLUME,WM8994_MIXOUTL_VOL_MASK|WM8994_MIXOUT_VU_MASK,VOL_NORMAL_REC);
	snd_soc_update_bits(codec, WM8994_RIGHT_OPGA_VOLUME,WM8994_MIXOUTR_VOL_MASK|WM8994_MIXOUT_VU_MASK,VOL_NORMAL_REC);
	snd_soc_update_bits(codec, WM8994_LEFT_OPGA_VOLUME,WM8994_MIXOUT_VU_MASK,WM8994_MIXOUT_VU);
	snd_soc_update_bits(codec, WM8994_RIGHT_OPGA_VOLUME,WM8994_MIXOUT_VU_MASK,WM8994_MIXOUT_VU);
	//snd_soc_update_bits(codec, WM8994_HPOUT2_VOLUME,WM8994_HPOUT2_VOL_MASK,VOL_NORMAL_HP2);
}

void SetVolume_Ring(struct snd_soc_codec *codec)
{
	dprintk ("%s() \n", __func__);
	// Speaker Boost
	snd_soc_update_bits(codec, WM8994_CLASSD,WM8994_SPKOUTL_BOOST_MASK,VOL_RING_SPK_BOOST);
	
	// Speaker
	snd_soc_update_bits(codec, WM8994_SPEAKER_VOLUME_LEFT,WM8994_SPKOUTL_VOL_MASK|WM8994_SPKOUT_VU_MASK,VOL_RING_SPK);
	snd_soc_update_bits(codec, WM8994_SPEAKER_VOLUME_RIGHT,WM8994_SPKOUTR_VOL_MASK|WM8994_SPKOUT_VU_MASK,VOL_RING_SPK);
	snd_soc_update_bits(codec, WM8994_SPEAKER_VOLUME_LEFT,WM8994_SPKOUT_VU_MASK,WM8994_SPKOUT_VU);
	snd_soc_update_bits(codec, WM8994_SPEAKER_VOLUME_RIGHT,WM8994_SPKOUT_VU_MASK,WM8994_SPKOUT_VU);

	// HP
	snd_soc_update_bits(codec, WM8994_LEFT_OUTPUT_VOLUME,WM8994_HPOUT1L_VOL_MASK|WM8994_HPOUT1_VU_MASK,VOL_RING_HP);
	snd_soc_update_bits(codec, WM8994_RIGHT_OUTPUT_VOLUME,WM8994_HPOUT1R_VOL_MASK|WM8994_HPOUT1_VU_MASK,VOL_RING_HP);
	snd_soc_update_bits(codec, WM8994_LEFT_OUTPUT_VOLUME,WM8994_HPOUT1_VU_MASK,WM8994_HPOUT1_VU);
	snd_soc_update_bits(codec, WM8994_RIGHT_OUTPUT_VOLUME,WM8994_HPOUT1_VU_MASK,WM8994_HPOUT1_VU);

	// REC
	snd_soc_update_bits(codec, WM8994_LEFT_OPGA_VOLUME,WM8994_MIXOUTL_VOL_MASK|WM8994_MIXOUT_VU_MASK,VOL_RING_REC);
	snd_soc_update_bits(codec, WM8994_RIGHT_OPGA_VOLUME,WM8994_MIXOUTR_VOL_MASK|WM8994_MIXOUT_VU_MASK,VOL_RING_REC);
	snd_soc_update_bits(codec, WM8994_LEFT_OPGA_VOLUME,WM8994_MIXOUT_VU_MASK,WM8994_MIXOUT_VU);
	snd_soc_update_bits(codec, WM8994_RIGHT_OPGA_VOLUME,WM8994_MIXOUT_VU_MASK,WM8994_MIXOUT_VU);
	//snd_soc_update_bits(codec, WM8994_HPOUT2_VOLUME,WM8994_HPOUT2_VOL_MASK,VOL_RING_HP2);
}


void SetVolume_Incall(struct snd_soc_codec *codec)
{
	dprintk ("%s() \n", __func__);
	// Speaker Boost
	snd_soc_update_bits(codec, WM8994_CLASSD,WM8994_SPKOUTL_BOOST_MASK,VOL_INCALL_SPK_BOOST);
	
	// Speaker
	snd_soc_update_bits(codec, WM8994_SPEAKER_VOLUME_LEFT,WM8994_SPKOUTL_VOL_MASK|WM8994_SPKOUT_VU_MASK,VOL_INCALL_SPK);
	snd_soc_update_bits(codec, WM8994_SPEAKER_VOLUME_RIGHT,WM8994_SPKOUTR_VOL_MASK|WM8994_SPKOUT_VU_MASK,VOL_INCALL_SPK);
	snd_soc_update_bits(codec, WM8994_SPEAKER_VOLUME_LEFT,WM8994_SPKOUT_VU_MASK,WM8994_SPKOUT_VU);
	snd_soc_update_bits(codec, WM8994_SPEAKER_VOLUME_RIGHT,WM8994_SPKOUT_VU_MASK,WM8994_SPKOUT_VU);

	// HP
	snd_soc_update_bits(codec, WM8994_LEFT_OUTPUT_VOLUME,WM8994_HPOUT1L_VOL_MASK|WM8994_HPOUT1_VU_MASK,VOL_INCALL_HP);
	snd_soc_update_bits(codec, WM8994_RIGHT_OUTPUT_VOLUME,WM8994_HPOUT1R_VOL_MASK|WM8994_HPOUT1_VU_MASK,VOL_INCALL_HP);
	snd_soc_update_bits(codec, WM8994_LEFT_OUTPUT_VOLUME,WM8994_HPOUT1_VU_MASK,WM8994_HPOUT1_VU);
	snd_soc_update_bits(codec, WM8994_RIGHT_OUTPUT_VOLUME,WM8994_HPOUT1_VU_MASK,WM8994_HPOUT1_VU);

	// REC
	snd_soc_update_bits(codec, WM8994_LEFT_OPGA_VOLUME,WM8994_MIXOUTL_VOL_MASK|WM8994_MIXOUT_VU_MASK,VOL_INCALL_REC);
	snd_soc_update_bits(codec, WM8994_RIGHT_OPGA_VOLUME,WM8994_MIXOUTR_VOL_MASK|WM8994_MIXOUT_VU_MASK,VOL_INCALL_REC);
	snd_soc_update_bits(codec, WM8994_LEFT_OPGA_VOLUME,WM8994_MIXOUT_VU_MASK,WM8994_MIXOUT_VU);
	snd_soc_update_bits(codec, WM8994_RIGHT_OPGA_VOLUME,WM8994_MIXOUT_VU_MASK,WM8994_MIXOUT_VU);
	snd_soc_update_bits(codec, WM8994_HPOUT2_VOLUME,WM8994_HPOUT2_VOL_MASK,VOL_INCALL_HP2);
	snd_soc_update_bits(codec, WM8994_AIF2_CONTROL_2,WM8994_AIF2DAC_BOOST_MASK,VOL_INCALL_AIF2DAC_BOOST);
}

void SetVolume_VoIP(struct snd_soc_codec *codec)
{
	dprintk ("%s() \n", __func__);
	// Speaker Boost
	snd_soc_update_bits(codec, WM8994_CLASSD,WM8994_SPKOUTL_BOOST_MASK,VOL_VOIP_SPK_BOOST);
	
	// Speaker
	snd_soc_update_bits(codec, WM8994_SPEAKER_VOLUME_LEFT,WM8994_SPKOUTL_VOL_MASK|WM8994_SPKOUT_VU_MASK,VOL_VOIP_SPK);
	snd_soc_update_bits(codec, WM8994_SPEAKER_VOLUME_RIGHT,WM8994_SPKOUTR_VOL_MASK|WM8994_SPKOUT_VU_MASK,VOL_VOIP_SPK);
	snd_soc_update_bits(codec, WM8994_SPEAKER_VOLUME_LEFT,WM8994_SPKOUT_VU_MASK,WM8994_SPKOUT_VU);
	snd_soc_update_bits(codec, WM8994_SPEAKER_VOLUME_RIGHT,WM8994_SPKOUT_VU_MASK,WM8994_SPKOUT_VU);

	// HP
	snd_soc_update_bits(codec, WM8994_LEFT_OUTPUT_VOLUME,WM8994_HPOUT1L_VOL_MASK|WM8994_HPOUT1_VU_MASK,VOL_VOIP_HP);
	snd_soc_update_bits(codec, WM8994_RIGHT_OUTPUT_VOLUME,WM8994_HPOUT1R_VOL_MASK|WM8994_HPOUT1_VU_MASK,VOL_VOIP_HP);
	snd_soc_update_bits(codec, WM8994_LEFT_OUTPUT_VOLUME,WM8994_HPOUT1_VU_MASK,WM8994_HPOUT1_VU);
	snd_soc_update_bits(codec, WM8994_RIGHT_OUTPUT_VOLUME,WM8994_HPOUT1_VU_MASK,WM8994_HPOUT1_VU);

	// REC
	snd_soc_update_bits(codec, WM8994_LEFT_OPGA_VOLUME,WM8994_MIXOUTL_VOL_MASK|WM8994_MIXOUT_VU_MASK,VOL_VOIP_REC);
	snd_soc_update_bits(codec, WM8994_RIGHT_OPGA_VOLUME,WM8994_MIXOUTR_VOL_MASK|WM8994_MIXOUT_VU_MASK,VOL_VOIP_REC);
	snd_soc_update_bits(codec, WM8994_LEFT_OPGA_VOLUME,WM8994_MIXOUT_VU_MASK,WM8994_MIXOUT_VU);
	snd_soc_update_bits(codec, WM8994_RIGHT_OPGA_VOLUME,WM8994_MIXOUT_VU_MASK,WM8994_MIXOUT_VU);
	snd_soc_update_bits(codec, WM8994_HPOUT2_VOLUME,WM8994_HPOUT2_VOL_MASK,VOL_VOIP_HP2);
	snd_soc_update_bits(codec, WM8994_AIF2_CONTROL_2,WM8994_AIF2DAC_BOOST_MASK,VOL_VOIP_AIF2DAC_BOOST);
}


void SetVolume_Mixerin_spk(struct snd_soc_codec *codec)
{	
	dprintk ("%s() \n", __func__);
	// Mixer in
	snd_soc_update_bits(codec, WM8994_INPUT_MIXER_3,WM8994_IN1L_MIXINL_VOL_MASK, VOL_IN1L_BOOST_SPK);
	snd_soc_update_bits(codec, WM8994_INPUT_MIXER_3,WM8994_IN2L_MIXINL_VOL_MASK, VOL_IN2L_BOOST_SPK);
	snd_soc_update_bits(codec, WM8994_INPUT_MIXER_4,WM8994_IN1R_MIXINR_VOL_MASK, VOL_IN1R_BOOST_SPK);
	
	// 0x19 = +21dB  0x1F = +30dB 0x07 = -6dB
	snd_soc_update_bits(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME,WM8994_IN1L_VOL_MASK, VOL_IN1L_SPK);
	snd_soc_update_bits(codec, WM8994_LEFT_LINE_INPUT_3_4_VOLUME,WM8994_IN2L_VOL_MASK, VOL_IN2L_SPK);
	snd_soc_update_bits(codec, WM8994_RIGHT_LINE_INPUT_1_2_VOLUME,WM8994_IN1R_VOL_MASK, VOL_IN1R_SPK);
}

void SetVolume_Mixerin_rec(struct snd_soc_codec *codec)
{	
	dprintk ("%s() \n", __func__);
	// Mixer in
	snd_soc_update_bits(codec, WM8994_INPUT_MIXER_3,WM8994_IN1L_MIXINL_VOL_MASK, VOL_IN1L_BOOST_REC);
	snd_soc_update_bits(codec, WM8994_INPUT_MIXER_3,WM8994_IN2L_MIXINL_VOL_MASK, VOL_IN2L_BOOST_REC);
	snd_soc_update_bits(codec, WM8994_INPUT_MIXER_4,WM8994_IN1R_MIXINR_VOL_MASK, VOL_IN1R_BOOST_REC);
	
	// 0x19 = +21dB  0x1F = +30dB 0x07 = -6dB
	snd_soc_update_bits(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME,WM8994_IN1L_VOL_MASK, VOL_IN1L_REC);
	snd_soc_update_bits(codec, WM8994_LEFT_LINE_INPUT_3_4_VOLUME,WM8994_IN2L_VOL_MASK, VOL_IN2L_REC);
	snd_soc_update_bits(codec, WM8994_RIGHT_LINE_INPUT_1_2_VOLUME,WM8994_IN1R_VOL_MASK, VOL_IN1R_REC);
}

void SetVolume_ADC1(struct snd_soc_codec *codec)
{	
	dprintk ("%s() \n", __func__);
	
	// AIF1ADC1
	snd_soc_update_bits(codec, WM8994_AIF1_ADC1_RIGHT_VOLUME,WM8994_AIF1ADC1R_VOL_MASK, VOL_AIF1_ADC1_RIGHT);
	snd_soc_update_bits(codec, WM8994_AIF1_ADC1_LEFT_VOLUME,WM8994_AIF1ADC1L_VOL_MASK, VOL_AIF1_ADC1_LEFT);
}


void OpenAIF2(struct snd_soc_codec *codec)
{		
	dprintk ("%s \n", __func__);
	
	//snd_soc_write(codec,WM8994_POWER_MANAGEMENT_4, 0x00 );	
	snd_soc_update_bits(codec, WM8994_POWER_MANAGEMENT_4,WM8994_AIF2ADCL_ENA_MASK|WM8994_AIF2ADCR_ENA_MASK, 0x00);
	
	// Audio Interface Input/Output (I/O) Configuration
	snd_soc_write(codec,WM8994_GPIO_2, 0x8101);  // MCLK2 - Pull-up and pull-down resistors disabled 0xA101
	snd_soc_write(codec,WM8994_GPIO_3, 0x8101);  // BCLK2 - Pull-up and pull-down resistors disabled
	snd_soc_write(codec,WM8994_GPIO_4, 0x8101);  // LRCLK2 - Pull-up and pull-down resistors disabled
	snd_soc_write(codec,WM8994_GPIO_5, 0x8101);  // DACDAT2 - Pull-up and pull-down resistors disabled	
 
 	// Clocking	&  AIF2 Interface
	snd_soc_write(codec, WM8994_OVERSAMPLING, WM8994_ADC_OSR128 );	// Select High Power ADC/DMIC Oversample Rate, Select Low Power DAC Oversample Rate (Default)	
	snd_soc_write(codec, WM8994_AIF2_CONTROL_1, 0x411B );	// AIF2 Word Length = 16-bits, AIF2 Format = DSP,BCLK2 inverted
	snd_soc_write(codec, WM8994_AIF2_CONTROL_2, 0x4000 );	// Disable AIF2 DSP Mono Mode
#ifdef CONFIG_MACH_M040
	snd_soc_write(codec, WM8994_AIF2_CONTROL_2, 0x4000 );	// Disable AIF2 DSP Mono Mode
	// snd_soc_write(codec, WM8994_AIF2_MASTER_SLAVE, 0x0000 );	// AIF2 Slave Mode
	snd_soc_write(codec, WM8994_AIF2_MASTER_SLAVE, 0x4000 );	// AIF2 Master Mode
	snd_soc_write(codec, WM8994_AIF2_BCLK, 0x00A0 );	// AIF2CLK / 16
	snd_soc_write(codec, WM8994_AIF2ADC_LRCLK, 0x0040 );	// BCLK2 / 64
	snd_soc_write(codec, WM8994_AIF2DAC_LRCLK, 0x0003 );	// BCLK2 / 3

	snd_soc_update_bits(codec, WM8994_CLOCKING_1,WM8994_AIF2DSPCLK_ENA_MASK|WM8994_SYSDSPCLK_ENA_MASK, WM8994_AIF2DSPCLK_ENA|WM8994_SYSDSPCLK_ENA);
	snd_soc_update_bits(codec, WM8994_CLOCKING_1,WM8994_SYSCLK_SRC_MASK, 0x00);// Set the core clock source to AIF1CLK

	snd_soc_write(codec, WM8994_AIF2_RATE, 0x0005 );		// AIF2 Sample Rate = 8 kHz, AIF2CLK/Fs ratio = 512
	snd_soc_write(codec,WM8994_FLL2_CONTROL_1, 0);			// Disable the FLL while reconfiguring
	snd_soc_write(codec,WM8994_FLL2_CONTROL_2, 0x1500 );	// FLL2_OUTDIV = 22, FLL2_FRATIO = 1
	// snd_soc_write(codec,WM8994_FLL2_CONTROL_3, 0xBE77 );	// FLL2_K = 0.744
	// snd_soc_write(codec,WM8994_FLL2_CONTROL_3, 0xFA8A );	// FLL2_K = 0.97867
	snd_soc_write(codec,WM8994_FLL2_CONTROL_3, 0x8264 );	// FLL2_K = 0.50934
	snd_soc_write(codec,WM8994_FLL2_CONTROL_4, 0x00E0 );	// FLL2_N = 7
	snd_soc_write(codec,WM8994_FLL2_CONTROL_5, 0x0C88 );	// FLL@_REFCLK_DIV = MCLK / 2, FLL2_REFCLK_SRC = MCLK1
	snd_soc_write(codec,WM8994_FLL2_CONTROL_1, WM8994_FLL2_ENA);
#else
	snd_soc_write(codec, WM8994_AIF2_MASTER_SLAVE, 0x0000 );	// AIF2 Slave Mode (Default Register Value)
	snd_soc_write(codec, WM8994_AIF2_BCLK, 0x00A0 );	// 
	snd_soc_write(codec, WM8994_AIF2ADC_LRCLK, 0x0040 );	// 
	snd_soc_write(codec, WM8994_AIF2DAC_LRCLK, 0x0003 );	// 

	snd_soc_update_bits(codec, WM8994_CLOCKING_1,WM8994_AIF2DSPCLK_ENA_MASK|WM8994_SYSDSPCLK_ENA_MASK, WM8994_AIF2DSPCLK_ENA|WM8994_SYSDSPCLK_ENA);
	snd_soc_update_bits(codec, WM8994_CLOCKING_1,WM8994_SYSCLK_SRC_MASK, 0x00);// Set the core clock source to AIF1CLK
	
	// FLL2 Setting
#ifdef CONFIG_MACH_M030
	if(machine_is_m030()){
		snd_soc_write(codec, WM8994_AIF2_RATE, 0x0005 );		// AIF2 Sample Rate = 8 kHz, AIF2CLK/Fs ratio = 512
		snd_soc_write(codec,WM8994_FLL2_CONTROL_1, 0);			// Disable the FLL while reconfiguring 
		snd_soc_write(codec,WM8994_FLL2_CONTROL_2, 0x1500 );	
		snd_soc_write(codec,WM8994_FLL2_CONTROL_3, 0x8264 );	
		snd_soc_write(codec,WM8994_FLL2_CONTROL_4, 0x00E0 );
		snd_soc_write(codec,WM8994_FLL2_CONTROL_5, 0x0C80 );	
		snd_soc_write(codec,WM8994_FLL2_CONTROL_1, WM8994_FLL2_ENA);	
	}else{
#endif
		snd_soc_write(codec, WM8994_AIF2_RATE, 0x0001 );		// AIF2 Sample Rate = 8 kHz, AIF2CLK/Fs ratio = 128
		snd_soc_write(codec,WM8994_FLL2_CONTROL_1, 0);			// Disable the FLL while reconfiguring 
		snd_soc_write(codec,WM8994_FLL2_CONTROL_2, 0x2F01 );		// FLL2 Control (2)(241H):  2F01  FLL2_OUTDIV=48, FLL2_FRATIO=2
		snd_soc_write(codec,WM8994_FLL2_CONTROL_3, 0x2E8C );		// FLL2 Control (3)(242H):  2E8C  FLL2_K=0.18182
		snd_soc_write(codec,WM8994_FLL2_CONTROL_4, 0x1740 );		// FLL2 Control (4)(243H):  1740  FLL2_N=186
		snd_soc_write(codec,WM8994_FLL2_CONTROL_5, 0x0C83 );		// FLL2 Control (5)(244H):  0C83  FLL2_BYP=FLL2, FLL2_FRC_NCO_VAL=01_1001, FLL2_FRC_NCO=0, FLL2_REFCLK_DIV=MCLK / 1, FLL2_REFCLK_SRC=BCLK2
		snd_soc_write(codec,WM8994_FLL2_CONTROL_1, WM8994_FLL2_ENA);	
#ifdef CONFIG_MACH_M030
	}
#endif
#endif
	snd_soc_write(codec,WM8994_AIF2_CLOCKING_1, 0x0019 );	// Enable AIF2 Clock, AIF2 Clock Source = FLL2/BCLK2
	
	// Unmutes
	snd_soc_write(codec, WM8994_AIF2_DAC_FILTERS_1, WM8994_AIF2DAC_MONO);// Unmute the AIF2 DAC path,AIF2DAC input path Mono Mixer control

	// Digital Path Enables and Unmutes 	
	snd_soc_update_bits(codec, WM8994_POWER_MANAGEMENT_5 , WM8994_AIF2DACL_ENA_MASK| WM8994_AIF2DACR_ENA_MASK,WM8994_AIF2DACL_ENA | WM8994_AIF2DACR_ENA );

	// Analogue Input Configuration
	snd_soc_update_bits(codec, WM8994_POWER_MANAGEMENT_2,WM8994_IN1L_ENA_MASK|WM8994_IN1R_ENA_MASK|WM8994_IN2L_ENA_MASK|WM8994_MIXINL_ENA_MASK|WM8994_MIXINR_ENA_MASK, 
			WM8994_IN1L_ENA|WM8994_IN1R_ENA|WM8994_IN2L_ENA|WM8994_MIXINL_ENA|WM8994_MIXINR_ENA);
	msleep(20);
	
	snd_soc_update_bits(codec, WM8994_POWER_MANAGEMENT_4,WM8994_AIF2ADCL_ENA_MASK|WM8994_AIF2ADCR_ENA_MASK|WM8994_ADCL_ENA_MASK|WM8994_ADCR_ENA_MASK, 
			WM8994_AIF2ADCL_ENA|WM8994_AIF2ADCR_ENA|WM8994_ADCL_ENA|WM8994_ADCR_ENA);
		
}


void CloseAIF2(struct snd_soc_codec *codec)
{		
	dprintk ("%s \n", __func__);
	
	// Unmutes
	snd_soc_write(codec, WM8994_AIF2_DAC_FILTERS_1, WM8994_AIF2DAC_MUTE);// Mute the AIF2 DAC path
	//snd_soc_update_bits(codec, WM8994_AIF2_CONTROL_1,0x3,0x00); 

	//snd_soc_update_bits(codec, WM8994_POWER_MANAGEMENT_4,WM8994_AIF2ADCL_ENA_MASK|WM8994_AIF2ADCR_ENA_MASK,0x00); 
	snd_soc_update_bits(codec, WM8994_POWER_MANAGEMENT_5 , WM8994_AIF2DACL_ENA_MASK| WM8994_AIF2DACR_ENA_MASK,0x00 );

	snd_soc_update_bits(codec, WM8994_CLOCKING_1,WM8994_AIF2DSPCLK_ENA_MASK, 0x00); // Disable AIF2DSPCLK
	
	// FLL2 Setting
	snd_soc_update_bits(codec, WM8994_FLL2_CONTROL_1,WM8994_FLL2_ENA_MASK, 0x00); // Disable FLL2
	
	snd_soc_update_bits(codec, WM8994_AIF2_CLOCKING_1,WM8994_AIF2CLK_ENA_MASK, 0x00); // Disable AIF2 Clock

#ifdef CONFIG_AUDIENCE_ES305B
	es305b_setmode(ES305B_SUSPEND);
#endif
#ifdef CONFIG_AUDIENCE_A1028
	//a1028_setmode(NULL,A1028_SUSPEND);
#endif
}

void SetAIF3_2_AIF2(struct snd_soc_codec *codec,bool enable)
{
	dprintk ("%s(%d) \n", __func__,enable);
	
	if( enable )
	{		
		// AIF3 input/Output (I/O) Configuration
		snd_soc_write(codec, WM8994_GPIO_8, 0xA100 ); // DACDAT3
		snd_soc_write(codec, WM8994_GPIO_9, 0xA100 ); // ADCDAT3
		snd_soc_write(codec, WM8994_GPIO_10, 0xA100 );// LRCLK3
		snd_soc_write(codec, WM8994_GPIO_11, 0xA100 );// BCLK3

		// AIF2 ADCDAT SRC select GPIO8/DACDAT3
		// AIF3 ADCDAT SRC select GPIO5/DACDAT2
		//snd_soc_update_bits(codec, WM8994_POWER_MANAGEMENT_6,
			//WM8994_AIF3_ADCDAT_SRC_MASK|WM8994_AIF2_ADCDAT_SRC_MASK, 
			//(2<<WM8994_AIF3_ADCDAT_SRC_SHIFT)|(1<<WM8994_AIF2_ADCDAT_SRC_SHIFT));
			
		snd_soc_write(codec, WM8994_POWER_MANAGEMENT_6,
			(2<<WM8994_AIF3_ADCDAT_SRC_SHIFT)|(1<<WM8994_AIF2_ADCDAT_SRC_SHIFT));

	}
	else
	{
		// AIF2 ADCDAT SRC select AIF2 ADCDAT2
		//snd_soc_update_bits(codec, WM8994_POWER_MANAGEMENT_6,WM8994_AIF2_ADCDAT_SRC_MASK, 0x00);
		snd_soc_write(codec, WM8994_POWER_MANAGEMENT_6,0x00);
	}
}

void OpenBTRing(struct snd_soc_codec *codec)
{
	snd_soc_write(codec, WM8994_AIF1_CLOCKING_1, 0x0001); // * AIF1 Clocking (1)(200H): 0001  AIF1CLK_SRC=MCLK1, AIF1CLK_INV=0, AIF1CLK_DIV=AIF1CLK, AIF1CLK_ENA=1
	snd_soc_write(codec, WM8994_AIF2_CLOCKING_1, 0x0001); // * AIF2 Clocking (1)(204H): 0001  AIF2CLK_SRC=MCLK1, AIF2CLK_INV=0, AIF2CLK_DIV=AIF2CLK, AIF2CLK_ENA=1
	snd_soc_write(codec, WM8994_CLOCKING_1, 0x000E); // * Clocking (1)(208H):      000E  DSP2CLK_ENA=0, DSP2CLK_SRC=AIF1CLK Source, TOCLK_ENA=0, AIF1DSPCLK_ENA=1, AIF2DSPCLK_ENA=1, SYSDSPCLK_ENA=1, SYSCLK_SRC=AIF1CLK
	snd_soc_write(codec, WM8994_AIF2_RATE, 0x0003); // * AIF2 Rate(211H): 0009  AIF2_SR=8 kHz, AIF2CLK_RATE=256
	snd_soc_write(codec, WM8994_POWER_MANAGEMENT_5, 0x030F); // * Power Management (5)(05H): 030F  AIF2DACL_ENA=0, AIF2DACR_ENA=0, AIF1DAC2L_ENA=0, AIF1DAC2R_ENA=0, AIF1DAC1L_ENA=1, AIF1DAC1R_ENA=1, DAC2L_ENA=1, DAC2R_ENA=1, DAC1L_ENA=1, DAC1R_ENA=1
	snd_soc_write(codec, WM8994_POWER_MANAGEMENT_6, 0x0008); // * Power Management (6)(06H): 0008  AIF3ADC_SRC=None, AIF2DAC_SRC=Left and Right inputs from AIF2, AIF3_TRI=0, AIF3_ADCDAT_SRC=AIF2 ADCDAT2, AIF2_ADCDAT_SRC=AIF2 ADCDAT2, AIF2_DACDAT_SRC=DACDAT2, AIF1_DACDAT_SRC=DACDAT1
	snd_soc_write(codec, WM8994_POWER_MANAGEMENT_4, 0x3000); // * Power Management (4)(04H): 3000  AIF2ADCL_ENA=1, AIF2ADCR_ENA=1, AIF1ADC2L_ENA=0, AIF1ADC2R_ENA=0, AIF1ADC1L_ENA=0, AIF1ADC1R_ENA=0, DMIC2L_ENA=0, DMIC2R_ENA=0, DMIC1L_ENA=0, DMIC1R_ENA=0, ADCL_ENA=0, ADCR_ENA=0
	snd_soc_write(codec, WM8994_DAC2_LEFT_MIXER_ROUTING, 0x0001); // * DAC2 Left Mixer Routing(604H): 0001  ADCR_TO_DAC2L=0, ADCL_TO_DAC2L=0, AIF2DACL_TO_DAC2L=0, AIF1DAC2L_TO_DAC2L=0, AIF1DAC1L_TO_DAC2L=1
	snd_soc_write(codec, WM8994_DAC2_RIGHT_MIXER_ROUTING, 0x0001); // * DAC2 Right Mixer Routing(605H): 0001  ADCR_TO_DAC2R=0, ADCL_TO_DAC2R=0, AIF2DACR_TO_DAC2R=0, AIF1DAC2R_TO_DAC2R=0, AIF1DAC1R_TO_DAC2R=1
	snd_soc_write(codec, WM8994_DAC1_LEFT_VOLUME, 0x01C0); // * DAC1 Left Volume(610H):  01C0  DAC1L_MUTE=0, DAC1_VU=1, DAC1L_VOL=0dB
	snd_soc_write(codec, WM8994_DAC1_RIGHT_VOLUME, 0x01C0);// * DAC1 Right Volume(611H): 01C0  DAC1R_MUTE=0, DAC1_VU=1, DAC1R_VOL=0dB
	snd_soc_write(codec, WM8994_DAC2_LEFT_VOLUME, 0x01C0); // * DAC2 Left Volume(612H):  01C0  DAC2L_MUTE=0, DAC2_VU=1, DAC2L_VOL=0dB
	snd_soc_write(codec, WM8994_DAC2_RIGHT_VOLUME, 0x01C0);// * DAC2 Right Volume(613H): 01C0  DAC2R_MUTE=0, DAC2_VU=1, DAC2R_VOL=0dB
	snd_soc_write(codec, WM8994_OVERSAMPLING, 0x0000); // * Oversampling(620H):      0000  ADC_OSR128=Low Power, DAC_OSR128=Low Power
	snd_soc_write(codec, WM8994_GPIO_8 , 0xA100); // * GPIO 8(707H):            A100  GP8_DIR=Input, GP8_PU=0, GP8_PD=1, GP8_POL=Non-inverted, GP8_OP_CFG=CMOS, GP8_DB=Enabled, GP8_LVL=0, GP8_FN=DACDAT3
	snd_soc_write(codec, WM8994_GPIO_9 , 0x2100); // * GPIO 9(708H):            2100  GP9_DIR=Output, GP9_PU=0, GP9_PD=1, GP9_POL=Non-inverted, GP9_OP_CFG=CMOS, GP9_DB=Enabled, GP9_LVL=0, GP9_FN=ADCDAT3
	snd_soc_write(codec, WM8994_GPIO_10, 0xA100); // * GPIO 10(709H):           2100  GP10_DIR=Output, GP10_PU=0, GP10_PD=1, GP10_POL=Non-inverted, GP10_OP_CFG=CMOS, GP10_DB=Enabled, GP10_LVL=0, GP10_FN=LRCLK3
	snd_soc_write(codec, WM8994_GPIO_11, 0x2100); // * GPIO 11(70AH):           2100  GP11_DIR=Output, GP11_PU=0, GP11_PD=1, GP11_POL=Non-inverted, GP11_OP_CFG=CMOS, GP11_DB=Enabled, GP11_LVL=0, GP11_FN=BCLK3
	snd_soc_write(codec, WM8994_AIF2_CONTROL_1, 0x411B); // * AIF2 Control (1)(310H):  411B  AIF2_LRCLK_INV=1, AIF2_WL=16 bits, AIF2_FMT=DSP mode
	snd_soc_write(codec, WM8958_AIF3_CONTROL_1, 0x0018); // * AIF3 Control (1)(320H):  0018  AIF3_LRCLK_INV=0, AIF3_WL=16 bits, AIF3_FMT=DSP mode
	snd_soc_write(codec, WM8994_AIF1_MASTER_SLAVE, 0x4000); // * AIF1 Master/Slave(302H): 4000  AIF1_TRI=0, AIF1_MSTR=Master mode, AIF1_CLK_FRC=0, AIF1_LRCLK_FRC=0
	snd_soc_write(codec, WM8994_AIF2_MASTER_SLAVE, 0x4000); // * AIF2 Master/Slave(312H): 4000  AIF2_TRI=0, AIF2_MSTR=Master mode, AIF2_CLK_FRC=0, AIF2_LRCLK_FRC=0
	snd_soc_write(codec, WM8994_FLL1_CONTROL_1, 0x0000); // * FLL1 Control (1)(220H):  0000  FLL1_OSC_ENA=0, FLL1_ENA=0
	snd_soc_write(codec, WM8994_FLL1_CONTROL_2, 0x0700); // * FLL1 Control (2)(221H):  0700  FLL1_OUTDIV=8, FLL1_FRATIO=1
	snd_soc_write(codec, WM8994_FLL1_CONTROL_3, 0x86C2); // * FLL1 Control (3)(222H):  86C2  FLL1_K=0.5264
	snd_soc_write(codec, WM8994_FLL1_CONTROL_4, 0x00E0); // * FLL1 Control (4)(223H):  00E0  FLL1_N=7
	if (machine_is_m030())
		snd_soc_write(codec, WM8994_FLL1_CONTROL_5, 0x0C80); // * FLL1 Control (5)(224H):  0C80  FLL1_BYP=FLL1, FLL1_FRC_NCO_VAL=01_1001, FLL1_FRC_NCO=0, FLL1_REFCLK_DIV=MCLK / 1, FLL1_REFCLK_SRC=MCLK1
	else
		snd_soc_write(codec, WM8994_FLL1_CONTROL_5, 0x0C88); // * FLL1 Control (5)(224H):  0C88  FLL1_BYP=FLL1, FLL1_FRC_NCO_VAL=01_1001, FLL1_FRC_NCO=0, FLL1_REFCLK_DIV=MCLK / 2, FLL1_REFCLK_SRC=MCLK1
	snd_soc_write(codec, WM8994_FLL1_CONTROL_1, 0x0001); // * FLL1 Control (1)(220H):  0001  FLL1_OSC_ENA=0, FLL1_ENA=1
	snd_soc_write(codec, WM8994_AIF1_CLOCKING_1, 0x0011);// * AIF1 Clocking (1)(200H): 0011  AIF1CLK_SRC=FLL1/BCLK1, AIF1CLK_INV=0, AIF1CLK_DIV=AIF1CLK, AIF1CLK_ENA=1
	snd_soc_write(codec, WM8994_FLL2_CONTROL_1, 0x0000); // * FLL2 Control (1)(240H):  0000  FLL2_OSC_ENA=0, FLL2_ENA=0
	snd_soc_write(codec, WM8994_FLL2_CONTROL_2, 0x2F00); // * FLL2 Control (2)(241H):  2F00  FLL2_OUTDIV=48, FLL2_FRATIO=1
	snd_soc_write(codec, WM8994_FLL2_CONTROL_3, 0x3127); // * FLL2 Control (3)(242H):  3127  FLL2_K=0.192
	snd_soc_write(codec, WM8994_FLL2_CONTROL_4, 0x0100); // * FLL2 Control (4)(243H):  0100  FLL2_N=8
	if (machine_is_m030())
		snd_soc_write(codec, WM8994_FLL2_CONTROL_5, 0x0C80); // * FLL2 Control (5)(244H):  0C88  FLL2_BYP=FLL1, FLL2_FRC_NCO_VAL=01_1001, FLL2_FRC_NCO=0, FLL2_REFCLK_DIV=MCLK, FLL2_REFCLK_SRC=MCLK1
	else
		snd_soc_write(codec, WM8994_FLL2_CONTROL_5, 0x0C88); // * FLL2 Control (5)(244H):  0C88  FLL2_BYP=FLL1, FLL2_FRC_NCO_VAL=01_1001, FLL2_FRC_NCO=0, FLL2_REFCLK_DIV=MCLK / 2, FLL2_REFCLK_SRC=MCLK1
	snd_soc_write(codec, WM8994_FLL2_CONTROL_1, 0x0001); // * FLL2 Control (1)(240H):  0001  FLL2_OSC_ENA=0, FLL2_ENA=1
	snd_soc_write(codec, WM8994_AIF2_CLOCKING_1, 0x0019);// * AIF2 Clocking (1)(204H): 0019  AIF2CLK_SRC=FLL2/BCLK2, AIF2CLK_INV=0, AIF2CLK_DIV=AIF2CLK, AIF2CLK_ENA=1
}

int set_playback_path(struct snd_soc_codec *codec,u8 playback_path)
{
	int ret = 0;
	//struct wm8994_priv *wm8958 = snd_soc_codec_get_drvdata(codec);
	//u8 capture_path = wm8958->capture_path;
	
	dprintk ("%s() :playbackpath = %d\n", __func__,playback_path);
	
	SetSpkMute(codec,1);
	SetRecMute(codec,1);
	SetHpMute(codec,1);

	switch(playback_path)
	{
	case PLAYBACK_REC_NORMAL:
		CloseAIF2(codec);
		SetEQBase(codec,EQ_SPK_NORMAL);
		SetVolume_Normal(codec);
		SetRecMute(codec,0);
		break;
	case PLAYBACK_SPK_HP_NORMAL:
		CloseAIF2(codec);
		SetEQBase(codec,EQ_SPK_NORMAL);
		SetVolume_Normal(codec);
		SetSpkMute(codec,0);
		SetHpMute(codec,0);
		break;
	case PLAYBACK_SPK_NORMAL:
		CloseAIF2(codec);
		SetEQBase(codec,EQ_SPK_NORMAL);
		SetVolume_Normal(codec);
		SetSpkMute(codec,0);
		break;
	case PLAYBACK_HP_NORMAL:
		CloseAIF2(codec);
		SetEQBase(codec,EQ_HP_NORMAL);
		SetVolume_Normal(codec);
		SetHpMute(codec,0);
		break;
		
		
	case PLAYBACK_REC_RING:
		CloseAIF2(codec);
		SetEQBase(codec,EQ_SPK_NORMAL);
		SetVolume_Ring(codec);
		SetRecMute(codec,0);
		break;
	case PLAYBACK_SPK_HP_RING:
		CloseAIF2(codec);
		SetEQBase(codec,EQ_SPK_NORMAL);
		SetVolume_Ring(codec);
		SetSpkMute(codec,0);
		SetHpMute(codec,0);
		break;
	case PLAYBACK_SPK_RING:
		CloseAIF2(codec);
		SetEQBase(codec,EQ_SPK_NORMAL);
		SetVolume_Ring(codec);
		SetSpkMute(codec,0);
		break;
	case PLAYBACK_HP_RING:
		CloseAIF2(codec);
		SetEQBase(codec,EQ_HP_NORMAL);
		SetVolume_Ring(codec);
		SetHpMute(codec,0);
		break;
	case PLAYBACK_BT_RING:
		OpenBTRing(codec);
		SetVolume_Ring(codec);
		SetSpkMute(codec,0);
		SetHpMute(codec,0);
		break;
		
	case PLAYBACK_SPK_HP_INCALL:
		SetAIF3_2_AIF2(codec,false);
		OpenAIF2(codec);
		audio_switch(SWTICH_TO_BB);	
#ifdef CONFIG_AUDIENCE_ES305B
		es305b_setmode(ES305B_INCALL_SPEAKER_NB);
#endif
#ifdef CONFIG_AUDIENCE_A1028
		a1028_setmode(NULL,A1028_INCALL_SPEAKER);
		#endif	
		SetVolume_Incall(codec);
		SetVolume_Mixerin_spk(codec);
		SetSpkMute(codec,0);
		SetHpMute(codec,0);

		break;
		
	case PLAYBACK_SPK_INCALL:
		SetAIF3_2_AIF2(codec,false);
		OpenAIF2(codec);
		audio_switch(SWTICH_TO_BB);		
#ifdef CONFIG_AUDIENCE_ES305B
		es305b_setmode(ES305B_INCALL_SPEAKER_NB);
#endif
#ifdef CONFIG_AUDIENCE_A1028
		a1028_setmode(NULL,A1028_INCALL_SPEAKER);
		#endif
		SetVolume_Incall(codec);
		SetVolume_Mixerin_spk(codec);
		SetSpkMute(codec,0);

		break;
		
	case PLAYBACK_HP_INCALL:
		SetAIF3_2_AIF2(codec,false);
		OpenAIF2(codec);
		audio_switch(SWTICH_TO_BB);
#ifdef CONFIG_AUDIENCE_ES305B
		es305b_setmode(ES305B_INCALL_HEADPHONE_NB);
#endif
#ifdef CONFIG_AUDIENCE_A1028
		a1028_setmode(NULL,A1028_INCALL_HEADSET);
		#endif
		SetVolume_Incall(codec);
		SetVolume_Mixerin_spk(codec);
		SetHpMute(codec,0);
		
		break;
			
	case PLAYBACK_HS_INCALL:
		SetAIF3_2_AIF2(codec,false);
		OpenAIF2(codec);
		audio_switch(SWTICH_TO_BB);
#ifdef CONFIG_AUDIENCE_ES305B
		es305b_setmode(ES305B_INCALL_HEADPHONE_NB);
#endif
#ifdef CONFIG_AUDIENCE_A1028
		a1028_setmode(NULL,A1028_INCALL_HEADSET);
		#endif
		SetVolume_Incall(codec);
		SetVolume_Mixerin_spk(codec);
		SetHpMute(codec,0);
		
		break;

	case PLAYBACK_SPK_HS_INCALL:	
		SetAIF3_2_AIF2(codec,false);
		OpenAIF2(codec);
		audio_switch(SWTICH_TO_BB);	
#ifdef CONFIG_AUDIENCE_ES305B
		es305b_setmode(ES305B_INCALL_HEADPHONE_NB);
#endif
#ifdef CONFIG_AUDIENCE_A1028
		a1028_setmode(NULL,A1028_INCALL_HEADSET);
		#endif	
		SetVolume_Incall(codec);
		SetVolume_Mixerin_spk(codec);
		SetSpkMute(codec,0);
		SetHpMute(codec,0);
		
		break;

	case PLAYBACK_REC_INCALL:	
		SetAIF3_2_AIF2(codec,false);
		OpenAIF2(codec);
		audio_switch(SWTICH_TO_BB);	
#ifdef CONFIG_AUDIENCE_ES305B
		es305b_setmode(ES305B_INCALL_RECEIVER_NB);
#endif
#ifdef CONFIG_AUDIENCE_A1028
		a1028_setmode(NULL,A1028_INCALL_RECEIVER);
		#endif
		SetVolume_Incall(codec);
		SetVolume_Mixerin_rec(codec);
		SetRecMute(codec,0);
		
		break;	

	case PLAYBACK_BT_INCALL:	
		OpenAIF2(codec);
		SetAIF3_2_AIF2(codec,true);
		audio_switch(SWTICH_TO_BB);	
#ifdef CONFIG_AUDIENCE_ES305B
		es305b_setmode(ES305B_INCALL_BT);
#endif
#ifdef CONFIG_AUDIENCE_A1028
		a1028_setmode(NULL,A1028_INCALL_BT);
		#endif
		
		break;	
				
	case PLAYBACK_SPK_HP_VOIP:
		audio_switch(SWTICH_TO_AP);		
#ifdef CONFIG_AUDIENCE_ES305B
		es305b_setmode(ES305B_VOIP_SPEAKER_NB);
#endif
#ifdef CONFIG_AUDIENCE_A1028
		a1028_setmode(NULL,A1028_INCALL_SPEAKER);
		#endif
		SetAIF3_2_AIF2(codec,false);
		SetVolume_VoIP(codec);
		SetVolume_Mixerin_spk(codec);
		SetSpkMute(codec,0);
		SetHpMute(codec,0);

		break;
		
	case PLAYBACK_SPK_VOIP:
		audio_switch(SWTICH_TO_AP);		
#ifdef CONFIG_AUDIENCE_ES305B
		es305b_setmode(ES305B_VOIP_SPEAKER_NB);
#endif
#ifdef CONFIG_AUDIENCE_A1028
		a1028_setmode(NULL,A1028_INCALL_SPEAKER);
		#endif
		SetAIF3_2_AIF2(codec,false);
		SetVolume_VoIP(codec);
		SetVolume_Mixerin_spk(codec);
		SetSpkMute(codec,0);

		break;
	case PLAYBACK_HP_VOIP:
		audio_switch(SWTICH_TO_AP);
#ifdef CONFIG_AUDIENCE_ES305B
		es305b_setmode(ES305B_VOIP_HEADPHONE_NB);
#endif
#ifdef CONFIG_AUDIENCE_A1028
		a1028_setmode(NULL,A1028_INCALL_HEADSET);
		#endif
		SetAIF3_2_AIF2(codec,false);
		SetVolume_VoIP(codec);
		SetVolume_Mixerin_spk(codec);
		SetHpMute(codec,0);
		
		break;
			
	case PLAYBACK_HS_VOIP:
		audio_switch(SWTICH_TO_AP);
#ifdef CONFIG_AUDIENCE_ES305B
		es305b_setmode(ES305B_VOIP_HEADPHONE_NB);
#endif
#ifdef CONFIG_AUDIENCE_A1028
		a1028_setmode(NULL,A1028_INCALL_HEADSET);
		#endif
		SetAIF3_2_AIF2(codec,false);
		SetVolume_VoIP(codec);
		SetVolume_Mixerin_spk(codec);
		SetHpMute(codec,0);
		
		break;

	case PLAYBACK_SPK_HS_VOIP:	
		audio_switch(SWTICH_TO_AP);		
#ifdef CONFIG_AUDIENCE_ES305B
		es305b_setmode(ES305B_VOIP_HEADPHONE_NB);
#endif
#ifdef CONFIG_AUDIENCE_A1028
		a1028_setmode(NULL,A1028_INCALL_HEADSET);
		#endif
		SetAIF3_2_AIF2(codec,false);
		SetVolume_VoIP(codec);
		SetVolume_Mixerin_spk(codec);
		SetSpkMute(codec,0);
		SetHpMute(codec,0);
		
		break;


	case PLAYBACK_REC_VOIP:	
		audio_switch(SWTICH_TO_AP);	
#ifdef CONFIG_AUDIENCE_ES305B
		es305b_setmode(ES305B_VOIP_RECEIVER_NB);
#endif
#ifdef CONFIG_AUDIENCE_A1028
		a1028_setmode(NULL,A1028_INCALL_RECEIVER);
		#endif
		SetAIF3_2_AIF2(codec,false);
		SetVolume_VoIP(codec);
		SetVolume_Mixerin_rec(codec);
		SetRecMute(codec,0);
		
		break;	

	case PLAYBACK_BT_VOIP:	
		audio_switch(SWTICH_TO_AP);	
		SetAIF3_2_AIF2(codec,true);
#ifdef CONFIG_AUDIENCE_ES305B
		es305b_setmode(ES305B_VOIP_BT);
#endif
#ifdef CONFIG_AUDIENCE_A1028
		a1028_setmode(NULL,A1028_INCALL_BT);
		#endif
		
		break;	
	case PLAYBACK_NONE:
		break;
		
	default:			
		printk ("%s() :invalid path = %d\n", __func__,playback_path);
		ret = -1;
		break;			
	}
	return ret;	
}

int set_capture_path(struct snd_soc_codec *codec,u8 capture_path)
{
	int ret = 0;
	
	dprintk ("%s() :capture_path %d\n", __func__,capture_path);
	SetVolume_ADC1(codec);
	switch(capture_path)
	{
		case CAPTURE_MAIN_MIC_NORMAL:
			SetVolume_Mixerin_spk(codec);
			break;			
			
		case CAPTURE_SECOND_MIC_NORMAL:
			SetVolume_Mixerin_rec(codec);
			break;		
			
		case CAPTURE_HAND_MIC_NORMAL:
			SetVolume_Mixerin_spk(codec);
			break;		
			
		case CAPTURE_MAIN_MIC_INCALL:
			audio_switch(SWTICH_TO_BB);
			break;
			
		case CAPTURE_SECOND_MIC_INCALL:
			audio_switch(SWTICH_TO_BB);
			break;

		case CAPTURE_HAND_MIC_INCALL:	
			audio_switch(SWTICH_TO_BB);
			break;
			
		case CAPTURE_MAIN_MIC_VOIP:
			audio_switch(SWTICH_TO_AP);
			break;
			
		case CAPTURE_SECOND_MIC_VOIP:
			audio_switch(SWTICH_TO_AP);
			break;

		case CAPTURE_HAND_MIC_VOIP:	
			audio_switch(SWTICH_TO_AP);
			break;
			
		case CAPTURE_NONE:
			printk ("%s() :invalid path = %d\n", __func__,capture_path);
			ret = -1;
			break;
			
		default:			
			break;			
	}
	return ret;	
}

#endif	//CONFIG_SND_SOC_MX_WM8958
