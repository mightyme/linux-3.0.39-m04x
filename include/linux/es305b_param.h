#ifndef __LINUX_ES305B_PARAM_H_
#define __LINUX_ES305B_PARAM_H_

static const u8 incall_ct_buf[] =
{
// !Preset id: 0 ; Handson_NB (CT)
	0x80, 0x52, 0x00, 0x00, // ; 0x8052:Disable digital pass through
	0x80, 0x26, 0x00, 0x01, // ; 0x8026:SelectRouting, 0x0001:
	0x80, 0x1B, 0x00, 0x0C, // ; 0x801B:SetDigitalInputGain, 0x00:PCM-A left, 0x0C:(12 dB)
	0x80, 0x1B, 0x01, 0x09, // ; 0x801B:SetDigitalInputGain, 0x01:PCM-A right, 0x09:(9 dB)
	0x80, 0x15, 0x00, 0xFC, // ; 0x8015:SetDigitalOutputGain, 0x00:PCM-A left, 0xFC:(-4 dB)
	0x80, 0x1B, 0x04, 0x00, // ; 0x801B:SetDigitalInputGain, 0x04:PCM-C left, 0x00:(0 dB)
	0x80, 0x15, 0x04, 0xFC, // ; 0x8015:SetDigitalOutputGain, 0x04:PCM-C left, 0xFC:(-4 dB)
	0x80, 0x1C, 0x00, 0x01, // ; 0x801C:VoiceProcessingOn, 0x0001:On

	0x80, 0x17, 0x00, 0x1C, 0x80, 0x18, 0x00, 0x1F, // ; 0x8017:SetAlgorithmParmID, 0x001C:0x8018:SetAlgorithmParm, 0x001F:Reset all
	0x80, 0x17, 0x00, 0x4B, 0x80, 0x18, 0x00, 0x06, // ; 0x8017:SetAlgorithmParmID, 0x004B:Tx Noise Suppression Level, 0x8018:SetAlgorithmParm, 0x0006:Level 6
	0x80, 0x17, 0x00, 0x02, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0002:Microphone Configuration, 0x8018:SetAlgorithmParm, 0x0000:2-mic Close Talk (CT)
	0x80, 0x17, 0x00, 0x15, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0015:Side Tone Mode, 0x8018:SetAlgorithmParm, 0x0000:Off
	0x80, 0x17, 0x00, 0x16, 0x80, 0x18, 0xFF, 0xEE, // ; 0x8017:SetAlgorithmParmID, 0x0016:   Side Tone Gain (dB), 0x8018:SetAlgorithmParm, 0xFFEE:(-18 dB)
	0x80, 0x17, 0x00, 0x03, 0x80, 0x18, 0x00, 0x01, // ; 0x8017:SetAlgorithmParmID, 0x0003:AEC Mode, 0x8018:SetAlgorithmParm, 0x0001:AEC On (auto select mode)
	0x80, 0x17, 0x00, 0x12, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0012:   Downlink Speaker Volume, 0x8018:SetAlgorithmParm, 0x0000:(0 dB)
	0x80, 0x17, 0x00, 0x23, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0023:   Use AEC Comfort Noise Fill, 0x8018:SetAlgorithmParm, 0x0000:No
	0x80, 0x17, 0x00, 0x34, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0034:   Echo Suppression Enhancement, 0x8018:SetAlgorithmParm, 0x0000:(0 dB)
	0x80, 0x17, 0x00, 0x2E, 0x80, 0x18, 0xFF, 0xC4, // ; 0x8017:SetAlgorithmParmID, 0x002E:   AEC Comfort Noise Gain, 0x8018:SetAlgorithmParm, 0xFFC4:(-60 dB)
	0x80, 0x17, 0x01, 0x03, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0103:   DT ESE Back Off, 0x8018:SetAlgorithmParm, 0:(0 dB)
	0x80, 0x17, 0x00, 0x04, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0004:Use AGC, 0x8018:SetAlgorithmParm, 0x0000:No
	0x80, 0x17, 0x00, 0x28, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0028:Use Rx AGC, 0x8018:SetAlgorithmParm, 0x0000:No
	0x80, 0x17, 0x00, 0x29, 0x80, 0x18, 0xFF, 0xE6, // ; 0x8017:SetAlgorithmParmID, 0x0029:   Rx AGC Target Level (dB), 0x8018:SetAlgorithmParm, 0xFFE6:(-26 dB)
	0x80, 0x17, 0x00, 0x2A, 0x80, 0x18, 0xFF, 0xA6, // ; 0x8017:SetAlgorithmParmID, 0x002A:   Rx AGC Noise Floor (dB), 0x8018:SetAlgorithmParm, 0xFFA6:(-90 dB)
	0x80, 0x17, 0x00, 0x2B, 0x80, 0x18, 0x00, 0x04, // ; 0x8017:SetAlgorithmParmID, 0x002B:   Rx AGC SNR Improve (dB), 0x8018:SetAlgorithmParm, 0x0004:(4 dB)
	0x80, 0x17, 0x00, 0x2C, 0x80, 0x18, 0x00, 0x04, // ; 0x8017:SetAlgorithmParmID, 0x002C:   Rx AGC Up Rate (dBS), 0x8018:SetAlgorithmParm, 0x0004:(4 dBS)
	0x80, 0x17, 0x00, 0x2D, 0x80, 0x18, 0x00, 0x04, // ; 0x8017:SetAlgorithmParmID, 0x002D:   Rx AGC Down Rate (dBS), 0x8018:SetAlgorithmParm, 0x0004:(4 dBS)
	0x80, 0x17, 0x01, 0x02, 0x80, 0x18, 0x00, 0x0F, // ; 0x8017:SetAlgorithmParmID, 0x0102:   Rx AGC Max Gain (dB), 0x8018:SetAlgorithmParm, 0x000F:(15 dB)
	0x80, 0x17, 0x00, 0x3F, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x003F:   Rx AGC Guard Band (dB), 0x8018:SetAlgorithmParm, 0x0000:(0 dB)
	0x80, 0x17, 0x00, 0x09, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0009:Speaker Enhancement Mode, 0x8018:SetAlgorithmParm, 0x0000:SE Off (HPF only)
	0x80, 0x17, 0x00, 0x0E, 0x80, 0x18, 0x00, 0x01, // ; 0x8017:SetAlgorithmParmID, 0x000E:Far End Noise Suppression, 0x8018:SetAlgorithmParm, 0x0001:On (auto select mode)
	0x80, 0x17, 0x00, 0x4C, 0x80, 0x18, 0x00, 0x06, // ; 0x8017:SetAlgorithmParmID, 0x004C:   Rx Noise Suppression Level, 0x8018:SetAlgorithmParm, 0x0006:Level 6
	0x80, 0x17, 0x00, 0x42, 0x80, 0x18, 0xFF, 0xFD, // ; SetAlgorithmParmID, 0x0042: Tx-in  Limiter Max Level (dB) -- SetAlgorithmParm, 0xFFFD: (-3 dB)
	0x80, 0x17, 0x00, 0x40, 0x80, 0x18, 0xFF, 0xFD, // ; SetAlgorithmParmID, 0x0040: Tx-out Limiter Max Level (dB) -- SetAlgorithmParm, 0xFFFD: (-3 dB)
	0x80, 0x17, 0x00, 0x0D, 0x80, 0x18, 0xFF, 0xFD, // ; 0x8017:SetAlgorithmParmID, 0x000D:Rx-out Limiter Max Level,   0x8018:SetAlgorithmParm,0xFFFD:( -3 dB)
	0x80, 0x17, 0x00, 0x20, 0x80, 0x18, 0x00, 0x01, // ; 0x8017:SetAlgorithmParmID, 0x0020:Tx PostEq Mode, 0x8018:SetAlgorithmParm, 0x0001:On
	0x80, 0x17, 0x00, 0x1F, 0x80, 0x18, 0x00, 0x01, // ; 0x8017:SetAlgorithmParmID, 0x001F:Rx PostEq Mode, 0x8018:SetAlgorithmParm, 0x0001:On
	0x80, 0x17, 0x00, 0x30, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0030:Tx MBC Mode, 0x8018:SetAlgorithmParm, 0x0000:Off
	0x80, 0x17, 0x00, 0x31, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0031:Rx MBC Mode, 0x8018:SetAlgorithmParm, 0x0000:Off
	0x80, 0x17, 0x00, 0x1A, 0x80, 0x18, 0x00, 0x01, // ; 0x8017:SetAlgorithmParmID, 0x001A:Use Tx ComfortNoise, 0x8018:SetAlgorithmParm, 0x0001:Yes
	0x80, 0x17, 0x00, 0x1B, 0x80, 0x18, 0xFF, 0xB5, // ; 0x8017:SetAlgorithmParmID, 0x001B:   Tx ComfortNoise gain (dB), 0x8018:SetAlgorithmParm, 0xFFB5:(-75 dB)
	0x80, 0x17, 0x00, 0x1E, 0x80, 0x18, 0x00, 0x01, // ; 0x8017:SetAlgorithmParmID, 0x001E:PST, 0x8018:SetAlgorithmParm, 0x0001:1
	0x80, 0x17, 0x00, 0x63, 0x80, 0x18, 0xFF, 0xA0, // ; SetAlgorithmParmID, 0x0063:    AEC Rx Energy Threshold (dB) -- SetAlgorithmParm, 0xFFA0: (-96 dB)
	0x80, 0x17, 0x00, 0x64, 0x80, 0x18, 0x00, 0x64, // ; SetAlgorithmParmID, 0x0064:    AEC Rx Energy Threshold Time (ms) -- SetAlgorithmParm, 0x0064: (100 ms)
};

static const u8 incall_dv_buf[] =
{
	0x80, 0x31, 0x00, 0x03,

#if 0
	// !Preset id: 3 ; Handsfree_NB (DV)
	0x80, 0x52, 0x00, 0x00, // ; 0x8052:Disable digital pass through
	0x80, 0x26, 0x00, 0x01, // ; 0x8026:SelectRouting, 0x0001:
	0x80, 0x1C, 0x00, 0x01, // ; 0x801C:VoiceProcessingOn, 0x0000:On
	0x80, 0x1B, 0x00, 0x06, // ; 0x801B:SetDigitalInputGain, 0x00:PCM-A left, 0x06:(6 dB)
	0x80, 0x1B, 0x01, 0x06, // ; 0x801B:SetDigitalInputGain, 0x01:PCM-A right, 0x06:(6 dB)
	0x80, 0x15, 0x00, 0x00, // ; 0x8015:SetDigitalOutputGain, 0x00:PCM-A left, 0x00:(0 dB)
	0x80, 0x1B, 0x04, 0x00, // ; 0x801B:SetDigitalInputGain, 0x04:PCM-C left, 0x00:(0 dB)
	0x80, 0x15, 0x04, 0xFB, // ; 0x8015:SetDigitalOutputGain, 0x04:PCM-C left, 0xFB:(-5 dB)

	0x80, 0x17, 0x00, 0x1C, 0x80, 0x18, 0x00, 0x1F, // ; 0x8017:SetAlgorithmParmID, 0x001C:0x8018:SetAlgorithmParm, 0x001F:Reset all
	0x80, 0x17, 0x00, 0x4B, 0x80, 0x18, 0x00, 0x04, // ; 0x8017:SetAlgorithmParmID, 0x004B:Tx Noise Suppression Level, 0x8018:SetAlgorithmParm, 0x0004:Level 4
	0x80, 0x17, 0x00, 0x02, 0x80, 0x18, 0x00, 0x02, // ; 0x8017:SetAlgorithmParmID, 0x0002:Microphone Configuration, 0x8018:SetAlgorithmParm, 0x0002:1-mic Desktop/Vehicle (DV)
	0x80, 0x17, 0x00, 0x15, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0015:Side Tone Mode, 0x8018:SetAlgorithmParm, 0x0000:Off
	0x80, 0x17, 0x00, 0x03, 0x80, 0x18, 0x00, 0x01, // ; 0x8017:SetAlgorithmParmID, 0x0003:AEC Mode, 0x8018:SetAlgorithmParm, 0x0001:AEC On (auto select mode)
	0x80, 0x17, 0x00, 0x12, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0012:   Downlink Speaker Volume, 0x8018:SetAlgorithmParm, 0x0000:(0 dB)
	0x80, 0x17, 0x00, 0x23, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0023:   Use AEC Comfort Noise Fill, 0x8018:SetAlgorithmParm, 0x0000:No
	0x80, 0x17, 0x00, 0x34, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0034:   Echo Suppression Enhancement, 0x8018:SetAlgorithmParm, 0x0000:(0 dB)
	0x80, 0x17, 0x00, 0x36, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0036:   Half-duplex, 0x8018:SetAlgorithmParm, 0x0000:No
	0x80, 0x17, 0x00, 0x37, 0x80, 0x18, 0xFF, 0xE2, // ; 0x8017:SetAlgorithmParmID, 0x0037:   HD Tx interrupt level, 0x8018:SetAlgorithmParm, 0xFFE2:(-30 dB)
	0x80, 0x17, 0x00, 0x2E, 0x80, 0x18, 0xFF, 0xF0, // ; 0x8017:SetAlgorithmParmID, 0x002E:   AEC Comfort Noise Gain, 0x8018:SetAlgorithmParm, 0xFFF0:(-16 dB)
	0x80, 0x17, 0x01, 0x03, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0103:   DT ESE Back Off, 0x8018:SetAlgorithmParm, 0x0000:(0 dB)
	0x80, 0x17, 0x00, 0x04, 0x80, 0x18, 0x00, 0x01, // ; 0x8017:SetAlgorithmParmID, 0x0004:Use Tx AGC, 0x8018:SetAlgorithmParm, 0x0001:Yes
	0x80, 0x17, 0x00, 0x05, 0x80, 0x18, 0xFF, 0xE6, // ; 0x8017:SetAlgorithmParmID, 0x0005:   AGC Target Level (dB), 0x8018:SetAlgorithmParm, 0xFFE6:(-26 dB)
	0x80, 0x17, 0x00, 0x06, 0x80, 0x18, 0xFF, 0xC4, // ; 0x8017:SetAlgorithmParmID, 0x0006:   AGC Noise Floor (dB), 0x8018:SetAlgorithmParm, 0xFFBF:(-60 dB)
	0x80, 0x17, 0x00, 0x07, 0x80, 0x18, 0x00, 0x04, // ; 0x8017:SetAlgorithmParmID, 0x0007:   AGC SNR Improve (dB), 0x8018:SetAlgorithmParm, 0x0004:(4 dB)
	0x80, 0x17, 0x00, 0x26, 0x80, 0x18, 0x00, 0x04, // ; 0x8017:SetAlgorithmParmID, 0x0026:   AGC Up Rate (dBS), 0x8018:SetAlgorithmParm, 0x0004:(4 dBS)
	0x80, 0x17, 0x00, 0x27, 0x80, 0x18, 0x00, 0x04, // ; 0x8017:SetAlgorithmParmID, 0x0027:   AGC Down Rate (dBS), 0x8018:SetAlgorithmParm, 0x0004:(4 dBS)
	0x80, 0x17, 0x01, 0x00, 0x80, 0x18, 0x00, 0x14, // ; 0x8017:SetAlgorithmParmID, 0x0100:   AGC Max Gain (dB), 0x8018:SetAlgorithmParm, 0x0014:(20 dB)
	0x80, 0x17, 0x00, 0x28, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0028:Use Rx AGC, 0x8018:SetAlgorithmParm, 0x0000:No
	0x80, 0x17, 0x00, 0x09, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0009:Speaker Enhancement Mode, 0x8018:SetAlgorithmParm, 0x0000:SE Off (HPF only)
	0x80, 0x17, 0x00, 0x0E, 0x80, 0x18, 0x00, 0x01, // ; 0x8017:SetAlgorithmParmID, 0x000E:Far End Noise Suppression, 0x8018:SetAlgorithmParm, 0x0001:On (auto select mode)
	0x80, 0x17, 0x00, 0x4C, 0x80, 0x18, 0x00, 0x08, // ; 0x8017:SetAlgorithmParmID, 0x004C:   Rx Noise Suppression Level, 0x8018:SetAlgorithmParm, 0x0008:Level 8
	0x80, 0x17, 0x00, 0x42, 0x80, 0x18, 0xFF, 0xFD, // ; SetAlgorithmParmID, 0x0042: Tx-in  Limiter Max Level (dB) -- SetAlgorithmParm, 0xFFFD: (-3 dB)
	0x80, 0x17, 0x00, 0x40, 0x80, 0x18, 0xFF, 0xFD, // ; SetAlgorithmParmID, 0x0040: Tx-out Limiter Max Level (dB) -- SetAlgorithmParm, 0xFFFD: (-3 dB)
	0x80, 0x17, 0x00, 0x0D, 0x80, 0x18, 0xFF, 0xF7, // ; 0x8017:SetAlgorithmParmID, 0x000D:Rx-out Limiter Max Level,   0x8018:SetAlgorithmParm,0xFFF7:( -9 dB)
	0x80, 0x17, 0x00, 0x20, 0x80, 0x18, 0x00, 0x01, // ; 0x8017:SetAlgorithmParmID, 0x0020:Tx PostEq Mode, 0x8018:SetAlgorithmParm, 0x0001:On
	0x80, 0x17, 0x00, 0x1F, 0x80, 0x18, 0x00, 0x01, // ; 0x8017:SetAlgorithmParmID, 0x001F:Rx PostEq Mode, 0x8018:SetAlgorithmParm, 0x0001:On
	0x80, 0x17, 0x00, 0x30, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0030:Tx MBC Mode, 0x8018:SetAlgorithmParm, 0x0000:Off
	0x80, 0x17, 0x00, 0x31, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0031:Rx MBC Mode, 0x8018:SetAlgorithmParm, 0x0000:Off
	0x80, 0x17, 0x00, 0x1A, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x001A:Use Tx ComfortNoise, 0x8018:SetAlgorithmParm, 0x0000:No
	0x80, 0x17, 0x00, 0x63, 0x80, 0x18, 0xFF, 0xA0, // ; SetAlgorithmParmID, 0x0063:    AEC Rx Energy Threshold (dB) -- SetAlgorithmParm, 0xFFA0: (-96 dB)
	0x80, 0x17, 0x00, 0x64, 0x80, 0x18, 0x00, 0x64, // ; SetAlgorithmParmID, 0x0064:    AEC Rx Energy Threshold Time (ms) -- SetAlgorithmParm, 0x0064: (100 ms)
#endif
};

static const u8 incall_whs_buf[] =
{
	0x80, 0x52, 0x00, 0x00, // ; 0x8052:
	0x80, 0x26, 0x00, 0x01, // ; 0x8026:SelectRouting, 0x0001:
	0x80, 0x1C, 0x00, 0x00, // ; 0x801C:VoiceProcessingOn, 0x0000:Off
};

static const u8 incall_bt_buf[] =
{
	0x80, 0x52, 0x00, 0x00, // ; 0x8052:
	0x80, 0x26, 0x00, 0x95, // ; 0x8026:SelectRouting, 0x0095:149
	0x80, 0x1C, 0x00, 0x00, // ; 0x801C:VoiceProcessingOn, 0x0000:Off
};

static const u8 incall_bt_vpoff_buf[] =
{
	0x80, 0x52, 0x00, 0x00, // ; 0x8052:
	0x80, 0x26, 0x00, 0x95, // ; 0x8026:SelectRouting, 0x0095:149
	0x80, 0x1C, 0x00, 0x00, // ; 0x801C:VoiceProcessingOn, 0x0000:Off
};

static const u8 voip_ct_buf[] =
{
	0x80, 0x52, 0x00, 0x00, // ; 0x8052:Disable digital pass through
	0x80, 0x26, 0x00, 0x93, // ; 0x8026:SelectRouting, 0x0093:
	0x80, 0x1B, 0x00, 0x0C, // ; 0x801B:SetDigitalInputGain, 0x00:PCM-A left, 0x0C:(12 dB)
	0x80, 0x1B, 0x01, 0x09, // ; 0x801B:SetDigitalInputGain, 0x01:PCM-A right, 0x09:(9 dB)
	0x80, 0x15, 0x00, 0xF8, // ; 0x8015:SetDigitalOutputGain, 0x00:PCM-A left, 0xF8:(-8 dB)
	0x80, 0x1B, 0x04, 0x00, // ; 0x801B:SetDigitalInputGain, 0x04:PCM-C left, 0x00:(0 dB)
	0x80, 0x15, 0x04, 0xFC, // ; 0x8015:SetDigitalOutputGain, 0x04:PCM-C left, 0xFC:(-4 dB)
	0x80, 0x1C, 0x00, 0x01, // ; 0x801C:VoiceProcessingOn, 0x0001:On
	0x80, 0x0C, 0x0A, 0x00, 0x80, 0x0D, 0x00, 0x0F, // ; 0x800C: SetDeviceParmID, 0x0B: PCM1, 0x00: PCM WordLength0x800D: SetDeviceParm, 0x000F: 16 Bits
	0x80, 0x0C, 0x0A, 0x02, 0x80, 0x0D, 0x00, 0x00, // ; 0x800C: SetDeviceParmID, 0x0B: PCM1, 0x02: PCM DelFromFsTx0x800D: SetDeviceParm, 0x0000: (0 clocks)
	0x80, 0x0C, 0x0A, 0x03, 0x80, 0x0D, 0x00, 0x01, // ; 0x800C: SetDeviceParmID, 0x0B: PCM1, 0x03: PCM DelFromFsRx0x800D: SetDeviceParm, 0x0001: (1 clocks)
	0x80, 0x0C, 0x0A, 0x04, 0x80, 0x0D, 0x00, 0x02, // ; 0x800C: SetDeviceParmID, 0x0B: PCM1, 0x04: PCM Latch Edge0x800D: SetDeviceParm, 0x0002:TxFalling/RxFalling
	0x80, 0x0C, 0x0A, 0x05, 0x80, 0x0D, 0x00, 0x01, // ; 0x800C: SetDeviceParmID, 0x0B: PCM1, 0x05: PCM Endianness0x800D: SetDeviceParm, 0x0001:Big Endian
	0x80, 0x0C, 0x0A, 0x06, 0x80, 0x0D, 0x00, 0x00, // ; 0x800C: SetDeviceParmID, 0x0B: PCM1, 0x06: PCM Tristate Enable0x800D: SetDeviceParm, 0x0000: Disable
	0x80, 0x0C, 0x0A, 0x07, 0x80, 0x0D, 0x00, 0x00, // ; 0x800C: SetDeviceParmID, 0x0B: PCM1, 0x07: PCM Audio Port Mode0x800D: SetDeviceParm, 0x0000: PCM
	0x80, 0x0C, 0x0B, 0x00, 0x80, 0x0D, 0x00, 0x0F, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x00: PCM WordLength0x800D: SetDeviceParm, 0x000F: 16 Bits
	0x80, 0x0C, 0x0B, 0x02, 0x80, 0x0D, 0x00, 0x01, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x02: PCM DelFromFsTx0x800D: SetDeviceParm, 0x0001: (1 clocks)
	0x80, 0x0C, 0x0B, 0x03, 0x80, 0x0D, 0x00, 0x01, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x03: PCM DelFromFsRx0x800D: SetDeviceParm, 0x0001: (1 clocks)
	0x80, 0x0C, 0x0B, 0x04, 0x80, 0x0D, 0x00, 0x02, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x04: PCM Latch Edge0x800D: SetDeviceParm, 0x0002:TxFalling/RxFalling
	0x80, 0x0C, 0x0B, 0x05, 0x80, 0x0D, 0x00, 0x01, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x05: PCM Endianness0x800D: SetDeviceParm, 0x0001:Big Endian
	0x80, 0x0C, 0x0B, 0x06, 0x80, 0x0D, 0x00, 0x00, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x06: PCM Tristate Enable0x800D: SetDeviceParm, 0x0000: Disable
	0x80, 0x0C, 0x0B, 0x07, 0x80, 0x0D, 0x00, 0x00, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x07: PCM Audio Port Mode0x800D: SetDeviceParm, 0x0000: PCM

	0x80, 0x17, 0x00, 0x1C, 0x80, 0x18, 0x00, 0x1F, // ; 0x8017:SetAlgorithmParmID, 0x001C:0x8018:SetAlgorithmParm, 0x001F:Reset all
	0x80, 0x17, 0x00, 0x4B, 0x80, 0x18, 0x00, 0x06, // ; 0x8017:SetAlgorithmParmID, 0x004B:Tx Noise Suppression Level, 0x8018:SetAlgorithmParm, 0x0006:Level 6
	0x80, 0x17, 0x00, 0x02, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0002:Microphone Configuration, 0x8018:SetAlgorithmParm, 0x0000:2-mic Close Talk (CT)
	0x80, 0x17, 0x00, 0x15, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0015:Side Tone Mode, 0x8018:SetAlgorithmParm, 0x0000:Off
	0x80, 0x17, 0x00, 0x16, 0x80, 0x18, 0xFF, 0xEE, // ; 0x8017:SetAlgorithmParmID, 0x0016:   Side Tone Gain (dB), 0x8018:SetAlgorithmParm, 0xFFEE:(-18 dB)
	0x80, 0x17, 0x00, 0x03, 0x80, 0x18, 0x00, 0x01, // ; 0x8017:SetAlgorithmParmID, 0x0003:AEC Mode, 0x8018:SetAlgorithmParm, 0x0001:AEC On (auto select mode)
	0x80, 0x17, 0x00, 0x12, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0012:   Downlink Speaker Volume, 0x8018:SetAlgorithmParm, 0x0000:(0 dB)
	0x80, 0x17, 0x00, 0x23, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0023:   Use AEC Comfort Noise Fill, 0x8018:SetAlgorithmParm, 0x0000:No
	0x80, 0x17, 0x00, 0x34, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0034:   Echo Suppression Enhancement, 0x8018:SetAlgorithmParm, 0x0000:(0 dB)
	0x80, 0x17, 0x00, 0x2E, 0x80, 0x18, 0xFF, 0xC4, // ; 0x8017:SetAlgorithmParmID, 0x002E:   AEC Comfort Noise Gain, 0x8018:SetAlgorithmParm, 0xFFC4:(-60 dB)
	0x80, 0x17, 0x01, 0x03, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0103:   DT ESE Back Off, 0x8018:SetAlgorithmParm, 0:(0 dB)
	0x80, 0x17, 0x00, 0x04, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0004:Use AGC, 0x8018:SetAlgorithmParm, 0x0000:No
	0x80, 0x17, 0x00, 0x28, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0028:Use Rx AGC, 0x8018:SetAlgorithmParm, 0x0000:No
	0x80, 0x17, 0x00, 0x29, 0x80, 0x18, 0xFF, 0xE6, // ; 0x8017:SetAlgorithmParmID, 0x0029:   Rx AGC Target Level (dB), 0x8018:SetAlgorithmParm, 0xFFE6:(-26 dB)
	0x80, 0x17, 0x00, 0x2A, 0x80, 0x18, 0xFF, 0xA6, // ; 0x8017:SetAlgorithmParmID, 0x002A:   Rx AGC Noise Floor (dB), 0x8018:SetAlgorithmParm, 0xFFA6:(-90 dB)
	0x80, 0x17, 0x00, 0x2B, 0x80, 0x18, 0x00, 0x04, // ; 0x8017:SetAlgorithmParmID, 0x002B:   Rx AGC SNR Improve (dB), 0x8018:SetAlgorithmParm, 0x0004:(4 dB)
	0x80, 0x17, 0x00, 0x2C, 0x80, 0x18, 0x00, 0x04, // ; 0x8017:SetAlgorithmParmID, 0x002C:   Rx AGC Up Rate (dBS), 0x8018:SetAlgorithmParm, 0x0004:(4 dBS)
	0x80, 0x17, 0x00, 0x2D, 0x80, 0x18, 0x00, 0x04, // ; 0x8017:SetAlgorithmParmID, 0x002D:   Rx AGC Down Rate (dBS), 0x8018:SetAlgorithmParm, 0x0004:(4 dBS)
	0x80, 0x17, 0x01, 0x02, 0x80, 0x18, 0x00, 0x0F, // ; 0x8017:SetAlgorithmParmID, 0x0102:   Rx AGC Max Gain (dB), 0x8018:SetAlgorithmParm, 0x000F:(15 dB)
	0x80, 0x17, 0x00, 0x3F, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x003F:   Rx AGC Guard Band (dB), 0x8018:SetAlgorithmParm, 0x0000:(0 dB)
	0x80, 0x17, 0x00, 0x09, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0009:Speaker Enhancement Mode, 0x8018:SetAlgorithmParm, 0x0000:SE Off (HPF only)
	0x80, 0x17, 0x00, 0x0E, 0x80, 0x18, 0x00, 0x01, // ; 0x8017:SetAlgorithmParmID, 0x000E:Far End Noise Suppression, 0x8018:SetAlgorithmParm, 0x0001:On (auto select mode)
	0x80, 0x17, 0x00, 0x4C, 0x80, 0x18, 0x00, 0x06, // ; 0x8017:SetAlgorithmParmID, 0x004C:   Rx Noise Suppression Level, 0x8018:SetAlgorithmParm, 0x0006:Level 6
	0x80, 0x17, 0x00, 0x42, 0x80, 0x18, 0xFF, 0xFD, // ; SetAlgorithmParmID, 0x0042: Tx-in  Limiter Max Level (dB) -- SetAlgorithmParm, 0xFFFD: (-3 dB)
	0x80, 0x17, 0x00, 0x40, 0x80, 0x18, 0xFF, 0xFD, // ; SetAlgorithmParmID, 0x0040: Tx-out Limiter Max Level (dB) -- SetAlgorithmParm, 0xFFFD: (-3 dB)
	0x80, 0x17, 0x00, 0x0D, 0x80, 0x18, 0xFF, 0xFD, // ; 0x8017:SetAlgorithmParmID, 0x000D:Rx-out Limiter Max Level,   0x8018:SetAlgorithmParm,0xFFFD:( -3 dB)
	0x80, 0x17, 0x00, 0x20, 0x80, 0x18, 0x00, 0x01, // ; 0x8017:SetAlgorithmParmID, 0x0020:Tx PostEq Mode, 0x8018:SetAlgorithmParm, 0x0001:On
	0x80, 0x17, 0x00, 0x1F, 0x80, 0x18, 0x00, 0x01, // ; 0x8017:SetAlgorithmParmID, 0x001F:Rx PostEq Mode, 0x8018:SetAlgorithmParm, 0x0001:On
	0x80, 0x17, 0x00, 0x30, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0030:Tx MBC Mode, 0x8018:SetAlgorithmParm, 0x0000:Off
	0x80, 0x17, 0x00, 0x31, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0031:Rx MBC Mode, 0x8018:SetAlgorithmParm, 0x0000:Off
	0x80, 0x17, 0x00, 0x1A, 0x80, 0x18, 0x00, 0x01, // ; 0x8017:SetAlgorithmParmID, 0x001A:Use Tx ComfortNoise, 0x8018:SetAlgorithmParm, 0x0001:Yes
	0x80, 0x17, 0x00, 0x1B, 0x80, 0x18, 0xFF, 0xB5, // ; 0x8017:SetAlgorithmParmID, 0x001B:   Tx ComfortNoise gain (dB), 0x8018:SetAlgorithmParm, 0xFFB5:(-75 dB)
	0x80, 0x17, 0x00, 0x1E, 0x80, 0x18, 0x00, 0x01, // ; 0x8017:SetAlgorithmParmID, 0x001E:PST, 0x8018:SetAlgorithmParm, 0x0001:1
	0x80, 0x17, 0x00, 0x63, 0x80, 0x18, 0xFF, 0xA0, // ; SetAlgorithmParmID, 0x0063:    AEC Rx Energy Threshold (dB) -- SetAlgorithmParm, 0xFFA0: (-96 dB)
	0x80, 0x17, 0x00, 0x64, 0x80, 0x18, 0x00, 0x64, // ; SetAlgorithmParmID, 0x0064:    AEC Rx Energy Threshold Time (ms) -- SetAlgorithmParm, 0x0064: (100 ms)

};

static const u8 voip_dv_buf[] =
{
	0x80, 0x52, 0x00, 0x00, // ; 0x8052:Disable digital pass through
	0x80, 0x26, 0x00, 0x93, // ; 0x8026:SelectRouting, 0x0093:
	0x80, 0x1C, 0x00, 0x01, // ; 0x801C:VoiceProcessingOn, 0x0000:On
	0x80, 0x1B, 0x00, 0x06, // ; 0x801B:SetDigitalInputGain, 0x00:PCM-A left, 0x06:(6 dB)
	0x80, 0x1B, 0x01, 0x06, // ; 0x801B:SetDigitalInputGain, 0x01:PCM-A right, 0x06:(6 dB)
	0x80, 0x15, 0x00, 0x00, // ; 0x8015:SetDigitalOutputGain, 0x00:PCM-A left, 0x00:(0 dB)
	0x80, 0x1B, 0x04, 0x00, // ; 0x801B:SetDigitalInputGain, 0x04:PCM-C left, 0x00:(0 dB)
	0x80, 0x15, 0x04, 0xFB, // ; 0x8015:SetDigitalOutputGain, 0x04:PCM-C left, 0xFB:(-5 dB)
	0x80, 0x0C, 0x0A, 0x00, 0x80, 0x0D, 0x00, 0x0F, // ; 0x800C: SetDeviceParmID, 0x0B: PCM1, 0x00: PCM WordLength0x800D: SetDeviceParm, 0x000F: 16 Bits
	0x80, 0x0C, 0x0A, 0x02, 0x80, 0x0D, 0x00, 0x00, // ; 0x800C: SetDeviceParmID, 0x0B: PCM1, 0x02: PCM DelFromFsTx0x800D: SetDeviceParm, 0x0000: (0 clocks)
	0x80, 0x0C, 0x0A, 0x03, 0x80, 0x0D, 0x00, 0x01, // ; 0x800C: SetDeviceParmID, 0x0B: PCM1, 0x03: PCM DelFromFsRx0x800D: SetDeviceParm, 0x0001: (1 clocks)
	0x80, 0x0C, 0x0A, 0x04, 0x80, 0x0D, 0x00, 0x02, // ; 0x800C: SetDeviceParmID, 0x0B: PCM1, 0x04: PCM Latch Edge0x800D: SetDeviceParm, 0x0002:TxFalling/RxFalling
	0x80, 0x0C, 0x0A, 0x05, 0x80, 0x0D, 0x00, 0x01, // ; 0x800C: SetDeviceParmID, 0x0B: PCM1, 0x05: PCM Endianness0x800D: SetDeviceParm, 0x0001:Big Endian
	0x80, 0x0C, 0x0A, 0x06, 0x80, 0x0D, 0x00, 0x00, // ; 0x800C: SetDeviceParmID, 0x0B: PCM1, 0x06: PCM Tristate Enable0x800D: SetDeviceParm, 0x0000: Disable
	0x80, 0x0C, 0x0A, 0x07, 0x80, 0x0D, 0x00, 0x00, // ; 0x800C: SetDeviceParmID, 0x0B: PCM1, 0x07: PCM Audio Port Mode0x800D: SetDeviceParm, 0x0000: PCM
	0x80, 0x0C, 0x0B, 0x00, 0x80, 0x0D, 0x00, 0x0F, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x00: PCM WordLength0x800D: SetDeviceParm, 0x000F: 16 Bits
	0x80, 0x0C, 0x0B, 0x02, 0x80, 0x0D, 0x00, 0x01, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x02: PCM DelFromFsTx0x800D: SetDeviceParm, 0x0001: (1 clocks)
	0x80, 0x0C, 0x0B, 0x03, 0x80, 0x0D, 0x00, 0x01, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x03: PCM DelFromFsRx0x800D: SetDeviceParm, 0x0001: (1 clocks)
	0x80, 0x0C, 0x0B, 0x04, 0x80, 0x0D, 0x00, 0x02, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x04: PCM Latch Edge0x800D: SetDeviceParm, 0x0002:TxFalling/RxFalling
	0x80, 0x0C, 0x0B, 0x05, 0x80, 0x0D, 0x00, 0x01, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x05: PCM Endianness0x800D: SetDeviceParm, 0x0001:Big Endian
	0x80, 0x0C, 0x0B, 0x06, 0x80, 0x0D, 0x00, 0x00, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x06: PCM Tristate Enable0x800D: SetDeviceParm, 0x0000: Disable
	0x80, 0x0C, 0x0B, 0x07, 0x80, 0x0D, 0x00, 0x00, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x07: PCM Audio Port Mode0x800D: SetDeviceParm, 0x0000: PCM

	0x80, 0x17, 0x00, 0x1C, 0x80, 0x18, 0x00, 0x1F, // ; 0x8017:SetAlgorithmParmID, 0x001C:0x8018:SetAlgorithmParm, 0x001F:Reset all
	0x80, 0x17, 0x00, 0x4B, 0x80, 0x18, 0x00, 0x04, // ; 0x8017:SetAlgorithmParmID, 0x004B:Tx Noise Suppression Level, 0x8018:SetAlgorithmParm, 0x0004:Level 4
	0x80, 0x17, 0x00, 0x02, 0x80, 0x18, 0x00, 0x02, // ; 0x8017:SetAlgorithmParmID, 0x0002:Microphone Configuration, 0x8018:SetAlgorithmParm, 0x0002:1-mic Desktop/Vehicle (DV)
	0x80, 0x17, 0x00, 0x15, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0015:Side Tone Mode, 0x8018:SetAlgorithmParm, 0x0000:Off
	0x80, 0x17, 0x00, 0x03, 0x80, 0x18, 0x00, 0x01, // ; 0x8017:SetAlgorithmParmID, 0x0003:AEC Mode, 0x8018:SetAlgorithmParm, 0x0001:AEC On (auto select mode)
	0x80, 0x17, 0x00, 0x12, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0012:   Downlink Speaker Volume, 0x8018:SetAlgorithmParm, 0x0000:(0 dB)
	0x80, 0x17, 0x00, 0x23, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0023:   Use AEC Comfort Noise Fill, 0x8018:SetAlgorithmParm, 0x0000:No
	0x80, 0x17, 0x00, 0x34, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0034:   Echo Suppression Enhancement, 0x8018:SetAlgorithmParm, 0x0000:(0 dB)
	0x80, 0x17, 0x00, 0x36, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0036:   Half-duplex, 0x8018:SetAlgorithmParm, 0x0000:No
	0x80, 0x17, 0x00, 0x37, 0x80, 0x18, 0xFF, 0xE2, // ; 0x8017:SetAlgorithmParmID, 0x0037:   HD Tx interrupt level, 0x8018:SetAlgorithmParm, 0xFFE2:(-30 dB)
	0x80, 0x17, 0x00, 0x2E, 0x80, 0x18, 0xFF, 0xF0, // ; 0x8017:SetAlgorithmParmID, 0x002E:   AEC Comfort Noise Gain, 0x8018:SetAlgorithmParm, 0xFFF0:(-16 dB)
	0x80, 0x17, 0x01, 0x03, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0103:   DT ESE Back Off, 0x8018:SetAlgorithmParm, 0x0000:(0 dB)
	0x80, 0x17, 0x00, 0x04, 0x80, 0x18, 0x00, 0x01, // ; 0x8017:SetAlgorithmParmID, 0x0004:Use Tx AGC, 0x8018:SetAlgorithmParm, 0x0001:Yes
	0x80, 0x17, 0x00, 0x05, 0x80, 0x18, 0xFF, 0xE6, // ; 0x8017:SetAlgorithmParmID, 0x0005:   AGC Target Level (dB), 0x8018:SetAlgorithmParm, 0xFFE6:(-26 dB)
	0x80, 0x17, 0x00, 0x06, 0x80, 0x18, 0xFF, 0xC4, // ; 0x8017:SetAlgorithmParmID, 0x0006:   AGC Noise Floor (dB), 0x8018:SetAlgorithmParm, 0xFFBF:(-60 dB)
	0x80, 0x17, 0x00, 0x07, 0x80, 0x18, 0x00, 0x04, // ; 0x8017:SetAlgorithmParmID, 0x0007:   AGC SNR Improve (dB), 0x8018:SetAlgorithmParm, 0x0004:(4 dB)
	0x80, 0x17, 0x00, 0x26, 0x80, 0x18, 0x00, 0x04, // ; 0x8017:SetAlgorithmParmID, 0x0026:   AGC Up Rate (dBS), 0x8018:SetAlgorithmParm, 0x0004:(4 dBS)
	0x80, 0x17, 0x00, 0x27, 0x80, 0x18, 0x00, 0x04, // ; 0x8017:SetAlgorithmParmID, 0x0027:   AGC Down Rate (dBS), 0x8018:SetAlgorithmParm, 0x0004:(4 dBS)
	0x80, 0x17, 0x01, 0x00, 0x80, 0x18, 0x00, 0x14, // ; 0x8017:SetAlgorithmParmID, 0x0100:   AGC Max Gain (dB), 0x8018:SetAlgorithmParm, 0x0014:(20 dB)
	0x80, 0x17, 0x00, 0x28, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0028:Use Rx AGC, 0x8018:SetAlgorithmParm, 0x0000:No
	0x80, 0x17, 0x00, 0x09, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0009:Speaker Enhancement Mode, 0x8018:SetAlgorithmParm, 0x0000:SE Off (HPF only)
	0x80, 0x17, 0x00, 0x0E, 0x80, 0x18, 0x00, 0x01, // ; 0x8017:SetAlgorithmParmID, 0x000E:Far End Noise Suppression, 0x8018:SetAlgorithmParm, 0x0001:On (auto select mode)
	0x80, 0x17, 0x00, 0x4C, 0x80, 0x18, 0x00, 0x08, // ; 0x8017:SetAlgorithmParmID, 0x004C:   Rx Noise Suppression Level, 0x8018:SetAlgorithmParm, 0x0008:Level 8
	0x80, 0x17, 0x00, 0x42, 0x80, 0x18, 0xFF, 0xFD, // ; SetAlgorithmParmID, 0x0042: Tx-in  Limiter Max Level (dB) -- SetAlgorithmParm, 0xFFFD: (-3 dB)
	0x80, 0x17, 0x00, 0x40, 0x80, 0x18, 0xFF, 0xFD, // ; SetAlgorithmParmID, 0x0040: Tx-out Limiter Max Level (dB) -- SetAlgorithmParm, 0xFFFD: (-3 dB)
	0x80, 0x17, 0x00, 0x0D, 0x80, 0x18, 0xFF, 0xF7, // ; 0x8017:SetAlgorithmParmID, 0x000D:Rx-out Limiter Max Level,   0x8018:SetAlgorithmParm,0xFFF7:( -9 dB)
	0x80, 0x17, 0x00, 0x20, 0x80, 0x18, 0x00, 0x01, // ; 0x8017:SetAlgorithmParmID, 0x0020:Tx PostEq Mode, 0x8018:SetAlgorithmParm, 0x0001:On
	0x80, 0x17, 0x00, 0x1F, 0x80, 0x18, 0x00, 0x01, // ; 0x8017:SetAlgorithmParmID, 0x001F:Rx PostEq Mode, 0x8018:SetAlgorithmParm, 0x0001:On
	0x80, 0x17, 0x00, 0x30, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0030:Tx MBC Mode, 0x8018:SetAlgorithmParm, 0x0000:Off
	0x80, 0x17, 0x00, 0x31, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x0031:Rx MBC Mode, 0x8018:SetAlgorithmParm, 0x0000:Off
	0x80, 0x17, 0x00, 0x1A, 0x80, 0x18, 0x00, 0x00, // ; 0x8017:SetAlgorithmParmID, 0x001A:Use Tx ComfortNoise, 0x8018:SetAlgorithmParm, 0x0000:No
	0x80, 0x17, 0x00, 0x63, 0x80, 0x18, 0xFF, 0xA0, // ; SetAlgorithmParmID, 0x0063:    AEC Rx Energy Threshold (dB) -- SetAlgorithmParm, 0xFFA0: (-96 dB)
	0x80, 0x17, 0x00, 0x64, 0x80, 0x18, 0x00, 0x64, // ; SetAlgorithmParmID, 0x0064:    AEC Rx Energy Threshold Time (ms) -- SetAlgorithmParm, 0x0064: (100 ms)
};

static const u8 voip_whs_buf[] =
{
	0x80, 0x52, 0x00, 0x00, // disable digital pass through
	0x80, 0x26, 0x00, 0x93, // select path route 0x0093
	0x80, 0x0C, 0x0A, 0x00, 0x80, 0x0D, 0x00, 0x0F, // ; 0x800C: SetDeviceParmID, 0x0A: PCM1, 0x00: PCM WordLength0x800D: SetDeviceParm, 0x000F: 16 Bits
	0x80, 0x0C, 0x0A, 0x02, 0x80, 0x0D, 0x00, 0x00, // ; 0x800C: SetDeviceParmID, 0x0A: PCM1, 0x02: PCM DelFromFsTx0x800D: SetDeviceParm, 0x0000: (0 clocks)
	0x80, 0x0C, 0x0A, 0x03, 0x80, 0x0D, 0x00, 0x01, // ; 0x800C: SetDeviceParmID, 0x0A: PCM1, 0x03: PCM DelFromFsRx0x800D: SetDeviceParm, 0x0001: (1 clocks)
	0x80, 0x0C, 0x0A, 0x04, 0x80, 0x0D, 0x00, 0x02, // ; 0x800C: SetDeviceParmID, 0x0A: PCM1, 0x04: PCM Latch Edge0x800D: SetDeviceParm, 0x0002:TxFalling/RxFalling
	0x80, 0x0C, 0x0A, 0x05, 0x80, 0x0D, 0x00, 0x01, // ; 0x800C: SetDeviceParmID, 0x0A: PCM1, 0x05: PCM Endianness0x800D: SetDeviceParm, 0x0001:Big Endian
	0x80, 0x0C, 0x0A, 0x06, 0x80, 0x0D, 0x00, 0x00, // ; 0x800C: SetDeviceParmID, 0x0A: PCM1, 0x06: PCM Tristate Enable0x800D: SetDeviceParm, 0x0000: Disable
	0x80, 0x0C, 0x0A, 0x07, 0x80, 0x0D, 0x00, 0x00, // ; 0x800C: SetDeviceParmID, 0x0A: PCM1, 0x07: PCM Audio Port Mode0x800D: SetDeviceParm, 0x0000: PCM

	0x80, 0x0C, 0x0B, 0x00, 0x80, 0x0D, 0x00, 0x0F, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x00: PCM WordLength0x800D: SetDeviceParm, 0x000F: 16 Bits
	0x80, 0x0C, 0x0B, 0x02, 0x80, 0x0D, 0x00, 0x01, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x02: PCM DelFromFsTx0x800D: SetDeviceParm, 0x0001: (1 clocks)
	0x80, 0x0C, 0x0B, 0x03, 0x80, 0x0D, 0x00, 0x01, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x03: PCM DelFromFsRx0x800D: SetDeviceParm, 0x0001: (1 clocks)
	0x80, 0x0C, 0x0B, 0x04, 0x80, 0x0D, 0x00, 0x02, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x04: PCM Latch Edge0x800D: SetDeviceParm, 0x0002:TxFalling/RxFalling
	0x80, 0x0C, 0x0B, 0x05, 0x80, 0x0D, 0x00, 0x01, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x05: PCM Endianness0x800D: SetDeviceParm, 0x0001:Big Endian
	0x80, 0x0C, 0x0B, 0x06, 0x80, 0x0D, 0x00, 0x00, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x06: PCM Tristate Enable0x800D: SetDeviceParm, 0x0000: Disable
	0x80, 0x0C, 0x0B, 0x07, 0x80, 0x0D, 0x00, 0x00, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x07: PCM Audio Port Mode0x800D: SetDeviceParm, 0x0000: PCM
};

static const u8 voip_bt_buf[] =
{
	0x80, 0x52, 0x00, 0x00, // disable digital pass through
	0x80, 0x26, 0x00, 0x96, // select path route 0x0096
	0x80, 0x0C, 0x0D, 0x00, 0x80, 0x0D, 0x00, 0x0F, // ; 0x800C: SetDeviceParmID, 0x0B: PCM1, 0x00: PCM WordLength0x800D: SetDeviceParm, 0x000F: 16 Bits
	0x80, 0x0C, 0x0D, 0x02, 0x80, 0x0D, 0x00, 0x00, // ; 0x800C: SetDeviceParmID, 0x0B: PCM1, 0x02: PCM DelFromFsTx0x800D: SetDeviceParm, 0x0000: (0 clocks)
	0x80, 0x0C, 0x0D, 0x03, 0x80, 0x0D, 0x00, 0x01, // ; 0x800C: SetDeviceParmID, 0x0B: PCM1, 0x03: PCM DelFromFsRx0x800D: SetDeviceParm, 0x0001: (1 clocks)
	0x80, 0x0C, 0x0D, 0x04, 0x80, 0x0D, 0x00, 0x01, // ; 0x800C: SetDeviceParmID, 0x0B: PCM1, 0x04: PCM Latch Edge0x800D: SetDeviceParm, 0x0001:TxRising / RxRising
	0x80, 0x0C, 0x0D, 0x05, 0x80, 0x0D, 0x00, 0x01, // ; 0x800C: SetDeviceParmID, 0x0B: PCM1, 0x05: PCM Endianness0x800D: SetDeviceParm, 0x0001:Big Endian
	0x80, 0x0C, 0x0D, 0x06, 0x80, 0x0D, 0x00, 0x00, // ; 0x800C: SetDeviceParmID, 0x0B: PCM1, 0x06: PCM Tristate Enable0x800D: SetDeviceParm, 0x0000: Disable
	0x80, 0x0C, 0x0D, 0x07, 0x80, 0x0D, 0x00, 0x00, // ; 0x800C: SetDeviceParmID, 0x0B: PCM1, 0x07: PCM Audio Port Mode0x800D: SetDeviceParm, 0x0000: PCM

	0x80, 0x0C, 0x0B, 0x00, 0x80, 0x0D, 0x00, 0x0F, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x00: PCM WordLength0x800D: SetDeviceParm, 0x000F: 16 Bits
	0x80, 0x0C, 0x0B, 0x02, 0x80, 0x0D, 0x00, 0x01, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x02: PCM DelFromFsTx0x800D: SetDeviceParm, 0x0001: (1 clocks)
	0x80, 0x0C, 0x0B, 0x03, 0x80, 0x0D, 0x00, 0x01, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x03: PCM DelFromFsRx0x800D: SetDeviceParm, 0x0001: (1 clocks)
	0x80, 0x0C, 0x0B, 0x04, 0x80, 0x0D, 0x00, 0x02, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x04: PCM Latch Edge0x800D: SetDeviceParm, 0x0002:TxFalling/RxFalling
	0x80, 0x0C, 0x0B, 0x05, 0x80, 0x0D, 0x00, 0x01, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x05: PCM Endianness0x800D: SetDeviceParm, 0x0001:Big Endian
	0x80, 0x0C, 0x0B, 0x06, 0x80, 0x0D, 0x00, 0x00, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x06: PCM Tristate Enable0x800D: SetDeviceParm, 0x0000: Disable
	0x80, 0x0C, 0x0B, 0x07, 0x80, 0x0D, 0x00, 0x00, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x07: PCM Audio Port Mode0x800D: SetDeviceParm, 0x0000: PCM
	//
};


static const u8 voip_bt_vpoff_buf[] =
{
	0x80, 0x52, 0x00, 0x00, // disable digital pass through
	0x80, 0x26, 0x00, 0x96, // select path route 0x0096
	0x80, 0x0C, 0x0D, 0x00, 0x80, 0x0D, 0x00, 0x0F, // ; 0x800C: SetDeviceParmID, 0x0B: PCM1, 0x00: PCM WordLength0x800D: SetDeviceParm, 0x000F: 16 Bits
	0x80, 0x0C, 0x0D, 0x02, 0x80, 0x0D, 0x00, 0x00, // ; 0x800C: SetDeviceParmID, 0x0B: PCM1, 0x02: PCM DelFromFsTx0x800D: SetDeviceParm, 0x0000: (0 clocks)
	0x80, 0x0C, 0x0D, 0x03, 0x80, 0x0D, 0x00, 0x01, // ; 0x800C: SetDeviceParmID, 0x0B: PCM1, 0x03: PCM DelFromFsRx0x800D: SetDeviceParm, 0x0001: (1 clocks)
	0x80, 0x0C, 0x0D, 0x04, 0x80, 0x0D, 0x00, 0x01, // ; 0x800C: SetDeviceParmID, 0x0B: PCM1, 0x04: PCM Latch Edge0x800D: SetDeviceParm, 0x0001:TxRising / RxRising
	0x80, 0x0C, 0x0D, 0x05, 0x80, 0x0D, 0x00, 0x01, // ; 0x800C: SetDeviceParmID, 0x0B: PCM1, 0x05: PCM Endianness0x800D: SetDeviceParm, 0x0001:Big Endian
	0x80, 0x0C, 0x0D, 0x06, 0x80, 0x0D, 0x00, 0x00, // ; 0x800C: SetDeviceParmID, 0x0B: PCM1, 0x06: PCM Tristate Enable0x800D: SetDeviceParm, 0x0000: Disable
	0x80, 0x0C, 0x0D, 0x07, 0x80, 0x0D, 0x00, 0x00, // ; 0x800C: SetDeviceParmID, 0x0B: PCM1, 0x07: PCM Audio Port Mode0x800D: SetDeviceParm, 0x0000: PCM

	0x80, 0x0C, 0x0B, 0x00, 0x80, 0x0D, 0x00, 0x0F, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x00: PCM WordLength0x800D: SetDeviceParm, 0x000F: 16 Bits
	0x80, 0x0C, 0x0B, 0x02, 0x80, 0x0D, 0x00, 0x01, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x02: PCM DelFromFsTx0x800D: SetDeviceParm, 0x0001: (1 clocks)
	0x80, 0x0C, 0x0B, 0x03, 0x80, 0x0D, 0x00, 0x01, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x03: PCM DelFromFsRx0x800D: SetDeviceParm, 0x0001: (1 clocks)
	0x80, 0x0C, 0x0B, 0x04, 0x80, 0x0D, 0x00, 0x02, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x04: PCM Latch Edge0x800D: SetDeviceParm, 0x0002:TxFalling/RxFalling
	0x80, 0x0C, 0x0B, 0x05, 0x80, 0x0D, 0x00, 0x01, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x05: PCM Endianness0x800D: SetDeviceParm, 0x0001:Big Endian
	0x80, 0x0C, 0x0B, 0x06, 0x80, 0x0D, 0x00, 0x00, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x06: PCM Tristate Enable0x800D: SetDeviceParm, 0x0000: Disable
	0x80, 0x0C, 0x0B, 0x07, 0x80, 0x0D, 0x00, 0x00, // ; 0x800C: SetDeviceParmID, 0x0B: PCM2, 0x07: PCM Audio Port Mode0x800D: SetDeviceParm, 0x0000: PCM
};

static const u8 bt_ring_buf[] = {
	0x80, 0x52, 0x00, 0xF3,
};

#endif /* __LINUX_ES305B_PARAM_H_ */
