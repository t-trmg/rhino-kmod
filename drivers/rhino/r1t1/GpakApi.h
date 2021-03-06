/*
 * Copyright (c) 2005 , Adaptive Digital Technologies, Inc.
 *
 * File Name: GpakApi.h
 *
 * Description:
 *   This file contains the function prototypes and data types for the user
 *   API functions that communicate with DSPs executing G.PAK software. The
 *   file is used by application software in the host processor connected to
 *   C55X G.PAK DSPs via a Host Port Interface.
 *
 * Version: 1.0
 *
 * Revision History:
 *   06/15/05 - Initial release.
 *   11/15/2006  - 24 TDM-TDM Channels EC release
 */

#ifndef _GPAKAPI_H				/* prevent multiple inclusion */
#define _GPAKAPI_H
#include "GpakErrs.h"
#include "gpakenum.h"
#include "r1t1.h"

typedef unsigned long int GPAK_FILE_ID;	/* G.PAK Download file identifier */

#define BL_DSP_BOOTLOADER_ENTRY 0x7020	/* DSP bootloader entry-point, */
										/* HOST have to put this address in 0x60-61 */
										/* after DSP bootloader bin file downloaded */

typedef unsigned short int DSP_WORD;	/* 16 bit DSP word */
typedef unsigned int DSP_ADDRESS;	/* 32 bit DSP address */
//typedef unsigned long int DSP_ADDRESS;  /* 32 bit DSP address */

#define DSP_DEBUG_BUFF_SIZE 42	// units of 16-bit words

/* Definition of an Asynchronous Event Data Structure */

typedef union {
	struct {
		GpakToneCodes_t ToneCode;	// detected tone code
		unsigned short int ToneDuration;	// tone duration
		GpakTdmDirection Direction;	// detected on A r B side
		short int DebugToneStatus;	// reserved for debug info
	} toneEvent;

} GpakAsyncEventData_t;

/* Definition of an Echo Canceller Parameters information structure. */

typedef struct {
	short int EcanTapLength;	// Echo Can Num Taps (tail length) 
	short int EcanNlpType;		// Echo Can NLP Type 
	short int EcanAdaptEnable;	// Echo Can Adapt Enable flag 
	short int EcanG165DetEnable;	// Echo Can G165 Detect Enable flag 
	short int EcanDblTalkThresh;	// Echo Can Double Talk threshold 
	short int EcanNlpThreshold;	// Echo Can NLP threshold 
	short int EcanNlpConv;		// Dynamic NLP control, NLP limit when EC about to converged
	short int EcanNlpUnConv;	// Dynamic NLP control, NLP limit when EC not converged yet
	short int EcanNlpMaxSuppress;	// suppression level for NLP_SUPP mode
	short int EcanCngThreshold;	// Echo Can CNG Noise threshold 
	short int EcanAdaptLimit;	// Echo Can Max Adapts per frame 
	short int EcanCrossCorrLimit;	// Echo Can Cross Correlation limit 
	short int EcanNumFirSegments;	// Echo Can Num FIR Segments 
	short int EcanFirSegmentLen;	// Echo Can FIR Segment Length 
} GpakEcanParms_t;

/* Definition of a Channel Configuration information structure. */

typedef struct {
	GpakSerialPort_t PcmInPortA;	// A side PCM Input Serial Port Id 
	unsigned short int PcmInSlotA;	// A side PCM Input Time Slot 
	GpakSerialPort_t PcmOutPortA;	// A side PCM Output Serial Port Id 
	unsigned short int PcmOutSlotA;	// A side PCM Output Time Slot 
	GpakSerialPort_t PcmInPortB;	// B side PCM Input Serial Port Id 
	unsigned short int PcmInSlotB;	// B side PCM Input Time Slot 
	GpakSerialPort_t PcmOutPortB;	// B side PCM Output Serial Port Id 
	unsigned short int PcmOutSlotB;	// B side PCM Output Time Slot 
	GpakToneTypes ToneTypesA;	// A side Tone Detect Types
	GpakToneTypes ToneTypesB;	// B side Tone Detect Types
	GpakActivation EcanEnableA;	// Echo Cancel A Enabled
	GpakActivation EcanEnableB;	// Echo Cancel B Enabled
	GpakEcanParms_t EcanParametersA;	// Echo Cancel parameters 
	GpakEcanParms_t EcanParametersB;	// Echo Cancel parameters 
	GpakCompandModes SoftwareCompand;	// software companding
	GpakRate_t FrameRate;		// Gpak Frame Rate 
	GpakActivation MuteToneA;	// A side mute DTMF Enabled
	GpakActivation MuteToneB;	// B side mute DTMF Enabled
	GpakActivation FaxCngDetA;	// A side FaxCng Tone Detector Enabled
	GpakActivation FaxCngDetB;	// B side FaxCng Tone Detector Enabled

} GpakChannelConfig_t;


/* Definition of a Serial Port Configuration Structure */
typedef struct {
	GpakSlotCfg_t SlotsSelect1;	// port 1 Slot selection
	unsigned short int FirstBlockNum1;	// port 1 first group Block Number
	unsigned short int FirstSlotMask1;	// port 1 first group Slot Mask
	unsigned short int SecBlockNum1;	// port 1 second group Block Number
	unsigned short int SecSlotMask1;	// port 1 second group Slot Mask

	GpakSerWordSize_t SerialWordSize1;	// port 1 serial word size 
	GpakCompandModes CompandingMode1;	// port 1 companding mode
	GpakSerFrameSyncPol_t TxFrameSyncPolarity1;	// port 1 Tx Frame Sync Polarity
	GpakSerFrameSyncPol_t RxFrameSyncPolarity1;	// port 1 Rx Frame Sync Polarity
	GpakSerClockPol_t TxClockPolarity1;	// port 1 Tx Clock Polarity
	GpakSerClockPol_t RxClockPolarity1;	// port 1 Rx Clock Polarity
	GpakSerDataDelay_t TxDataDelay1;	// port 1 Tx data delay
	GpakSerDataDelay_t RxDataDelay1;	// port 1 Rx data delay
	GpakActivation DxDelay1;	// port 1 DX Delay

	unsigned short int ThirdSlotMask1;	// port 1 3rd group Slot Mask
	unsigned short int FouthSlotMask1;	// port 1 4th group Slot Mask
	unsigned short int FifthSlotMask1;	// port 1 5th group Slot Mask
	unsigned short int SixthSlotMask1;	// port 1 6th group Slot Mask
	unsigned short int SevenSlotMask1;	// port 1 7th group Slot Mask
	unsigned short int EightSlotMask1;	// port 1 8th group Slot Mask


	GpakSlotCfg_t SlotsSelect2;	// port 2 Slot selection
	unsigned short int FirstBlockNum2;	// port 2 first group Block Number
	unsigned short int FirstSlotMask2;	// port 2 first group Slot Mask
	unsigned short int SecBlockNum2;	// port 2 second group Block Number
	unsigned short int SecSlotMask2;	// port 2 second group Slot Mask
	GpakSerWordSize_t SerialWordSize2;	// port 2 serial word size 
	GpakCompandModes CompandingMode2;	// port 2 companding mode
	GpakSerFrameSyncPol_t TxFrameSyncPolarity2;	// port 2 Tx Frame Sync Polarity
	GpakSerFrameSyncPol_t RxFrameSyncPolarity2;	// port 2 Rx Frame Sync Polarity
	GpakSerClockPol_t TxClockPolarity2;	// port 2 Tx Clock Polarity
	GpakSerClockPol_t RxClockPolarity2;	// port 2 Rx Clock Polarity
	GpakSerDataDelay_t TxDataDelay2;	// port 2 Tx data delay
	GpakSerDataDelay_t RxDataDelay2;	// port 2 Rx data delay
	GpakActivation DxDelay2;	// port 2 DX Delay

	unsigned short int ThirdSlotMask2;	// port 2 3rd group Slot Mask
	unsigned short int FouthSlotMask2;	// port 2 4th group Slot Mask
	unsigned short int FifthSlotMask2;	// port 2 5th group Slot Mask
	unsigned short int SixthSlotMask2;	// port 2 6th group Slot Mask
	unsigned short int SevenSlotMask2;	// port 2 7th group Slot Mask
	unsigned short int EightSlotMask2;	// port 2 8th group Slot Mask

	GpakSlotCfg_t SlotsSelect3;	// port 3 Slot selection
	unsigned short int FirstBlockNum3;	// port 3 first group Block Number
	unsigned short int FirstSlotMask3;	// port 3 first group Slot Mask
	unsigned short int SecBlockNum3;	// port 3 second group Block Number
	unsigned short int SecSlotMask3;	// port 3 second group Slot Mask
	GpakSerWordSize_t SerialWordSize3;	// port 3 serial word size 
	GpakCompandModes CompandingMode3;	// port 3 companding mode
	GpakSerFrameSyncPol_t TxFrameSyncPolarity3;	// port 3 Tx Frame Sync Polarity
	GpakSerFrameSyncPol_t RxFrameSyncPolarity3;	// port 3 Rx Frame Sync Polarity
	GpakSerClockPol_t TxClockPolarity3;	// port 3 Tx Clock Polarity
	GpakSerClockPol_t RxClockPolarity3;	// port 3 Rx Clock Polarity
	GpakSerDataDelay_t TxDataDelay3;	// port 3 Tx data delay
	GpakSerDataDelay_t RxDataDelay3;	// port 3 Rx data delay
	GpakActivation DxDelay3;	// port 3 DX Delay

	unsigned short int ThirdSlotMask3;	// port 3 3rd group Slot Mask
	unsigned short int FouthSlotMask3;	// port 3 4th group Slot Mask
	unsigned short int FifthSlotMask3;	// port 3 5th group Slot Mask
	unsigned short int SixthSlotMask3;	// port 3 6th group Slot Mask
	unsigned short int SevenSlotMask3;	// port 3 7th group Slot Mask
	unsigned short int EightSlotMask3;	// port 3 8th group Slot Mask

} GpakPortConfig_t;

/* Definition of a Tone Generation Parameter Structure */
/*
typedef struct
{
    GpakToneGenType_t ToneType;         // Tone Type
    unsigned short int Frequency[4];    // Frequency (Hz)
    short int Level[4];                 // Frequency's Level (1 dBm)
    unsigned short int OnTime[4];       // On Times (msecs)
    unsigned short int OffTime[4];      // Off Times (msecs)
} GpakToneGenParms_t;
*/
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

/* gpakConfigureChannel return status. */
typedef enum {
	CcsSuccess = 0,				/* Channel Configured successfully */
	CcsParmError = 1,			/* Channel Config Parameter error */
	CcsInvalidChannel = 2,		/* invalid channel */
	CcsInvalidDsp = 3,			/* invalid DSP */
	CcsDspCommFailure = 4		/* failed to communicate with DSP */
} gpakConfigChanStatus_t;

/*
 * gpakConfigureChannel - Configure a DSP's Channel.
 *
 * FUNCTION
 *  This function configures a DSP's Channel.
 *
 * RETURNS
 *  Status code indicating success or a specific error.
 *
 */
extern gpakConfigChanStatus_t gpakConfigureChannel(struct r1t1_card *r1t1_card,	/* Card containing the DSP */
												   unsigned short int DspId,	// DSP identifier
												   unsigned short int ChannelId,	// channel identifier
												   GpakChanType ChannelType,	// channel type
												   GpakChannelConfig_t * pChanConfig,	// pointer to channel config info 
												   GPAK_ChannelConfigStat_t * pStatus	// pointer to Channel Config Status
	);


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

/* gpakTearDownChannel return status. */
typedef enum {
	TdsSuccess = 0,				/* Channel Tear Down successful */
	TdsError = 1,				/* Channel Tear Down error */
	TdsInvalidChannel = 2,		/* invalid channel */
	TdsInvalidDsp = 3,			/* invalid DSP */
	TdsDspCommFailure = 4		/* failed to communicate with DSP */
} gpakTearDownStatus_t;

/*
 * gpakTearDownChannel - Tear Down a DSP's Channel.
 *
 * FUNCTION
 *  This function tears down a DSP's Channel.
 *
 * RETURNS
 *  Status code indicating success or a specific error.
 *
 */

extern gpakTearDownStatus_t gpakTearDownChannel(struct r1t1_card *r1t1_card,	/* Card containing the DSP */
												unsigned short int DspId,	// DSP identifier
												unsigned short int ChannelId,	// channel identifier
												GPAK_TearDownChanStat_t * pStatus	// pointer to Tear Down Status
	);

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

/* gpakAlgControl return status. */
typedef enum {
	AcSuccess = 0,				/* control successful */
	AcInvalidChannel = 1,		/* invalid channel identifier */
	AcInvalidDsp = 2,			/* invalid DSP */
	AcParmError = 3,			/* invalid control parameter */
	AcDspCommFailure = 4		/* failed to communicate with DSP */
} gpakAlgControlStat_t;

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * gpakAlgControl - Control an Algorithm.
 *
 * FUNCTION
 *  This function controls an Algorithm
 *
 * RETURNS
 *  Status code indicating success or a specific error.
 *
 */
extern gpakAlgControlStat_t gpakAlgControl(struct r1t1_card *r1t1_card,	/* Card containing the DSP */
										   unsigned short int DspId,	// DSP identifier
										   unsigned short int ChannelId,	// channel identifier
										   GpakAlgCtrl_t ControlCode,	// algorithm control code
										   GPAK_AlgControlStat_t * pStatus	// pointer to return status
	);


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

/* gpakConfigurePorts return status. */
typedef enum {
	CpsSuccess = 0,				/* Serial Ports configured successfully */
	CpsParmError = 1,			/* Configure Ports Parameter error */
	CpsInvalidDsp = 2,			/* invalid DSP */
	CpsDspCommFailure = 3		/* failed to communicate with DSP */
} gpakConfigPortStatus_t;

/*
 * gpakConfigurePorts - Configure a DSP's serial ports.
 *
 * FUNCTION
 *  This function configures a DSP's serial ports.
 *
 * RETURNS
 *  Status code indicating success or a specific error.
 *
 */
extern gpakConfigPortStatus_t gpakConfigurePorts(struct r1t1_card *r1t1_card,	/* Card containing the DSP */
												 unsigned short int DspId,	// DSP identifier
												 GpakPortConfig_t * pPortConfig,	// pointer to Port Config info
												 GPAK_PortConfigStat_t * pStatus	// pointer to Port Config Status
	);

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

/* gpakDownloadDsp return status. */
typedef enum {
	GdlSuccess = 0,				/* DSP download successful */
	GdlFileReadError = 1,		/* error reading Download file */
	GdlInvalidFile = 2,			/* invalid Download file content */
	GdlInvalidDsp = 3,			/* invalid DSP */
	GdlDspCommFailure = 4		/* failed to communicate with DSP */
} gpakDownloadStatus_t;

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * gpakDownloadLoader - Download the DSP's Loader program.
 *
 * FUNCTION
 *  This function reads the DSP's Loader program from the specified file and
 *  writes the image to the DSP's memory.
 *
 * RETURNS
 *  Status code indicating success or a specific error.
 *
 */

extern gpakDownloadStatus_t gpakDownloadLoader(struct r1t1_card *r1t1_card,	/* Card containing the DSP */
											   unsigned short int DspId,	/* DSP Identifier (0 to MaxDSPCores-1) */
											   GPAK_FILE_ID FileId	/* G.PAK Loader program File Identifier */
	);

/*
 * gpakDownloadDsp - Download a DSP's Program and initialized Data memory.
 *
 * FUNCTION
 *  This function reads a DSP's Program and Data memory image from the
 *  specified file and writes the image to the DSP's memory.
 *
 * RETURNS
 *  Status code indicating success or a specific error.
 *
 */
extern gpakDownloadStatus_t gpakDownloadDsp_5510(struct r1t1_card *r1t1_card,	/* Card containing the DSP */
												 unsigned short int DspId,	// DSP identifier
												 GPAK_FILE_ID FileId	// G.PAK download file identifier
	);

extern gpakDownloadStatus_t gpakDownloadDsp_5507(struct r1t1_card *r1t1_card,	/* Card containing the DSP */
												 unsigned short int DspId,	// DSP identifier
												 GPAK_FILE_ID FileId	// G.PAK download file identifier
	);

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

/* gpakReadEventFIFOMessage return status */
typedef enum {
	RefEventAvail = 0,			/* an event was successfully read from the fifo */
	RefNoEventAvail = 1,		/* no event was in the fifo */
	RefInvalidDsp = 2,			/* invalid DSP identifier */
	RefInvalidEvent = 3,		/* invalid event */
	RefDspCommFailure = 4		/* error communicating with DSP */
} gpakReadEventFIFOMessageStat_t;

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * gpakReadEventFIFOMessage - read from the event fifo
 * 
 * FUNCTION
 *  This function reads a single event from the event fifo if one is available
 * 
 * RETURNS
 *  Status  code indicating success or a specific error.
 *
 * Note: This function should be called in a loop until the return status 
 *       indicates that the fifo is empty.
 */
extern gpakReadEventFIFOMessageStat_t gpakReadEventFIFOMessage(struct r1t1_card *r1t1_card,	// Card containing the DSP
															   unsigned short int DspId,	// DSP identifier
															   unsigned short int *pChannelId,	// pointer to channel identifier
															   GpakAsyncEventCode_t * pEventCode,	// pointer to Event Code
															   GpakAsyncEventData_t * pEventData	// pointer to Event Data Struct
	);


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

/* gpakPingDsp return status values */
typedef enum {
	PngSuccess = 0,				/* DSP responded successfully */
	PngInvalidDsp = 1,			/* invalid DSP identifier */
	PngDspCommFailure = 2		/* error communicating with DSP */
} gpakPingDspStat_t;

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * gpakPingDsp - ping the DSP to see if it's alive
 * 
 * FUNCTION
 *  This function checks if the DSP is still communicating with the host
 * 
 * RETURNS
 *  Status  code indicating success or a specific error.
 */
extern gpakPingDspStat_t gpakPingDsp(struct r1t1_card *r1t1_card,	/* Card containing the DSP */
									 unsigned short int DspId,	// DSP identifier
									 unsigned short int *pDspSwVersion	// DSP software version
	);

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

/* gpakSerialTxFixedValue return status values */
typedef enum {
	TfvSuccess = 0,				/* operation successful */
	TfvInvalidChannel = 1,		/* invalid channel identifier */
	TfvInvalidDsp = 2,			/* invalid DSP identifier */
	TfvDspCommFailure = 3		/* failed to communicate with DSP */
} gpakSerialTxFixedValueStat_t;

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * gpakSerialTxFixedValue - transmit a fixed value on a timeslot
 * 
 * FUNCTION
 *  This function controls transmission of a fixed value out onto a serial 
 *  port's timeslot.
 * 
 * RETURNS
 *  Status  code indicating success or a specific error.
 */
extern gpakSerialTxFixedValueStat_t gpakSerialTxFixedValue(struct r1t1_card *r1t1_card,	/* Card containing the DSP */
														   unsigned short int DspId,	// DSP identifier
														   unsigned short int ChannelId,	// channel identifier
														   GpakSerialPort_t PcmOutPort,	// PCM Output Serial Port Id
														   unsigned short int PcmOutSlot,	// PCM Output Time Slot
														   unsigned short int Value,	// 16-bit value 
														   GpakActivation State	// activation state
	);

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

/* gpakControlTdmLoopBack return status values */
typedef enum {
	ClbSuccess = 0,				/* operation successful */
	ClbSerPortInactive = 1,		/* serial port is inactive */
	ClbInvalidDsp = 2,			/* invalid DSP identifier */
	ClbDspCommFailure = 3		/* failed to communicate with DSP */
} gpakControlTdmLoopBackStat_t;

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * gpakControlTdmLoopBack - control a serial port's loopback state
 * 
 * FUNCTION
 *  This function enables/disables the tdm input to output looback mode on a
 *  serial port
 * 
 * RETURNS
 *  Status  code indicating success or a specific error.
 */

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
gpakControlTdmLoopBackStat_t gpakControlTdmLoopBack(struct r1t1_card *r1t1_card,	/* Card containing the DSP */
													unsigned short int DspId,	// DSP identifier
													GpakSerialPort_t SerialPort,	// Serial Port Id
													GpakActivation LoopBackState	// Loopback State
	);

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

/* gpakReadCpuUsage return status values */
typedef enum {
	RcuSuccess = 0,				/* operation successful */
	RcuInvalidDsp = 1,			/* invalid DSP identifier */
	RcuDspCommFailure = 2		/* communication failure */
} gpakReadCpuUsageStat_t;

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * gpakReadCpuUsage - read the cpu usage statistics
 * 
 * FUNCTION
 *  This function reads cpu utilization from the DSP.
 * 
 * RETURNS
 *  Status  code indicating success or a specific error.
 */
extern gpakReadCpuUsageStat_t gpakReadCpuUsage(struct r1t1_card *r1t1_card,	/* Card containing the DSP */
											   unsigned short int DspId,	// DSP identifier
											   unsigned short int *pPeakUsage,	// pointer to peak usage variable
											   unsigned short int *pPrev1SecPeakUsage	// peak usage over previous 1 second
	);

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

/* gpakResetCpuUsageStats return status values */
typedef enum {
	RstcSuccess = 0,			/* operation successful */
	RstcInvalidDsp = 1,			/* invalid DSP identifier */
	RstcDspCommFailure = 2		/* communication failure */
} gpakResetCpuUsageStat_t;
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * gpakResetCpuUsageStats - reset the cpu usage statistics
 * 
 * FUNCTION
 *  This function resets the cpu utilization statistics
 * 
 * RETURNS
 *  Status  code indicating success or a specific error.
 */
extern gpakResetCpuUsageStat_t gpakResetCpuUsageStats(struct r1t1_card *r1t1_card,	/* Card containing the DSP */
													  unsigned short int DspId	// DSP identifier
	);


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

/* gpakReadFramingStats return status values */
typedef enum {
	RfsSuccess = 0,				/* operation successful */
	RfsInvalidDsp = 1,			/* invalid DSP identifier */
	RfsDspCommFailure = 2		/* communication failure */
} gpakReadFramingStatsStatus_t;

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * gpakReadFramingStats
 * 
 * FUNCTION
 *  This function reads a DSP's framing interrupt statistics
 * 
 * RETURNS
 *  Status  code indicating success or a specific error.
 */
extern gpakReadFramingStatsStatus_t gpakReadFramingStats(struct r1t1_card *r1t1_card,	/* Card containing the DSP */
														 unsigned short int DspId,	// DSP identifier
														 unsigned short int *pFramingError1Count,	// port 1 Framing error count
														 unsigned short int *pFramingError2Count,	// port 2 Framing error count
														 unsigned short int *pFramingError3Count,	// port 3 Framing error count
														 unsigned short int *pDmaStopErrorCount,	// DMA-stoppage error count
														 unsigned short int *pDmaSlipStatsBuffer	// DMA slips count
	);

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

/* gpakResetFramingStats return values */
typedef enum {
	RstfSuccess = 0,			/* operation successful */
	RstfInvalidDsp = 1,			/* invalid DSP identifier */
	RstfDspCommFailure = 2		/* communication failure */
} gpakResetFramingStatsStatus_t;

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * gpakResetFramingStats - reset a DSP's framing interrupt statistics
 * 
 * FUNCTION
 *  This function resets a DSP's framing interrupt statistics
 * 
 * RETURNS
 *  Status  code indicating success or a specific error.
 */
extern gpakResetFramingStatsStatus_t gpakResetFramingStats(struct r1t1_card *r1t1_card,	/* Card containing the DSP */
														   unsigned short int DspId	// DSP identifier
	);


typedef enum {
	RmmSuccess = 0,
	RmmInvalidDsp = 1,
	RmmSizeTooBig = 2,
	RmmFailure = 3,
	RmmInvalidAddress = 4
} gpakReadDSPMemoryStat_t;
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * gpakResetFramingStats - read a section of DSP memory
 *                         to get access DSP registers, since 0x00--0x60 not HPI-accessable
 * 
 * FUNCTION
 *  This function resets a DSP's framing interrupt statistics
 * 
 * RETURNS
 *  Status  code indicating success or a specific error.
 */

extern gpakReadDSPMemoryStat_t gpakReadDSPMemoryMap(struct r1t1_card *r1t1_card,	// Card containing the DSP
													unsigned short int DspId,	// Dsp Identifier
													unsigned short int *pDest,	// Buffer on host to hold DSP memory map
													DSP_ADDRESS BufrBaseAddress,	// DSP memory users want to read out
													unsigned short int MemoryLength_Word16	// Length of memory section read out, unit is 16-bit word
	);

#endif // end multiple inclusion
