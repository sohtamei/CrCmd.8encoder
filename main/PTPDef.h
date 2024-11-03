#ifndef PTPTDEF
#define PTPTDEF

#define PTP_DT_UNDEF 0x0000
#define PTP_DT_INT8 0x0001
#define PTP_DT_UINT8 0x0002
#define PTP_DT_INT16 0x0003
#define PTP_DT_UINT16 0x0004
#define PTP_DT_INT32 0x0005
#define PTP_DT_UINT32 0x0006
#define PTP_DT_INT64 0x0007
#define PTP_DT_UINT64 0x0008
#define PTP_DT_INT128 0x0009
#define PTP_DT_UINT128 0x000A
#define PTP_DT_AINT8 0x4001
#define PTP_DT_AUINT8 0x4002
#define PTP_DT_AINT16 0x4003
#define PTP_DT_AUINT16 0x4004
#define PTP_DT_AINT32 0x4005
#define PTP_DT_AUINT32 0x4006
#define PTP_DT_AINT64 0x4007
#define PTP_DT_AUINT64 0x4008
#define PTP_DT_AINT128 0x4009
#define PTP_DT_AUINT128 0x400A
#define PTP_DT_STR 0xFFFF

enum {
	PTP_OC_GetDeviceInfo		= 0x1001,
	PTP_OC_OpenSession			= 0x1002,
	PTP_OC_CloseSession			= 0x1003,
	PTP_OC_GetStorageID			= 0x1004,
	PTP_OC_GetStorageInfo		= 0x1005,
	PTP_OC_GetNumObjects		= 0x1006,
	PTP_OC_GetObjectHandles		= 0x1007,
	PTP_OC_GetObjectInfo		= 0x1008,
	PTP_OC_GetObject			= 0x1009,
	PTP_OC_GetThumb 			= 0x100A,
	PTP_OC_DeleteObject			= 0x100B,
	PTP_OC_SendObject			= 0x100D,
	PTP_OC_GetPartialObject		= 0x101B,

	PTP_OC_SDIOConnect					= 0x9201,
	PTP_OC_SDIOGetExtDeviceInfo			= 0x9202,
	PTP_OC_SDIOSetExtDevicePropValue	= 0x9205,
	PTP_OC_SDIOControlDevice			= 0x9207,
	PTP_OC_SDIOGetAllExtDevicePropInfo	= 0x9209,
	PTP_OC_SDIOOpenSession 				= 0x9210,
	PTP_OC_SDIOGetPartialLargeObject	= 0x9211,
	PTP_OC_SDIOSetContentsTransferMode	= 0x9212,
	PTP_OC_SDIOGetDisplayStringList 	= 0x9215,
	PTP_OC_SDIOGetLensInformation		= 0x9223,
	PTP_OC_SDIOOperationResultsSupported = 0x922F,
};


enum {
	PTP_SDIE_ObjectAdded = 0xC201,
	PTP_SDIE_ObjectRemoved = 0xC202,
	PTP_SDIE_DevicePropChanged = 0xC203,
	PTP_SDIE_CapturedEvent = 0xC206,
	PTP_SDIE_CWBCapturedResult = 0xC208,
	PTP_SDIE_CameraSettingReadResult = 0xC209,
	PTP_SDIE_FTPSettingReadResult = 0xC20A,
	PTP_SDIE_MediaFormatResult = 0xC20B,
	PTP_SDIE_FTPDisplayNameListChanged = 0xC20C,
	PTP_SDIE_ContentsTransferEvent = 0xC20D,
	PTP_SDIE_DisplayListChangedEvent = 0xC20F,
	PTP_SDIE_FocusPositionResult = 0xC218,
	PTP_SDIE_LensInformationChanged = 0xC21B,
	PTP_SDIE_OperationResults = 0xC222,
	PTP_SDIE_AFStatus = 0xC223,
	PTP_SDIE_MovieRecOperationResults = 0xC224,
};

enum {
	DPC_MOVIE_REC				= 0xD2C8,
	DPC_S2_BUTTON				= 0xD2C2,
	DPC_S1_BUTTON				= 0xD2C1,
	DPC_AE_LOCK					= 0xD2C3,
	DPC_AF_LOCK					= 0xD2C9,
	DPC_LIVEVIEW_MODE			= 0xD26A,
	DPC_DRIVE_MODE				= 0x5013,
	DPC_ASPECT_RATIO			= 0xD211,
	DPC_IMAGE_SIZE				= 0xD203,
	DPC_WHITE_BALANCE			= 0x5005,
	DPC_EXPOSURE_COMPENSATION	= 0x5010,		//
	DPC_FNUMBER					= 0x5007,		//
	DPC_SHOOTING_FILE_INFOMATION= 0xD215,
	DPC_FOCUS_MODE				= 0x500A,
	DPC_EXPOSURE_MODE			= 0x500E,		//
	DPC_DRO_HDR_MODE			= 0xD201,
	DPC_FOCUS_AREA_X_Y			= 0xD2DC,
	DPC_FOCUS_AREA				= 0xD22C,
	DPC_LIVEVIEW_STATUS			= 0xD221,
	DPC_SHUTTER_SPEED			= 0xD20D,		//
	DPC_COLOR_TEMP				= 0xD20F,
	DPC_ISO						= 0xD21E,		//
	DPC_FLASH_MODE				= 0x500C,
	DPC_METERING_MODE			= 0x500B,
	DPC_PICTURE_EFFECTS			= 0xD21B,
	DPC_BATTERY_LEVEL			= 0xD20E,
	DPC_POSITION_KEY			= 0xD25A,		//
	DPC_AELOCK_INDICATION		= 0xD217,
	DPC_AFLOCK_INDICATION		= 0xD21F,
	DPC_WHITEBALANCE_AB			= 0xD21C,
	DPC_WHITEBALANCE_GM			= 0xD210,
	DPC_COMPRESSION_SETTING		= 0x5004,
	DPC_IMAGE_FILE_FORMAT		= 0x5004,
	DPC_VIEW					= 0xD231,
	DPC_FLASH_COMP				= 0xD200,
	DPC_WIRELESS_FLASH			= 0xD262,
	DPC_AWB_LOCK				= 0xD2D9,
	DPC_NORMAL					= 0xD2D4,
	DPC_AF_STATUS				= 0xD213,
	DPC_PICTURE_PROFILE			= 0xD23F,
	DPC_CREATIVE_STYLE			= 0xD240,
	DPC_MOVIE_FORMAT			= 0xD241,
	DPC_MOVIE_QUALITY			= 0xD242,
	DPC_SAVE_MEDIA				= 0xD222,		//
	DPC_ZOOM_SETTING			= 0xD25F,
	DPC_ZOOM_SCALE				= 0xD25C,
	DPC_ZOOM_OPTIC				= 0xD25D,
	DPC_NEAR_FAR				= 0xD2D1,
	DPC_ZOOM					= 0xD2DD,
	DPC_MEDIA_FORMAT			= 0xD2CA,
	DPC_JPEG_QUALITY			= 0xD252,
	DPC_FILE_FORMAT				= 0xD253,
	DPC_FOCUS_MAGNIFY			= 0xD254,
	DPC_USB_POWER_SUPPLY		= 0xD150,
};

#endif // PTPTDEF
