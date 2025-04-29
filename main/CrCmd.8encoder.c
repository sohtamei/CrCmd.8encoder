// カメラUSB挿抜

/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "driver/gpio.h"
#include "usb/usb_host.h"

#include "PTPDef.h"
#include "_8encoder.h"

static const char *TAG = "DAEMON";


#define DAEMON_TASK_PRIORITY    2
#define CLASS_TASK_PRIORITY     3

#define CLIENT_NUM_EVENT_MSG    5

#define PARAM_NUM  5

struct _Param {
	uint16_t pcode;
	uint16_t datatype;
	uint8_t  getset;		// 0-R, 1-R/W
	uint8_t  isenabled;		// 0-invalid, 1-R/W, 2-R
	int32_t  current;
	uint8_t  formflag;
	int32_t* enums;
	int      enumNum;

	int32_t  currentIndex;
	uint32_t led;
} paramTable[PARAM_NUM] = {
	{DPC_SHUTTER_SPEED,			0,0,0,0,0,NULL,0,0,0},
	{DPC_FNUMBER,				0,0,0,0,0,NULL,0,0,0},
	{DPC_EXPOSURE_COMPENSATION,	0,0,0,0,0,NULL,0,0,0},
	{DPC_ISO,					0,0,0,0,0,NULL,0,0,0},
	{DPC_EXPOSURE_MODE,			0,0,0,0,0,NULL,0,0,0},
};

#define TIMEOUT_MS 100

#define SetL32(buf,data) {uint32_t d=(data);(buf)[0]=(d);(buf)[1]=(d)>>8;(buf)[2]=(d)>>16;(buf)[3]=(d)>>24;}
#define SetL16(buf,data) {uint32_t d=(data);(buf)[0]=(d);(buf)[1]=(d)>>8;}

#define GetL32(buf) (((buf)[0]<<0)|((buf)[1]<<8)|((buf)[2]<<16)|((buf)[3]<<24))
#define GetL16(buf) (((buf)[0]<<0)|((buf)[1]<<8))

#define numof(a) (sizeof(a)/sizeof((a)[0]))

enum {
	EVENT_NewDev  = 0x01,
	EVENT_CloseDev = 0x20,
	EVENT_DP_Changed = 0x02,
};

typedef struct {
	uint8_t                  dev_addr;
	usb_host_client_handle_t client_hdl;
	usb_device_handle_t      dev_hdl;
	uint32_t                 clientEvent;
	uint32_t                 ptpEvent;
} class_driver_t;
class_driver_t g_driver_obj = {0};

SemaphoreHandle_t sem_class;

usb_transfer_t *xfer_out = NULL;
usb_transfer_t *xfer_in = NULL;
usb_transfer_t *event_in = NULL;


static int updateDeviceProp(int onlyDiff);


#define GPIO_USB_DIR		4
#define GPIP_5V_LDO			36
#define GPIO_NEOPIXEL		35


//------------ usb host ------------

#define RECV_BUFFER  32768
#define BULK_IN_EP_ADDR		0x81
#define BULK_OUT_EP_ADDR	0x02
#define INT_IN_EP_ADDR		0x83

static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
{
	class_driver_t *driver_obj = (class_driver_t*)arg;
	switch(event_msg->event) {
	case USB_HOST_CLIENT_EVENT_NEW_DEV:
		if(driver_obj->dev_addr == 0) {
			driver_obj->dev_addr = event_msg->new_dev.address;
			driver_obj->clientEvent |= EVENT_NewDev;
		}
		break;
	case USB_HOST_CLIENT_EVENT_DEV_GONE:
		if(driver_obj->dev_hdl != NULL) {
			driver_obj->clientEvent |= EVENT_CloseDev;
		}
		break;
	default:
		//Should never occur
		abort();
	}
}

static void xfer_cb(usb_transfer_t *transfer)
{
	assert(transfer);
	//hid_device_t *hid_device = (hid_device_t *)transfer->context;
	xSemaphoreGive(sem_class);
}

static void event_cb(usb_transfer_t *transfer)
{
	assert(transfer);
	//hid_device_t *hid_device = (hid_device_t *)transfer->context;
	//ESP_LOG_BUFFER_HEXDUMP(TAG, event_in->data_buffer, event_in->actual_num_bytes, ESP_LOG_INFO);

	uint16_t code = GetL16(event_in->data_buffer+6);
	switch(code) {
	case PTP_SDIE_DevicePropChanged:
		g_driver_obj.ptpEvent |= EVENT_DP_Changed;
		break;
	case PTP_SDIE_ObjectAdded:
	case PTP_SDIE_ObjectRemoved:
	case PTP_SDIE_CapturedEvent:
	case PTP_SDIE_CWBCapturedResult:
	case PTP_SDIE_CameraSettingReadResult:
	case PTP_SDIE_FTPSettingReadResult:
	case PTP_SDIE_MediaFormatResult:
	case PTP_SDIE_FTPDisplayNameListChanged:
	case PTP_SDIE_ContentsTransferEvent:
	case PTP_SDIE_DisplayListChangedEvent:
	case PTP_SDIE_FocusPositionResult:
	case PTP_SDIE_LensInformationChanged:
	case PTP_SDIE_OperationResults:
	case PTP_SDIE_AFStatus:
	case PTP_SDIE_MovieRecOperationResults:
	default:
		break;
	}

	int ret = usb_host_transfer_submit(event_in);
	if(ret) ESP_LOGE(TAG, "Error(%x)", ret);
}

int usb_host_init(void)
{
	ESP_LOGI(TAG, "Installing USB Host Library");
	usb_host_config_t host_config = {
		.skip_phy_setup = false,
		.intr_flags = ESP_INTR_FLAG_LEVEL1,
	//	.enum_filter_cb = 
	};
	ESP_ERROR_CHECK(usb_host_install(&host_config));

	// 
	usb_host_client_config_t client_config = {
		.is_synchronous = false,
		.max_num_event_msg = CLIENT_NUM_EVENT_MSG,
		.async = {
			.client_event_callback = client_event_cb,
			.callback_arg = (void*)&g_driver_obj,
		},
	};
	ESP_ERROR_CHECK(usb_host_client_register(&client_config, &g_driver_obj.client_hdl));

	ESP_ERROR_CHECK(usb_host_transfer_alloc(256, 0, &xfer_out));
	xfer_out->callback = xfer_cb;
	xfer_out->context = NULL;
	xfer_out->bEndpointAddress = BULK_OUT_EP_ADDR;
	xfer_out->timeout_ms = TIMEOUT_MS;

	ESP_ERROR_CHECK(usb_host_transfer_alloc(RECV_BUFFER, 0, &xfer_in));
	xfer_in->callback = xfer_cb;
	xfer_in->context = NULL;
	xfer_in->bEndpointAddress = BULK_IN_EP_ADDR;
	xfer_in->timeout_ms = TIMEOUT_MS;

	ESP_ERROR_CHECK(usb_host_transfer_alloc(256, 0, &event_in));
	event_in->callback = event_cb;
	event_in->context = NULL;
	event_in->bEndpointAddress = INT_IN_EP_ADDR;
//	event_in->timeout_ms = TIMEOUT_MS;
	return 0;
}

int usb_host_connect(void)
{
	assert(g_driver_obj.dev_addr != 0);
	ESP_LOGI(TAG, "Opening device at address %d", g_driver_obj.dev_addr);
	ESP_ERROR_CHECK(usb_host_device_open(g_driver_obj.client_hdl, g_driver_obj.dev_addr, &g_driver_obj.dev_hdl));
	ESP_ERROR_CHECK(usb_host_interface_claim(g_driver_obj.client_hdl, g_driver_obj.dev_hdl, 0/*bInterfaceNumber*/, 0/*bAlternateSetting*/));

	xfer_out->device_handle = g_driver_obj.dev_hdl;
	xfer_in->device_handle = g_driver_obj.dev_hdl;
	event_in->device_handle = g_driver_obj.dev_hdl;

	assert(g_driver_obj.dev_hdl != NULL);
	usb_device_info_t dev_info;
	ESP_ERROR_CHECK(usb_host_device_info(g_driver_obj.dev_hdl, &dev_info));
	ESP_LOGI(TAG, "\t%s speed", (dev_info.speed == USB_SPEED_LOW) ? "Low" : "Full");
	ESP_LOGI(TAG, "\tbConfigurationValue %d", dev_info.bConfigurationValue);

	const usb_device_desc_t *dev_desc;
	ESP_ERROR_CHECK(usb_host_get_device_descriptor(g_driver_obj.dev_hdl, &dev_desc));
	usb_print_device_descriptor(dev_desc);

	const usb_config_desc_t *config_desc;
	ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(g_driver_obj.dev_hdl, &config_desc));
	usb_print_config_descriptor(config_desc, NULL);

	ESP_ERROR_CHECK(usb_host_device_info(g_driver_obj.dev_hdl, &dev_info));
	if(dev_info.str_desc_manufacturer) {
		ESP_LOGI(TAG, "Getting Manufacturer string descriptor");
		usb_print_string_descriptor(dev_info.str_desc_manufacturer);
	}
	if(dev_info.str_desc_product) {
		ESP_LOGI(TAG, "Getting Product string descriptor");
		usb_print_string_descriptor(dev_info.str_desc_product);
	}
	if(dev_info.str_desc_serial_num) {
		ESP_LOGI(TAG, "Getting Serial Number string descriptor");
		usb_print_string_descriptor(dev_info.str_desc_serial_num);
	}

	event_in->num_bytes = 256;
	int ret = usb_host_transfer_submit(event_in);
	if(ret) ESP_LOGE(TAG, "Error(%x)", ret);
	return 0;
}

static void class_driver_task(void *arg)
{
	while (1) {
		if(g_driver_obj.clientEvent == 0) {
			usb_host_client_handle_events(g_driver_obj.client_hdl, portMAX_DELAY);    // ★

		} else if(g_driver_obj.clientEvent & EVENT_NewDev) {
			g_driver_obj.clientEvent &= ~EVENT_NewDev;
			usb_host_connect();

		} else if(g_driver_obj.clientEvent & EVENT_CloseDev) {
			g_driver_obj.clientEvent &= ~EVENT_CloseDev;
			ESP_ERROR_CHECK(usb_host_device_close(g_driver_obj.client_hdl, g_driver_obj.dev_hdl));
			g_driver_obj.dev_hdl = NULL;
			g_driver_obj.dev_addr = 0;
			//We need to exit the event handler loop
			break;
		}
	}
	ESP_LOGI(TAG, "Deregistering Client");
	ESP_ERROR_CHECK(usb_host_client_deregister(g_driver_obj.client_hdl));

	xSemaphoreGive(sem_class);
	vTaskSuspend(NULL);
	return;
}

static void host_lib_daemon_task(void *arg)
{
	bool has_clients = true;
	bool has_devices = true;
	while (has_clients || has_devices) {
		uint32_t event_flags;
		ESP_ERROR_CHECK(usb_host_lib_handle_events(portMAX_DELAY, &event_flags));   // ★
		if(event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
			has_clients = false;
		}
		if(event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
			has_devices = false;
		}
	}
	ESP_LOGI(TAG, "No more clients and devices");

	ESP_ERROR_CHECK(usb_host_uninstall());
	xSemaphoreGive(sem_class);
	vTaskSuspend(NULL);
}

//-------------------- PTP ------------------

static uint32_t ptp_index = 0;

static int usb_ptp_transfer(uint32_t pcode,
							uint32_t num, uint32_t param0, uint32_t param1, uint32_t param2, uint32_t param3, uint32_t param4,
							const uint8_t* outbuf, uint32_t outsize,
							uint8_t** inbuf, uint32_t* insize)
{
	int ret;
	BaseType_t received;

	assert(g_driver_obj.dev_hdl != NULL);

	// cmd

	xfer_out->num_bytes = 12+num*4;
	SetL32(xfer_out->data_buffer+0, 12+num*4);
	SetL16(xfer_out->data_buffer+4, 1);
	SetL16(xfer_out->data_buffer+6, pcode);
	SetL32(xfer_out->data_buffer+8, ptp_index++);
	for(int i = 0; i < num; i++) {
		switch(i) {
		case 0: SetL32(xfer_out->data_buffer+12+0, param0); break;
		case 1: SetL32(xfer_out->data_buffer+12+4, param1); break;
		case 2: SetL32(xfer_out->data_buffer+12+8, param2); break;
		case 3: SetL32(xfer_out->data_buffer+12+12, param3); break;
		case 4: SetL32(xfer_out->data_buffer+12+16, param4); break;
		}
	}

	ret = usb_host_transfer_submit(xfer_out);
	if(ret) ESP_LOGE(TAG, "Error(%x)", ret);
	received = xSemaphoreTake(sem_class, pdMS_TO_TICKS(100));
	if(received != pdTRUE) {
		ESP_LOGE(TAG, "Control Transfer Timeout");
		return -1;
	}

	// data(send)

	if(outbuf) {
		vTaskDelay(2);
		xfer_out->num_bytes = 12+outsize;
		SetL32(xfer_out->data_buffer+0, 12+outsize);
		SetL16(xfer_out->data_buffer+4, 2);
		memcpy(xfer_out->data_buffer+12, outbuf, outsize);

		ret = usb_host_transfer_submit(xfer_out);
		if(ret) ESP_LOGE(TAG, "Error(%x)", ret);
		received = xSemaphoreTake(sem_class, pdMS_TO_TICKS(100));
		if(received != pdTRUE) {
			ESP_LOGE(TAG, "Control Transfer Timeout");
			return -1;
		}
	}

	xfer_in->num_bytes = RECV_BUFFER;
	ret = usb_host_transfer_submit(xfer_in);
	if(ret) ESP_LOGE(TAG, "Error(%x)", ret);
	received = xSemaphoreTake(sem_class, pdMS_TO_TICKS(100));
	if(received != pdTRUE) {
		ESP_LOGE(TAG, "Control Transfer Timeout");
		return -1;
	}

	if(GetL16(xfer_in->data_buffer+4) == 2) {
	//	ESP_LOG_BUFFER_HEXDUMP(TAG, xfer_in->data_buffer, xfer_in->actual_num_bytes, ESP_LOG_INFO);
		if(inbuf && insize) {
			if(xfer_in->actual_num_bytes > 12 && GetL16(xfer_in->data_buffer+0) == xfer_in->actual_num_bytes) {
				*insize = xfer_in->actual_num_bytes-12;
				*inbuf = (uint8_t*)malloc(*insize);
				if(!*inbuf) ESP_LOGE(TAG, "alloc error %ld", *insize);
				memcpy(*inbuf, xfer_in->data_buffer+12, *insize);
			} else {
				ESP_LOGE(TAG, "recv error");
			}
		}

		ret = usb_host_transfer_submit(xfer_in);
		if(ret) ESP_LOGE(TAG, "Error(%x)", ret);
		received = xSemaphoreTake(sem_class, pdMS_TO_TICKS(100));
		if(received != pdTRUE) {
			ESP_LOGE(TAG, "Control Transfer Timeout");
			return -1;
		}
	}
	ESP_LOG_BUFFER_HEXDUMP(TAG, xfer_in->data_buffer, xfer_in->actual_num_bytes, ESP_LOG_INFO);
	return 0;
}

//-------------------- client ------------------

int incParam(int index, int diff)
{
	if(paramTable[index].isenabled != 1) return -1;

	int currentIndex = paramTable[index].currentIndex + diff;
	if(currentIndex < 0)
		currentIndex = 0;

	if(currentIndex > paramTable[index].enumNum-1)
		currentIndex = paramTable[index].enumNum-1;

	if(paramTable[index].currentIndex == currentIndex) return 0;

	paramTable[index].currentIndex = currentIndex;

	int32_t data = paramTable[index].enums[currentIndex];
	int size = 0;
	uint8_t buf[8]; 

	switch(paramTable[index].datatype) {
	case PTP_DT_INT8:
	case PTP_DT_UINT8:
		buf[0] = data;
		size = 1;
		break;

	case PTP_DT_INT16:
	case PTP_DT_UINT16:
		SetL32(buf, data);
		size = 2;
		break;

	case PTP_DT_INT32:
	case PTP_DT_UINT32:
		SetL32(buf, data);
		size = 4;
		break;

	default:
		return -1;
	}
	return usb_ptp_transfer(PTP_OC_SDIOSetExtDevicePropValue, 1, paramTable[index].pcode,0,0,0,0, buf,size, NULL,NULL);
}

int32_t getVariableVal(int datatype, uint8_t** dpp)
{
	int32_t val = 0;
	switch(datatype) {
	case PTP_DT_INT8:	val=*( int8_t*)(*dpp); (*dpp)++; break;
	case PTP_DT_UINT8:	val=*(uint8_t*)(*dpp); (*dpp)++; break;

	case PTP_DT_INT16:	val=( int16_t)GetL16(*dpp); (*dpp)+=2; break;
	case PTP_DT_UINT16:	val=(uint16_t)GetL16(*dpp); (*dpp)+=2; break;

	case PTP_DT_INT32:	val=( int32_t)GetL32(*dpp); (*dpp)+=4; break;
	case PTP_DT_UINT32:	val=(uint32_t)GetL32(*dpp); (*dpp)+=4; break;

	case PTP_DT_INT64:	(*dpp)+=8; break;
	case PTP_DT_UINT64:	(*dpp)+=8; break;

	case PTP_DT_STR: {
		int strLen = *(uint8_t*)((*dpp)++);
		(*dpp) += strLen*2;
		break;
		}
	default:
		ESP_LOGE(TAG, "unknown datatype");
		break;
	}
	return val;
}

static int updateDeviceProp(int onlyDiff)
{
	uint8_t* recv_buf = NULL;
	uint32_t recv_size = 0;
	int ret = usb_ptp_transfer(PTP_OC_SDIOGetAllExtDevicePropInfo, 1, onlyDiff,0/*Device Property Option*/,0,0,0, NULL,0, &recv_buf,&recv_size);
	if(ret) return ret;

	if(!recv_buf || !recv_size)
		return -1;

	int propNum = GetL32(recv_buf+0);
	(void)propNum;
	uint8_t* dp = recv_buf+8;
	for(; dp < recv_buf+recv_size;) {
		uint16_t pcode    = GetL16(dp); dp+=2;
		uint16_t datatype = GetL16(dp); dp+=2;
		uint8_t getset    = *dp++;
		uint8_t isenabled = *dp++;
		int32_t factory = getVariableVal(datatype, &dp);
		(void)factory;
		int32_t current = getVariableVal(datatype, &dp);
		uint8_t formflag = *dp++;

		int i;
		int index = -1;
		for(i = 0; i < PARAM_NUM; i++) {
			if(paramTable[i].pcode == pcode) {
				paramTable[i].datatype = datatype;
				paramTable[i].getset = getset;
				paramTable[i].isenabled = isenabled;
				paramTable[i].current = current;
				paramTable[i].formflag = formflag;
				paramTable[i].currentIndex = 0;
				paramTable[i].enumNum = 0;
				if(paramTable[i].enums) {
					free(paramTable[i].enums);
					paramTable[i].enums = NULL;
				}
				index = i;
				break;
			}
		}

		switch(formflag) {
		case 0:
			break;
		case 1:		// range
			getVariableVal(datatype, &dp);
			getVariableVal(datatype, &dp);
			getVariableVal(datatype, &dp);
			break;
		case 2: {	// enum
			uint16_t num;
			int data;

			// set
			num = GetL16(dp); dp+=2;
			for(i = 0; i < num; i++)
				data = getVariableVal(datatype, &dp);

			// set/get
			num = GetL16(dp); dp+=2;
			if(index >= 0) {
				paramTable[index].enumNum = num;
				paramTable[index].enums = (int32_t*)malloc(num*sizeof(int32_t));
			}
			for(i = 0; i < num; i++) {
				data = getVariableVal(datatype, &dp);
				if(index >= 0) {
					paramTable[index].enums[i] = data;
					if(data == current)
						paramTable[index].currentIndex = i;
				}
			}
			break;
		  }
		default:
			ESP_LOGE(TAG, "Control Transfer Timeout");
			break;
		}
		if(index >= 0) {
			ESP_LOGI(TAG, "%04x, %d,%d,%d,%d, %ld,%ld", pcode, datatype,getset,isenabled,formflag, paramTable[index].currentIndex,current);
			int led = (paramTable[index].isenabled==1 ? 0x101010: 0x000000);
			if(paramTable[index].led != led) {
				paramTable[index].led = led;
				_8encoder_write(_8ENCODER_REG_RGB, index, led);
			}
		}
	}
	if(recv_buf) free(recv_buf);
	return 0;
}

void app_main(void)
{
	gpio_set_direction(GPIP_5V_LDO, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIP_5V_LDO, 1);			// ON

	usb_host_init();

	gpio_set_direction(GPIO_USB_DIR, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_USB_DIR, 0);		// USB host

	sem_class = xSemaphoreCreateBinary();

	TaskHandle_t daemon_task_hdl;
	TaskHandle_t class_driver_task_hdl;

	//Create daemon task
	xTaskCreatePinnedToCore(host_lib_daemon_task,
							"daemon",
							4096,
							(void*)sem_class,
							DAEMON_TASK_PRIORITY,
							&daemon_task_hdl,
							0);

	//Create the class driver task
	xTaskCreatePinnedToCore(class_driver_task,
							"class",
							4096,
							(void*)sem_class,
							CLASS_TASK_PRIORITY,
							&class_driver_task_hdl,
							0);

	int i;

	_8encoder_init();

	while(g_driver_obj.dev_hdl == NULL)
		vTaskDelay(100);
	vTaskDelay(50);

	// open session, connect(v3)
	usb_ptp_transfer(PTP_OC_OpenSession, 1, 1,0,0,0,0, NULL,0, NULL,NULL);
	usb_ptp_transfer(PTP_OC_SDIOConnect, 3, 1,0,0,0,0, NULL,0, NULL,NULL);
	usb_ptp_transfer(PTP_OC_SDIOConnect, 3, 2,0,0,0,0, NULL,0, NULL,NULL);
	usb_ptp_transfer(PTP_OC_SDIOGetExtDeviceInfo, 1, 0x012c,0,0,0,0, NULL,0, NULL,NULL);
	usb_ptp_transfer(PTP_OC_SDIOConnect, 3, 3,0,0,0,0, NULL,0, NULL,NULL);

	vTaskDelay(200);

	// set up parametors
	uint8_t buf[32];
	buf[0] = 0x01;			// PC Remote
	usb_ptp_transfer(PTP_OC_SDIOSetExtDevicePropValue, 1, DPC_POSITION_KEY,0,0,0,0, buf,1, NULL,NULL);

	SetL16(buf, 0x0010);	// Camera
	usb_ptp_transfer(PTP_OC_SDIOSetExtDevicePropValue, 1, DPC_SAVE_MEDIA,0,0,0,0, buf,2, NULL,NULL);

	buf[0] = 0x01;			// off
	usb_ptp_transfer(PTP_OC_SDIOSetExtDevicePropValue, 1, DPC_USB_POWER_SUPPLY,0,0,0,0, buf,1, NULL,NULL);

	vTaskDelay(100);

	updateDeviceProp(false);

	int32_t cnt_last[PARAM_NUM] = {0};
	int32_t cnt_cur[PARAM_NUM] = {0};

	int ret = 0;
	(void)ret;
	ret = _8encoder_read(_8ENCODER_REG_COUNTER, cnt_last, PARAM_NUM);
//	if(!ret) printf("%3ld,%3ld,%3ld,%3ld,\n", cnt_last[0],cnt_last[1],cnt_last[2],cnt_last[3]);//,cnt_last[4],cnt_last[5],cnt_last[6],cnt_last[7]);

//	int32_t tmp[8];
//	ret = _8encoder_read(_8ENCODER_REG_BUTTON, tmp, 8);
//	if(!ret) printf("%ld\n", tmp[0]);

	while(1) {
		ret = _8encoder_read(_8ENCODER_REG_COUNTER, cnt_cur, PARAM_NUM);
	//	if(!ret) printf("%3ld,%3ld,%3ld,%3ld,\n", cnt_cur[0],cnt_cur[1],cnt_cur[2],cnt_cur[3]);//,cnt_cur[4],cnt_cur[5],cnt_cur[6],cnt_cur[7]);

		if(g_driver_obj.ptpEvent & EVENT_DP_Changed) {
			g_driver_obj.ptpEvent &= ~EVENT_DP_Changed;
			updateDeviceProp(true);
		}

		for(i = 0; i < PARAM_NUM; i++) {
			int diff = cnt_cur[i] - cnt_last[i];
		//	if(diff < 0) diff += 1;
			diff = diff>>1;
			if(diff) {
				incParam(i, diff);
				cnt_last[i] = cnt_cur[i] & ~1;
			}
		}
		vTaskDelay(5);
	}

	vTaskDelay(200);

	usb_ptp_transfer(PTP_OC_CloseSession, 1, 1,0,0,0,0, NULL,0, NULL,NULL);

	//Wait for the tasks to complete
	for (i = 0; i < 2; i++) {
		xSemaphoreTake(sem_class, portMAX_DELAY);
	}

	_8encoder_delete();

	//Delete the tasks
	vTaskDelete(class_driver_task_hdl);
	vTaskDelete(daemon_task_hdl);
}




/*
esp_err_t usb_control_transfer(void)
{
	ESP_LOGI(TAG, "setconfig1");
	usb_transfer_t *transfer;
	
	ESP_ERROR_CHECK(usb_host_transfer_alloc(USB_SETUP_PACKET_SIZE+2, 0, &transfer));
	//ESP_ERROR_CHECK(usb_host_transfer_alloc(BULK_PACKET_SIZE, 0, &transfer));

	transfer->device_handle = g_driver_obj.dev_hdl;
	transfer->callback = xfer_cb;
	transfer->context = NULL;
	transfer->bEndpointAddress = 0;
	transfer->timeout_ms = TIMEOUT_MS;


	transfer->num_bytes = USB_SETUP_PACKET_SIZE+2;
  //uint8_t tmp[8] = {0x00,0x09,0x01,0x00,0x00,0x00,0x00,0x00};
	uint8_t tmp[8] = {0x80,0x00,0x00,0x00,0x00,0x00,0x02,0x00};
	memcpy(transfer->data_buffer, tmp, 8);

	ESP_ERROR_CHECK(usb_host_transfer_submit_control(g_driver_obj.client_hdl, transfer));

	BaseType_t received = xSemaphoreTake(sem_class, pdMS_TO_TICKS(100));
	if(received != pdTRUE) {
		ESP_LOGE(TAG, "Control Transfer Timeout");
		return ESP_ERR_TIMEOUT;
	}

	transfer->num_bytes = 8;
	uint8_t tmp2[8] = {0x00,0x09,0x01,0x00,0x00,0x00,0x00,0x00};
	memcpy(transfer->data_buffer, tmp2, 8);

	ESP_ERROR_CHECK(usb_host_transfer_submit_control(g_driver_obj.client_hdl, transfer));

	received = xSemaphoreTake(sem_class, pdMS_TO_TICKS(100));
	if(received != pdTRUE) {
		ESP_LOGE(TAG, "Control Transfer Timeout");
		return ESP_ERR_TIMEOUT;
	}
	ESP_LOGI(TAG, "setconfig3");
	usb_host_transfer_free(transfer);

	return ESP_OK;
}
*/

