// GetDP
// LED制御
// ExpProgMode改善
// 安定？
// 5V電源問題
// 

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
#include "usb/usb_host.h"

#include "PTPDef.h"
#include "_8encoder.h"

static const char *TAG = "DAEMON";


#define DAEMON_TASK_PRIORITY    2
#define CLASS_TASK_PRIORITY     3

#define BULK_PACKET_SIZE	512
#define BULK_IN_EP_ADDR		0x81
#define BULK_OUT_EP_ADDR	0x02
#define INT_IN_EP_ADDR		0x83

#define CLIENT_NUM_EVENT_MSG        5


#define PARAM_NUM  5


#define TIMEOUT_MS 100

#define SetL32(buf,data) {uint32_t d=(data);(buf)[0]=(d);(buf)[1]=(d)>>8;(buf)[2]=(d)>>16;(buf)[3]=(d)>>24;}
#define SetL16(buf,data) {uint32_t d=(data);(buf)[0]=(d);(buf)[1]=(d)>>8;}

#define GetL32(buf) (((buf)[0]<<0)|((buf)[1]<<8)|((buf)[2]<<16)|((buf)[3]<<24))
#define GetL16(buf) (((buf)[0]<<0)|((buf)[1]<<8))


enum {
	ACTION_OPEN_DEV = 0x01,
	ACTION_EVENT_DEV = 0x02,
	ACTION_CLOSE_DEV = 0x20,
};

typedef struct {
	usb_host_client_handle_t client_hdl;
	uint8_t                  dev_addr;
	usb_device_handle_t      dev_hdl;
	uint32_t                 actions;
} class_driver_t;
class_driver_t g_driver_obj = {0};

SemaphoreHandle_t sem_class;

usb_transfer_t *xfer_out = NULL;
usb_transfer_t *xfer_in = NULL;
usb_transfer_t *event_in = NULL;

static void xfer_done(usb_transfer_t *transfer)
{
	assert(transfer);
	//hid_device_t *hid_device = (hid_device_t *)transfer->context;
	xSemaphoreGive(sem_class);
}

static void event_done(usb_transfer_t *transfer)
{
	assert(transfer);
	//hid_device_t *hid_device = (hid_device_t *)transfer->context;
	int ret = usb_host_transfer_submit(event_in);
	if(ret) ESP_LOGE(TAG, "Error(%x)", ret);
	g_driver_obj.actions |= ACTION_EVENT_DEV;
}

/*
esp_err_t usb_control_transfer(void)
{
	ESP_LOGI(TAG, "setconfig1");
	usb_transfer_t *transfer;
	
	ESP_ERROR_CHECK(usb_host_transfer_alloc(USB_SETUP_PACKET_SIZE+2, 0, &transfer));
	//ESP_ERROR_CHECK(usb_host_transfer_alloc(BULK_PACKET_SIZE, 0, &transfer));

	transfer->device_handle = g_driver_obj.dev_hdl;
	transfer->callback = xfer_done;
	transfer->context = NULL;
	transfer->bEndpointAddress = 0;
	transfer->timeout_ms = TIMEOUT_MS;


	transfer->num_bytes = USB_SETUP_PACKET_SIZE+2;
  //uint8_t tmp[8] = {0x00,0x09,0x01,0x00,0x00,0x00,0x00,0x00};
	uint8_t tmp[8] = {0x80,0x00,0x00,0x00,0x00,0x00,0x02,0x00};
	memcpy(transfer->data_buffer, tmp, 8);

	ESP_ERROR_CHECK(usb_host_transfer_submit_control(g_driver_obj.client_hdl, transfer));

	BaseType_t received = xSemaphoreTake(sem_class, pdMS_TO_TICKS(100));
	if (received != pdTRUE) {
		ESP_LOGE(TAG, "Control Transfer Timeout");
		return ESP_ERR_TIMEOUT;
	}

	transfer->num_bytes = 8;
	uint8_t tmp2[8] = {0x00,0x09,0x01,0x00,0x00,0x00,0x00,0x00};
	memcpy(transfer->data_buffer, tmp2, 8);

	ESP_ERROR_CHECK(usb_host_transfer_submit_control(g_driver_obj.client_hdl, transfer));

	received = xSemaphoreTake(sem_class, pdMS_TO_TICKS(100));
	if (received != pdTRUE) {
		ESP_LOGE(TAG, "Control Transfer Timeout");
		return ESP_ERR_TIMEOUT;
	}
	ESP_LOGI(TAG, "setconfig3");
	usb_host_transfer_free(transfer);

	return ESP_OK;
}
*/

//-------------------- PTP ------------------

static uint32_t ptp_index = 0;

static void usb_ptp_transfer(uint32_t pcode, 
							uint32_t num, uint32_t param0, uint32_t param1, uint32_t param2, uint32_t param3, uint32_t param4,
							const uint8_t* outbuf, uint32_t outlen)
{
	int ret;
	BaseType_t received;

	assert(g_driver_obj.dev_hdl != NULL);
	ESP_LOGI(TAG, "bulkout");

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

	//ESP_ERROR_CHECK(usb_host_transfer_submit(xfer_out));
	ret = usb_host_transfer_submit(xfer_out);
	if(ret) ESP_LOGE(TAG, "Error(%x)", ret);
	received = xSemaphoreTake(sem_class, pdMS_TO_TICKS(100));
	if (received != pdTRUE) {
		ESP_LOGE(TAG, "Control Transfer Timeout");
		return;
	}

	if(outbuf) {
		vTaskDelay(2);
		xfer_out->num_bytes = 12+outlen;
		SetL32(xfer_out->data_buffer+0, 12+outlen);
		SetL16(xfer_out->data_buffer+4, 2);
		memcpy(xfer_out->data_buffer+12, outbuf, outlen);

		ret = usb_host_transfer_submit(xfer_out);
		if(ret) ESP_LOGE(TAG, "Error(%x)", ret);
		received = xSemaphoreTake(sem_class, pdMS_TO_TICKS(100));
		if (received != pdTRUE) {
			ESP_LOGE(TAG, "Control Transfer Timeout");
			return;
		}
	}

	xfer_in->num_bytes = 8192;
	ret = usb_host_transfer_submit(xfer_in);
	if(ret) ESP_LOGE(TAG, "Error(%x)", ret);
	received = xSemaphoreTake(sem_class, pdMS_TO_TICKS(100));
	if (received != pdTRUE) {
		ESP_LOGE(TAG, "Control Transfer Timeout");
		return;
	}
	ESP_LOG_BUFFER_HEXDUMP(TAG, xfer_in->data_buffer, xfer_in->actual_num_bytes, ESP_LOG_INFO);

	if(GetL16(xfer_in->data_buffer+4) == 2) {
		ret = usb_host_transfer_submit(xfer_in);
		if(ret) ESP_LOGE(TAG, "Error(%x)", ret);
		received = xSemaphoreTake(sem_class, pdMS_TO_TICKS(100));
		if (received != pdTRUE) {
			ESP_LOGE(TAG, "Control Transfer Timeout");
			return;
		}
		ESP_LOG_BUFFER_HEXDUMP(TAG, xfer_in->data_buffer, xfer_in->actual_num_bytes, ESP_LOG_INFO);
	}
}

//-------------------- client ------------------

static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
{
	class_driver_t *driver_obj = (class_driver_t*)arg;
	switch (event_msg->event) {
		case USB_HOST_CLIENT_EVENT_NEW_DEV:
			if (driver_obj->dev_addr == 0) {
				driver_obj->dev_addr = event_msg->new_dev.address;
				driver_obj->actions |= ACTION_OPEN_DEV;
			}
			break;

		case USB_HOST_CLIENT_EVENT_DEV_GONE:
			if (driver_obj->dev_hdl != NULL) {
				driver_obj->actions |= ACTION_CLOSE_DEV;
			}
			break;
		default:
			//Should never occur
			abort();
	}
}

static void class_driver_task(void *arg)
{
	//Wait until daemon task has installed USB Host Library
	xSemaphoreTake(sem_class, portMAX_DELAY);

	ESP_LOGI(TAG, "1.Registering Client");
	usb_host_client_config_t client_config = {
		.is_synchronous = false,
		.max_num_event_msg = CLIENT_NUM_EVENT_MSG,
		.async = {
			.client_event_callback = client_event_cb,
			.callback_arg = (void*)&g_driver_obj,
		},
	};
	ESP_ERROR_CHECK(usb_host_client_register(&client_config, &g_driver_obj.client_hdl));

	//ESP_ERROR_CHECK(usb_host_transfer_alloc(BULK_PACKET_SIZE, 0, &xfer_out));
	size_t out_worst_case_size = 256;//MAX(256, sizeof(usb_setup_packet_t));
	ESP_ERROR_CHECK(usb_host_transfer_alloc(out_worst_case_size, 0, &xfer_out));
	xfer_out->callback = xfer_done;
	xfer_out->context = NULL;//hid_device;
	xfer_out->bEndpointAddress = BULK_OUT_EP_ADDR;
	xfer_out->timeout_ms = TIMEOUT_MS;

	ESP_ERROR_CHECK(usb_host_transfer_alloc(8192, 0, &xfer_in));
	xfer_in->callback = xfer_done;
	xfer_in->context = NULL;//hid_device;
	xfer_in->bEndpointAddress = BULK_IN_EP_ADDR;
	xfer_in->timeout_ms = TIMEOUT_MS;

	ESP_ERROR_CHECK(usb_host_transfer_alloc(256, 0, &event_in));
	event_in->callback = event_done;
	event_in->context = NULL;//hid_device;
	event_in->bEndpointAddress = INT_IN_EP_ADDR;
//	event_in->timeout_ms = TIMEOUT_MS;

	while (1) {
		if(g_driver_obj.actions == 0) {
			usb_host_client_handle_events(g_driver_obj.client_hdl, portMAX_DELAY);    // ★

		} else if(g_driver_obj.actions & ACTION_OPEN_DEV) {
			g_driver_obj.actions &= ~ACTION_OPEN_DEV;
			assert(g_driver_obj.dev_addr != 0);
			ESP_LOGI(TAG, "2.Opening device at address %d", g_driver_obj.dev_addr);
			ESP_ERROR_CHECK(usb_host_device_open(g_driver_obj.client_hdl, g_driver_obj.dev_addr, &g_driver_obj.dev_hdl));
			ESP_ERROR_CHECK(usb_host_interface_claim(g_driver_obj.client_hdl, g_driver_obj.dev_hdl, 0/*bInterfaceNumber*/, 0/*bAlternateSetting*/));

			xfer_out->device_handle = g_driver_obj.dev_hdl;
			xfer_in->device_handle = g_driver_obj.dev_hdl;
			event_in->device_handle = g_driver_obj.dev_hdl;

			assert(g_driver_obj.dev_hdl != NULL);
			ESP_LOGI(TAG, "3.Getting device information");
			usb_device_info_t dev_info;
			ESP_ERROR_CHECK(usb_host_device_info(g_driver_obj.dev_hdl, &dev_info));
			ESP_LOGI(TAG, "\t%s speed", (dev_info.speed == USB_SPEED_LOW) ? "Low" : "Full");
			ESP_LOGI(TAG, "\tbConfigurationValue %d", dev_info.bConfigurationValue);
			//Todo: Print string descriptors

			ESP_LOGI(TAG, "4.Getting device descriptor");
			const usb_device_desc_t *dev_desc;
			ESP_ERROR_CHECK(usb_host_get_device_descriptor(g_driver_obj.dev_hdl, &dev_desc));
			usb_print_device_descriptor(dev_desc);

			ESP_LOGI(TAG, "5.Getting config descriptor");
			const usb_config_desc_t *config_desc;
			ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(g_driver_obj.dev_hdl, &config_desc));
			usb_print_config_descriptor(config_desc, NULL);

			ESP_ERROR_CHECK(usb_host_device_info(g_driver_obj.dev_hdl, &dev_info));
			if (dev_info.str_desc_manufacturer) {
				ESP_LOGI(TAG, "Getting Manufacturer string descriptor");
				usb_print_string_descriptor(dev_info.str_desc_manufacturer);
			}
			if (dev_info.str_desc_product) {
				ESP_LOGI(TAG, "Getting Product string descriptor");
				usb_print_string_descriptor(dev_info.str_desc_product);
			}
			if (dev_info.str_desc_serial_num) {
				ESP_LOGI(TAG, "Getting Serial Number string descriptor");
				usb_print_string_descriptor(dev_info.str_desc_serial_num);
			}

			event_in->num_bytes = 256;
			int ret = usb_host_transfer_submit(event_in);
			if(ret) ESP_LOGE(TAG, "Error(%x)", ret);
		} else if(g_driver_obj.actions & ACTION_CLOSE_DEV) {
			g_driver_obj.actions &= ~ACTION_CLOSE_DEV;
			ESP_ERROR_CHECK(usb_host_device_close(g_driver_obj.client_hdl, g_driver_obj.dev_hdl));
			g_driver_obj.dev_hdl = NULL;
			g_driver_obj.dev_addr = 0;
			//We need to exit the event handler loop
			break;
		} else if(g_driver_obj.actions & ACTION_EVENT_DEV) {
			g_driver_obj.actions &= ~ACTION_EVENT_DEV;
		}
	}
	ESP_LOGI(TAG, "Deregistering Client");
	ESP_ERROR_CHECK(usb_host_client_deregister(g_driver_obj.client_hdl));

	xSemaphoreGive(sem_class);
	vTaskSuspend(NULL);
	return;
}

//-------------------- daemon ------------------

static void host_lib_daemon_task(void *arg)
{
	ESP_LOGI(TAG, "Installing USB Host Library");
	usb_host_config_t host_config = {
		.skip_phy_setup = false,
		.intr_flags = ESP_INTR_FLAG_LEVEL1,
	};
	ESP_ERROR_CHECK(usb_host_install(&host_config));

	// daemon->class
	xSemaphoreGive(sem_class);
	vTaskDelay(10); //Short delay to let client task spin up

	bool has_clients = true;
	bool has_devices = true;
	while (has_clients || has_devices) {
		uint32_t event_flags;
		ESP_ERROR_CHECK(usb_host_lib_handle_events(portMAX_DELAY, &event_flags));   // ★
		if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
			has_clients = false;
		}
		if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
			has_devices = false;
		}
	}
	ESP_LOGI(TAG, "No more clients and devices");

	ESP_ERROR_CHECK(usb_host_uninstall());
	xSemaphoreGive(sem_class);
	vTaskSuspend(NULL);
}

#define numof(a) (sizeof(a)/sizeof((a)[0]))
const uint32_t dpc_exposure_mode[] = {
	0x00000001, 0x00010002, 0x00020003, 0x00030004, 0x00048000, 0x00078050, 0x00078051, 0x00078052, 0x00078053, 0x00098059, 0x0009805A, 0x0009805B, 0x0009805C, 
};

const uint32_t dpc_shutter_speed[] = {
	0x00000000, 0x012C000A, 0x00FA000A, 0x00C8000A, 0x0096000A, 0x0082000A, 0x0064000A, 0x0050000A, 0x003C000A, 0x0032000A, 0x0028000A, 0x0020000A, 0x0019000A, 0x0014000A, 0x0010000A, 0x000D000A, 
	0x000A000A, 0x0008000A, 0x0006000A, 0x0005000A, 0x0004000A, 0x00010003, 0x00010004, 0x00010005, 0x00010006, 0x00010008, 0x0001000A, 0x0001000D, 0x0001000F, 0x00010014, 0x00010019, 0x0001001E, 
	0x00010028, 0x00010032, 0x0001003C, 0x00010050, 0x00010064, 0x0001007D, 0x000100A0, 0x000100C8, 0x000100FA, 0x00010140, 0x00010190, 0x000101F4, 0x00010280, 0x00010320, 0x000103E8, 0x000104E2, 
	0x00010640, 0x000107D0, 0x000109C4, 0x00010C80, 0x00010FA0, 0x00011388, 0x00011900, 0x00011F40, 0x00012710, 0x00013200, 0x00013E80, 0x00017D00, 
};

const uint16_t dpc_fnumber[] = {
	0x0190, 0x01C2, 0x01F4, 0x0230, 0x0276, 0x02C6, 0x0320, 0x0384, 0x03E8, 0x044C, 0x0514, 0x0578, 0x0640, 0x0708, 0x07D0, 0x0898, 
};

const int16_t dpc_exposure_compensation[] = {
	0x1388, 0x125C, 0x10CC, 0x0FA0, 0x0E74, 0x0CE4, 0x0BB8, 0x0A8C, 0x08FC, 0x07D0, 0x06A4, 0x0514, 0x03E8, 0x02BC, 0x012C, 0x0000, 
	0xFED4, 0xFD44, 0xFC18, 0xFAEC, 0xF95C, 0xF830, 0xF704, 0xF574, 0xF448, 0xF31C, 0xF18C, 0xF060, 0xEF34, 0xEDA4, 0xEC78, 
};

const uint32_t dpc_iso[] = {
	0x00FFFFFF, 0x10000032, 0x10000040, 0x10000050, 0x00000064, 0x0000007D, 0x000000A0, 0x000000C8, 0x000000FA, 0x00000140, 0x00000190, 0x000001F4, 0x00000280, 0x00000320, 0x000003E8, 0x000004E2, 
	0x00000640, 0x000007D0, 0x000009C4, 0x00000C80, 0x00000FA0, 0x00001388, 0x00001900, 0x00001F40, 0x00002710, 0x00003200, 0x00003E80, 0x00004E20, 0x00006400, 0x00007D00, 0x00009C40, 0x0000C800, 
	0x1000FA00, 0x10013880, 0x10019000, 0x1001F400, 0x10027100, 0x10032000, 
};

/*
DPC_FNUMBER
DPC_EXPOSURE_MODE
DPC_SHUTTER_SPEED
DPC_ISO
DPC_POSITION_KEY
0x00, 0x01,
*/

int setParam(int param, int id)
{
	uint8_t buf32[4]; 
	uint8_t buf16[2]; 

	switch(param) {
	case 0:
		SetL32(buf32, dpc_exposure_mode[id]);
		usb_ptp_transfer(PTP_OC_SDIOSetExtDevicePropValue, 1, DPC_EXPOSURE_MODE,0,0,0,0, buf32,sizeof(buf32));
		break;

	case 1:
		SetL32(buf32, dpc_shutter_speed[id]);
		usb_ptp_transfer(PTP_OC_SDIOSetExtDevicePropValue, 1, DPC_SHUTTER_SPEED,0,0,0,0, buf32,sizeof(buf32));
		break;

	case 2:
		SetL16(buf16, dpc_fnumber[id]);
		usb_ptp_transfer(PTP_OC_SDIOSetExtDevicePropValue, 1, DPC_FNUMBER,0,0,0,0, buf16,sizeof(buf16));
		break;

	case 3:
		SetL16(buf16, dpc_exposure_compensation[id]);
		usb_ptp_transfer(PTP_OC_SDIOSetExtDevicePropValue, 1, DPC_EXPOSURE_COMPENSATION,0,0,0,0, buf16,sizeof(buf16));
		break;

	case 4:
		SetL32(buf32, dpc_iso[id]);
		usb_ptp_transfer(PTP_OC_SDIOSetExtDevicePropValue, 1, DPC_ISO,0,0,0,0, buf32,sizeof(buf32));
		break;
	}
	return 0;
}

void app_main(void)
{
	sem_class = xSemaphoreCreateBinary();

	ESP_LOGI(TAG, "send data %d", __LINE__);

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

	vTaskDelay(100);

	// open session, connect(v3)
	usb_ptp_transfer(PTP_OC_OpenSession, 1, 1,0,0,0,0, NULL,0);
	usb_ptp_transfer(PTP_OC_SDIOConnect, 3, 1,0,0,0,0, NULL,0);
	usb_ptp_transfer(PTP_OC_SDIOConnect, 3, 2,0,0,0,0, NULL,0);
	usb_ptp_transfer(PTP_OC_SDIOGetExtDeviceInfo, 1, 0x012c,0,0,0,0, NULL,0);
	usb_ptp_transfer(PTP_OC_SDIOConnect, 3, 3,0,0,0,0, NULL,0);

	vTaskDelay(200);

	// init-position key, 
	// 保存先=カメラ、USB給電OFF
	uint8_t buf8[1];
	buf8[0] = 0x01;
	usb_ptp_transfer(PTP_OC_SDIOSetExtDevicePropValue, 1, DPC_POSITION_KEY,0,0,0,0, buf8,sizeof(buf8));

	vTaskDelay(100);

	for(i = 0; i < PARAM_NUM; i++) {
		_8encoder_write(_8ENCODER_REG_RGB+i*3, 0x101010);
	}

	int id_cur[PARAM_NUM] = {0,36,0,15,17};
	int id_max[PARAM_NUM] = {numof(dpc_exposure_mode)-1, numof(dpc_shutter_speed)-1, numof(dpc_fnumber)-1, numof(dpc_exposure_compensation)-1, numof(dpc_iso)-1};
	int32_t cnt_last[PARAM_NUM] = {0};
	int32_t cnt_cur[PARAM_NUM] = {0};

	for(i = 0; i < PARAM_NUM; i++) {
		setParam(i, id_cur[i]);
		if(i==0) vTaskDelay(50);
	}

	int ret = 0;
	ret = _8encoder_read(_8ENCODER_REG_COUNTER, cnt_last, PARAM_NUM);
//	if(!ret) printf("%3ld,%3ld,%3ld,%3ld,\n", cnt_last[0],cnt_last[1],cnt_last[2],cnt_last[3]);//,cnt_last[4],cnt_last[5],cnt_last[6],cnt_last[7]);

//	int32_t tmp[8];
//	ret = _8encoder_read(_8ENCODER_REG_BUTTON, tmp, 8);
//	if(!ret) printf("%ld\n", tmp[0]);

	while(1) {
		ret = _8encoder_read(_8ENCODER_REG_COUNTER, cnt_cur, PARAM_NUM);
	//	if(!ret) printf("%3ld,%3ld,%3ld,%3ld,\n", cnt_cur[0],cnt_cur[1],cnt_cur[2],cnt_cur[3]);//,cnt_cur[4],cnt_cur[5],cnt_cur[6],cnt_cur[7]);

		for(i = 0; i < PARAM_NUM; i++) {
			int diff = cnt_cur[i] - cnt_last[i];
		//	if(diff < 0) diff += 1;
			diff = diff>>1;
			if(diff) {
				int new_id = id_cur[i] + diff;
				if(new_id < 0) new_id = 0;
				if(new_id > id_max[i]) new_id = id_max[i];

				if(id_cur[i] != new_id) {
printf("%ld %ld %d %d\n", cnt_last[i], cnt_cur[i], id_cur[i], new_id);
					id_cur[i] = new_id;
					setParam(i, id_cur[i]);
					if(i == 0) {
						vTaskDelay(50);
						for(int j = 1; j < PARAM_NUM; j++) {
							setParam(j, id_cur[j]);
						}
					}
				}
				cnt_last[i] = cnt_cur[i] & ~1;
			}
		}
		vTaskDelay(5);
	}

	vTaskDelay(200);

	usb_ptp_transfer(PTP_OC_CloseSession, 1, 1,0,0,0,0, NULL,0);

	//Wait for the tasks to complete
	for (i = 0; i < 2; i++) {
		xSemaphoreTake(sem_class, portMAX_DELAY);
	}

	_8encoder_delete();

	//Delete the tasks
	vTaskDelete(class_driver_task_hdl);
	vTaskDelete(daemon_task_hdl);
}

