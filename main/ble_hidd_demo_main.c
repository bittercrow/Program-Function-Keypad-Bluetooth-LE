#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
//
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
//
#include "driver/rtc_io.h"
//
#include "esp_bt.h"
#include "esp_hidd_prf_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "driver/gpio.h"`
#include "hid_dev.h"
#include "key_code_list.h"
//
#include "driver/ledc.h"                                                                          

// TAGs
#define GATTS_TAG "MY_KEYPAD"
#define HID_TAG "HID TAG"

// LEDC
#define LEDC_LS_TIMER           LEDC_TIMER_0
#define LEDC_LS_MODE            LEDC_LOW_SPEED_MODE
#define LEDC_LS_CH0_GPIO        33
#define LEDC_LS_CH0_CHANNEL     LEDC_CHANNEL_0
#define LEDC_TEST_DUTY          (4000)

// Key Pad GPIO Pins
#define GPIO_OUTPUT_IO_0    18
#define GPIO_OUTPUT_IO_1    19
#define GPIO_OUTPUT_IO_2    21
#define GPIO_OUTPUT_IO_3    22
#define GPIO_OUTPUT_IO_4    23
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1) |(1ULL<<GPIO_OUTPUT_IO_2) | (1ULL<<GPIO_OUTPUT_IO_3) | (1ULL<<GPIO_OUTPUT_IO_4))
    
#define GPIO_INPUT_IO_0     27 
#define GPIO_INPUT_IO_1     14
#define GPIO_INPUT_IO_2     12
#define GPIO_INPUT_IO_3     13
#define GPIO_INPUT_IO_4     25
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1) | (1ULL<<GPIO_INPUT_IO_2) | (1ULL<<GPIO_INPUT_IO_3) | (1ULL<<GPIO_INPUT_IO_4))
#define ESP_INTR_FLAG_DEFAULT 0

#define WAIT_TIME 100
#define GPIO_HIGH_LEVEL 1
#define GPIO_LOW_LEVEL 0
#define KEY_ROW 4
#define KEY_COL 5

// Deep sleep mode
#define SLEEP_DELAY_TIME 30 //min

//
static int scanned_output;
static int scanned_input;
static bool scan_enable = true;
static char *key_array1[KEY_ROW][KEY_COL]={
    {NULL,     "KB001 ", "KB002 ", "KB003 ", "KB004 "}, // Empty pace is Enter for Rhino.
    {"KB010 ", "KB011 ", "KB012 ", "KB013 ", "KB014 "},
    {"KB020 ", "KB021 ", "KB022 ", "KB023 ", "KB024 "},
    {"KB030 ", "KB031 ", "KB032 ", "KB033 ", "KB034 "},
};
static char *key_array2[KEY_ROW][KEY_COL]={
    {NULL,     "KB101 ", "KB102 ", "KB103 ", "KB104 "},
    {"KB110 ", "KB111 ", "KB112 ", "KB113 ", "KB114 "},
    {"KB120 ", "KB121 ", "KB122 ", "KB123 ", "KB124 "},
    {"KB130 ", "KB131 ", "KB132 ", "KB133 ", "KB134 "},
};
enum key_layout{
    ARRAY1 = 1,
    ARRAY2,
} layout = ARRAY2;
enum queue_gate{
    OPEN,
    CLOSE,
} gate = OPEN;
char *selected_array[KEY_ROW][KEY_COL];

static xQueueHandle gpio_evt_queue = NULL;
static uint16_t hid_conn_id = 0;
static bool sec_conn = false;

static TimerHandle_t xTimer;

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

const char hid_device_name[] = "MY-KEYPAD";
//const char hid_device_name_flipmouse[] = "FLipMouse";
static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x30,
    .appearance = 0x03c0,       //HID Generic, 0x03c1,       //HID KEYBOARD,
    .manufacturer_len = 0,      //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //test_manufacturer,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    /*
    FLAG
    ESP_BLE_ADV_FLAG_LIMIT_DISC         
    ESP_BLE_ADV_FLAG_GEN_DISC           
    ESP_BLE_ADV_FLAG_BREDR_NOT_SPT     
    ESP_BLE_ADV_FLAG_DMT_CONTROLLER_SPT 
    ESP_BLE_ADV_FLAG_DMT_HOST_SPT       
    ESP_BLE_ADV_FLAG_NON_LIMIT_DISC
    */
    .flag = 0x6,//=(ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT)
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x30,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/**
 * LEDC
 */

/*
* Prepare and set configuration of timers
* that will be used by LED Controller
*/
static ledc_timer_config_t ledc_timer = {
    .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
    .freq_hz = 5000,                      // frequency of PWM signal
    .speed_mode = LEDC_LS_MODE,           // timer mode
    .timer_num = LEDC_LS_TIMER            // timer index
};

/*
* Prepare individual configuration
* for each channel of LED Controller
* by selecting:
* - controller's channel number
* - output duty cycle, set initially to 0
* - GPIO number where LED is connected to
* - speed mode, either high or low
* - timer servicing selected channel
*   Note: if different channels use one timer,
*         then frequency and bit_num of these channels
*         will be the same
*/
static ledc_channel_config_t ledc_channel = {
    .channel    = LEDC_LS_CH0_CHANNEL,
    .duty       = 0,//LED off
    .gpio_num   = LEDC_LS_CH0_GPIO,
    .speed_mode = LEDC_LS_MODE,
    .timer_sel  = LEDC_LS_TIMER
};

static void led_on(void)
{
    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, LEDC_TEST_DUTY);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
}

static void led_off(void)
{
    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 0);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
}

/**
 * GPIO ISR
 */
static void switch_key_layout(uint32_t *io_num)
{
    if(gpio_get_level(*io_num)==GPIO_HIGH_LEVEL){
        gate = OPEN;
        return;
    }else if(gate == CLOSE){
        return;
    }
    switch(layout){
        case ARRAY1:
            memcpy(selected_array, key_array1, KEY_ROW * KEY_COL * sizeof(key_array1[0][0]));
            layout = ARRAY2; //next layout
            led_off();
            printf("Key_array1\n");
            break;
        case ARRAY2:
            memcpy(selected_array, key_array2, KEY_ROW * KEY_COL * sizeof(key_array2[0][0]));  
            layout = ARRAY1;  //next layout  
            led_on();                  
            printf("Key_array2\n");
            break;
    }   
    gate = CLOSE;
    return;
} 

void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void gpio_kb_task(void* arg)
{
    char *key;
    KEYMAP map;
    uint32_t io_num;
    memcpy(selected_array, key_array1, KEY_ROW * KEY_COL * sizeof(key_array1[0][0]));
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) { //the loop stop and wait for the next queue here
            if (!sec_conn) continue;
            
            switch(io_num){
                case GPIO_INPUT_IO_0:scanned_input = 0; break;
                case GPIO_INPUT_IO_1:scanned_input = 1; break;
                case GPIO_INPUT_IO_2:scanned_input = 2; break;
                case GPIO_INPUT_IO_3:scanned_input = 3; break;                
                case GPIO_INPUT_IO_4:
                    switch_key_layout(&io_num);
                    continue;   
            }

            if((gpio_get_level(io_num)==GPIO_LOW_LEVEL) & scan_enable)
            {
                scan_enable = false;
                key = selected_array[scanned_input][scanned_output];
                printf("Command -> %s \n", key);
                do{
                    map = keymap[(uint8_t)*key];
                    // void esp_hidd_send_keyboard_value(uint16_t conn_id, key_mask_t special_key_mask, uint8_t *keyboard_cmd, uint8_t num_key)
                    esp_hidd_send_keyboard_value(hid_conn_id, map.modifier, &map.usage, 1);      
                    map = keymap[0];
                    esp_hidd_send_keyboard_value(hid_conn_id, map.modifier, &map.usage, 1);
                }while(*++key);  

                xTimerReset(xTimer, 10);              
            }
            else if(gpio_get_level(io_num)==GPIO_HIGH_LEVEL){
                scan_enable = true;
            }           
        }
    }           
}

static void gpio_kb_init(void)
{
    gpio_config_t io_conf;

    // OUTPUTs
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;  // Interrupt disable     
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;

    gpio_config(&io_conf);

    // INPUTs
    io_conf.intr_type = GPIO_PIN_INTR_ANYEDGE; // Interrupt, both rising and falling edges
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
  
    gpio_config(&io_conf);


    //Create Queue and Task
    gpio_evt_queue = xQueueCreate(1, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_kb_task, "gpio_kb_task", 2048, NULL, 10, NULL);


    //ISR 
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
    gpio_isr_handler_add(GPIO_INPUT_IO_2, gpio_isr_handler, (void*) GPIO_INPUT_IO_2);
    gpio_isr_handler_add(GPIO_INPUT_IO_3, gpio_isr_handler, (void*) GPIO_INPUT_IO_3);
    gpio_isr_handler_add(GPIO_INPUT_IO_4, gpio_isr_handler, (void*) GPIO_INPUT_IO_4);

    //Key matrix scan starts
    int output_sel;
    while(1) {
        for(scanned_output=0; scanned_output<5; scanned_output++){
            output_sel = 1 << scanned_output;
            if(scan_enable){
                //scan       
                gpio_set_level(GPIO_OUTPUT_IO_0, ~output_sel&1);
                gpio_set_level(GPIO_OUTPUT_IO_1, ~(output_sel>>1)&1);
                gpio_set_level(GPIO_OUTPUT_IO_2, ~(output_sel>>2)&1);
                gpio_set_level(GPIO_OUTPUT_IO_3, ~(output_sel>>3)&1);
                gpio_set_level(GPIO_OUTPUT_IO_4, ~(output_sel>>4)&1);
                vTaskDelay(40 / portTICK_RATE_MS);
            }
        }
    }
}

/**
 * Deep Sleep Mode
 */
static void deep_sleep_init()
{
    //これ不要。 off にすると誤動作
    /**
     * ESP_PD_OPTION_OFF,  //!< Power down the power domain in sleep mode
     * ESP_PD_OPTION_ON,   //!< Keep power domain enabled during sleep mode
     * ESP_PD_OPTION_AUTO //!< Keep power domain enabled in sleep mode, if it is needed by one of the wakeup options. Otherwise power it down. 
     */
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO); 
    
    /* GPIO Pin Configuration*/
    gpio_num_t gpio_num = GPIO_INPUT_IO_4;
    rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis(gpio_num);
    rtc_gpio_pullup_en(gpio_num);
  
    /* Disconnect GPIO12 and GPIO15 to remove current drain through
     * pullup/pulldown resistors.
     * GPIO15 may be connected to ground to suppress boot messages.
     * GPIO12 may be pulled high to select flash voltage.
     */
    rtc_gpio_isolate(GPIO_NUM_12);
    rtc_gpio_isolate(GPIO_NUM_15);
    rtc_gpio_isolate(GPIO_NUM_4);//Battery checker

    esp_sleep_enable_ext0_wakeup(GPIO_INPUT_IO_4, GPIO_LOW_LEVEL);

    esp_deep_sleep_start();
}

/**
 * FreeRTOS Timer
 */
void vTimerCallback( TimerHandle_t xTimer )
{
    printf("Timer Callback: Deep Sleep starting\n");
        
    //Disable BT before deep sleep
    esp_bluedroid_disable();
    esp_bt_controller_disable();
    //Start Deep sleep
    deep_sleep_init();
}

static void dsleep_timer_init(void)
{
    xTimer = xTimerCreate
    ( 
        /* Just a text name, not used by the RTOS kernel. */
        "Timer",
        /* The timer period in ticks, must be greater than 0. */
	    (SLEEP_DELAY_TIME * 60000 / portTICK_PERIOD_MS),//pdMS_TO_TICKS(1000),
        /* The timers will auto-reload themselves when they expire. */
        pdFALSE,//pdTRUE,
        /* The ID is used to store a count of the number of times the timer has expired, which is initialised to 0. */
        (void*) 0,
        /* Each timer calls the same callback when it expires. */
        vTimerCallback
    );

    xTimerStart(xTimer, 0);//portMAX_DELAY
    printf("Timer start\n");
}

/**
 * BLE
 */
static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch(event) {
        case ESP_HIDD_EVENT_REG_FINISH: {
            if (param->init_finish.state == ESP_HIDD_INIT_OK) {
                //esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};//最初からコメントアウト
                esp_ble_gap_set_device_name(hid_device_name);
                esp_ble_gap_config_adv_data(&hidd_adv_data); 
            }
            break;
        }
        case ESP_BAT_EVENT_REG: {
            ESP_LOGE(GATTS_TAG,"%s(), ESP_BAT_EVENT_REG", __func__);
            break;
        }
        case ESP_HIDD_EVENT_DEINIT_FINISH:{
            ESP_LOGE(GATTS_TAG,"%s(), ESP_HIDD_EVENT_DEINIT_FINISH", __func__);
	        break;
        }
		case ESP_HIDD_EVENT_BLE_CONNECT: {
            hid_conn_id = param->connect.conn_id;
            sec_conn = true;
            ESP_LOGE(GATTS_TAG,"%s(), ESP_HIDD_EVENT_BLE_CONNECT", __func__);
            break;
        }
        case ESP_HIDD_EVENT_BLE_DISCONNECT: {
            sec_conn = false;
            hid_conn_id = 0;
            ESP_LOGE(GATTS_TAG,"%s(), ESP_HIDD_EVENT_BLE_DISCONNECT", __func__);
            esp_ble_gap_start_advertising(&hidd_adv_params);
            break;
        }
        case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT: {
            ESP_LOGE(GATTS_TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
            ESP_LOG_BUFFER_HEX(GATTS_TAG, param->vendor_write.data, param->vendor_write.length);
        } 
        default:{
            ESP_LOGE(GATTS_TAG,"%s(), unhandled event: %d", __func__,event);
            break;
        }
    }
    return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        for(int i = 0; i < ESP_BD_ADDR_LEN; i++) {
             ESP_LOGD(GATTS_TAG, "%x:",param->ble_security.ble_req.bd_addr[i]);
        }
        /* send the positive(true) security response to the peer device to accept the security request.
        If not accept the security request, should sent the security response with negative(false) accept value*/
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
	    break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        sec_conn = true;
        ESP_LOGE(GATTS_TAG, "staus =%s, ESP_GAP_BLE_AUTH_CMPL_EVT", 
        param->ble_security.auth_cmpl.success ? "success" : "fail");
        break;
    //unused events 
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        ESP_LOGE(GATTS_TAG, "advertising %s, ESP_GAP_BLE_ADV_START_COMPLETE_EVT", 
        (param->adv_start_cmpl.status==ESP_BT_STATUS_SUCCESS) ? "success" : "fail");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(GATTS_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
        param->update_conn_params.status,
        param->update_conn_params.min_int,
        param->update_conn_params.max_int,
        param->update_conn_params.conn_int,
        param->update_conn_params.latency,
        param->update_conn_params.timeout);
        break;    
    case ESP_GAP_BLE_PASSKEY_REQ_EVT:                           /* passkey request event */
        //esp_ble_passkey_reply(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, true, 0x00);
        ESP_LOGI(GATTS_TAG,"ESP_GAP_BLE_PASSKEY_REQ_EVT");
        break;
    case ESP_GAP_BLE_OOB_REQ_EVT:                                /* OOB request event */
        ESP_LOGI(GATTS_TAG,"ESP_GAP_BLE_OOB_REQ_EVT");
        break;
    case ESP_GAP_BLE_LOCAL_IR_EVT:                               /* BLE local IR event */
        ESP_LOGI(GATTS_TAG,"ESP_GAP_BLE_LOCAL_IR_EVT");
        break;
    case ESP_GAP_BLE_LOCAL_ER_EVT:                               /* BLE local ER event */
        ESP_LOGI(GATTS_TAG,"ESP_GAP_BLE_LOCAL_ER_EVT");
        break;
    case ESP_GAP_BLE_NC_REQ_EVT:
        ESP_LOGI(GATTS_TAG,"ESP_GAP_BLE_NC_REQ_EVT");
        break;   
    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:  ///the app will receive this evt when the IO  has Output capability and the peer device IO has Input capability.
        ///show the passkey number to the user to input it in the peer deivce.
        ESP_LOGI(GATTS_TAG,"The passkey Notify number:%d", param->ble_security.key_notif.passkey);
        break;
    case ESP_GAP_BLE_KEY_EVT:
        //shows the ble key info share with peer device to the user.
        ESP_LOGI(GATTS_TAG,"key type = %d", param->ble_security.ble_key.key_type);
        break;
    default:
        ESP_LOGW(GATTS_TAG,"unhandled event: %d",event);
        break;
    }
}

/**
 * MAIN
 */
void app_main()
{
    esp_err_t ret;
    
    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
     
    // 
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(HID_TAG, "%s initialize controller failed\n", __func__);
        return;
    }

    /*
    ESP_BT_MODE_IDLE       = 0x00,   //!< Bluetooth is not running 
    ESP_BT_MODE_BLE        = 0x01,   //!< Run BLE mode 
    ESP_BT_MODE_CLASSIC_BT = 0x02,   //!< Run Classic BT mode 
    ESP_BT_MODE_BTDM       = 0x03,   //!< Run dual mode 
    */
    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (ret) {
        ESP_LOGE(HID_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(HID_TAG,"%s init bluedroid failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(HID_TAG,"%s init bluedroid failed\n", __func__);
        return;
    }
    
    //load HID country code for locale before initialising HID  *****
    //hidd_set_countrycode(get_hid_country_code(config.locale));

    if((ret = esp_hidd_profile_init()) != ESP_OK) {
        ESP_LOGE(HID_TAG,"%s init bluedroid failed\n", __func__);
    }

    ///register the callback function to the gap module
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(HID_TAG, "gap register callback error, error code = %x", ret);
        return;
    }
    ret = esp_hidd_register_callbacks(hidd_event_callback);
    if (ret){
        ESP_LOGE(HID_TAG, "hidd register error callback, error code = %x", ret);
        return;
    }
    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;     //bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           //set the IO capability to No output No input
    //esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT;           // Passkey Request
    //esp_ble_io_cap_t iocap = ESP_IO_CAP_IO;           //
    //esp_ble_io_cap_t iocap = ESP_IO_CAP_IN;           //
    
    uint8_t key_size = 16;      //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    
    /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribut to you,
    and the response key means which key you can distribut to the Master;
    If your BLE device act as a master, the response key means you hope which types of key of the slave should distribut to you, 
    and the init key means which key you can distribut to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    // Init deep sleep timer
    dsleep_timer_init();

    // Configure LEDC
    ledc_timer_config(&ledc_timer);
    ledc_channel_config(&ledc_channel);

    // Init the gpio pin
    gpio_kb_init();

}

