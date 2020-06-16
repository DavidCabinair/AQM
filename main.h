#ifndef MAIN_H
#define MAIN_H
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include "ble_hids.h"
#include "ble_nus.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_freertos.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_timer.h"
#include "nrf_twi_mngr.h"
#include "nrf_drv_spi.h"
#include "nrf_delay.h"
#include "app_timer.h"
#include "app_uart.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_drv_clock.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_rng.h"
#include "nrf_drv_spi.h"
#include "nrf_ble_scan.h"
#include "nrf_pwr_mgmt.h"
#include "cmdlist.h"
#include "nrf_queue.h"
#include "nrf_soc.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "Time_convert.h"
//for I2C
#include "sensor_rtc_led_func.h"
//for I2C

//for encrp alograthim 
#include "uECC.h"
#include "sha256.h"
#include "aes.h"
//for encrp alograthim 
#define TWI_SCL_M           22         //I2C SCL
#define TWI_SDA_M           23         //I2C SDA

//#define DEVICE_NAME                         "Nordic_NUS"                            /**< Name of device. Will be included in the advertising data. */
//#define test_SN_number                      "2108314321"
#define DEVICE_NAME                         "CAAQM"                                  /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                   "NordicSemiconductor"                   /**< Manufacturer. Will be passed to Device Information Service. */
#define LINK_TOTAL                      NRF_SDH_BLE_PERIPHERAL_LINK_COUNT + \
                                        NRF_SDH_BLE_CENTRAL_LINK_COUNT

#define APP_BLE_OBSERVER_PRIO               3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                    300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
//#define APP_ADV_DURATION                    18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_ADV_DURATION                    0

#define NUS_SERVICE_UUID_TYPE               BLE_UUID_TYPE_VENDOR_BEGIN               /*< UUID type for the Nordic UART Service (vendor specific). */    


#define INFO_Upload_INTERVAL               2000                                     //  m_ble_upload_timer timer interval 
#define TWI_Op_INTERVAL                    2000                                     //  m_twi_op_timer timer interval                                           

#define SENSOR_CONTACT_DETECTED_INTERVAL    5000                                    //< Sensor Contact Detected toggle interval (ms). 


#define MIN_CONN_INTERVAL                   MSEC_TO_UNITS(400, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(650, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.65 second). */
//#define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(2000, UNIT_1_25_MS)  
#define SLAVE_LATENCY                       0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY      5000                                    /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY       30000                                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT        3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                      1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                      1                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                  0                                       /**< Keypress notifications not enabled. */
//#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                  /**< No I/O capabilities. */
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_DISPLAY_ONLY            /**< Display I/O capabilities. */     
#define SEC_PARAM_OOB                       0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define OSTIMER_WAIT_FOR_QUEUE              2                                       /**< Number of ticks to wait for the timer queue to be ready */





#define SAADC_OVERSAMPLE NRF_SAADC_OVERSAMPLE_4X  //Oversampling setting for the SAADC. Setting oversample to 4x This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times. Enable BURST mode to make the SAADC sample 4 times when triggering SAMPLE task once.
#define SAADC_BURST_MODE 1                        //Set to 1 to enable BURST mode, otherwise set to 0.



#define portTICK_PERIOD_MS  configTICK_RATE_HZ  // configTICK_RATE_HZ is 1024 = 1.024ms
#define portTICK_RATE_MS    portTICK_PERIOD_MS  // 1.024ms

//UART
#define RX_PIN_NUMBER  8
#define TX_PIN_NUMBER  6
#define CTS_PIN_NUMBER 7
#define RTS_PIN_NUMBER 5
#define HWFC           true
//UART

//internal flash
#define  FsstartAdd   0x75000
#define  FsendAdd     0x77fff
#define  FlasePage1   0x76000
#define  FlasePage2   0x77000
//internal flash

//hid
#define MOVEMENT_SPEED                  5                                           /**< Number of pixels by which the cursor is moved each time a button is pushed. */
#define INPUT_REPORT_COUNT              3                                           /**< Number of input reports in this application. */
#define INPUT_REP_BUTTONS_LEN           3                                           /**< Length of Mouse Input Report containing button data. */
#define INPUT_REP_MOVEMENT_LEN          3                                           /**< Length of Mouse Input Report containing movement data. */
#define INPUT_REP_MEDIA_PLAYER_LEN      1                                           /**< Length of Mouse Input Report containing media player data. */
#define INPUT_REP_BUTTONS_INDEX         0                                           /**< Index of Mouse Input Report containing button data. */
#define INPUT_REP_MOVEMENT_INDEX        1                                           /**< Index of Mouse Input Report containing movement data. */
#define INPUT_REP_MPLAYER_INDEX         2                                           /**< Index of Mouse Input Report containing media player data. */
#define INPUT_REP_REF_BUTTONS_ID        1                                           /**< Id of reference to Mouse Input Report containing button data. */
#define INPUT_REP_REF_MOVEMENT_ID       2                                           /**< Id of reference to Mouse Input Report containing movement data. */
#define INPUT_REP_REF_MPLAYER_ID        3                                           /**< Id of reference to Mouse Input Report containing media player data. */

#define BASE_USB_HID_SPEC_VERSION       0x0101                                      /**< Version number of base USB HID Specification implemented by this application. */

 /**< Random numbers buffer size. */
#define RANDOM_BUFF_SIZE    8     


//RTC
/**
 * @brief Number of RTC ticks between interrupts
 */
 // NRFX_RTC_US_TO_TICKS(us,freq) (((us) * (freq)) / 1000000U)
 //current 10ms
//#define BLINK_RTC_TICKS   (RTC_US_TO_TICKS(1000ULL, RTC_DEFAULT_CONFIG_FREQUENCY))
#define BLINK_RTC_TICKS   (RTC_US_TO_TICKS(10000ULL, RTC_DEFAULT_CONFIG_FREQUENCY))


//RTC

#define CMD_MAX_length       50
//for saadc
#define SAMPLES_IN_BUFFER     3
//for saadc

//for bluetooth passkey
#define STATIC_PASSKEY                  "111111"                        //static password
//#define STATIC_PASSKEY                 "123456"                          //static password
//for bluetooth passkey

#define UART_TX_BUF_SIZE                256                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                     /**< UART RX buffer size. */

/*encryption connect*/
static bool encription_connect_flag=false;
static uint16_t package_ID=0;
static uint8_t external_pulic_key[64]={0};
static uint8_t Rev_public_length=0;
static uint8_t AQM_public_key[64]={0};
static uint8_t AQM_private_key[32]={0};
static WORD   AQM_Key_sechedule[60]={0};
static uint8_t AQM_share_key[32]={0};
static uint8_t  signaturep[64]={0};
static uint8_t  signing_flag=false;
static uint8_t  testsign[]={0xB6,0xC0,0xE3,0xCD,0x20,0xFF,0xFC,0x2D,0x76,0x99,0x6D,0xE2,0xC2,0x8E,0xE5,0x84,0xC4,0x7D,0xBD,0x02,0xBB,0x37,0xFE,0x17,0x6B,0xDB,0xBA,0x56,0x92,0xB0,0xBC,0x26,0xFA,0x23,0x23,0x41,0xCD,0x56,0x21,0x4E,0xAA,0x3A,0x7D,0x46,0x87,0x99,0x83,0x40,0x50,0xF7,0x22,0xDF,0x78,0x7F,0x66,0x17,0x2D,0xEE,0xDC,0x5B,0xE4,0x4C,0x50,0xF8};
static uint8_t  encrypt_connect_status=0; //0: no encrypt connect 1:encypt  
//static const uint8_t public_key_sample[]={0xf8,0xe2,0xfb,0x14,0x5e,0xca,0xa8,0xc7,0xd7,0x96,0xd3,0x8b,0x88,0x49,0xc8,0x81,0x01,0x96,0x2c,0x8a,0x20,0xc3,0x45,0x22,0xc1,0xfa,0xd9,0x1a,0x9d,0xf6,0x6b,0x21,0x42,0x8a,0x3b,0x7d,0x2f,0x3b,0xf5,0x62,0xc6,0x06,0x28,0x82,0x60,0xe1,0x65,0xa0,0xb5,0x7f,0xdd,0x67,0xcb,0xc0,0xd8,0x98,0x97,0xa7,0x21,0x2d,0xdb,0x3c,0x47,0x85};
//static const uint8_t private_key_sample[]={0x71,0x56,0xD1,0x94,0xB6,0xEB,0xB9,0xCF,0xBE,0x2B,0x84,0x2E,0xC9,0x10,0xF9,0x45,0x35,0xEF,0xCA,0x7F,0x61,0xAC,0x45,0x40,0x05,0xCF,0xC8,0x29,0xA4,0xEF,0x67,0x44};
/* Dummy data to read/write to flash. */
static uint32_t Flash_W_data          = 0XFFFFFFFF;
static uint32_t Flash_R_data          = 0XFFFFFFFF;
static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);


//for UART
static uint8_t data_array[CMD_MAX_length];
//for UART 

//for spi
static uint8_t    spi_tx_buf[6];     //spi data send  using dma must setting as static
static uint8_t    spi_rx_buf[6];     //spi data receive
//for spi

//global value

//for signing data 
static uint8_t  signing_data[40]={0};
static uint8_t  signing_data_length=0;
//for signing data 

// for sec peer
static pm_peer_id_t m_peer_to_be_deleted = PM_PEER_ID_INVALID;
// for sec peer

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                   /**< BLE NUS service instance. */
BLE_HIDS_DEF(m_hids,                                                                /**< HID service instance. */
             NRF_SDH_BLE_TOTAL_LINK_COUNT,
             INPUT_REP_BUTTONS_LEN,
             INPUT_REP_MOVEMENT_LEN,
             INPUT_REP_MEDIA_PLAYER_LEN);
BLE_BAS_DEF(m_bas);                                                 /**< Battery service instance. */
BLE_HRS_DEF(m_hrs);                                                 /**< Heart rate service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
//NRF_BLE_QWR_DEF(m_qwr);                                             /**< Context for the Queued Write module.*/
NRF_BLE_QWRS_DEF(m_qwr, NRF_SDH_BLE_TOTAL_LINK_COUNT);               /**< Context for the Queued Write module.*/
NRF_BLE_SCAN_DEF(m_scan);                                            /**< Scanning Module instance. */

BLE_ADVERTISING_DEF(m_advertising);                                 /**< Advertising module instance. */

static uint16_t m_conn_handle         = BLE_CONN_HANDLE_INVALID;    /**< Handle of the first   connection. */
static uint16_t m_sec_handle          = BLE_CONN_HANDLE_INVALID;    /**< Handle of the Second  connection. */
static uint16_t m_thrid_handle        = BLE_CONN_HANDLE_INVALID;    /**< Handle of the third  connection. */
static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
//static bool     m_rr_interval_enabled = true;                       /**< Flag for enabling and disabling the registration of new RR interval measurements (the purpose of disabling this is just to test sending HRM without RR interval data. */


static bool keyexist  = false;

//instance setting 


static char const m_target_periph_name[] = "CAAQS";                                 /**< Name of the device to try to **/

static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE},
    {BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE, BLE_UUID_TYPE_BLE}
};



//@RTC instance 
static const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2); /**< Declaring an instance of nrf_drv_rtc for RTC2. */


/**
 * @brief RTC configuration
 */
static nrf_drv_rtc_config_t const m_rtc_config = NRF_DRV_RTC_DEFAULT_CONFIG;
static uint8_t RTC_TIME_FROM_APP[4]={0};


/**
 * @brief flash reading per page 0x1000
 */
static const uint32_t public_key1= 0x75000;       //public key address start
static const uint32_t public_key2= 0x75010;
static const uint32_t public_key3= 0x75020;
static const uint32_t public_key4= 0x75030;       //public key address end

static const uint32_t private_key1=0x75080;
static const uint32_t private_key2=0x75090;
static const uint32_t externalpublic_key1=0x76020; 
static const uint32_t externalpublic_key2=0x76030;
static const uint32_t externalpublic_key3=0x76040;
static const uint32_t externalpublic_key4=0x76050;
static const uint32_t share_key1 = 0x76060;
static const uint32_t share_key2 = 0x76070;

static const uint32_t ble_passkey_add1 = 0x76000;
static const uint32_t ble_passkey_add2 = 0x76010;
static const uint32_t encryp_key_add = 0x76020;
static const uint32_t serial_num = 0x77000;
static const uint32_t product_time1=0x77010;
static const uint32_t product_time2=0x77020;
static const uint32_t FWversion=0x77030;

/**
 * @brief SAADC parameter
 */
static nrf_saadc_value_t       m_buffer_pool[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t       m_ppi_channel;
static uint32_t                m_adc_evt_counter = 0;
static bool                    m_saadc_calibrate = false;  

/**
 * @brief Timer parameter
 */
static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(1);


/**
 * @brief AQS checking logic parameter
 */
static bool AQS_checking_flag = false;  // disable AQS checking: ture enable AQS checking: false 
static bool Find_AQS = false; //default is false once find AQS device  

/**
 * @brief Timer Count
 */
static uint16_t seccounter = 0;
static uint16_t mincounter = 0;
static uint16_t millisecondcnt =0;
static uint16_t secondcnt = 0;
static uint16_t testcnt = 0;

/**
 * @brief advertising start
 */
static void advertising_start(void * p_erase_bonds);

/**
 * @brief BLE connection 
 */
static bool connection_withoutbonding_flag = false;  // connnect timeout with no bonding 
static uint16_t connection_timeout_count = 0;
static bool bonding_connect_flag = false;  // the connect with bonding
static uint16_t onfly_count = 0;
//static bool sleep_flag = 0; //BLE bonding connect but no any information transfer then will go to sleep mode
static uint8_t datatransmissioncount = 0 ; 
static bool onfly_mode = 0;



/**
 * @brief data buffer   
 */
static uint8_t data_log=0;
static uint8_t log_number=0;

/**
 * @brief Receive Queue for TWI structure define
 */

/**
 * @brief Command Queue for TWI structure define
 */


/**
 * @brief Command receive from BLE or UART
 */
 
typedef struct 
{
  uint8_t    function;
  uint8_t    type;
  uint8_t    terminal;  // 0x00: init value, 0x0a from UART, 0x0b from BLE
  uint8_t   CMDdatalength;
  uint8_t    CMDdata[64];
}UART_BLE_Rev_cmd;



/*
typedef struct 
{
 struct RGB_LED     LP1;     //3
 struct RGB_LED     LP2;      //6
 struct RG_LED      LP3;    //8
 struct R_LED       LP4;     //9
 struct R_LED       LP5;      //10
 struct R_LED       LP6;      //11
 struct RGB_LED     LC1;     //14
 struct RG_LED      LC2;      //16
 struct R_LED       LC3;    //17

}UI_struct;
*/
static uint8_t LIGHT_val=0;
//For driver value

//Function
static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt);
static int GenHashKey(uint8_t * output_buff,uint8_t * source_buff, uint8_t length );
static int Sign_Registration(uint8_t * signature, uint8_t * source_buff,uint8_t length );
static int Encrypt_connect_withAES256(uint8_t * encrypt_data, uint8_t * source_buff,uint8_t *Rev_publickey );
//static int Encrypt_connect_withAES256(uint8_t encrypt_data[], uint8_t source_buff[],uint8_t *Rev_publickey);
static int key_generation();
#endif //MAIN_H