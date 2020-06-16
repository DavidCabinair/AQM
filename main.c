/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
// Board/nrf6310/ble/ble_app_hrs_rtx/main.c
/**
 *
 * @brief Heart Rate Service Sample Application with RTX main file.
 *
 * This file contains the source code for a sample application using RTX and the
 * Heart Rate service (and also Battery and Device Information services).
 * This application uses the @ref srvlib_conn_params module.
 */

#include "main.h"



NRF_QUEUE_DEF(UART_BLE_Rev_cmd, m_buf_queue, 20, NRF_QUEUE_MODE_NO_OVERFLOW);

/**@brief instance for MCU internal flash
 *
 * @param[in] event_type  p_context
 */
NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    //application range is 0x0002 0000 - 0x0007 8000 (352 kB)
    //per page size 0x1000
    //use two page
    .start_addr = FsstartAdd,
    .end_addr   = FsendAdd,
};


/**@brief instance flash evt handle
 *
 * @param[in] event_type  p_evt
 */
static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        default:
            break;
    }
}
/**@brief flash writing func
 *
 * @param[in] event_type  p_evt
 */
void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)
{
    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(p_fstorage))
    {
       // power_manage();
       (void) sd_app_evt_wait();
    }
}


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for timer_handler
 *
 * @param[in] event_type  p_context
 */
void timer_handler(nrf_timer_event_t event_type, void * p_context)
{
     NRF_LOG_INFO("timer_handler");
}



/**@brief   Function for handling app_uart events.
*/
void uart_event_handle(app_uart_evt_t * p_event)
{
     ret_code_t err_code ;
     static uint16_t index = 0;
     uint8_t num=0;
     UART_BLE_Rev_cmd Rev_cmd;
     /*
     Rev_cmd.function=fun_Error_response; //default set as error response
     Rev_cmd.type=sub_tye;
     Rev_cmd.terminal=0x0A;
     */
     switch (p_event->evt_type)
     {
        case APP_UART_DATA_READY:
             UNUSED_VARIABLE(app_uart_get(&data_array[index]));
             index++;
             if(index==8)
              {
                 if(data_array[0]==fun_Test_BIST)
                 {
                    Rev_cmd.function=fun_Test_BIST;
                    Rev_cmd.type =data_array[1];
                    Rev_cmd.CMDdatalength=6;
                    Rev_cmd.terminal=0x0A;
                /*
                  for(uint8_t i=6;i>0;i--)
                  {                   
                    Rev_cmd.CMDdata[num]=data_array[i-1];
                    num++;
                  }
                 */
                  Rev_cmd.CMDdata[0]=data_array[2];
                  Rev_cmd.CMDdata[1]=data_array[3];
                  Rev_cmd.CMDdata[2]=data_array[4];
                  Rev_cmd.CMDdata[3]=data_array[5];
                  Rev_cmd.CMDdata[4]=data_array[6];
                  Rev_cmd.CMDdata[5]=data_array[7];

                  index=0;
                 } 
                 err_code = nrf_queue_push(&m_buf_queue, &Rev_cmd);
                 APP_ERROR_CHECK(err_code);
              }
            
             else if(index==20)
             {
              if(data_array[0]==fun_set_production_data)
              {
                   Rev_cmd.function=fun_set_production_data;
                   Rev_cmd.type =data_array[1];
                   Rev_cmd.CMDdatalength=18;
                   Rev_cmd.terminal=0x0A;
                  for(uint8_t i=2; i<20;i++)
                  {
                    Rev_cmd.CMDdata[num]=data_array[i];
                    num++;
                  }
                  index=0;
              }
              err_code = nrf_queue_push(&m_buf_queue, &Rev_cmd);
              APP_ERROR_CHECK(err_code);
             }
               break;
        case APP_UART_COMMUNICATION_ERROR:
            NRF_LOG_ERROR("Communication error occurred while handling UART.");
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            NRF_LOG_ERROR("Error occurred in FIFO module used by UART.");
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
     
     }
    

}


/**@brief Function for twi_handler I2C event
 *
 * @param[in] nrf_drv_twi_evt_t
 */

static void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{

    switch (p_event->type)
    {
            case NRF_DRV_TWI_EVT_DONE:
            
              break;
        default:
            break;
    
    }

}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    bool delete_bonds = false;
    ret_code_t err_code;

    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
          pm_conn_sec_status_t conn_sec_status;
          //NRF_LOG_INFO("nPM_EVT_CONN_SEC_SUCCEEDED");
          // Check if the link is authenticated (meaning at least MITM).
          err_code = pm_conn_sec_status_get(p_evt->conn_handle, &conn_sec_status);
          APP_ERROR_CHECK(err_code);
          if (conn_sec_status.mitm_protected)
          {
            
            NRF_LOG_INFO("Link secured. Role: %d. conn_handle: %d, Procedure: %d",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
            bonding_connect_flag=true;
            connection_withoutbonding_flag=false;
            onfly_count=0;
                        
          }
          else
          {
            // The peer did not use MITM, disconnect.
            NRF_LOG_INFO("Collector did not use MITM, disconnecting");
            err_code = pm_peer_id_get(m_conn_handle, &m_peer_to_be_deleted);
            APP_ERROR_CHECK(err_code);
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
          }
        
        }break;
        
        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            NRF_LOG_INFO("Reject pairing request from already bonded");
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = true};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
           // bonding_connect_flag=true;
            connection_withoutbonding_flag=false;
            onfly_count=0;
        }break;
         case PM_EVT_CONN_SEC_FAILED:
          m_conn_handle = BLE_CONN_HANDLE_INVALID;
          break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(&delete_bonds);
            break;

        default:
            break;
    }
}


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/** @brief: Function for handling the RTC2 interrupts. 500ms tick
 * Triggered on TICK and COMPARE0 match.
 */
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{ 
    ret_code_t err_code;
  
    err_code = nrf_drv_rtc_cc_set(
        &rtc,
        0,
        (nrf_rtc_cc_get(rtc.p_reg, 0) + BLINK_RTC_TICKS) & RTC_COUNTER_COUNTER_Msk,
        true); 
    APP_ERROR_CHECK(err_code);
   // NRF_LOG_INFO("rtc_handler");

    seccounter++;
    if(seccounter>=100)
    {
      testcnt++;
      seccounter=0;
      connection_timeout_count++;
      secondcnt++;
      datatransmissioncount++;
     
    }
    mincounter++;
    if(mincounter>=6000)
    {
      mincounter=0;
      onfly_count++;
    }
    if(secondcnt>65530)
    {
      secondcnt=65530;
    }
    if(connection_timeout_count>65530)
    {
      connection_timeout_count=65530;
    }
    if(onfly_count>65530)
    {
      onfly_count=65530;
    }
    if(datatransmissioncount>65530)
    {
        datatransmissioncount=65530;
    }

   // NRF_LOG_INFO("rtc_handler");
}


/** @brief Function initialization and configuration of RTC driver instance.
 */
static void rtc_config(void)
{
    uint32_t err_code;

     //Initialize RTC instance
    err_code = nrf_drv_rtc_init(&rtc, &m_rtc_config, rtc_handler);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_rtc_cc_set(&rtc, 0, BLINK_RTC_TICKS, true);
    APP_ERROR_CHECK(err_code);
    nrf_drv_rtc_enable(&rtc);


  /*  
    //Initialize RTC instance
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = 4095;
    

    //Enable tick event & interrupt
    nrf_drv_rtc_tick_enable(&rtc,true);

    //Set compare channel to trigger interrupt after COMPARE_COUNTERTIME seconds
  //  err_code = nrf_drv_rtc_cc_set(&rtc,0,COMPARE_COUNTERTIME * 8,true);
    err_code = nrf_drv_rtc_cc_set(&rtc,0,1000 * 8,true);
    APP_ERROR_CHECK(err_code);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc);
    */
}






/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

//for NUS 
     memset(&gap_conn_params, 0, sizeof(gap_conn_params));

     gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
     gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
     gap_conn_params.slave_latency     = SLAVE_LATENCY;
     gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

     err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
     APP_ERROR_CHECK(err_code);

}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    
    ret_code_t err_code ;
    UART_BLE_Rev_cmd Rev_cmd;
    uint8_t num=0;
    uint8_t recv_length=0;
 //   Rev_cmd.function=fun_Error_response; //default set as error response
  //  Rev_cmd.type=sub_tye;
 //   Rev_cmd.terminal=0x0B;
   // uint8_t Recarry[CMD_MAX_length]={0};
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {     
      NRF_LOG_INFO("Received data from BLE NUS. Writing data on UART.");
     if(p_evt->params.rx_data.p_data[0]==fun_active_data_signing)
     {  
        Rev_cmd.function=fun_active_data_signing;
        Rev_cmd.terminal=0x0B;
        Rev_cmd.type=p_evt->params.rx_data.p_data[1];
        Rev_cmd.CMDdatalength=0;
        err_code = nrf_queue_push(&m_buf_queue, &Rev_cmd);
        APP_ERROR_CHECK(err_code);
     }
     if(p_evt->params.rx_data.length==66) //ENCRYPTED CONNECT
      {
        if(p_evt->params.rx_data.p_data[0]==fun_Open_connection)
        {
           // recv_length=p_evt->params.rx_data.length-2;
            Rev_cmd.function=fun_Open_connection;
            Rev_cmd.type =p_evt->params.rx_data.p_data[1];
            Rev_cmd.CMDdatalength=66;
            Rev_cmd.terminal=0x0B;
            Rev_public_length=66;
            
           // memset(Rev_pulic_key,0,sizeof(Rev_pulic_key));
           memcpy(Rev_cmd.CMDdata,(p_evt->params.rx_data.p_data)+2,64);

           /*
            for(uint8_t i=0; i<128;i++)
            {
               // Rev_pulic_key[i]=params.rx_data.p_data[i+2];
               Rev_cmd.CMDdata[i]=p_evt->params.rx_data.p_data[i+2];
            }
            */
         err_code = nrf_queue_push(&m_buf_queue, &Rev_cmd);
         APP_ERROR_CHECK(err_code);
                    
        }
         
      
      }
    
       if( p_evt->params.rx_data.length == 8) // BIST Test mode
       {             
           if(p_evt->params.rx_data.p_data[0]==fun_Test_BIST)
           {              
              Rev_cmd.function=fun_Test_BIST;
              Rev_cmd.type =p_evt->params.rx_data.p_data[1];
              Rev_cmd.CMDdatalength=6;
              Rev_cmd.terminal=0x0B;
              for(uint8_t i = 0; i<Rev_cmd.CMDdatalength;i++)
              {
                Rev_cmd.CMDdata[i]=p_evt->params.rx_data.p_data[i+2];
              }
              err_code = nrf_queue_push(&m_buf_queue, &Rev_cmd);
              APP_ERROR_CHECK(err_code);
           }
            
       }
       else if( p_evt->params.rx_data.length == 20) //set production data
       {
            if(p_evt->params.rx_data.p_data[0]==fun_set_production_data)
            {
               Rev_cmd.function=fun_set_production_data;
               Rev_cmd.type =p_evt->params.rx_data.p_data[1];
               Rev_cmd.CMDdatalength=18;
               Rev_cmd.terminal=0x0B;
               for(uint8_t i=0; i<20;i++)
               {
                Rev_cmd.CMDdata[i]=p_evt->params.rx_data.p_data[i+2];
               }     
               err_code = nrf_queue_push(&m_buf_queue, &Rev_cmd);
               APP_ERROR_CHECK(err_code);
            }
            
       }
      else if(p_evt->params.rx_data.length == 6) //set device time
      {     
           if(p_evt->params.rx_data.p_data[0]==fun_set_device_time)
            {
               Rev_cmd.function=fun_set_device_time;
               Rev_cmd.type =p_evt->params.rx_data.p_data[1];
               Rev_cmd.CMDdatalength=4;
               Rev_cmd.terminal=0x0B;
               for(uint8_t i=2; i<6;i++)
               {
                Rev_cmd.CMDdata[num]=p_evt->params.rx_data.p_data[i];
                num++;
               }
             err_code = nrf_queue_push(&m_buf_queue, &Rev_cmd);
             APP_ERROR_CHECK(err_code);
            }
            
      
      }
      else if(p_evt->params.rx_data.length == 6)// encrpyted connect  
      {
         if(p_evt->params.rx_data.p_data[0]==fun_Encrypted_connect)
         {
             Rev_cmd.function=fun_Encrypted_connect;
             Rev_cmd.type =p_evt->params.rx_data.p_data[1];
             err_code = nrf_queue_push(&m_buf_queue, &Rev_cmd);
             APP_ERROR_CHECK(err_code);
         }
      }
      else if(p_evt->params.rx_data.length == 3)
      {
          if(p_evt->params.rx_data.p_data[0]==fun_Data_log_request)
          {
             Rev_cmd.function=fun_Data_log_request;
             Rev_cmd.type =p_evt->params.rx_data.p_data[1];
             Rev_cmd.CMDdatalength=1;
             Rev_cmd.CMDdata[0]=p_evt->params.rx_data.p_data[2];  
             err_code = nrf_queue_push(&m_buf_queue, &Rev_cmd);
             APP_ERROR_CHECK(err_code); 
          }
           
      }
      else if(p_evt->params.rx_data.length == 2)
      {
         if(p_evt->params.rx_data.p_data[0]==fun_disconnect)// fun_disconnect
         {
             Rev_cmd.function=fun_disconnect;
             Rev_cmd.type =p_evt->params.rx_data.p_data[1];
             encription_connect_flag=false;
             err_code = nrf_queue_push(&m_buf_queue, &Rev_cmd);
             APP_ERROR_CHECK(err_code);
         }
         
         else if(p_evt->params.rx_data.p_data[0]==fun_Reg_request)//REGISTRATION
         {
            Rev_cmd.function=fun_Reg_request;
            Rev_cmd.type =p_evt->params.rx_data.p_data[1];
            Rev_cmd.CMDdatalength=0;
            Rev_cmd.terminal=0x0B;  
            err_code = nrf_queue_push(&m_buf_queue, &Rev_cmd);
           APP_ERROR_CHECK(err_code);
         }
         else if(p_evt->params.rx_data.p_data[0]==fun_request_publickey)//REQUEST PUBLIC KEY
         {
            Rev_cmd.function=fun_request_publickey;
            Rev_cmd.type =p_evt->params.rx_data.p_data[1];
            Rev_cmd.CMDdatalength=0;
            Rev_cmd.terminal=0x0B;  
            err_code = nrf_queue_push(&m_buf_queue, &Rev_cmd);
            APP_ERROR_CHECK(err_code);
         }
         /*
         else if(p_evt->params.rx_data.p_data[0]==fun_active_data_signing)
         {
            Rev_cmd.function=fun_active_data_signing;
            Rev_cmd.type =p_evt->params.rx_data.p_data[1];
            Rev_cmd.CMDdatalength=0;
            Rev_cmd.terminal=0x0B; 
         }
         */
             
      }    
       
    }
     if (p_evt->type == BLE_NUS_EVT_TX_RDY)
     {
         NRF_LOG_INFO("BLE_NUS_EVT_TX_RDY");
     }
}


/**@brief Function for handling events from the GATT library. */

void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
    
}

/**@brief Function for initializing the GATT module. */
static void gatt_init(void)
{
   // ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
    //for nus
    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID,204);
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;
     switch(p_scan_evt->scan_evt_id)
     {
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
        {
          //NRF_LOG_INFO("NRF_BLE_SCAN_EVT_CONNECTING_ERROR");
        }break;
        case NRF_BLE_SCAN_EVT_FILTER_MATCH:
        {
         // NRF_LOG_INFO("NRF_BLE_SCAN_EVT_FILTER_MATCH")
          Find_AQS=true;
        }break;
        
        case NRF_BLE_SCAN_EVT_NOT_FOUND:
        {
         // NRF_LOG_INFO("NRF_BLE_SCAN_EVT_NOT_FOUND")
        }break;
        
       default:
         break;
     }
}

/**@brief Function for starting scanning. */
static void scan_start(void)
{
    ret_code_t ret;
    
    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);

    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ret);
}


/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;
    memset(&init_scan, 0, sizeof(init_scan));
    
   // init_scan.connect_if_match = true;
    init_scan.connect_if_match = false;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_ble_scan_filter_set(&m_scan,SCAN_NAME_FILTER,m_target_periph_name); 
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
    APP_ERROR_CHECK(err_code);

     // err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_nus_uuid);
    //  APP_ERROR_CHECK(err_code);

    //  err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
    //  APP_ERROR_CHECK(err_code);

    //  err_code = nrf_ble_scan_filter_set(&m_scan,SCAN_ADDR_FILTER,test_periph_addr); 
    //  APP_ERROR_CHECK(err_code);

   //  err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_ADDR_FILTER, false);
   //  APP_ERROR_CHECK(err_code);

}


/**@brief Function for initializing HID Service.
 */
static void hids_init(void)
{
   ret_code_t                err_code;
   ble_hids_init_t           hids_init_obj;
   ble_hids_inp_rep_init_t * p_input_report;
   uint8_t                   hid_info_flags;
   static ble_hids_inp_rep_init_t inp_rep_array[INPUT_REPORT_COUNT];
   static uint8_t rep_map_data[] ={0x00,0x00};
    memset(inp_rep_array, 0, sizeof(inp_rep_array));
     // Initialize HID Service.
    p_input_report                      = &inp_rep_array[INPUT_REP_BUTTONS_INDEX];
    p_input_report->max_len             = INPUT_REP_BUTTONS_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_BUTTONS_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    p_input_report->sec.cccd_wr = SEC_JUST_WORKS;
    p_input_report->sec.wr      = SEC_JUST_WORKS;
    p_input_report->sec.rd      = SEC_JUST_WORKS;

    p_input_report                      = &inp_rep_array[INPUT_REP_MOVEMENT_INDEX];
    p_input_report->max_len             = INPUT_REP_MOVEMENT_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_MOVEMENT_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    p_input_report->sec.cccd_wr = SEC_JUST_WORKS;
    p_input_report->sec.wr      = SEC_JUST_WORKS;
    p_input_report->sec.rd      = SEC_JUST_WORKS;

    p_input_report                      = &inp_rep_array[INPUT_REP_MPLAYER_INDEX];
    p_input_report->max_len             = INPUT_REP_MEDIA_PLAYER_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_MPLAYER_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    p_input_report->sec.cccd_wr = SEC_JUST_WORKS;
    p_input_report->sec.wr      = SEC_JUST_WORKS;
    p_input_report->sec.rd      = SEC_JUST_WORKS;

    hid_info_flags = HID_INFO_FLAG_REMOTE_WAKE_MSK | HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK;

    memset(&hids_init_obj, 0, sizeof(hids_init_obj));

    hids_init_obj.evt_handler                    = on_hids_evt;
    hids_init_obj.error_handler                  = service_error_handler;
    hids_init_obj.is_kb                          = false;
    hids_init_obj.is_mouse                       = true;
    hids_init_obj.inp_rep_count                  = INPUT_REPORT_COUNT;
    hids_init_obj.p_inp_rep_array                = inp_rep_array;
    hids_init_obj.outp_rep_count                 = 0;
    hids_init_obj.p_outp_rep_array               = NULL;
    hids_init_obj.feature_rep_count              = 0;
    hids_init_obj.p_feature_rep_array            = NULL;
    hids_init_obj.rep_map.data_len               = sizeof(rep_map_data);
    hids_init_obj.rep_map.p_data                 = rep_map_data;
    hids_init_obj.hid_information.bcd_hid        = BASE_USB_HID_SPEC_VERSION;
    hids_init_obj.hid_information.b_country_code = 0;
    hids_init_obj.hid_information.flags          = hid_info_flags;
    hids_init_obj.included_services_count        = 0;
    hids_init_obj.p_included_services_array      = NULL;

    hids_init_obj.rep_map.rd_sec         = SEC_JUST_WORKS;
    hids_init_obj.hid_information.rd_sec = SEC_JUST_WORKS;

    hids_init_obj.boot_mouse_inp_rep_sec.cccd_wr = SEC_JUST_WORKS;
    hids_init_obj.boot_mouse_inp_rep_sec.wr      = SEC_JUST_WORKS;
    hids_init_obj.boot_mouse_inp_rep_sec.rd      = SEC_JUST_WORKS;

    hids_init_obj.protocol_mode_rd_sec = SEC_JUST_WORKS;
    hids_init_obj.protocol_mode_wr_sec = SEC_JUST_WORKS;
    hids_init_obj.ctrl_point_wr_sec    = SEC_JUST_WORKS;

    err_code = ble_hids_init(&m_hids, &hids_init_obj);
    APP_ERROR_CHECK(err_code);
 
}

/**@brief Function for handling HID events.
 *
 * @details This function will be called for all HID events which are passed to the application.
 *
 * @param[in]   p_hids  HID service structure.
 * @param[in]   p_evt   Event received from the HID service.
 */
static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt)
{
     switch (p_evt->evt_type)
     {
         default:
            // No implementation needed.
            break;
     }
}

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};
    uint8_t            body_sensor_location;

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

  /*
    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);
*/
    for (uint32_t i = 0; i < LINK_TOTAL; i++)
    {
        err_code = nrf_ble_qwr_init(&m_qwr[i], &qwr_init);
        APP_ERROR_CHECK(err_code);
    }

    

    //Initialize hid serivce
    hids_init();

    // Initialize nus serivce 
    nus_init.data_handler = nus_data_handler;
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}





/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module. */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));
    //NUS
    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}



/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
              NRF_LOG_INFO("Connected");
              connection_timeout_count=0;
              connection_withoutbonding_flag=true;
              m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
              err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
              APP_ERROR_CHECK(err_code);
            break;
        case BLE_GAP_EVT_DISCONNECTED:
             connection_withoutbonding_flag=false;
             bonding_connect_flag=false;
             NRF_LOG_INFO("Connection 0x%x has been disconnected. Reason: 0x%X",
                 p_ble_evt->evt.gap_evt.conn_handle,
                 p_ble_evt->evt.gap_evt.params.disconnected.reason);
              m_conn_handle = BLE_CONN_HANDLE_INVALID;
              if (m_peer_to_be_deleted != PM_PEER_ID_INVALID)
              {
                err_code = pm_peer_delete(m_peer_to_be_deleted);
                APP_ERROR_CHECK(err_code);
                NRF_LOG_DEBUG("Collector's bond deleted");
                m_peer_to_be_deleted = PM_PEER_ID_INVALID;
              }
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
       case BLE_GAP_EVT_AUTH_KEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
            break;

        case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
            break;

         case BLE_GAP_EVT_AUTH_STATUS:
            
             NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                          p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                          p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
              bonding_connect_flag=true;
            // connection_flag=true;
            break;
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}

/**@brief Function for the GPIO initialization. */
static void GPIO_init(void)
{

}


/**@brief Function for the Peer Manager initialization. */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage. */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality. */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;
   
 
    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
   // init.advdata.include_appearance = true;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    //init.advdata.flags              =BLE_GAP_ADV_FLAG_LE_BR_EDR_HOST;
    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);

    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV,m_advertising.adv_handle ,-8);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@callback Function for saadc.
 */
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
   // NRF_LOG_INFO("saadc_callback");
    ret_code_t err_code ;
    /*
    Recmsg saadcmsg;
    if(p_event->type == NRF_DRV_SAADC_EVT_DONE) 
    {
       err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
       APP_ERROR_CHECK(err_code);
       saadcmsg.item=0xA2;  
       saadcmsg.address=0x00;
       saadcmsg.data[0]=(p_event->data.done.p_buffer[0])<<8;
       saadcmsg.data[1]=(p_event->data.done.p_buffer[0]);
    
    }
*/
}
/**@brief Function for initializing saac_init. the saac using oversample brust mode
 *
 * @param[out] none
 */
void saadc_init(void)
{
   ret_code_t err_code;
   nrf_saadc_channel_config_t channel_config1 = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN4); // for testing 
 //  nrf_saadc_channel_config_t channel_config2 = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN6); //for AQM


   err_code = nrf_drv_saadc_init(NULL, saadc_callback);
   APP_ERROR_CHECK(err_code);

   err_code = nrf_drv_saadc_channel_init(4, &channel_config1);
   APP_ERROR_CHECK(err_code); 
    /*
   err_code = nrf_drv_saadc_channel_init(6, &channel_config1);
   APP_ERROR_CHECK(err_code);  
    */

   err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
   APP_ERROR_CHECK(err_code);

   err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
   APP_ERROR_CHECK(err_code);


    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

     //ppi config
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);
    
    /* setup m_timer for compare event every 400ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer,400);

     nrf_drv_timer_extended_compare(&m_timer,
                                     NRF_TIMER_CC_CHANNEL0,
                                     ticks,
                                     NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                     false);
     nrf_drv_timer_enable(&m_timer);

     uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
                                                                                  NRF_TIMER_CC_CHANNEL0);
     uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

     /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
     err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
     APP_ERROR_CHECK(err_code);

     err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
                                            timer_compare_event_addr,
                                            saadc_sample_task_addr);
     APP_ERROR_CHECK(err_code);
     saadc_sampling_event_enable();  // saadc enable


}

/**
 * @brief saadc_ppi enable function.
 *
 * @param[in] void  
 */

void saadc_sampling_event_enable(void)
{  
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the uart init 
 */
static void UART_init(void)
{
     uint32_t err_code;
     app_uart_comm_params_t comm_params =
     {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
       // .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud250000
     };
      APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
      APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/** @brief Function for getting vector of random numbers.
 *
 * @param[out] p_buff       Pointer to unit8_t buffer for storing the bytes.
 * @param[in]  length       Number of bytes to take from pool and place in p_buff.
 *
 * @retval     Number of bytes actually placed in p_buff.
 */
static uint8_t random_vector_generate(uint8_t * p_buff, uint8_t size)
{
       uint32_t err_code;
       uint8_t  available;

       nrf_drv_rng_bytes_available(&available);
       uint8_t length = MIN(size, available);

       err_code = nrf_drv_rng_rand(p_buff, length);
       APP_ERROR_CHECK(err_code);

}

/** @brief Function for getting vector of random numbers by uECC
 *
 * @param[out] p_buff       Pointer to unit8_t buffer for storing the bytes.
 * @param[in]  length       Number of bytes to take from pool and place in p_buff.
 *
 * @retval     Number of bytes actually placed in p_buff.
 */
static int default_RNG(uint8_t *p_dest, unsigned p_size)
{
   uint8_t key[1]={0};
   while(p_size) {
   random_vector_generate(key,1);
   long v =key[0]; 
   unsigned l_amount = MIN(p_size, sizeof(long));
   memcpy(p_dest, &v, l_amount);
   p_size -= l_amount;
   p_dest += l_amount;
   }
   return 1;
}

/**@brief Function for static passkey.
 */
 static void set_static_passkey()
{
    static ble_opt_t    m_static_pin_option;
    uint32_t            err_code;
    uint8_t             passkey1[4] = {0};
    uint8_t             passkey2[4] = {0};
    uint8_t             readpasskey[8] = {0};
    uint32_t            passkey_val;
  //  uint8_t testString[]= test_SN_number;
    err_code=nrf_fstorage_read(&fstorage, ble_passkey_add1, passkey1, 4);
    APP_ERROR_CHECK(err_code);
    
    err_code=nrf_fstorage_read(&fstorage, ble_passkey_add2, passkey2, 4);
    APP_ERROR_CHECK(err_code);
   
    passkey_val=passkey1[3]<<24;
    passkey_val=passkey_val+(passkey1[2]<<16);
    passkey_val=passkey_val+(passkey1[1]<<8);
    passkey_val=passkey_val+passkey1[0];
    
    for(uint8_t i=0;i<8; i++)
    {
      if(i<4)
      {
         readpasskey[i]=passkey1[i];
      }
      else
      {
        readpasskey[i]=passkey2[i-4];
      }
    }

    if(passkey_val  == 0xffffffff)  //first version
    {
      uint8_t passkey[] = STATIC_PASSKEY;
      m_static_pin_option.gap_opt.passkey.p_passkey = &passkey[0];
      keyexist=false;
      err_code =  sd_ble_opt_set(BLE_GAP_OPT_PASSKEY, &m_static_pin_option);
      APP_ERROR_CHECK(err_code);
    }
    else
    {
      // uint8_t passkey[] =passkey_val;
     
      m_static_pin_option.gap_opt.passkey.p_passkey = &readpasskey[0];
      keyexist=true;
      err_code =  sd_ble_opt_set(BLE_GAP_OPT_PASSKEY, &m_static_pin_option);
      APP_ERROR_CHECK(err_code);
    }
    
}

/**@brief Function for starting advertising & scan. */
static void adv_scan_start(void)
{
    ret_code_t err_code;
   
    //check if there are no flash operations in progress
    if (!nrf_fstorage_is_busy(NULL))
    {
        // Start scanning for peripherals and initiate connection to devices which
        // advertise Heart Rate or Running speed and cadence UUIDs.        
        // Start advertising.
        err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for starting advertising. */
static void advertising_start(void * p_erase_bonds)
{
    bool erase_bonds = *(bool*)p_erase_bonds;

    if (erase_bonds)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for get mac address.
 */
 static void getMACaddress()
 {
      ble_gap_addr_t MAC_address;
      sd_ble_gap_addr_get(&MAC_address);

      MAC_address.addr[5];
      MAC_address.addr[4];
      MAC_address.addr[3];
      MAC_address.addr[2];
      MAC_address.addr[1];
      MAC_address.addr[0];
 //     NRF_LOG_INFO("\r\MAC\r\n");
 }
/**@brief Function for BLE connection status check
 */
static void BLE_connection_check()
{
  ret_code_t err_code;
  if(connection_withoutbonding_flag==true)
  {
      if((connection_timeout_count>50)&&(bonding_connect_flag==false)) //connect without binding over than 50 sec then disconnect
      {
           if(m_conn_handle==NULL)
           {
              err_code=sd_ble_gap_disconnect(m_conn_handle,BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
              APP_ERROR_CHECK(err_code);
              NRF_LOG_INFO("connection_without_bonding");
           }
           
      }
      
  }
  else
  {
    if(bonding_connect_flag==true)
    {
      if(onfly_count>=5) //the App connect to AQM larger than 5minute and no any communication AQM go to sleep mode
       {
          onfly_mode==true;
       //   NRF_LOG_INFO("onfly_mode"); //onfly mode
       }
      else
      {
          onfly_mode=false;
         //  NRF_LOG_INFO("no_onfly_mode");
      }
    }
  }

}

/**@brief Function for flash operation & log read 
 */
static void flash_log_check(void)
{
   ret_code_t err_code;
 
   uint16_t Sendbackmsglength=0;
   uint32_t Timecounting=0;
   uint8_t RTC_count[4]={0};
   
  if( data_log != 0 )
  {         
            uint8_t Sendmsg[300]={0};
            Sendbackmsglength=3+(data_log*19);           
            Sendmsg[0]=0x02;
            Sendmsg[1]=0x01;
            Sendmsg[2]=data_log; 
            Timecounting=RTC_TIME_FROM_APP[3]<<24;
            Timecounting=Timecounting+(RTC_TIME_FROM_APP[2]<<16);
            Timecounting=Timecounting+(RTC_TIME_FROM_APP[1]<<8);
            Timecounting=Timecounting+RTC_TIME_FROM_APP[0];
            for(uint8_t i= 0; i<data_log; i++)
            {             
              Timecounting=Timecounting+i;
               RTC_count[0]=Timecounting>>24;
               RTC_count[1]=Timecounting>>16;
               RTC_count[2]=Timecounting>>8;
               RTC_count[3]=Timecounting;    
                Sendmsg[3+(i*19)]=RTC_count[0];//Time stamp start UNIX Time 1590980121
                Sendmsg[4+(i*19)]=RTC_count[1];//
                Sendmsg[5+(i*19)]=RTC_count[2];
                Sendmsg[6+(i*19)]=RTC_count[3];  
                Sendmsg[7+(i*19)]=0x03;//PM1 start 1000
                Sendmsg[8+(i*19)]=0xE8;//PM1 end
                Sendmsg[9+(i*19)]=0x01;//PM2.5 start 500
                Sendmsg[10+(i*19)]=0xF4;//PM2.5 end
                Sendmsg[11+(i*19)]=0x01;//PM10 start 300
                Sendmsg[12+(i*19)]=0x2c;//PM10 end 
                Sendmsg[13+(i*19)]=0x01;//C02 start 410
                Sendmsg[14+(i*19)]=0x9A;//C02 end 
                Sendmsg[15+(i*19)]=0x00;//tVOC start level2
                Sendmsg[16+(i*19)]=0x02;//tVOC end 
                Sendmsg[17+(i*19)]=0x00;//Temp start 25 
                Sendmsg[18+(i*19)]=0x19;//Temp end 
                Sendmsg[19+(i*19)]=0x00;//Humidity  start 50%
                Sendmsg[20+(i*19)]=0x32;//Humidity end
                Sendmsg[21+(i*19)]=0x0A;//Light 10
            }
           
           do 
           {
              err_code = ble_nus_data_send(&m_nus,Sendmsg,&Sendbackmsglength,m_conn_handle);
              if ((err_code != NRF_ERROR_INVALID_STATE) &&
                  (err_code != NRF_ERROR_RESOURCES) &&
                  (err_code != NRF_ERROR_NOT_FOUND))
               {
                  APP_ERROR_CHECK(err_code);
               }
          }while (err_code == NRF_ERROR_RESOURCES);
        data_log=0;
       
     // data_log=0;
  }

}
/**@brief Function for BLE connection command read
 */
static void BLE_cmd_read_send()
{
  UART_BLE_Rev_cmd  Revcmd;
  ret_code_t err_code;
  uint8_t Sendmsg[80]={0};
  uint8_t passkey1[4]={0};
  uint8_t passkey2[4]={0};
 // uint8_t testval[58]={0};
  uint16_t Sendbackmsglength=0;
  uint8_t connect_Termnial=0;
  bool sendackflag=false; 
  if(!nrf_queue_is_empty(&m_buf_queue))
  {
     err_code = nrf_queue_pop(&m_buf_queue, &Revcmd);
     APP_ERROR_CHECK(err_code);
     onfly_count=0;
     switch(Revcmd.function)
      {        
              case fun_Open_connection:
                   if(Revcmd.type=sub_type_from_user)
                   {
                      uint8_t readbuff[16]={0};
                      uint8_t SNnumber[10]={0};
                      uint8_t encrpy_data[10]={0};
                      connect_Termnial=Revcmd.terminal;
                      Sendbackmsglength=12;
                      Sendmsg[0]=0x01;
                      Sendmsg[1]=0x10;
                      err_code=nrf_fstorage_read(&fstorage, serial_num, &readbuff, 16);
                      APP_ERROR_CHECK(err_code);
                      memcpy(external_pulic_key,Revcmd.CMDdata,64);
                      memcpy(SNnumber,readbuff,10);        
                      Encrypt_connect_withAES256(encrpy_data,SNnumber,external_pulic_key);                     
                      memcpy(Sendmsg+2,encrpy_data,10);
                   }
              break;
              case fun_active_data_signing:
                   if(Revcmd.type==sub_tye)
                   {
                     uint8_t encrpy_data[4]={0};
                     Sendbackmsglength=6;
                     connect_Termnial=Revcmd.terminal;
                     Sendmsg[0]=0x01;
                     Sendmsg[1]=0x10;
                     Encrypt_connect_withAES256(encrpy_data,RTC_TIME_FROM_APP,AQM_public_key);
                     memcpy(Sendmsg+2,encrpy_data,4);
                   }
                   break;
              case fun_Reg_request:
                   if(Revcmd.type==sub_tye)
                    { 
                      Sendmsg[0]=fun_Reg_request;
                      Sendmsg[1]=0x02;
                      Sendbackmsglength=66;
                      uint8_t SNnumber[16]={0};
                      connect_Termnial=Revcmd.terminal;
                      /* for static test sign
                      for(uint8_t i=0;i<64;i++)
                      {
                        Sendmsg[i+2]=testsign[i];
                      }
                      */
                      err_code=nrf_fstorage_read(&fstorage, serial_num, &SNnumber, 16);
                      APP_ERROR_CHECK(err_code);
                      Sign_Registration(signaturep,SNnumber,10);
                      encription_connect_flag=true;
                      memcpy(Sendmsg+2,signaturep,64);

                    }
                  break;
              case fun_set_production_data: // SN number 10byte TIME 32byte FWversion 4byte head 2byte
                   if(Revcmd.type==sub_tye) 
                   {
                       uint8_t SNnumber[16]={0};
                       uint8_t TimeData[4]={0};
                       uint8_t FW[4]={0};
                       uint8_t flashreadmsg[4]={0};
                       uint8_t i=0;
                       connect_Termnial=Revcmd.terminal;
                       for(i=0; i<10; i++)
                       {
                          SNnumber[i]=Revcmd.CMDdata[i];
                       }
                       for(i=0;i<4;i++)
                       {
                          TimeData[i]=Revcmd.CMDdata[i+10];
                       }
                       for(i=0;i<4;i++)
                       {
                          FW[i]=Revcmd.CMDdata[i+14];
                       }
                      err_code=nrf_fstorage_read(&fstorage, serial_num, &flashreadmsg, 4);
                      APP_ERROR_CHECK(err_code);
                      if((flashreadmsg[0]==0xff)&&(flashreadmsg[1]==0xff)&&(flashreadmsg[2]==0xff)&&(flashreadmsg[3]==0xff))
                      {
                          err_code=nrf_fstorage_write(&fstorage, serial_num, &SNnumber, 16, NULL);   
                          wait_for_flash_ready(&fstorage);
                          err_code=err_code+(nrf_fstorage_write(&fstorage, product_time1, &TimeData, 4, NULL)); 
                          wait_for_flash_ready(&fstorage);
                          err_code=err_code+(nrf_fstorage_write(&fstorage, FWversion, &FW, 4, NULL));
                          wait_for_flash_ready(&fstorage);
                           if(err_code == 0)//writing ok 
                           {
                              Sendbackmsglength=20;
                              Sendmsg[0]=0x2F;
                              Sendmsg[1]=sub_type_from_user;
                              for(i=0;i<10;i++)
                              {
                                Sendmsg[i+2]=SNnumber[i];
                              }
                              for(i=0;i<4;i++)
                              {
                                Sendmsg[i+12]=TimeData[i];
                              }
                              for(i=0;i<4;i++)
                              {
                                Sendmsg[i+16]=FW[i];
                              }  
                           }
                           
                           else
                           {
                             Sendbackmsglength=2;
                             Sendmsg[0]=0XFF;
                             Sendmsg[1]=msg_unexpect;
                           }
                           
                      }
                      else 
                      {     
                            Sendbackmsglength=2;
                            Sendmsg[0]=0XFF;
                            Sendmsg[1]=msg_unexpect;
                      
                      }                     
                   }
                   break;
              case fun_set_device_time:
                    if(Revcmd.type==sub_tye) 
                    {
                      //set_rtc_time()
                      Sendbackmsglength=2;
                      connect_Termnial=Revcmd.terminal;                      
                      memcpy(&RTC_TIME_FROM_APP,&(Revcmd.CMDdata),4);
                      Sendmsg[0]=0X55;
                      Sendmsg[1]=sub_tye;
                    }
                   break;
              case fun_Test_BIST:
                     if(Revcmd.type==Gen_BLE_key && keyexist==false)
                     {
                          Sendbackmsglength=2;
                          connect_Termnial=Revcmd.terminal;
                          passkey1[0]=Revcmd.CMDdata[0];
                          passkey1[1]=Revcmd.CMDdata[1];
                          passkey1[2]=Revcmd.CMDdata[2];
                          passkey1[3]=Revcmd.CMDdata[3];
                          passkey2[0]=Revcmd.CMDdata[4];
                          passkey2[1]=Revcmd.CMDdata[5];
                          err_code=nrf_fstorage_write(&fstorage, ble_passkey_add1, &passkey1, 4, NULL);
                          wait_for_flash_ready(&fstorage);
                          err_code=err_code+(nrf_fstorage_write(&fstorage, ble_passkey_add2, &passkey2, 4, NULL));
                          wait_for_flash_ready(&fstorage);
                          if(err_code == 0)//writing ok 
                          {
                            Sendmsg[0]=0X55;
                            Sendmsg[1]=Gen_BLE_key;
                          
                          }
                          else
                          {
                            Sendmsg[0]=0XFF;
                            Sendmsg[1]=msg_unexpect;
                          }
                 
                     }
                     else if(Revcmd.type==tst_sensor_tst)
                     {
                        Sendbackmsglength=22;
                        connect_Termnial=Revcmd.terminal;
                        Sendmsg[0]=0x22;
                        Sendmsg[1]=0x01;
                        Sendmsg[2]=0x00;//Status
                        Sendmsg[3]=RTC_TIME_FROM_APP[0];//Time stamp start UNIX Time 1590980121
                        Sendmsg[4]=RTC_TIME_FROM_APP[1];
                        Sendmsg[5]=RTC_TIME_FROM_APP[2];
                        Sendmsg[6]=RTC_TIME_FROM_APP[3];//Time stamp end
                        Sendmsg[7]=0x03;//PM1 start 1000
                        Sendmsg[8]=0xE8;//PM1 end
                        Sendmsg[9]=0x01;//PM2.5 start 500
                        Sendmsg[10]=0xF4;//PM2.5 end
                        Sendmsg[11]=0x01;//PM10 start 300
                        Sendmsg[12]=0x2c;//PM10 end 
                        Sendmsg[13]=0x01;//C02 start 410
                        Sendmsg[14]=0x9A;//C02 end 
                        Sendmsg[15]=0x00;//tVOC start level2
                        Sendmsg[16]=0x02;//tVOC end 
                        Sendmsg[17]=0x00;//Temp start 25 
                        Sendmsg[18]=0x19;//Temp end 
                        Sendmsg[19]=0x00;//Humidity  start 50%
                        Sendmsg[20]=0x32;//Humidity end
                        Sendmsg[21]=0x0A;//Light 10
                     }
                     else if(Revcmd.type==tst_control_led)
                     {  
                        Sendbackmsglength=2;
                        connect_Termnial=Revcmd.terminal;
                        Sendmsg[0]=0X55;
                        Sendmsg[1]=tst_control_led;

                     }
                     else if(Revcmd.type==tst_RTC_tst)
                     {
                        Sendbackmsglength=2;
                        connect_Termnial=Revcmd.terminal;
                        Sendmsg[0]=0X55;
                        Sendmsg[1]=tst_RTC_tst;

                     }
                     else if(Revcmd.type==Gen_enc_key)
                     {
                       uint8_t buffer[16]={0};
                       //connect_Termnial=0x0A;
                       err_code=key_generation();
                       if(err_code==0)
                       {
                        Sendbackmsglength=2;
                        connect_Termnial=Revcmd.terminal;
                        Sendmsg[0]=0X55;
                        Sendmsg[1]=Mode_operation;
                       }
                       else
                       {
                        Sendbackmsglength=2;
                        connect_Termnial=Revcmd.terminal;
                        Sendmsg[0]=0XFF;
                        Sendmsg[1]=msg_unexpect;
                       }
                                          
                     }
                     else if (Revcmd.type==Mode_operation)
                     {
                        Sendbackmsglength=2;
                        connect_Termnial=Revcmd.terminal;
                        Sendmsg[0]=0X55;
                        Sendmsg[1]=Mode_operation;
                     }
                     else if (Revcmd.type==tst_flash_tst)
                     {
                        Sendbackmsglength=2;
                        connect_Termnial=Revcmd.terminal;
                        Sendmsg[0]=0X55;
                        Sendmsg[1]=tst_flash_tst;

                     }
                     
                     else
                     {
                         Sendbackmsglength=2;
                         connect_Termnial=Revcmd.terminal;
                         Sendmsg[0]=0XFF;
                         Sendmsg[1]=msg_unexpect;
                     }
                     
              break;
              case fun_Data_log_request:
                    if(Revcmd.type==sub_tye)
                    {
                      connect_Termnial=0x0c; // no need to send 
                      data_log=Revcmd.CMDdata[0];
                      log_number=0;
                    }
              break;
              case fun_request_publickey:
                    if(Revcmd.type==sub_tye)
                    {
                       Sendbackmsglength=66;
                       Sendmsg[0]=0xcc;
                       Sendmsg[1]=0x01;
                       connect_Termnial=Revcmd.terminal; 
                       memcpy(Sendmsg+2,AQM_public_key,64);
                    }
              break;
              case fun_disconnect:
                   encription_connect_flag=false;
                   package_ID=0;
              break;
              default:
              break;            
           }
           if(connect_Termnial==0x0A)
           { 
              for(uint8_t i = 0; i < Sendbackmsglength; i++)
              {
                  do
                  {
                      err_code = app_uart_put(Sendmsg[i]);
                      if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                      {
                          NRF_LOG_ERROR("app_uart_put failed for index 0x%04x.", i);
                          APP_ERROR_CHECK(err_code);
                      }
                  } while (err_code == NRF_ERROR_BUSY);
              }
              if(sendackflag==true)
              {
                  Sendmsg[0]=0x55;
                  Sendmsg[1]=0x01;
                  for(uint8_t i = 0; i < 2; i++)
                  {
                      do
                      {
                          err_code = app_uart_put(Sendmsg[i]);
                          if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                          {
                              NRF_LOG_ERROR("app_uart_put failed for index 0x%04x.", i);
                              APP_ERROR_CHECK(err_code);
                          }
                      } while (err_code == NRF_ERROR_BUSY);
                      sendackflag=false;
                  }

              }
           }
           else if(connect_Termnial==0x0B)
           {   
               do 
               {
                  err_code = ble_nus_data_send(&m_nus,Sendmsg,&Sendbackmsglength,m_conn_handle);
                  if ((err_code != NRF_ERROR_INVALID_STATE) &&
                      (err_code != NRF_ERROR_RESOURCES) &&
                      (err_code != NRF_ERROR_NOT_FOUND))
                   {
                      APP_ERROR_CHECK(err_code);
                   }
              }while (err_code == NRF_ERROR_RESOURCES);

              if(sendackflag==true)
              {
                Sendmsg[0]=0x55;
                Sendmsg[1]=0x01;
                Sendbackmsglength=2;
                do 
                 {
                    err_code = ble_nus_data_send(&m_nus,Sendmsg,&Sendbackmsglength,m_conn_handle);
                    if ((err_code != NRF_ERROR_INVALID_STATE) &&
                        (err_code != NRF_ERROR_RESOURCES) &&
                        (err_code != NRF_ERROR_NOT_FOUND))
                     {
                        APP_ERROR_CHECK(err_code);
                     }
                }while (err_code == NRF_ERROR_RESOURCES);
                sendackflag=false;
              }
           }   
  }
  if((datatransmissioncount>=1)&&(onfly_mode==false)&&(bonding_connect_flag==true)) //datat transmission each 1 sec
  {
       NRF_LOG_INFO("data transimission");
      datatransmissioncount=0;
      Sendbackmsglength=25;
      if(package_ID==65535)
      {
        package_ID=0;
      }
      Sendmsg[0]=0x00;
      Sendmsg[1]=(package_ID>>8);
      Sendmsg[2]=package_ID;
      Sendmsg[3]=0x00;//status
      Sendmsg[4]=0x5E;//Time stamp start UNIX Time 1590980121
      Sendmsg[5]=0xD4;
      Sendmsg[6]=0x6E;
      Sendmsg[7]=0x19;//Time stamp end
      Sendmsg[8]=0x03;//PM1 start 1000
      Sendmsg[9]=0xE8;//PM1 end
      Sendmsg[10]=0x01;//PM2.5 start 500
      Sendmsg[11]=0xF4;//PM2.5 end
      Sendmsg[12]=0x01;//PM10 start 300
      Sendmsg[13]=0x2c;//PM10 end 
      Sendmsg[14]=0x01;//C02 start 410
      Sendmsg[15]=0x9A;//C02 end 
      Sendmsg[16]=0x00;//tVOC start level2
      Sendmsg[17]=0x02;//tVOC end 
      Sendmsg[18]=0x00;//Temp start 25 
      Sendmsg[19]=0x19;//Temp end 
      Sendmsg[20]=0x00;//Humidity  start 50%
      Sendmsg[21]=0x32;//Humidity end
      Sendmsg[22]=0x0A;//Light 10
      Sendmsg[23]=0xF0;//Device id1
      Sendmsg[24]=0x0D;//Device id2
      package_ID++;
       do 
       {
          err_code = ble_nus_data_send(&m_nus,Sendmsg,&Sendbackmsglength,m_conn_handle);
          if ((err_code != NRF_ERROR_INVALID_STATE) &&
              (err_code != NRF_ERROR_RESOURCES) &&
              (err_code != NRF_ERROR_NOT_FOUND))
           {
              APP_ERROR_CHECK(err_code);
           }
      }while (err_code == NRF_ERROR_RESOURCES);
  
  }
  else if(datatransmissioncount>=1) 
  {
      datatransmissioncount=0;
  }

}
static void test_send()
{
   ret_code_t err_code;
   uint8_t Sendmsg[3]={0};
   uint16_t Sendbackmsglength=0;
   Sendbackmsglength=3;
   if(testcnt==2)
   {  testcnt=0;
      if(package_ID==65535)
      {
         package_ID=0;
      }
      Sendmsg[0]=0xff;
      Sendmsg[1]=(package_ID>>8);
      Sendmsg[2]=package_ID;
      package_ID++;
       do 
       {
          err_code = ble_nus_data_send(&m_nus,Sendmsg,&Sendbackmsglength,m_conn_handle);
          if ((err_code != NRF_ERROR_INVALID_STATE) &&
              (err_code != NRF_ERROR_RESOURCES) &&
              (err_code != NRF_ERROR_NOT_FOUND))
           {
              APP_ERROR_CHECK(err_code);
           }
      }while (err_code == NRF_ERROR_RESOURCES);
   }
}


/**@brief Function for initializing the random number.
 */
static void rng_init(void)
{
  uint32_t err_code;
  err_code = nrf_drv_rng_init(NULL);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing fucntion check.
 */
static void init_fuction_check()
{
  ret_code_t err_code;
  uint8_t readbuf[16] = {0};
  uint8_t readprivatebuf[16]={0};

  //get the private key & public_key
  err_code=nrf_fstorage_read(&fstorage, private_key1, &readprivatebuf, 16);
  APP_ERROR_CHECK(err_code);
  err_code=nrf_fstorage_read(&fstorage, public_key1, &readbuf, 16);
  APP_ERROR_CHECK(err_code);
  if((readprivatebuf[0]!=0xff)&&(readprivatebuf[1]!=0xff)&&(readprivatebuf[2]!=0xff)&&(readprivatebuf[3]!=0xff)) //if private key not empty
  {
      memcpy(AQM_private_key,readprivatebuf,16);
      err_code=nrf_fstorage_read(&fstorage, private_key2, &readprivatebuf, 16);
      APP_ERROR_CHECK(err_code);
      memcpy(AQM_private_key+16,readprivatebuf,16);
     
  }
  if((readbuf[0]!=0xff)&&(readbuf[1]!=0xff)&&(readbuf[2]!=0xff)&&(readbuf[3]!=0xff))
  {
       memcpy(AQM_public_key,readbuf,16);
       err_code=nrf_fstorage_read(&fstorage, public_key2, &readbuf, 16);
       APP_ERROR_CHECK(err_code); 
       memcpy(AQM_public_key+16,readbuf,16);
       err_code=nrf_fstorage_read(&fstorage, public_key3, &readbuf, 16);
       APP_ERROR_CHECK(err_code);
       memcpy(AQM_public_key+32,readbuf,16);
       err_code=nrf_fstorage_read(&fstorage, public_key4, &readbuf, 16);
       APP_ERROR_CHECK(err_code);
       memcpy(AQM_public_key+48,readbuf,16);
  }


}
static int Encrypt_connect_withAES256(uint8_t * encrypt_data, uint8_t * source_buff,uint8_t *Rev_publickey )
{
    int i, c;
    uint8_t secret_key[32];
    uECC_set_rng(&default_RNG);
    const struct uECC_Curve_t * curves[5];
    int num_curves = 0;

    #if uECC_SUPPORTS_secp160r1
    curves[num_curves++] = uECC_secp160r1();
#endif
#if uECC_SUPPORTS_secp192r1
    curves[num_curves++] = uECC_secp192r1();
#endif
#if uECC_SUPPORTS_secp224r1
    curves[num_curves++] = uECC_secp224r1();
#endif
#if uECC_SUPPORTS_secp256r1
    curves[num_curves++] = uECC_secp256r1();
#endif

#if uECC_SUPPORTS_secp256k1
    curves[num_curves++] = uECC_secp256k1();
#endif

  if (!uECC_shared_secret(Rev_publickey, AQM_private_key, secret_key, curves[4])) {
                printf("uECC_secret() AQM_test_common failed\n");
                return 1;
     }
    //aes encrption 
    aes_key_setup(secret_key, AQM_Key_sechedule, 256);
    for(int idx = 0; idx < 255; idx++)
    {
      //aes_encrypt(&source_buff,&encrypt_data,&AQM_Key_sechedule,256);
      aes_encrypt(source_buff,encrypt_data,AQM_Key_sechedule,256);
    }
     return 0;
}

static int Sign_Registration(uint8_t * signature, uint8_t * source_buff,uint8_t length )
{   
    int i, c;
    uint8_t hash[32] = {0};
    SHA256_CTX sha;
    uECC_set_rng(&default_RNG);
    const struct uECC_Curve_t * curves[5];
    int num_curves = 0;

    #if uECC_SUPPORTS_secp256k1
    curves[num_curves++] = uECC_secp256k1();
    #endif
    sha256_init(&sha);
    sha256_update(&sha, source_buff, length);
    sha256_final(&sha, hash);

    if (!uECC_sign(AQM_private_key, hash, sizeof(hash), signature, curves[c])) 
     {
         printf("uECC_sign() failed\n");
         return 1;
      }

    
  // if (!uECC_make_key(public, private, curves[c])) {
  /*
   if (!uECC_make_key(AQM_public_key, AQM_private_key, curves[c])) {
          printf("uECC_make_key() AQM_test_hash failed\n");
          return 1;
    }
   
    for (c = 0; c < num_curves; ++c) {
        for (i = 0; i < 256; ++i) {
            printf(".");
            fflush(stdout);			
			sha256_init(&sha);
			sha256_update(&sha, source_buff, length);
                        sha256_final(&sha, hash);
        }
        printf("\n");
    }
     
     for (c = 0; c < num_curves; ++c) {
        for (i = 0; i < 256; ++i) {
            printf(".");
            fflush(stdout);

            if (!uECC_sign(AQM_private_key, hash, sizeof(hash), signature, curves[c])) {
                printf("uECC_sign() failed\n");
                return 1;
            }

        }
        printf("\n");
    }   
      */
   return 0;
}

static int GenHashKey(uint8_t * output_buff,uint8_t * source_buff, uint8_t length )
{
    int i, c;
    ret_code_t       err_code;  
    uint8_t hash[32] = {0};
    // sha256
    SHA256_CTX sha;
    uECC_set_rng(&default_RNG);
    const struct uECC_Curve_t * curves[5];
    int num_curves = 0;

    #if uECC_SUPPORTS_secp256k1
    curves[num_curves++] = uECC_secp256k1();
    #endif

   // if (!uECC_make_key(public, private, curves[c])) {
    if (!uECC_make_key(AQM_public_key, AQM_private_key, curves[c])) 
    {
            printf("uECC_make_key() AQM_test_hash failed\n");
            return 1;
    }
     
      for (c = 0; c < num_curves; ++c) {
        for (i = 0; i < 256; ++i) {
            printf(".");
            fflush(stdout);			
			sha256_init(&sha);
                        sha256_update(&sha, source_buff, length);
                        sha256_final(&sha, output_buff);
        }
        printf("\n");
    }
    
}
static int key_generation()
{
    int i, c;
    ret_code_t       err_code;
    uint8_t storage_arry[16]={0};
    uint8_t storage_arry1[16]={0};
    uint8_t SNnumber[16]={0};
    uECC_set_rng(&default_RNG);
    const struct uECC_Curve_t * curves[5];  
    memset(AQM_private_key,0,32);
    memset(AQM_public_key,0,64);
    int num_curves = 0;

    #if uECC_SUPPORTS_secp256k1
    curves[num_curves++] = uECC_secp256k1();
    #endif
   
    if (!uECC_make_key(AQM_public_key, AQM_private_key, curves[c])) 
    {
          printf("uECC_make_key() AQM_key_generation failed\n");
          return 1;
    }
   /* 
     printf("Testing 256 key generations\n");
     for (c = 0; c < num_curves; ++c) {
      for (i = 0; i < 256; ++i) {
          printf(".");
          fflush(stdout);

         // if (!uECC_make_key(public, private, curves[c])) {
          if (!uECC_make_key(AQM_public_key, AQM_private_key, curves[c])) {
              printf("uECC_make_key() AQM_key_generation failed\n");
              return 1;
          }
      }
    
      printf("\n");
  }*/

   err_code = nrf_fstorage_erase(&fstorage,public_key1,1,NULL);
   wait_for_flash_ready(&fstorage);
   APP_ERROR_CHECK(err_code);
   //private key
   memcpy(storage_arry,AQM_private_key,16);
   err_code=nrf_fstorage_write(&fstorage, private_key1, &storage_arry, 16, NULL);
   wait_for_flash_ready(&fstorage);
   APP_ERROR_CHECK(err_code);
   memcpy(storage_arry,AQM_private_key+16,16);
   err_code=nrf_fstorage_write(&fstorage, private_key2, &storage_arry, 16, NULL);
   wait_for_flash_ready(&fstorage);
   APP_ERROR_CHECK(err_code);
   //public key
   memcpy(storage_arry,AQM_public_key,16);
   err_code=nrf_fstorage_write(&fstorage, public_key1, &storage_arry, 16, NULL);
   wait_for_flash_ready(&fstorage);
   APP_ERROR_CHECK(err_code);
   
   memcpy(storage_arry,AQM_public_key+16,16);
   err_code=nrf_fstorage_write(&fstorage, public_key2, &storage_arry, 16, NULL);
   wait_for_flash_ready(&fstorage);
   APP_ERROR_CHECK(err_code);

   memcpy(storage_arry,AQM_public_key+32,16);
   err_code=nrf_fstorage_write(&fstorage, public_key3, &storage_arry, 16, NULL);
   wait_for_flash_ready(&fstorage);
   APP_ERROR_CHECK(err_code);

   memcpy(storage_arry,AQM_public_key+48,16);
   err_code=nrf_fstorage_write(&fstorage, public_key4, &storage_arry, 16, NULL);
   wait_for_flash_ready(&fstorage);
   APP_ERROR_CHECK(err_code);

 return 0;
}


void test_aes256()
{
	uint8_t i;
	
	WORD key_schedule[60], idx;
	uint8_t enc_buf[16];
	uint8_t dec_buf[16];
	uint8_t plaintext[] = {0x6b,0xc1,0xbe,0xe2,0x2e,0x40,0x9f,0x96,0xe9,0x3d,0x7e,0x11,0x73,0x93,0x17,0x2a};
	uint8_t ciphertext[] = {0x59,0x1c,0xcb,0x10,0xd4,0x10,0xed,0x26,0xdc,0x5b,0xa7,0x4a,0x31,0x36,0x28,0x70};
	uint8_t key[] = {0x55,0x3d,0xeb,0x10,0x15,0xca,0x71,0xbe,0x2b,0x73,0xae,0xf0,0x85,0x7d,0x77,0x81,0x1f,0x35,0x2c,0x07,0x3b,0x61,0x08,0xd7,0x2d,0x98,0x10,0xa3,0x09,0x14,0xdf,0xf4};
	
	aes_key_setup(key, key_schedule, 256);
	
	for(idx = 0; idx < 255; idx++)
	{
		aes_encrypt(&plaintext, &enc_buf, &key_schedule, 256);
		aes_decrypt(&enc_buf, &dec_buf, &key_schedule, 256);
	}


	
	printf("\ncipher = ");
	
	for(i = 0; i < 16; i++)
	{
		printf("%02x",enc_buf[i]);
	}
	
	printf("\ndecrypt = ");
	
	for(i = 0; i < 16; i++)
	{
		printf("%02x",dec_buf[i]);
	}
	
	printf("\nkey = ");
	
	for(i = 0; i < 32; i++)
	{
		printf("%02x",key[i]);
	}
	
	printf("\n");
	
	
}


int main(void)
{
    bool erase_bonds;
    ret_code_t       err_code;
    nrf_fstorage_api_t * p_fs_api;
    p_fs_api = &nrf_fstorage_sd;
    // Initialize modules.
    log_init();
    rng_init();
    
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);

    //Configure and initialize the flash storage
    err_code = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
    APP_ERROR_CHECK(err_code);

    // Configure and initialize the BLE stack.
    ble_stack_init();
    
    // Initialize modules.
    timers_init();
    rtc_config();
  //  buttons_leds_init(&erase_bonds);
    
    //setup the static passkey 
    set_static_passkey();
    /* Configure LED-pins as outputs */
    bsp_board_init(BSP_INIT_LEDS);

    gap_params_init();
    gatt_init();
    saadc_init();
    services_init();
    advertising_init();
    conn_params_init();
    peer_manager_init();
    getMACaddress(); 
 //   UART_init();
    twi_init();
    scan_init(); 
    NRF_LOG_INFO("AQM started.");
    scan_start();
    secondcnt=0;
    if(AQS_checking_flag==true)
    {
     ret_code_t err_code;
     UART_BLE_Rev_cmd  Revcmd;
        while(1)
        {
          if(!nrf_queue_is_empty(&m_buf_queue))
          {
              err_code = nrf_queue_pop(&m_buf_queue, &Revcmd);
              APP_ERROR_CHECK(err_code);
              if(Revcmd.function==fun_Test_BIST)
              {
                if(Revcmd.type==disable_AQS_Check)
                {
                    break;
                }
              }
          }
          if(secondcnt<30)  //small than 30 sec
          {
              if(Find_AQS==true)
              {
                break;
              }
          }
          else if(secondcnt>30) // larger than 30sec only can be waky up by UART cmd
          {         
               nrf_ble_scan_stop();
          }
        }
    }
    nrf_ble_scan_stop();
    adv_scan_start();
    rtc_time_t testTime;
    uint32_t UnixTimsStamp=1590641957;
    covUnixTimeStp2Beijing(UnixTimsStamp, &testTime);
    testcnt=0;
  //  test_aes256();
    //for init the value 
    datatransmissioncount=0;
    data_log=0;
    //init function
    init_fuction_check();
    while (1)
    {
      BLE_cmd_read_send();
   //   test_send();
      BLE_connection_check();
      flash_log_check();
      idle_state_handle();
      NRF_LOG_FLUSH();
    }
   
}