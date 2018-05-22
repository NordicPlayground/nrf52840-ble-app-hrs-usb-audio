#include "application_usb_audio.h"
#include "nrf_error.h"
#include "app_usbd.h"
#include "app_usbd_core.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_audio.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/**
 * @brief USB audio samples size
 */
#define BUFFER_SIZE  (48)

/**
 * @brief Enable power USB detection
 *
 * Configure if example supports USB port connection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif

/**
 * @brief Audio class user event handler
 */
static void hp_audio_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                     app_usbd_audio_user_event_t   event);
static void mic_audio_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                      app_usbd_audio_user_event_t   event);

/* Channels and feature controls configuration */

/**
 * @brief   Input terminal channel configuration
 */
#define HP_TERMINAL_CH_CONFIG()                                                                       \
        (APP_USBD_AUDIO_IN_TERM_CH_CONFIG_LEFT_FRONT | APP_USBD_AUDIO_IN_TERM_CH_CONFIG_RIGHT_FRONT)

/**
 * @brief   Feature controls
 *
 *      general
 *      channel 0
 *      channel 1
 */
#define HP_FEATURE_CONTROLS()                                               \
        APP_USBD_U16_TO_RAW_DSC(APP_USBD_AUDIO_FEATURE_UNIT_CONTROL_MUTE),  \
        APP_USBD_U16_TO_RAW_DSC(APP_USBD_AUDIO_FEATURE_UNIT_CONTROL_MUTE),  \
        APP_USBD_U16_TO_RAW_DSC(APP_USBD_AUDIO_FEATURE_UNIT_CONTROL_MUTE)



/**
 * @brief   Input terminal channel configuration
 */
#define MIC_TERMINAL_CH_CONFIG()                                                                       \
        (APP_USBD_AUDIO_IN_TERM_CH_CONFIG_LEFT_FRONT | APP_USBD_AUDIO_IN_TERM_CH_CONFIG_RIGHT_FRONT)

/**
 * @brief   Feature controls
 *
 *      general
 *      channel 0
 *      channel 1
 */
#define MIC_FEATURE_CONTROLS()                                                                     \
        APP_USBD_U16_TO_RAW_DSC(APP_USBD_AUDIO_FEATURE_UNIT_CONTROL_MUTE),                         \
        APP_USBD_U16_TO_RAW_DSC(APP_USBD_AUDIO_FEATURE_UNIT_CONTROL_MUTE),                         \
        APP_USBD_U16_TO_RAW_DSC(APP_USBD_AUDIO_FEATURE_UNIT_CONTROL_MUTE)


/* Microphone descriptors */

/**
 * @brief   Audio class specific format descriptor
 */
APP_USBD_AUDIO_FORMAT_DESCRIPTOR(mic_form_desc, 
                                 APP_USBD_AUDIO_AS_FORMAT_I_DSC(    /* Format type 1 descriptor */
                                    2,                              /* Number of channels */
                                    2,                              /* Subframe size */
                                    16,                             /* Bit resolution */
                                    1,                              /* Frequency type */
                                    APP_USBD_U24_TO_RAW_DSC(48000)) /* Frequency */
                                );

/**
 * @brief   Audio class input terminal descriptor
 */
APP_USBD_AUDIO_INPUT_DESCRIPTOR(mic_inp_desc, 
                                APP_USBD_AUDIO_INPUT_TERMINAL_DSC(
                                    1,                                     /* Terminal ID */
                                    APP_USBD_AUDIO_TERMINAL_IN_MICROPHONE, /* Terminal type */
                                    2,                                     /* Number of channels */
                                    MIC_TERMINAL_CH_CONFIG())              /* Channels config */
                                );

/**
 * @brief   Audio class output terminal descriptor
 */
APP_USBD_AUDIO_OUTPUT_DESCRIPTOR(mic_out_desc, 
                                 APP_USBD_AUDIO_OUTPUT_TERMINAL_DSC(
                                    3,                                     /* Terminal ID */
                                    APP_USBD_AUDIO_TERMINAL_USB_STREAMING, /* Terminal type */
                                    2)                                     /* Source ID */
                                );

/**
 * @brief   Audio class feature unit descriptor
 */
APP_USBD_AUDIO_FEATURE_DESCRIPTOR(mic_fea_desc, 
                                  APP_USBD_AUDIO_FEATURE_UNIT_DSC(
                                    2,                      /* Unit ID */
                                    1,                      /* Source ID */
                                    MIC_FEATURE_CONTROLS()) /* List of controls */
                                 );

/* Headphones descriptors */

/**
 * @brief   Audio class specific format III descriptor
 */
APP_USBD_AUDIO_FORMAT_DESCRIPTOR(hp_form_desc, 
                                    APP_USBD_AUDIO_AS_FORMAT_III_DSC( /* Format type 3 descriptor */
                                    2,                                /* Number of channels */
                                    2,                                /* Subframe size */
                                    16,                               /* Bit resolution */
                                    1,                                /* Frequency type */
                                    APP_USBD_U24_TO_RAW_DSC(48000))   /* Frequency */
                                );

/**
 * @brief   Audio class input terminal descriptor
 */
APP_USBD_AUDIO_INPUT_DESCRIPTOR(hp_inp_desc, 
                                APP_USBD_AUDIO_INPUT_TERMINAL_DSC(
                                    1,                                     /* Terminal ID */
                                    APP_USBD_AUDIO_TERMINAL_USB_STREAMING, /* Terminal type */
                                    2,                                     /* Number of channels */
                                    HP_TERMINAL_CH_CONFIG())               /* Channels config */
                               );

/**
 * @brief   Audio class output terminal descriptor
 */
APP_USBD_AUDIO_OUTPUT_DESCRIPTOR(hp_out_desc, 
                                 APP_USBD_AUDIO_OUTPUT_TERMINAL_DSC(
                                    3,                                      /* Terminal ID */
                                    APP_USBD_AUDIO_TERMINAL_OUT_HEADPHONES, /* Terminal type */
                                    2)                                      /* Source ID */
                                );

/**
 * @brief   Audio class feature unit descriptor
 */
APP_USBD_AUDIO_FEATURE_DESCRIPTOR(hp_fea_desc, 
                                  APP_USBD_AUDIO_FEATURE_UNIT_DSC(
                                    2,                     /* Unit ID */
                                    1,                     /* Source ID */
                                    HP_FEATURE_CONTROLS()) /* List of controls */
                                 );

/* Interfaces lists */

/**
 * @brief Interfaces list passed to @ref APP_USBD_AUDIO_GLOBAL_DEF
 */
#define HP_INTERFACES_CONFIG() APP_USBD_AUDIO_CONFIG_OUT(0, 1)

/**
 * @brief Interfaces list passed to @ref APP_USBD_AUDIO_GLOBAL_DEF
 */
#define MIC_INTERFACES_CONFIG() APP_USBD_AUDIO_CONFIG_IN(2, 3)

/*lint -save -e26 -e64 -e123 -e505 -e651*/


/**
 * @brief Headphone Audio class instance
 */
APP_USBD_AUDIO_GLOBAL_DEF(m_app_audio_headphone,
                          HP_INTERFACES_CONFIG(),
                          hp_audio_user_ev_handler,
                          &hp_form_desc,
                          &hp_inp_desc,
                          &hp_out_desc,
                          &hp_fea_desc,
                          0,
                          APP_USBD_AUDIO_AS_IFACE_FORMAT_PCM,
                          192,
                          APP_USBD_AUDIO_SUBCLASS_AUDIOSTREAMING
);



/**
 * @brief Microphone Audio class instance
 */
APP_USBD_AUDIO_GLOBAL_DEF(m_app_audio_microphone,
                          MIC_INTERFACES_CONFIG(),
                          mic_audio_user_ev_handler,
                          &mic_form_desc,
                          &mic_inp_desc,
                          &mic_out_desc,
                          &mic_fea_desc,
                          0,
                          APP_USBD_AUDIO_AS_IFACE_FORMAT_PCM,
                          192,
                          APP_USBD_AUDIO_SUBCLASS_AUDIOSTREAMING
);


/*lint -restore*/

/**
 * @brief Internal audio temporary buffer
 */
static int16_t  m_temp_buffer[2 * BUFFER_SIZE];



/**
 * @brief The size of last received block from the microphone
 */
static size_t m_temp_buffer_size;

/**
 * @brief Actual headphones mute
 */
static uint8_t  m_mute_hp;

/**
 * @brief Actual sampling frequency
 */
static uint32_t m_freq_hp;

/**
 * @brief Actual microphone mute state
 */
static uint8_t  m_mute_mic;

/**
 * @brief Actual microphone sampling frequency
 */
static uint32_t m_freq_mic;

static application_usb_audio_event_t        m_usb_audio_event;
static application_usb_audio_callback_t     m_usb_audio_callback;


/**
 * @brief Audio class specific request handle (headphones)
 */
static void hp_audio_user_class_req(app_usbd_class_inst_t const * p_inst)
{
    app_usbd_audio_t const * p_audio = app_usbd_audio_class_get(p_inst);
    app_usbd_audio_req_t * p_req = app_usbd_audio_class_request_get(p_audio);

    UNUSED_VARIABLE(m_mute_hp);
    UNUSED_VARIABLE(m_freq_hp);

    switch (p_req->req_target)
    {
        case APP_USBD_AUDIO_CLASS_REQ_IN:

            if (p_req->req_type == APP_USBD_AUDIO_REQ_SET_CUR)
            {
                //Only mute control is defined
                p_req->payload[0] = m_mute_hp;
            }

            break;
        case APP_USBD_AUDIO_CLASS_REQ_OUT:

            if (p_req->req_type == APP_USBD_AUDIO_REQ_SET_CUR)
            {
                //Only mute control is defined
                m_mute_hp = p_req->payload[0];
            }

            break;
        case APP_USBD_AUDIO_EP_REQ_IN:
            break;
        case APP_USBD_AUDIO_EP_REQ_OUT:

            if (p_req->req_type == APP_USBD_AUDIO_REQ_SET_CUR)
            {
                //Only set frequency is supported
                m_freq_hp = uint24_decode(p_req->payload);
            }

            break;
        default:
            break;
    }
}


/**
 * @brief Audio class specific request handle (microphone)
 */
static void mic_audio_user_class_req(app_usbd_class_inst_t const * p_inst)
{
    app_usbd_audio_t const * p_audio = app_usbd_audio_class_get(p_inst);
    app_usbd_audio_req_t * p_req = app_usbd_audio_class_request_get(p_audio);

    UNUSED_VARIABLE(m_mute_mic);
    UNUSED_VARIABLE(m_freq_mic);

    switch (p_req->req_target)
    {
        case APP_USBD_AUDIO_CLASS_REQ_IN:

            if (p_req->req_type == APP_USBD_AUDIO_REQ_SET_CUR)
            {
                //Only mute control is defined
                p_req->payload[0] = m_mute_mic;
            }

            break;
        case APP_USBD_AUDIO_CLASS_REQ_OUT:

            if (p_req->req_type == APP_USBD_AUDIO_REQ_SET_CUR)
            {
                //Only mute control is defined
                m_mute_mic = p_req->payload[0];
            }

            break;
        case APP_USBD_AUDIO_EP_REQ_IN:
            break;
        case APP_USBD_AUDIO_EP_REQ_OUT:

            if (p_req->req_type == APP_USBD_AUDIO_REQ_SET_CUR)
            {
                //Only set frequency is supported
                m_freq_mic = uint24_decode(p_req->payload);
            }

            break;
        default:
            break;
    }
}


/**
 * @brief User event handler @ref app_usbd_audio_user_ev_handler_t (headphones)
 */
static void hp_audio_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                     app_usbd_audio_user_event_t   event)
{
    app_usbd_audio_t const * p_audio = app_usbd_audio_class_get(p_inst);
    UNUSED_VARIABLE(p_audio);
    switch (event)
    {
        case APP_USBD_AUDIO_USER_EVT_CLASS_REQ:
            hp_audio_user_class_req(p_inst);
            break;
        case APP_USBD_AUDIO_USER_EVT_RX_DONE:
        {
            ret_code_t ret;
            /* Block from headphones copied into buffer, send it into microphone input */
            ret = app_usbd_audio_class_tx_start(&m_app_audio_microphone.base, m_temp_buffer, m_temp_buffer_size);
            if (NRF_SUCCESS == ret)
            {
                //bsp_board_led_invert(LED_AUDIO_RX);
            }
            break;
        }
        default:
            break;
    }
}


/**
 * @brief User event handler @ref app_usbd_audio_user_ev_handler_t (microphone)
 */
static void mic_audio_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                      app_usbd_audio_user_event_t   event)
{
    app_usbd_audio_t const * p_audio = app_usbd_audio_class_get(p_inst);
    UNUSED_VARIABLE(p_audio);

    switch (event)
    {
        case APP_USBD_AUDIO_USER_EVT_CLASS_REQ:
            mic_audio_user_class_req(p_inst);
            break;
        case APP_USBD_AUDIO_USER_EVT_TX_DONE:
        {
            //bsp_board_led_invert(LED_AUDIO_TX);
            break;
        }
        default:
            break;
    }
}
    

/**
 * @brief USBD library specific event handler.
 *
 * @param event     USBD library event.
 */
static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SOF:
        {
            if (APP_USBD_STATE_Configured != app_usbd_core_state_get())
            {
                break;
            }
            size_t rx_size = app_usbd_audio_class_rx_size_get(&m_app_audio_headphone.base);
            m_temp_buffer_size = rx_size;
            if (rx_size > 0)
            {
                ASSERT(rx_size <= sizeof(m_temp_buffer));
                ret_code_t ret;
                ret = app_usbd_audio_class_rx_start(&m_app_audio_headphone.base, m_temp_buffer, rx_size);
                if (NRF_SUCCESS != ret)
                {
                    NRF_LOG_ERROR("Cannot start RX transfer from headphone\r\n");
                }
            }
            break;
        }
        case APP_USBD_EVT_DRV_SUSPEND:
            //bsp_board_leds_off();
            break;
        case APP_USBD_EVT_DRV_RESUME:
            //bsp_board_led_on(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_STARTED:
            //bsp_board_led_on(LED_USB_START);
            break;
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            //bsp_board_leds_off();
            break;
        case APP_USBD_EVT_POWER_DETECTED:
            if(m_usb_audio_callback)
            {
                m_usb_audio_event.event_type = AUA_EVENT_USB_POWER_DETECTED;
                m_usb_audio_callback(&m_usb_audio_event);
            }

            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            if(m_usb_audio_callback)
            {
                m_usb_audio_event.event_type = AUA_EVENT_USB_POWER_REMOVED;
                m_usb_audio_callback(&m_usb_audio_event);
            }
            
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
            if(m_usb_audio_callback)
            {
                m_usb_audio_event.event_type = AUA_EVENT_USB_POWER_READY;
                m_usb_audio_callback(&m_usb_audio_event);
            }
            
            app_usbd_start();
            break;
        default:
            break;
    }
}

// TOO: Public interface functions

uint32_t application_usb_audio_init(application_usb_audio_config_t *config)
{
    uint32_t err_code;
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler,
        .enable_sof = true
    };
    
    err_code = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(err_code);

    app_usbd_class_inst_t const * class_inst_hp =
        app_usbd_audio_class_inst_get(&m_app_audio_headphone);
    err_code = app_usbd_class_append(class_inst_hp);
    APP_ERROR_CHECK(err_code);

    app_usbd_class_inst_t const * class_inst_mic =
        app_usbd_audio_class_inst_get(&m_app_audio_microphone);
    err_code = app_usbd_class_append(class_inst_mic);
    APP_ERROR_CHECK(err_code);
    
    m_usb_audio_callback = config->event_handler;
    
    return NRF_SUCCESS;
}


uint32_t application_usb_audio_events_enable(void)
{
    uint32_t err_code;
    
    if (USBD_POWER_DETECTION)
    {
        err_code = app_usbd_power_events_enable();
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        NRF_LOG_INFO("No USB power detection enabled\r\nStarting USB now");

        app_usbd_enable();
        app_usbd_start();
    }
    
    return NRF_SUCCESS;
}


uint32_t application_usb_audio_process_events(void)
{
    while (app_usbd_event_queue_process())
    {
        /* Nothing to do */
    }
    return NRF_SUCCESS;    
}

