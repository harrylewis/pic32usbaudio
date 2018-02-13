/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app_usb_audio_task.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2016-2017 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END

#include "app_usb_audio_task.h"

USB_DEVICE_AUDIO_RESULT AppUsbAudioError;
APP_USB_AUDIO_DATA AppUsbAudioData;

uint32_t rxBuffer[96];
#define SIZE 50
uint8_t buffer[SIZE];
DRV_I2S_BUFFER_HANDLE bufferHandle;
int count = 0;
int events = -1;

void func(void);

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************
void APP_USB_AUDIO_DeviceEventHandler(USB_DEVICE_EVENT event, void *pEventData, uintptr_t context) {
    
    volatile USB_DEVICE_EVENT_DATA_CONFIGURED* configuredEventData;
    
    switch (event) {
        
        case USB_DEVICE_EVENT_RESET:
        break;

        case USB_DEVICE_EVENT_DECONFIGURED:           
        break;

        case USB_DEVICE_EVENT_CONFIGURED:
            
            configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED *)pEventData;
            
            if(configuredEventData->configurationValue == 1) {
                USB_DEVICE_AUDIO_EventHandlerSet
                (
                    0,
                    APP_USB_AUDIO_DeviceAudioEventHandler ,
                    (uintptr_t)NULL
                );
                AppUsbAudioData.isConfigured = true;
            }
            
        break;

        case USB_DEVICE_EVENT_SUSPENDED:
        break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_POWER_DETECTED:
            USB_DEVICE_Attach (AppUsbAudioData.usbDevHandle);
        break;
        
        case USB_DEVICE_EVENT_POWER_REMOVED:
            USB_DEVICE_Detach (AppUsbAudioData.usbDevHandle);
        break;    
            
        case USB_DEVICE_EVENT_ERROR:
        default:
        break;
    }
}

////////////////////////////////////////////////////////////////////////////////
void APP_USB_AUDIO_DeviceAudioEventHandler(USB_DEVICE_AUDIO_INDEX iAudio, USB_DEVICE_AUDIO_EVENT event, void *pData, uintptr_t context) {
    
    volatile USB_DEVICE_AUDIO_EVENT_DATA_INTERFACE_SETTING_CHANGED *interfaceInfo;
    volatile USB_DEVICE_AUDIO_EVENT_DATA_READ_COMPLETE *readEventData;
    uint8_t entityID;
    uint8_t controlSelector;

    if ( iAudio == 0 )
    {
        switch (event)
        {
            case USB_DEVICE_AUDIO_EVENT_INTERFACE_SETTING_CHANGED:
                /* We have received a request from USB host to change the Interface-
                   Alternate setting.*/
//                interfaceInfo = (USB_DEVICE_AUDIO_EVENT_DATA_INTERFACE_SETTING_CHANGED *)pData;
//                AppUsbAudioData.activeInterfaceAlternateSetting = interfaceInfo->interfaceAlternateSetting;
//                AppUsbAudioData.state = APP_USB_AUDIO_STATE_USB_INTERFACE_ALTERNATE_SETTING_RCVD;
            break;

            case USB_DEVICE_AUDIO_EVENT_READ_COMPLETE:
            break;

            case USB_DEVICE_AUDIO_EVENT_WRITE_COMPLETE:
            break;


            case USB_DEVICE_AUDIO_EVENT_CONTROL_SET_CUR:
                
                entityID = ((USB_AUDIO_CONTROL_INTERFACE_REQUEST*)pData)->entityID;
                
                if (entityID == APP_ID_FEATURE_UNIT) {
                   controlSelector = ((USB_AUDIO_FEATURE_UNIT_CONTROL_REQUEST*)pData)->controlSelector;
                   if (controlSelector == USB_AUDIO_MUTE_CONTROL) {
                       //A control write transfer received from Host. Now receive data from Host.
                       USB_DEVICE_ControlReceive(AppUsbAudioData.usbDevHandle, (void *) &(AppUsbAudioData.dacMute), 1 );
                       // AppUsbAudioData.currentAudioControl = APP_USB_AUDIO_MUTE_CONTROL;
                   }
                }
                
            break;
                
            case USB_DEVICE_AUDIO_EVENT_CONTROL_GET_CUR:
                
                entityID = ((USB_AUDIO_CONTROL_INTERFACE_REQUEST*)pData)->entityID;
                
                if (entityID == APP_ID_FEATURE_UNIT) {
                   controlSelector = ((USB_AUDIO_FEATURE_UNIT_CONTROL_REQUEST*)pData)->controlSelector;
                   if (controlSelector == USB_AUDIO_MUTE_CONTROL) {
                       USB_DEVICE_ControlSend(AppUsbAudioData.usbDevHandle, (void *)&(AppUsbAudioData.dacMute), 1 );
                   }
                }
                
            break;
            
            case USB_DEVICE_AUDIO_EVENT_CONTROL_SET_MIN:
            case USB_DEVICE_AUDIO_EVENT_CONTROL_GET_MIN:
            case USB_DEVICE_AUDIO_EVENT_CONTROL_SET_MAX:
            case USB_DEVICE_AUDIO_EVENT_CONTROL_GET_MAX:
            case USB_DEVICE_AUDIO_EVENT_CONTROL_SET_RES:
            case USB_DEVICE_AUDIO_EVENT_CONTROL_GET_RES:
            case USB_DEVICE_AUDIO_EVENT_ENTITY_GET_MEM:
                USB_DEVICE_ControlStatus (AppUsbAudioData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_ERROR);
            break;
            
            case USB_DEVICE_AUDIO_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:
                
//                USB_DEVICE_ControlStatus(AppUsbAudioData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_OK );
//                if (AppUsbAudioData.currentAudioControl == APP_USB_AUDIO_MUTE_CONTROL) {
//                    AppUsbAudioData.state = APP_USB_AUDIO_STATE_MUTE_AUDIO_PLAYBACK;
//                    AppUsbAudioData.currentAudioControl = APP_USB_AUDIO_CONTROL_NONE;
//                }
                
            break;
            
            case  USB_DEVICE_AUDIO_EVENT_CONTROL_TRANSFER_DATA_SENT:
            break;
            
            default:
                SYS_ASSERT ( false , "Invalid callback" );
            break;
            
        } //end of switch ( callback )
    } //end of if  if ( iAudio == 0 )
} //end of function APP_AudioEventCallback

////////////////////////////////////////////////////////////////////////////////
/******************************************************
 * Application Initialize. It is
 * called from the SYS_Initialized() function.
 ******************************************************/
void APP_USB_AUDIO_Initialize(void) {     
    /* Device Layer Handle  */
    AppUsbAudioData.usbDevHandle = -1;
    /* USB Audio Instance index for this app object 0*/
    AppUsbAudioData.audioInstance = 0;
     /* app state */
    AppUsbAudioData.state = APP_USB_AUDIO_STATE_WAIT_FOR_I2S;
    /* device configured status */
    AppUsbAudioData.isConfigured = false; 
    
    AppUsbAudioData.i2sHandle = DRV_HANDLE_INVALID;
//    AppUsbAudioData.i2sBufferHandle = DRV_I2S_BUFFER_HANDLE_INVALID;
    AppUsbAudioData.i2sBufferEventHandler = (DRV_I2S_BUFFER_EVENT_HANDLER) APP_USB_AUDIO_I2SBufferEventHandler;
    AppUsbAudioData.context = (uintptr_t) 1;
            
}

////////////////////////////////////////////////////////////////////////////////
/**********************************************************
 * Application tasks routine. This function implements the
 * application state machine.
 ***********************************************************/
void APP_USB_AUDIO_Tasks (void) { 
    
    switch (AppUsbAudioData.state) {
        
        case APP_USB_AUDIO_STATE_WAIT_FOR_I2S:
            
            if (DRV_I2S_Status(sysObj.drvI2S0) == SYS_STATUS_READY) {
                AppUsbAudioData.state = APP_USB_AUDIO_STATE_I2S_OPEN;
            }
            
        break;
        
        case APP_USB_AUDIO_STATE_I2S_OPEN:
            
            AppUsbAudioData.i2sHandle = DRV_I2S_Open(DRV_I2S_INDEX_0, DRV_IO_INTENT_READ);
            
            if (AppUsbAudioData.i2sHandle != DRV_HANDLE_INVALID) {
                AppUsbAudioData.state = APP_USB_AUDIO_STATE_I2S_SET_BUFFER_HANDLER;
            }
            
        break;
        
        case APP_USB_AUDIO_STATE_I2S_SET_BUFFER_HANDLER:
            
            DRV_I2S_BaudSet(AppUsbAudioData.i2sHandle, 24576000, 6144000);
            
            DRV_I2S_BufferEventHandlerSet(AppUsbAudioData.i2sHandle, AppUsbAudioData.i2sBufferEventHandler, AppUsbAudioData.context); 
            
            AppUsbAudioData.state = APP_USB_AUDIO_STATE_SUBMIT_INITIAL_READ_REQUEST;
            
        break;
        
        case APP_USB_AUDIO_STATE_INIT:
            
            AppUsbAudioData.usbDevHandle = USB_DEVICE_Open(USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE);
            
            if(AppUsbAudioData.usbDevHandle != USB_DEVICE_HANDLE_INVALID) {
                USB_DEVICE_EventHandlerSet(AppUsbAudioData.usbDevHandle, APP_USB_AUDIO_DeviceEventHandler, 0);                
                AppUsbAudioData.state = APP_USB_AUDIO_STATE_WAIT_FOR_CONFIGURATION;
            } else {}
            
        break;

        case APP_USB_AUDIO_STATE_WAIT_FOR_CONFIGURATION:
            
            if (AppUsbAudioData.isConfigured == true) {                
                AppUsbAudioData.state = APP_USB_AUDIO_STATE_IDLE;
            }
            
        break;

        case APP_USB_AUDIO_STATE_SUBMIT_INITIAL_READ_REQUEST:
            
            
            DRV_I2S_BufferAddRead(AppUsbAudioData.i2sHandle, &AppUsbAudioData.i2sBufferHandle, &rxBuffer[0], sizeof(rxBuffer));
            //count = DRV_I2S_Read(AppUsbAudioData.i2sHandle, &buffer[0], SIZE);
            AppUsbAudioData.state = APP_USB_AUDIO_STATE_PROCESS_DATA;
            
            
        break;

        case APP_USB_AUDIO_STATE_PROCESS_DATA:
            
            AppUsbAudioData.state = APP_USB_AUDIO_STATE_INIT;
            
//            if (*AppUsbAudioData.i2sBufferHandle == DRV_I2S_BUFFER_HANDLE_INVALID) {
//                AppUsbAudioData.state = APP_USB_AUDIO_STATE_INIT;
//            }
            
        break;

        case APP_USB_AUDIO_STATE_MUTE_AUDIO_PLAYBACK:
        break;

        case APP_USB_AUDIO_STATE_USB_INTERFACE_ALTERNATE_SETTING_RCVD:
        break;

        case APP_USB_AUDIO_STATE_IDLE:
        break;

        case APP_USB_AUDIO_STATE_ERROR:
        default:
        break;
        
    }

}

void func(void) {
    return;
}


void APP_USB_AUDIO_I2SBufferEventHandler(DRV_I2S_BUFFER_EVENT event, DRV_I2S_BUFFER_HANDLE bufferHandle, uintptr_t context) {
    
    switch (event) {
        
        case DRV_I2S_BUFFER_EVENT_COMPLETE:
            AppUsbAudioData.state = APP_USB_AUDIO_STATE_INIT;
        break;
        
        case DRV_I2S_BUFFER_EVENT_ERROR:
            AppUsbAudioData.state = APP_USB_AUDIO_STATE_INIT;
        break;
        
        case DRV_I2S_BUFFER_EVENT_ABORT:
            AppUsbAudioData.state = APP_USB_AUDIO_STATE_INIT;
        break;
        
    }

}


////////////////////////////////////////////////////////////////////////////////


/*******************************************************************************
 End of File
 */