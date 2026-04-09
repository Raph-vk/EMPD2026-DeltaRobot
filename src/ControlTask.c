/*
 * ControlTask.c
 *
 * Created: 09-04-2026
 * Author: Raph van Koeveringe
 */

///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>
#include <stdbool.h>
#include <stdint.h>

///////////////////////////////////////////////////////////////////////////////
// FreeRTOS includes

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include "vPrintString.h"
#include "TaskSleep.h"

///////////////////////////////////////////////////////////////////////////////
// other includes

#include "ApplicationTasks.h"
#include "bits.h"
#include "LEDLib.h"

///////////////////////////////////////////////////////////////////////////////
// types

typedef enum
{
    STATE_INIT = 0,
    STATE_WAIT,
    STATE_HOMING,
    STATE_READY,
    STATE_RUN,
    STATE_PAUSE,
    STATE_FAULT
} ControlState_t;

typedef enum
{
    MOTION_CMD_DISABLED = 0,
    MOTION_CMD_HOMING,
    MOTION_CMD_HOLD,
    MOTION_CMD_RUN
} MotionCommand_t;

///////////////////////////////////////////////////////////////////////////////
// extern handles

extern EventGroupHandle_t handle_ThreadEventGroup;

extern SemaphoreHandle_t handle_StartSemaphore;
extern SemaphoreHandle_t handle_StopSemaphore;
extern SemaphoreHandle_t handle_RestartSemaphore;
extern SemaphoreHandle_t handle_NoodSemaphore;

extern SemaphoreHandle_t handle_HomeDoneSemaphore;
extern SemaphoreHandle_t handle_CycleDoneSemaphore;

extern QueueHandle_t handle_MotionCommandQueue;

///////////////////////////////////////////////////////////////////////////////
// locals

static ControlState_t currentState = STATE_INIT;

///////////////////////////////////////////////////////////////////////////////
// helpers

static void updateStateLeds(ControlState_t state)
{
    switch (state)
    {
        case STATE_INIT:
            led_DisplayValue(0x03);   // pas later aan naar jouw wens
            break;

        case STATE_WAIT:
            led_DisplayValue(0x01);   // voorbeeld
            break;

        case STATE_HOMING:
            led_DisplayValue(0x02);   // voorbeeld
            break;

        case STATE_READY:
            led_DisplayValue(0x04);   // voorbeeld
            break;

        case STATE_RUN:
            led_DisplayValue(0x08);   // voorbeeld
            break;

        case STATE_PAUSE:
            led_DisplayValue(0x05);   // voorbeeld
            break;

        case STATE_FAULT:
            led_DisplayValue(0x0F);   // voorbeeld
            break;

        default:
            led_DisplayValue(0x00);
            break;
    }
}

static void sendMotionCommand(MotionCommand_t cmd)
{
    if (handle_MotionCommandQueue != NULL)
    {
        xQueueOverwrite(handle_MotionCommandQueue, &cmd);
    }
}

static bool emergencyInputStillActive(void)
{
    // Later echte nood-input uitlezen
    return false;
}

static void enterState(ControlState_t newState)
{
    currentState = newState;
    updateStateLeds(currentState);

    switch (currentState)
    {
        case STATE_INIT:
            vPrintString("> STATE = INIT\r\n");
            sendMotionCommand(MOTION_CMD_DISABLED);
            break;

        case STATE_WAIT:
            vPrintString("> STATE = WAIT\r\n");
            sendMotionCommand(MOTION_CMD_DISABLED);
            break;

        case STATE_HOMING:
            vPrintString("> STATE = HOMING\r\n");
            sendMotionCommand(MOTION_CMD_HOMING);
            break;

        case STATE_READY:
            vPrintString("> STATE = READY\r\n");
            sendMotionCommand(MOTION_CMD_HOLD);
            break;

        case STATE_RUN:
            vPrintString("> STATE = RUN\r\n");
            sendMotionCommand(MOTION_CMD_RUN);
            break;

        case STATE_PAUSE:
            vPrintString("> STATE = PAUSE\r\n");
            sendMotionCommand(MOTION_CMD_HOLD);
            break;

        case STATE_FAULT:
            vPrintString("> STATE = FAULT\r\n");
            sendMotionCommand(MOTION_CMD_DISABLED);
            break;

        default:
            sendMotionCommand(MOTION_CMD_DISABLED);
            break;
    }
}

///////////////////////////////////////////////////////////////////////////////
// ControlTask

void ControlTask(void *pvParameters)
{
    (void)pvParameters;

    bool startPressed;
    bool stopPressed;
    bool resetPressed;
    bool noodActive;
    bool homeDone;
    bool cycleDone;

    vPrintString("> starting ControlTask\r\n");

    enterState(STATE_INIT);

    // Wacht tot InputHandlerTask en MotionControlTask actief zijn
    xEventGroupWaitBits(
        handle_ThreadEventGroup,
        BIT_0 | BIT_1,
        pdFALSE,
        pdTRUE,
        portMAX_DELAY
    );

    enterState(STATE_WAIT);

    while (true)
    {
        startPressed = (xSemaphoreTake(handle_StartSemaphore, 0) == pdTRUE);
        stopPressed  = (xSemaphoreTake(handle_StopSemaphore, 0) == pdTRUE);
        resetPressed = (xSemaphoreTake(handle_RestartSemaphore, 0) == pdTRUE);
        noodActive   = (xSemaphoreTake(handle_NoodSemaphore, 0) == pdTRUE);

        homeDone     = (xSemaphoreTake(handle_HomeDoneSemaphore, 0) == pdTRUE);
        cycleDone    = (xSemaphoreTake(handle_CycleDoneSemaphore, 0) == pdTRUE);

        // nood heeft altijd hoogste prioriteit
        if (noodActive)
        {
            enterState(STATE_FAULT);
        }

        switch (currentState)
        {
            case STATE_INIT:
            {
                break;
            }

            case STATE_WAIT:
            {
                if (startPressed || resetPressed)
                {
                    enterState(STATE_HOMING);
                }
                break;
            }

            case STATE_HOMING:
            {
                if (homeDone)
                {
                    enterState(STATE_READY);
                }
                break;
            }

            case STATE_READY:
            {
                if (startPressed)
                {
                    enterState(STATE_RUN);
                }
                break;
            }

            case STATE_RUN:
            {
                if (stopPressed)
                {
                    enterState(STATE_PAUSE);
                }
                else if (cycleDone)
                {
                    enterState(STATE_READY);
                }
                break;
            }

            case STATE_PAUSE:
            {
                if (startPressed)
                {
                    enterState(STATE_RUN);
                }
                else if (resetPressed)
                {
                    enterState(STATE_READY);
                }
                break;
            }

            case STATE_FAULT:
            {
                if (resetPressed && (emergencyInputStillActive() == false))
                {
                    enterState(STATE_WAIT);
                }
                break;
            }

            default:
            {
                enterState(STATE_FAULT);
                break;
            }
        }

        taskSleep(10);
    }
}