/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @brief The OpenPilot Modules do the majority of the control in OpenPilot.  The
 * @ref SystemModule "System Module" starts all the other modules that then take care
 * of all the telemetry and control algorithms and such.  This is done through the @ref PIOS
 * "PIOS Hardware abstraction layer" which then contains hardware specific implementations
 * (currently only STM32 supported)
 *
 * @{
 * @addtogroup SystemModule System Module
 * @brief Initializes PIOS and other modules runs monitoring
 * After initializing all the modules (currently selected by Makefile but in
 * future controlled by configuration on SD card) runs basic monitoring and
 * alarms.
 * @{
 *
 * @file       systemmod.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      System module
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <openpilot.h>

// private includes
#include "inc/systemmod.h"

// UAVOs
#include <objectpersistence.h>
#include <flightstatus.h>
#include <systemstats.h>
#include <systemsettings.h>
#include <i2cstats.h>
#include <taskinfo.h>
#include <watchdogstatus.h>
#include <taskinfo.h>
#include <hwsettings.h>
#include <pios_flashfs.h>
#if defined(PIOS_INCLUDE_RFM22B)
#include <oplinkstatus.h>
#endif

// Flight Libraries
#include <sanitycheck.h>


// #define DEBUG_THIS_FILE

#if defined(PIOS_INCLUDE_DEBUG_CONSOLE) && defined(DEBUG_THIS_FILE)
#define DEBUG_MSG(format, ...) PIOS_COM_SendFormattedString(PIOS_COM_DEBUG, format,##__VA_ARGS__)
#else
#define DEBUG_MSG(format, ...)
#endif

// Private constants
#define SYSTEM_UPDATE_PERIOD_MS        1000
#define LED_BLINK_RATE_HZ              5

#ifndef IDLE_COUNTS_PER_SEC_AT_NO_LOAD
#define IDLE_COUNTS_PER_SEC_AT_NO_LOAD 995998 // calibrated by running tests/test_cpuload.c
// must be updated if the FreeRTOS or compiler
// optimisation options are changed.
#endif

#if defined(PIOS_SYSTEM_STACK_SIZE)
#define STACK_SIZE_BYTES PIOS_SYSTEM_STACK_SIZE
#else
#define STACK_SIZE_BYTES 1024
#endif

#define TASK_PRIORITY    (tskIDLE_PRIORITY + 1)

// Private types

// Private variables
static uint32_t idleCounter;
static uint32_t idleCounterClear;
static xTaskHandle systemTaskHandle;
static xQueueHandle objectPersistenceQueue;
static bool stackOverflow;
static bool mallocFailed;
static HwSettingsData bootHwSettings;
#ifdef PIOS_INCLUDE_FLASH
static struct PIOS_FLASHFS_Stats fsStats;
#endif
// Private functions
static void objectUpdatedCb(UAVObjEvent *ev);
static void hwSettingsUpdatedCb(UAVObjEvent *ev);
#ifdef DIAG_TASKS
static void taskMonitorForEachCallback(uint16_t task_id, const struct pios_task_info *task_info, void *context);
#endif
static void updateStats();
static void updateSystemAlarms();
static void systemTask(void *parameters);
#ifdef DIAG_I2C_WDG_STATS
static void updateI2Cstats();
static void updateWDGstats();
#endif

extern uintptr_t pios_uavo_settings_fs_id;
extern uintptr_t pios_user_fs_id;

/**
 * Create the module task.
 * \returns 0 on success or -1 if initialization failed
 */
int32_t SystemModStart(void)
{
    // Initialize vars
    stackOverflow = false;
    mallocFailed  = false;
    // Create system task
    xTaskCreate(systemTask, (signed char *)"System", STACK_SIZE_BYTES / 4, NULL, TASK_PRIORITY, &systemTaskHandle);
    // Register task
    PIOS_TASK_MONITOR_RegisterTask(TASKINFO_RUNNING_SYSTEM, systemTaskHandle);

    return 0;
}

/**
 * Initialize the module, called on startup.
 * \returns 0 on success or -1 if initialization failed
 */
int32_t SystemModInitialize(void)
{
    // Must registers objects here for system thread because ObjectManager started in OpenPilotInit
    SystemSettingsInitialize();
    SystemStatsInitialize();
    FlightStatusInitialize();
    ObjectPersistenceInitialize();
#ifdef DIAG_TASKS
    TaskInfoInitialize();
#endif
#ifdef DIAG_I2C_WDG_STATS
    I2CStatsInitialize();
    WatchdogStatusInitialize();
#endif

    objectPersistenceQueue = xQueueCreate(1, sizeof(UAVObjEvent));
    if (objectPersistenceQueue == NULL) {
        return -1;
    }

    SystemModStart();

    return 0;
}

MODULE_INITCALL(SystemModInitialize, 0);
/**
 * System task, periodically executes every SYSTEM_UPDATE_PERIOD_MS
 */
static void systemTask(__attribute__((unused)) void *parameters)
{
    /* start the delayed callback scheduler */
    CallbackSchedulerStart();
    uint8_t cycleCount = 0;
    /* create all modules thread */
    MODULE_TASKCREATE_ALL;

    if (mallocFailed) {
        /* We failed to malloc during task creation,
         * system behaviour is undefined.  Reset and let
         * the BootFault code recover for us.
         */
        PIOS_SYS_Reset();
    }

#if defined(PIOS_INCLUDE_IAP)
    /* Record a successful boot */
    PIOS_IAP_WriteBootCount(0);
#endif

    // Initialize vars
    idleCounter = 0;
    idleCounterClear = 0;

    // Listen for SettingPersistance object updates, connect a callback function
    ObjectPersistenceConnectQueue(objectPersistenceQueue);

    // Load a copy of HwSetting active at boot time
    HwSettingsGet(&bootHwSettings);
    // Whenever the configuration changes, make sure it is safe to fly
    HwSettingsConnectCallback(hwSettingsUpdatedCb);

#ifdef DIAG_TASKS
    TaskInfoData taskInfoData;
#endif

    // Main system loop
    while (1) {
        // Update the system statistics
        updateStats();
        cycleCount = cycleCount > 0 ? cycleCount - 1 : 7;

        // Update the system alarms
        updateSystemAlarms();
#ifdef DIAG_I2C_WDG_STATS
        updateI2Cstats();
        updateWDGstats();
#endif

#ifdef DIAG_TASKS
        // Update the task status object
        PIOS_TASK_MONITOR_ForEachTask(taskMonitorForEachCallback, &taskInfoData);
        TaskInfoSet(&taskInfoData);
#endif

        // Flash the heartbeat LED
#if defined(PIOS_LED_HEARTBEAT)
        uint8_t armingStatus;
        FlightStatusArmedGet(&armingStatus);
        if ((armingStatus == FLIGHTSTATUS_ARMED_ARMED && (cycleCount & 0x1)) ||
            (armingStatus != FLIGHTSTATUS_ARMED_ARMED && (cycleCount & 0x4))) {
            PIOS_LED_On(PIOS_LED_HEARTBEAT);
        } else {
            PIOS_LED_Off(PIOS_LED_HEARTBEAT);
        }

        DEBUG_MSG("+ 0x%08x\r\n", 0xDEADBEEF);
#endif /* PIOS_LED_HEARTBEAT */

        // Turn on the error LED if an alarm is set
#if defined(PIOS_LED_ALARM)
        if (AlarmsHasCritical()) {
            PIOS_LED_On(PIOS_LED_ALARM);
        } else if ((AlarmsHasErrors() && (cycleCount & 0x1)) ||
                   (!AlarmsHasErrors() && AlarmsHasWarnings() && (cycleCount & 0x4))) {
            PIOS_LED_On(PIOS_LED_ALARM);
        } else {
            PIOS_LED_Off(PIOS_LED_ALARM);
        }
#endif /* PIOS_LED_ALARM */


        UAVObjEvent ev;
        int delayTime = SYSTEM_UPDATE_PERIOD_MS / portTICK_RATE_MS / (LED_BLINK_RATE_HZ * 2);

#if defined(PIOS_INCLUDE_RFM22B)
        // Update the OPLinkStatus UAVO
        OPLinkStatusData oplinkStatus;
        OPLinkStatusGet(&oplinkStatus);

        // Get the other device stats.
        PIOS_RFM2B_GetPairStats(pios_rfm22b_id, oplinkStatus.PairIDs, oplinkStatus.PairSignalStrengths, OPLINKSTATUS_PAIRIDS_NUMELEM);

        // Get the stats from the radio device
        struct rfm22b_stats radio_stats;
        PIOS_RFM22B_GetStats(pios_rfm22b_id, &radio_stats);

        // Update the OPLInk status
        static bool first_time = true;
        static uint16_t prev_tx_count = 0;
        static uint16_t prev_rx_count = 0;
        oplinkStatus.HeapRemaining = xPortGetFreeHeapSize();
        oplinkStatus.DeviceID = PIOS_RFM22B_DeviceID(pios_rfm22b_id);
        oplinkStatus.RxGood = radio_stats.rx_good;
        oplinkStatus.RxCorrected   = radio_stats.rx_corrected;
        oplinkStatus.RxErrors = radio_stats.rx_error;
        oplinkStatus.RxMissed = radio_stats.rx_missed;
        oplinkStatus.RxFailure     = radio_stats.rx_failure;
        oplinkStatus.TxDropped     = radio_stats.tx_dropped;
        oplinkStatus.TxResent = radio_stats.tx_resent;
        oplinkStatus.TxFailure     = radio_stats.tx_failure;
        oplinkStatus.Resets      = radio_stats.resets;
        oplinkStatus.Timeouts    = radio_stats.timeouts;
        oplinkStatus.RSSI        = radio_stats.rssi;
        oplinkStatus.LinkQuality = radio_stats.link_quality;
        if (first_time) {
            first_time = false;
        } else {
            uint16_t tx_count = radio_stats.tx_byte_count;
            uint16_t rx_count = radio_stats.rx_byte_count;
            uint16_t tx_bytes = (tx_count < prev_tx_count) ? (0xffff - prev_tx_count + tx_count) : (tx_count - prev_tx_count);
            uint16_t rx_bytes = (rx_count < prev_rx_count) ? (0xffff - prev_rx_count + rx_count) : (rx_count - prev_rx_count);
            oplinkStatus.TXRate = (uint16_t)((float)(tx_bytes * 1000) / SYSTEM_UPDATE_PERIOD_MS);
            oplinkStatus.RXRate = (uint16_t)((float)(rx_bytes * 1000) / SYSTEM_UPDATE_PERIOD_MS);
            prev_tx_count = tx_count;
            prev_rx_count = rx_count;
        }
        oplinkStatus.TXSeq     = radio_stats.tx_seq;
        oplinkStatus.RXSeq     = radio_stats.rx_seq;
        oplinkStatus.LinkState = radio_stats.link_state;
        OPLinkStatusSet(&oplinkStatus);
#endif /* if defined(PIOS_INCLUDE_RFM22B) */

        if (xQueueReceive(objectPersistenceQueue, &ev, delayTime) == pdTRUE) {
            // If object persistence is updated call the callback
            objectUpdatedCb(&ev);
        }
    }
}

/**
 * Function called in response to object updates
 */
static void objectUpdatedCb(UAVObjEvent *ev)
{
    ObjectPersistenceData objper;
    UAVObjHandle obj;

    // If the object updated was the ObjectPersistence execute requested action
    if (ev->obj == ObjectPersistenceHandle()) {
        // Get object data
        ObjectPersistenceGet(&objper);

        int retval = 1;
        FlightStatusData flightStatus;
        FlightStatusGet(&flightStatus);

        // When this is called because of this method don't do anything
        if (objper.Operation == OBJECTPERSISTENCE_OPERATION_ERROR || objper.Operation == OBJECTPERSISTENCE_OPERATION_COMPLETED) {
            return;
        }

        // Execute action if disarmed
        if (flightStatus.Armed != FLIGHTSTATUS_ARMED_DISARMED) {
            retval = -1;
        } else if (objper.Operation == OBJECTPERSISTENCE_OPERATION_LOAD) {
            if (objper.Selection == OBJECTPERSISTENCE_SELECTION_SINGLEOBJECT) {
                // Get selected object
                obj = UAVObjGetByID(objper.ObjectID);
                if (obj == 0) {
                    return;
                }
                // Load selected instance
                retval = UAVObjLoad(obj, objper.InstanceID);
            } else if (objper.Selection == OBJECTPERSISTENCE_SELECTION_ALLSETTINGS || objper.Selection == OBJECTPERSISTENCE_SELECTION_ALLOBJECTS) {
                retval = UAVObjLoadSettings();
            } else if (objper.Selection == OBJECTPERSISTENCE_SELECTION_ALLMETAOBJECTS || objper.Selection == OBJECTPERSISTENCE_SELECTION_ALLOBJECTS) {
                retval = UAVObjLoadMetaobjects();
            }
        } else if (objper.Operation == OBJECTPERSISTENCE_OPERATION_SAVE) {
            if (objper.Selection == OBJECTPERSISTENCE_SELECTION_SINGLEOBJECT) {
                // Get selected object
                obj = UAVObjGetByID(objper.ObjectID);
                if (obj == 0) {
                    return;
                }
                // Save selected instance
                retval = UAVObjSave(obj, objper.InstanceID);

                // Not sure why this is needed
                vTaskDelay(10);

                // Verify saving worked
                if (retval == 0) {
                    retval = UAVObjLoad(obj, objper.InstanceID);
                }
            } else if (objper.Selection == OBJECTPERSISTENCE_SELECTION_ALLSETTINGS || objper.Selection == OBJECTPERSISTENCE_SELECTION_ALLOBJECTS) {
                retval = UAVObjSaveSettings();
            } else if (objper.Selection == OBJECTPERSISTENCE_SELECTION_ALLMETAOBJECTS || objper.Selection == OBJECTPERSISTENCE_SELECTION_ALLOBJECTS) {
                retval = UAVObjSaveMetaobjects();
            }
        } else if (objper.Operation == OBJECTPERSISTENCE_OPERATION_DELETE) {
            if (objper.Selection == OBJECTPERSISTENCE_SELECTION_SINGLEOBJECT) {
                // Get selected object
                obj = UAVObjGetByID(objper.ObjectID);
                if (obj == 0) {
                    return;
                }
                // Delete selected instance
                retval = UAVObjDelete(obj, objper.InstanceID);
            } else if (objper.Selection == OBJECTPERSISTENCE_SELECTION_ALLSETTINGS || objper.Selection == OBJECTPERSISTENCE_SELECTION_ALLOBJECTS) {
                retval = UAVObjDeleteSettings();
            } else if (objper.Selection == OBJECTPERSISTENCE_SELECTION_ALLMETAOBJECTS || objper.Selection == OBJECTPERSISTENCE_SELECTION_ALLOBJECTS) {
                retval = UAVObjDeleteMetaobjects();
            }
        } else if (objper.Operation == OBJECTPERSISTENCE_OPERATION_FULLERASE) {
#if defined(PIOS_INCLUDE_FLASH_LOGFS_SETTINGS)
            retval = PIOS_FLASHFS_Format(0);
#else
            retval = -1;
#endif
        }
        switch (retval) {
        case 0:
            objper.Operation = OBJECTPERSISTENCE_OPERATION_COMPLETED;
            ObjectPersistenceSet(&objper);
            break;
        case -1:
            objper.Operation = OBJECTPERSISTENCE_OPERATION_ERROR;
            ObjectPersistenceSet(&objper);
            break;
        default:
            break;
        }
    }
}

/**
 * Called whenever hardware settings changed
 */
static void hwSettingsUpdatedCb(__attribute__((unused)) UAVObjEvent *ev)
{
    HwSettingsData currentHwSettings;

    HwSettingsGet(&currentHwSettings);
    // check whether the Hw Configuration has changed from the one used at boot time
    if (memcmp(&bootHwSettings, &currentHwSettings, sizeof(HwSettingsData)) != 0) {
        ExtendedAlarmsSet(SYSTEMALARMS_ALARM_BOOTFAULT, SYSTEMALARMS_ALARM_ERROR, SYSTEMALARMS_EXTENDEDALARMSTATUS_REBOOTREQUIRED, 0);
    }
}

#ifdef DIAG_TASKS
static void taskMonitorForEachCallback(uint16_t task_id, const struct pios_task_info *task_info, void *context)
{
    TaskInfoData *taskData = (TaskInfoData *)context;

    // By convention, there is a direct mapping between task monitor task_id's and members
    // of the TaskInfoXXXXElem enums
    PIOS_DEBUG_Assert(task_id < TASKINFO_RUNNING_NUMELEM);
    taskData->Running[task_id]        = task_info->is_running ? TASKINFO_RUNNING_TRUE : TASKINFO_RUNNING_FALSE;
    taskData->StackRemaining[task_id] = task_info->stack_remaining;
    taskData->RunningTime[task_id]    = task_info->running_time_percentage;
}
#endif

/**
 * Called periodically to update the I2C statistics
 */
#ifdef DIAG_I2C_WDG_STATS
static void updateI2Cstats()
{
#if defined(PIOS_INCLUDE_I2C)
    I2CStatsData i2cStats;
    I2CStatsGet(&i2cStats);

    struct pios_i2c_fault_history history;
    PIOS_I2C_GetDiagnostics(&history, &i2cStats.event_errors);

    for (uint8_t i = 0; (i < I2C_LOG_DEPTH) && (i < I2CSTATS_EVENT_LOG_NUMELEM); i++) {
        i2cStats.evirq_log[i] = history.evirq[i];
        i2cStats.erirq_log[i] = history.erirq[i];
        i2cStats.event_log[i] = history.event[i];
        i2cStats.state_log[i] = history.state[i];
    }
    i2cStats.last_error_type = history.type;
    I2CStatsSet(&i2cStats);
#endif
}

static void updateWDGstats()
{
    WatchdogStatusData watchdogStatus;

    watchdogStatus.BootupFlags = PIOS_WDG_GetBootupFlags();
    watchdogStatus.ActiveFlags = PIOS_WDG_GetActiveFlags();
    WatchdogStatusSet(&watchdogStatus);
}
#endif /* ifdef DIAG_I2C_WDG_STATS */

/**
 * Called periodically to update the system stats
 */
static uint16_t GetFreeIrqStackSize(void)
{
    uint32_t i = 0x200;

#if !defined(ARCH_POSIX) && !defined(ARCH_WIN32) && defined(CHECK_IRQ_STACK)
    extern uint32_t _irq_stack_top;
    extern uint32_t _irq_stack_end;
    uint32_t pattern    = 0x0000A5A5;
    uint32_t *ptr       = &_irq_stack_end;

#if 1 /* the ugly way accurate but takes more time, useful for debugging */
    uint32_t stack_size = (((uint32_t)&_irq_stack_top - (uint32_t)&_irq_stack_end) & ~3) / 4;

    for (i = 0; i < stack_size; i++) {
        if (ptr[i] != pattern) {
            i = i * 4;
            break;
        }
    }
#else /* faster way but not accurate */
    if (*(volatile uint32_t *)((uint32_t)ptr + IRQSTACK_LIMIT_CRITICAL) != pattern) {
        i = IRQSTACK_LIMIT_CRITICAL - 1;
    } else if (*(volatile uint32_t *)((uint32_t)ptr + IRQSTACK_LIMIT_WARNING) != pattern) {
        i = IRQSTACK_LIMIT_WARNING - 1;
    } else {
        i = IRQSTACK_LIMIT_WARNING;
    }
#endif
#endif /* if !defined(ARCH_POSIX) && !defined(ARCH_WIN32) && defined(CHECK_IRQ_STACK) */
    return i;
}

/**
 * Called periodically to update the system stats
 */
static void updateStats()
{
    static portTickType lastTickCount = 0;
    SystemStatsData stats;

    // Get stats and update
    SystemStatsGet(&stats);
    stats.FlightTime    = xTaskGetTickCount() * portTICK_RATE_MS;
#if defined(ARCH_POSIX) || defined(ARCH_WIN32)
    // POSIX port of FreeRTOS doesn't have xPortGetFreeHeapSize()
    stats.HeapRemaining = 10240;
#else
    stats.HeapRemaining = xPortGetFreeHeapSize();
#endif

    // Get Irq stack status
    stats.IRQStackRemaining = GetFreeIrqStackSize();

    // When idleCounterClear was not reset by the idle-task, it means the idle-task did not run
    if (idleCounterClear) {
        idleCounter = 0;
    }
#if !defined(ARCH_POSIX) && !defined(ARCH_WIN32)
#ifdef PIOS_INCLUDE_FLASH
    if (pios_uavo_settings_fs_id) {
        PIOS_FLASHFS_GetStats(pios_uavo_settings_fs_id, &fsStats);
        stats.SysSlotsFree   = fsStats.num_free_slots;
        stats.SysSlotsActive = fsStats.num_active_slots;
    }
    if (pios_user_fs_id) {
        PIOS_FLASHFS_GetStats(pios_user_fs_id, &fsStats);
        stats.UsrSlotsFree   = fsStats.num_free_slots;
        stats.UsrSlotsActive = fsStats.num_active_slots;
    }
#endif // PIOS_INCLUDE_FLASH
#endif
    portTickType now = xTaskGetTickCount();
    if (now > lastTickCount) {
        uint32_t dT = (xTaskGetTickCount() - lastTickCount) * portTICK_RATE_MS; // in ms
        stats.CPULoad = 100 - (uint8_t)roundf(100.0f * ((float)idleCounter / ((float)dT / 1000.0f)) / (float)IDLE_COUNTS_PER_SEC_AT_NO_LOAD);
    } // else: TickCount has wrapped, do not calc now
    lastTickCount    = now;
    idleCounterClear = 1;

#if defined(PIOS_INCLUDE_ADC) && defined(PIOS_ADC_USE_TEMP_SENSOR)
#if defined(STM32F4XX)
    float temp_voltage = 3.3f * PIOS_ADC_PinGet(3) / ((1 << 12) - 1);
    const float STM32_TEMP_V25 = 0.76f; /* V */
    const float STM32_TEMP_AVG_SLOPE = 2.5f; /* mV/C */
    stats.CPUTemp = (temp_voltage - STM32_TEMP_V25) * 1000.0f / STM32_TEMP_AVG_SLOPE + 25.0f;
#else
    float temp_voltage = 3.3f * PIOS_ADC_PinGet(0) / ((1 << 12) - 1);
    const float STM32_TEMP_V25 = 1.43f; /* V */
    const float STM32_TEMP_AVG_SLOPE = 4.3f; /* mV/C */
    stats.CPUTemp = (temp_voltage - STM32_TEMP_V25) * 1000.0f / STM32_TEMP_AVG_SLOPE + 25.0f;
#endif
#endif
    SystemStatsSet(&stats);
}

/**
 * Update system alarms
 */
static void updateSystemAlarms()
{
    SystemStatsData stats;
    UAVObjStats objStats;
    EventStats evStats;

    SystemStatsGet(&stats);

    // Check heap, IRQ stack and malloc failures
    if (mallocFailed || (stats.HeapRemaining < HEAP_LIMIT_CRITICAL)
#if !defined(ARCH_POSIX) && !defined(ARCH_WIN32) && defined(CHECK_IRQ_STACK)
        || (stats.IRQStackRemaining < IRQSTACK_LIMIT_CRITICAL)
#endif
        ) {
        AlarmsSet(SYSTEMALARMS_ALARM_OUTOFMEMORY, SYSTEMALARMS_ALARM_CRITICAL);
    } else if ((stats.HeapRemaining < HEAP_LIMIT_WARNING)
#if !defined(ARCH_POSIX) && !defined(ARCH_WIN32) && defined(CHECK_IRQ_STACK)
               || (stats.IRQStackRemaining < IRQSTACK_LIMIT_WARNING)
#endif
               ) {
        AlarmsSet(SYSTEMALARMS_ALARM_OUTOFMEMORY, SYSTEMALARMS_ALARM_WARNING);
    } else {
        AlarmsClear(SYSTEMALARMS_ALARM_OUTOFMEMORY);
    }

    // Check CPU load
    if (stats.CPULoad > CPULOAD_LIMIT_CRITICAL) {
        AlarmsSet(SYSTEMALARMS_ALARM_CPUOVERLOAD, SYSTEMALARMS_ALARM_CRITICAL);
    } else if (stats.CPULoad > CPULOAD_LIMIT_WARNING) {
        AlarmsSet(SYSTEMALARMS_ALARM_CPUOVERLOAD, SYSTEMALARMS_ALARM_WARNING);
    } else {
        AlarmsClear(SYSTEMALARMS_ALARM_CPUOVERLOAD);
    }

    // Check for stack overflow
    if (stackOverflow) {
        AlarmsSet(SYSTEMALARMS_ALARM_STACKOVERFLOW, SYSTEMALARMS_ALARM_CRITICAL);
    } else {
        AlarmsClear(SYSTEMALARMS_ALARM_STACKOVERFLOW);
    }

    // Check for event errors
    UAVObjGetStats(&objStats);
    EventGetStats(&evStats);
    UAVObjClearStats();
    EventClearStats();
    if (objStats.eventCallbackErrors > 0 || objStats.eventQueueErrors > 0 || evStats.eventErrors > 0) {
        AlarmsSet(SYSTEMALARMS_ALARM_EVENTSYSTEM, SYSTEMALARMS_ALARM_WARNING);
    } else {
        AlarmsClear(SYSTEMALARMS_ALARM_EVENTSYSTEM);
    }

    if (objStats.lastCallbackErrorID || objStats.lastQueueErrorID || evStats.lastErrorID) {
        SystemStatsData sysStats;
        SystemStatsGet(&sysStats);
        sysStats.EventSystemWarningID    = evStats.lastErrorID;
        sysStats.ObjectManagerCallbackID = objStats.lastCallbackErrorID;
        sysStats.ObjectManagerQueueID    = objStats.lastQueueErrorID;
        SystemStatsSet(&sysStats);
    }
}

/**
 * Called by the RTOS when the CPU is idle, used to measure the CPU idle time.
 */
void vApplicationIdleHook(void)
{
    // Called when the scheduler has no tasks to run
    if (idleCounterClear == 0) {
        ++idleCounter;
    } else {
        idleCounter = 0;
        idleCounterClear = 0;
    }
}

/**
 * Called by the RTOS when a stack overflow is detected.
 */
#define DEBUG_STACK_OVERFLOW 0
void vApplicationStackOverflowHook(__attribute__((unused)) xTaskHandle *pxTask,
                                   __attribute__((unused)) signed portCHAR *pcTaskName)
{
    stackOverflow = true;
#if DEBUG_STACK_OVERFLOW
    static volatile bool wait_here = true;
    while (wait_here) {
        ;
    }
    wait_here = true;
#endif
}

/**
 * Called by the RTOS when a malloc call fails.
 */
#define DEBUG_MALLOC_FAILURES 0
void vApplicationMallocFailedHook(void)
{
    mallocFailed = true;
#if DEBUG_MALLOC_FAILURES
    static volatile bool wait_here = true;
    while (wait_here) {
        ;
    }
    wait_here = true;
#endif
}

/**
 * @}
 * @}
 */
