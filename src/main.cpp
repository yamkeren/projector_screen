// ============================================================================
// main.cpp — Entry point for Rolling Projector Screen Controller
//
// Architecture:
//   All subsystem objects are instantiated here and passed by reference.
//   FreeRTOS tasks are pinned to cores with explicit priorities.
//   No global mutable state — all sharing goes through SharedState.
// ============================================================================

#include <Arduino.h>
#include <esp_task_wdt.h>
#include <soc/rtc_cntl_reg.h>

#include "config.h"
#include "core/system_types.h"
#include "core/shared_state.h"
#include "motion/motion_controller.h"
#include "safety/current_sensor.h"
#include "safety/fault_manager.h"
#include "network/wifi_manager.h"
#include "network/rest_api.h"
#include "power/power_manager.h"
#include "storage/persistent_storage.h"
#include "app/homing_machine.h"
#include "app/app_controller.h"
#include "audio/buzzer.h"

// ============================================================================
// Subsystem instances — owned by main, passed by reference
// ============================================================================
static SharedState       sharedState;
static MotionController  motionCtrl;
static CurrentSensor     currentSensor;
static FaultManager      faultMgr(sharedState, motionCtrl, currentSensor);
static WifiManager       wifiMgr(sharedState);
static RestApi           restApi(sharedState);
static PersistentStorage storage;
static HomingMachine     homingMachine(sharedState, motionCtrl);
static AppController     appCtrl(sharedState, motionCtrl, faultMgr, storage, homingMachine);
static PowerManager      powerMgr(sharedState, wifiMgr);
static Buzzer            buzzer;

// ============================================================================
// Task context structs — bundled references for each task function
// ============================================================================

struct MotionTaskCtx {
    MotionController& motion;
    AppController&    app;
    SharedState&      shared;
};

struct SafetyTaskCtx {
    FaultManager& faults;
};

struct NetworkTaskCtx {
    WifiManager& wifi;
    RestApi&     api;
    SharedState& shared;
};

struct AppTaskCtx {
    AppController& app;
    SharedState&   shared;
    MotionController& motion;
    Buzzer&        buzzer;
};

struct PowerTaskCtx {
    PowerManager&  power;
    AppController& app;
    SharedState&   shared;
};

// ============================================================================
// Task: MotionTask — Highest priority, deterministic 5ms loop on Core 1
// ============================================================================
static void motionTask(void* param) {
    auto* ctx = static_cast<MotionTaskCtx*>(param);
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        // Update motion controller (encoder read, PID, motor drive)
        ctx->motion.update();

        // Update homing state machine if active
        ctx->app.updateHoming();

        // Check if motion is complete
        if (ctx->motion.atTarget() &&
            ctx->shared.getState() == SystemState::MOVING) {
            ctx->shared.setState(SystemState::IDLE);
            ctx->shared.setEvent(EVT_MOTION_COMPLETE);
        }

        vTaskDelayUntil(&wake, pdMS_TO_TICKS(cfg::task::MOTION_PERIOD_MS));
    }
}

// ============================================================================
// Task: SafetyTask — High priority, 50ms loop on Core 1
// ============================================================================
static void safetyTask(void* param) {
    auto* ctx = static_cast<SafetyTaskCtx*>(param);
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        ctx->faults.evaluate();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(cfg::task::SAFETY_PERIOD_MS));
    }
}

// ============================================================================
// Task: NetworkTask — Medium priority on Core 0 (protocol stack core)
// ============================================================================
static void networkTask(void* param) {
    auto* ctx = static_cast<NetworkTaskCtx*>(param);
    TickType_t wake = xTaskGetTickCount();

    bool apiStarted = false;

    for (;;) {
        ctx->wifi.update();

        if (ctx->wifi.isConnected()) {
            if (!apiStarted) {
                ctx->api.begin();
                apiStarted = true;
                log_i("REST API started on %s:%d",
                      ctx->wifi.localIP().toString().c_str(),
                      cfg::network::HTTP_PORT);
            }
            ctx->api.handleClient();
        } else {
            if (apiStarted) {
                ctx->api.stop();
                apiStarted = false;
            }
        }

        vTaskDelayUntil(&wake, pdMS_TO_TICKS(cfg::task::NETWORK_PERIOD_MS));
    }
}

// ============================================================================
// Task: AppStateTask — Medium priority on Core 0
// ============================================================================
static void appStateTask(void* param) {
    auto* ctx = static_cast<AppTaskCtx*>(param);
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        ctx->app.update();
        ctx->buzzer.update();   // Tick buzzer playback FSM

        // Notify power manager of activity when moving
        if (ctx->shared.getState() == SystemState::MOVING ||
            ctx->shared.getState() == SystemState::HOMING) {
            // Activity is tracked by PowerManager::update() checking state
        }

        vTaskDelayUntil(&wake, pdMS_TO_TICKS(cfg::task::APP_PERIOD_MS));
    }
}

// ============================================================================
// Task: PowerTask — Lowest priority on Core 0
// ============================================================================
static void powerTask(void* param) {
    auto* ctx = static_cast<PowerTaskCtx*>(param);
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        ctx->power.update();

        // Feed watchdog from lowest-priority task (canary)
        esp_task_wdt_reset();

        vTaskDelayUntil(&wake, pdMS_TO_TICKS(cfg::task::POWER_PERIOD_MS));
    }
}

// ============================================================================
// Task context storage (static lifetime)
// ============================================================================
static MotionTaskCtx  motionCtx{motionCtrl, appCtrl, sharedState};
static SafetyTaskCtx  safetyCtx{faultMgr};
static NetworkTaskCtx networkCtx{wifiMgr, restApi, sharedState};
static AppTaskCtx     appCtx{appCtrl, sharedState, motionCtrl, buzzer};
static PowerTaskCtx   powerCtx{powerMgr, appCtrl, sharedState};

// ============================================================================
// Arduino setup() — initialization sequence
// ============================================================================
void setup() {
    Serial.begin(115200);
    vTaskDelay(pdMS_TO_TICKS(100)); // Allow UART to stabilize

    log_i("=== Projector Screen Controller v1.0 ===");
    log_i("Firmware build: %s %s", __DATE__, __TIME__);

    // --- Check reset reason -------------------------------------------------
    esp_reset_reason_t resetReason = esp_reset_reason();
    if (resetReason == ESP_RST_BROWNOUT) {
        log_e("BROWNOUT reset detected!");
        sharedState.addFault(FaultCode::BROWNOUT);
    }
    if (resetReason == ESP_RST_TASK_WDT || resetReason == ESP_RST_WDT) {
        log_e("Watchdog reset detected!");
        sharedState.addFault(FaultCode::WATCHDOG_RESET);
    }

    // --- Initialize subsystems in dependency order --------------------------
    log_i("Initializing storage...");
    if (!storage.begin()) {
        log_e("NVS init failed!");
    }

    log_i("Initializing motion controller...");
    motionCtrl.begin();

    log_i("Initializing current sensor...");
    if (!currentSensor.begin()) {
        log_w("INA226 init failed — current monitoring disabled");
    }

    log_i("Initializing fault manager...");
    faultMgr.begin();

    log_i("Initializing buzzer...");
    buzzer.begin();

    log_i("Initializing WiFi...");
    wifiMgr.begin();
    wifiMgr.setBuzzer(&buzzer);

    log_i("Initializing power manager...");
    powerMgr.begin();
    // Register pre-deep-sleep callback: stop motor + save NVS
    powerMgr.setPreSleepCallback([]() {
        motionCtrl.emergencyStop();
        appCtrl.prepareForSleep();
    });

    log_i("Initializing application controller...");
    appCtrl.begin();
    appCtrl.setBuzzer(&buzzer);
    homingMachine.setBuzzer(&buzzer);

    // --- Configure hardware watchdog ----------------------------------------
    esp_task_wdt_init(cfg::watchdog::TIMEOUT_SEC, true); // true = panic on timeout
    esp_task_wdt_add(NULL); // Add current task (if looper existed, but we use it in PowerTask)

    // --- Create FreeRTOS tasks ----------------------------------------------
    log_i("Starting FreeRTOS tasks...");

    TaskHandle_t hMotion = nullptr;
    TaskHandle_t hSafety = nullptr;
    TaskHandle_t hNetwork = nullptr;
    TaskHandle_t hApp = nullptr;
    TaskHandle_t hPower = nullptr;

    xTaskCreatePinnedToCore(
        motionTask, "Motion",
        cfg::task::MOTION_STACK, &motionCtx,
        cfg::task::MOTION_PRIO, &hMotion, cfg::task::MOTION_CORE);

    xTaskCreatePinnedToCore(
        safetyTask, "Safety",
        cfg::task::SAFETY_STACK, &safetyCtx,
        cfg::task::SAFETY_PRIO, &hSafety, cfg::task::SAFETY_CORE);

    xTaskCreatePinnedToCore(
        networkTask, "Network",
        cfg::task::NETWORK_STACK, &networkCtx,
        cfg::task::NETWORK_PRIO, &hNetwork, cfg::task::NETWORK_CORE);

    xTaskCreatePinnedToCore(
        appStateTask, "AppState",
        cfg::task::APP_STACK, &appCtx,
        cfg::task::APP_PRIO, &hApp, cfg::task::APP_CORE);

    xTaskCreatePinnedToCore(
        powerTask, "Power",
        cfg::task::POWER_STACK, &powerCtx,
        cfg::task::POWER_PRIO, &hPower, cfg::task::POWER_CORE);

    // Add power task to watchdog (it feeds WDT as canary)
    if (hPower) {
        esp_task_wdt_add(hPower);
    }

    log_i("All tasks started. System operational.");

    // If no faults from reset, and not homed, system is in BOOT
    // User must send /home command or system will attempt to resume from NVS
    if (sharedState.getState() == SystemState::BOOT) {
        log_i("System in BOOT state — homing recommended");
    }
}

// ============================================================================
// Arduino loop() — unused, all work in FreeRTOS tasks
// Delete the Arduino loopTask to free resources
// ============================================================================
void loop() {
    // Yield indefinitely — all work is in dedicated tasks
    vTaskDelete(NULL);
}
