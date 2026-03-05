// ============================================================================
// network_rest_test.cpp — WiFi connectivity and REST API validation
//
// Build:   pio run -e network_test -t upload
// Monitor: pio device monitor -e network_test
//
// Tests:
//   1. WiFi connection and IP address acquisition
//   2. WiFi reconnect after deliberate disconnect
//   3. REST API server startup on /status, /open, /close, /stop, /home, /reset
//   4. HTTP client self-test: GET /status — validate JSON response fields
//   5. HTTP client self-test: POST /open, /close, /stop — validate 200 OK
//   6. HTTP client self-test: POST /home, /reset — validate 200 OK
//   7. HTTP client self-test: GET /nonexistent — validate 404
//   8. Command queue round-trip via REST endpoints
//   9. WiFi manager state machine transitions
//  10. REST API stop/restart cycle
//
// Prerequisites:
//   - WiFi credentials in config.h must be valid
//   - ESP32 must be in range of the configured WiFi AP
//
// NOTE: Uses HTTPClient to make requests to itself (localhost).
// ============================================================================

#include <Arduino.h>
#include <esp_task_wdt.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#include "config.h"
#include "core/system_types.h"
#include "core/shared_state.h"
#include "network/wifi_manager.h"
#include "network/rest_api.h"
#include "audio/buzzer.h"

static Buzzer buzzer;
static SharedState  sharedState;
static WifiManager  wifiMgr(sharedState);
static RestApi      restApi(sharedState);

static uint8_t testNum   = 0;
static uint8_t passCount = 0;
static uint8_t failCount = 0;

static void check(const char* name, bool passed, const char* detail = nullptr) {
    testNum++;
    if (passed) passCount++;
    else failCount++;
    Serial.printf("  [%2d] %-48s %s\n", testNum, name, passed ? "PASS" : "** FAIL **");
    if (detail) Serial.printf("        %s\n", detail);
}

// Helper: build URL from local IP
static String baseUrl() {
    return "http://" + WiFi.localIP().toString() + ":" + String(cfg::network::HTTP_PORT);
}

// Helper: HTTP GET, returns status code (0 on failure)
static int httpGet(const String& path, String& body) {
    HTTPClient http;
    http.begin(baseUrl() + path);
    int code = http.GET();
    if (code > 0) {
        body = http.getString();
    }
    http.end();
    return code;
}

// Helper: HTTP POST, returns status code
static int httpPost(const String& path, String& body) {
    HTTPClient http;
    http.begin(baseUrl() + path);
    http.addHeader("Content-Type", "application/json");
    int code = http.POST("{}");
    if (code > 0) {
        body = http.getString();
    }
    http.end();
    return code;
}

// ============================================================================
// Test 1: WiFi connection
// ============================================================================
static void testWifiConnect() {
    Serial.println("\n--- Test Group: WiFi Connection ---");

    wifiMgr.begin();

    // Poll WiFiManager until connected or timeout
    uint32_t startMs = millis();
    uint32_t timeout = cfg::network::WIFI_CONNECT_TIMEOUT_MS + 5000;

    while (!wifiMgr.isConnected() && (millis() - startMs) < timeout) {
        wifiMgr.update();
        buzzer.update();
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    bool connected = wifiMgr.isConnected();
    check("WiFi connects to AP", connected);

    if (connected) {
        char detail[64];
        snprintf(detail, sizeof(detail), "IP: %s  RSSI: %d dBm",
                 WiFi.localIP().toString().c_str(), WiFi.RSSI());
        check("IP address acquired", WiFi.localIP() != IPAddress(0, 0, 0, 0), detail);

        bool strongSignal = (WiFi.RSSI() > -80);
        snprintf(detail, sizeof(detail), "RSSI: %d dBm", WiFi.RSSI());
        check("Signal strength adequate (> -80 dBm)", strongSignal, detail);

        // Verify shared state was updated
        check("SharedState wifi flag set", sharedState.getStatus().wifiConnected);
    } else {
        check("IP address acquired", false, "WiFi not connected - check SSID/pass in config.h");
        check("Signal strength adequate", false, "WiFi not connected");
        check("SharedState wifi flag set", false, "WiFi not connected");

        Serial.println("\n  *** WiFi connection failed — skipping REST API tests ***");
        Serial.println("  Verify cfg::network::WIFI_SSID and WIFI_PASS in config.h\n");
        return; // Can't test REST without WiFi
    }
}

// ============================================================================
// Test 2: WiFi manager state machine
// ============================================================================
static void testWifiStateMachine() {
    Serial.println("\n--- Test Group: WiFi State Machine ---");

    if (!wifiMgr.isConnected()) {
        check("WiFi SM: connected state", false, "Skipped — not connected");
        check("WiFi SM: disable → OFF", false, "Skipped");
        check("WiFi SM: enable → reconnect", false, "Skipped");
        return;
    }

    check("WiFi SM: state is CONNECTED",
          wifiMgr.state() == WifiManager::State::CONNECTED);

    // Disable WiFi
    wifiMgr.disable();
    check("WiFi SM: disable → OFF state",
          wifiMgr.state() == WifiManager::State::OFF);

    bool disconnected = (WiFi.status() != WL_CONNECTED);
    check("WiFi actually disconnected", disconnected);

    // Re-enable and wait for reconnect
    wifiMgr.enable();
    check("WiFi SM: enable → DISCONNECTED state",
          wifiMgr.state() == WifiManager::State::DISCONNECTED);

    uint32_t startMs = millis();
    while (!wifiMgr.isConnected() && (millis() - startMs) < 20000) {
        wifiMgr.update();
        buzzer.update();
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    bool reconnected = wifiMgr.isConnected();
    char detail[64];
    snprintf(detail, sizeof(detail), "Reconnected in %lu ms", millis() - startMs);
    check("WiFi reconnects after enable", reconnected, reconnected ? detail : "Timeout");
}

// ============================================================================
// Test 3: REST API server startup
// ============================================================================
static void testRestApiStartup() {
    Serial.println("\n--- Test Group: REST API Server ---");

    if (!wifiMgr.isConnected()) {
        check("REST API begin()", false, "Skipped — no WiFi");
        return;
    }

    restApi.begin();
    check("REST API begin() completes", true);
    check("REST API isRunning()", restApi.isRunning());

    char detail[80];
    snprintf(detail, sizeof(detail), "Listening on %s:%d",
             WiFi.localIP().toString().c_str(), cfg::network::HTTP_PORT);
    Serial.printf("        %s\n", detail);
}

// ============================================================================
// Test 4: GET /status — validate JSON response
// ============================================================================
static void testGetStatus() {
    Serial.println("\n--- Test Group: GET /status ---");

    if (!restApi.isRunning()) {
        check("GET /status returns 200", false, "Skipped — API not running");
        return;
    }

    // The REST API runs in the same task context, so we need to pump
    // handleClient() between our HTTP requests.
    // Since HTTPClient and WebServer both run on the same core, we
    // yield between request and server handling.

    // Set some known state first
    sharedState.setState(SystemState::IDLE);
    sharedState.setPosition(42);
    sharedState.setHomed(true);
    sharedState.setVelocity(1.5f);
    sharedState.setCurrentMa(250.0f);
    sharedState.setUptimeMs(millis());

    // Pump the server in a background approach:
    // Start a short-lived task to handle the HTTP server while we make requests
    TaskHandle_t serverTask = nullptr;
    static volatile bool serverRunning = true;
    serverRunning = true;

    xTaskCreatePinnedToCore(
        [](void* param) {
            while (serverRunning) {
                restApi.handleClient();
                buzzer.update();
                vTaskDelay(pdMS_TO_TICKS(5));
            }
            vTaskDelete(NULL);
        },
        "ServerPump", 4096, nullptr, 2, &serverTask, 0
    );

    vTaskDelay(pdMS_TO_TICKS(100)); // Let server task start

    String body;
    int code = httpGet("/status", body);

    check("GET /status returns 200", code == 200);

    if (code == 200) {
        char detail[128];
        snprintf(detail, sizeof(detail), "Response: %d bytes", body.length());
        check("Response body non-empty", body.length() > 10, detail);

        // Parse JSON
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, body);
        check("Response is valid JSON", !err);

        if (!err) {
            // Validate expected fields exist (ArduinoJson v7 idiom)
            check("JSON has 'state' field", doc["state"].is<const char*>());
            check("JSON has 'position' field", doc["position"].is<int>());
            check("JSON has 'homed' field", doc["homed"].is<bool>());
            check("JSON has 'velocity' field", doc["velocity"].is<float>());
            check("JSON has 'current_ma' field", doc["current_ma"].is<float>());
            check("JSON has 'faults' field", doc["faults"].is<uint32_t>());
            check("JSON has 'fault_names' array", doc["fault_names"].is<JsonArray>());
            check("JSON has 'wifi' field", doc["wifi"].is<bool>());
            check("JSON has 'uptime_ms' field", doc["uptime_ms"].is<unsigned long>());
            check("JSON has 'target' field", doc["target"].is<int>());
            check("JSON has 'limit_switch' field", doc["limit_switch"].is<bool>());
            check("JSON has 'homing_state' field", doc["homing_state"].is<const char*>());

            // Validate values match what we set
            const char* stateStr = doc["state"] | "null";
            check("state = 'idle'", strcmp(stateStr, "idle") == 0);

            int pos = doc["position"] | -1;
            check("position = 42", pos == 42);

            bool homed = doc["homed"] | false;
            check("homed = true", homed);

            bool wifi = doc["wifi"] | false;
            check("wifi = true", wifi);

            // Faults should be 0 (none)
            int faults = doc["faults"] | -1;
            check("faults = 0 (none)", faults == 0);
        }
    }

    // Stop server pump task
    serverRunning = false;
    vTaskDelay(pdMS_TO_TICKS(100));
}

// ============================================================================
// Test 5: POST endpoints — /open, /close, /stop
// ============================================================================
static void testPostCommands() {
    Serial.println("\n--- Test Group: POST Commands ---");

    if (!restApi.isRunning()) {
        check("POST /open returns 200", false, "Skipped — API not running");
        return;
    }

    // Set system to IDLE + homed so commands are accepted
    sharedState.setState(SystemState::IDLE);
    sharedState.setHomed(true);

    // Pump server in background
    static volatile bool serverRunning2 = true;
    serverRunning2 = true;
    TaskHandle_t serverTask = nullptr;

    xTaskCreatePinnedToCore(
        [](void* param) {
            while (serverRunning2) {
                restApi.handleClient();
                buzzer.update();
                vTaskDelay(pdMS_TO_TICKS(5));
            }
            vTaskDelete(NULL);
        },
        "ServerPump2", 4096, nullptr, 2, &serverTask, 0
    );

    vTaskDelay(pdMS_TO_TICKS(100));

    // POST /open
    String body;
    int code = httpPost("/open", body);
    check("POST /open returns 200", code == 200);
    if (code == 200) {
        JsonDocument doc;
        deserializeJson(doc, body);
        const char* status = doc["status"] | "null";
        const char* action = doc["action"] | "null";
        check("POST /open status=ok", strcmp(status, "ok") == 0);
        check("POST /open action=opening", strcmp(action, "opening") == 0);
    }

    // Drain the command queue
    Command cmd;
    bool gotOpen = false;
    while (sharedState.receiveCommand(cmd)) {
        if (cmd.type == CommandType::OPEN) gotOpen = true;
    }
    check("OPEN command was queued", gotOpen);

    // POST /close
    code = httpPost("/close", body);
    check("POST /close returns 200", code == 200);
    while (sharedState.receiveCommand(cmd)) {} // drain

    // POST /stop
    code = httpPost("/stop", body);
    check("POST /stop returns 200", code == 200);
    while (sharedState.receiveCommand(cmd)) {} // drain

    // POST /home
    code = httpPost("/home", body);
    check("POST /home returns 200", code == 200);
    while (sharedState.receiveCommand(cmd)) {} // drain

    // POST /reset
    code = httpPost("/reset", body);
    check("POST /reset returns 200", code == 200);
    while (sharedState.receiveCommand(cmd)) {} // drain

    // GET /nonexistent — should return 404
    code = httpGet("/nonexistent", body);
    check("GET /nonexistent returns 404", code == 404);
    if (code == 404) {
        JsonDocument doc;
        deserializeJson(doc, body);
        const char* msg = doc["message"] | "null";
        check("404 response has error message", strcmp(msg, "not_found") == 0);
    }

    // Stop server pump
    serverRunning2 = false;
    vTaskDelay(pdMS_TO_TICKS(100));
}

// ============================================================================
// Test 6: REST API stop/restart
// ============================================================================
static void testRestApiStopRestart() {
    Serial.println("\n--- Test Group: REST API Stop/Restart ---");

    if (!wifiMgr.isConnected()) {
        check("REST API stop", false, "Skipped — no WiFi");
        return;
    }

    restApi.stop();
    check("REST API stopped", !restApi.isRunning());

    // Try to connect — should fail (connection refused)
    // Don't actually test this since the port might still be bound briefly

    // Restart
    restApi.begin();
    check("REST API restarted", restApi.isRunning());

    // Quick server pump + self-test
    static volatile bool serverRunning3 = true;
    serverRunning3 = true;
    TaskHandle_t serverTask = nullptr;

    xTaskCreatePinnedToCore(
        [](void* param) {
            while (serverRunning3) {
                restApi.handleClient();
                buzzer.update();
                vTaskDelay(pdMS_TO_TICKS(5));
            }
            vTaskDelete(NULL);
        },
        "ServerPump3", 4096, nullptr, 2, &serverTask, 0
    );

    vTaskDelay(pdMS_TO_TICKS(200));

    String body;
    int code = httpGet("/status", body);
    check("GET /status works after restart", code == 200);

    serverRunning3 = false;
    vTaskDelay(pdMS_TO_TICKS(100));
}

// ============================================================================
// Test 7: Command queue stress — rapid-fire POST requests
// ============================================================================
static void testCommandQueueStress() {
    Serial.println("\n--- Test Group: Command Queue Stress ---");

    if (!restApi.isRunning()) {
        check("Queue stress", false, "Skipped — API not running");
        return;
    }

    // Drain any pending commands
    Command cmd;
    while (sharedState.receiveCommand(cmd)) {}

    // Set system ready
    sharedState.setState(SystemState::IDLE);
    sharedState.setHomed(true);

    // Pump server
    static volatile bool serverRunning4 = true;
    serverRunning4 = true;
    TaskHandle_t serverTask = nullptr;

    xTaskCreatePinnedToCore(
        [](void* param) {
            while (serverRunning4) {
                restApi.handleClient();
                buzzer.update();
                vTaskDelay(pdMS_TO_TICKS(2));
            }
            vTaskDelete(NULL);
        },
        "ServerPump4", 4096, nullptr, 2, &serverTask, 0
    );

    vTaskDelay(pdMS_TO_TICKS(100));

    // Send 8 rapid commands (queue capacity is 8)
    int successCount = 0;
    String body;
    for (int i = 0; i < 8; i++) {
        int code = httpPost("/stop", body);
        if (code == 200) successCount++;
        // Small delay to let server process
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    char detail[48];
    snprintf(detail, sizeof(detail), "%d/8 accepted", successCount);
    check("8 rapid commands: most accepted", successCount >= 6, detail);

    // 9th should get 503 (queue full) if queue hasn't been drained
    // But since there's a delay between requests, some may already be drained
    // Just verify the system stays stable
    int code = httpPost("/stop", body);
    check("System stable after rapid-fire", code == 200 || code == 503);

    // Drain
    int drained = 0;
    while (sharedState.receiveCommand(cmd)) drained++;
    snprintf(detail, sizeof(detail), "Drained %d commands", drained);
    check("Commands drained from queue", drained > 0, detail);

    serverRunning4 = false;
    vTaskDelay(pdMS_TO_TICKS(100));
}

// ============================================================================
// Test 8: Response timing — /status should respond within 100ms
// ============================================================================
static void testResponseTiming() {
    Serial.println("\n--- Test Group: Response Timing ---");

    if (!restApi.isRunning()) {
        check("Response timing", false, "Skipped — API not running");
        return;
    }

    static volatile bool serverRunning5 = true;
    serverRunning5 = true;
    TaskHandle_t serverTask = nullptr;

    xTaskCreatePinnedToCore(
        [](void* param) {
            while (serverRunning5) {
                restApi.handleClient();
                buzzer.update();
                vTaskDelay(pdMS_TO_TICKS(2));
            }
            vTaskDelete(NULL);
        },
        "ServerPump5", 4096, nullptr, 2, &serverTask, 0
    );

    vTaskDelay(pdMS_TO_TICKS(100));

    // Measure 5 GET /status requests
    uint32_t totalMs = 0;
    uint32_t maxMs = 0;
    int goodCount = 0;

    for (int i = 0; i < 5; i++) {
        String body;
        uint32_t start = millis();
        int code = httpGet("/status", body);
        uint32_t elapsed = millis() - start;

        if (code == 200) {
            goodCount++;
            totalMs += elapsed;
            if (elapsed > maxMs) maxMs = elapsed;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    if (goodCount > 0) {
        uint32_t avgMs = totalMs / goodCount;
        char detail[80];
        snprintf(detail, sizeof(detail), "avg=%lu ms, max=%lu ms (%d/5 OK)", avgMs, maxMs, goodCount);
        check("Average response < 100ms", avgMs < 100, detail);
        check("Max response < 250ms", maxMs < 250, detail);
    } else {
        check("Average response < 100ms", false, "All requests failed");
        check("Max response < 250ms", false, "All requests failed");
    }

    serverRunning5 = false;
    vTaskDelay(pdMS_TO_TICKS(100));
}

// ============================================================================
void setup() {
    buzzer.begin();
    wifiMgr.setBuzzer(&buzzer);
    Serial.begin(115200);
    vTaskDelay(pdMS_TO_TICKS(500));

    Serial.println("\n=============================================");
    Serial.println("  Network & REST API Validation Test Suite");
    Serial.println("=============================================");
    Serial.printf("  SSID: %s\n", cfg::network::WIFI_SSID);
    Serial.printf("  Port: %d\n", cfg::network::HTTP_PORT);
    Serial.println("  Requires valid WiFi credentials in config.h\n");

    testWifiConnect();
    testWifiStateMachine();
    testRestApiStartup();
    testGetStatus();
    testPostCommands();
    testRestApiStopRestart();
    testCommandQueueStress();
    testResponseTiming();

    // Clean up
    restApi.stop();
    wifiMgr.disable();

    Serial.println("\n=============================================");
    Serial.println("  RESULTS");
    Serial.println("=============================================");
    Serial.printf("  Passed: %d\n", passCount);
    Serial.printf("  Failed: %d\n", failCount);
    Serial.printf("  Total:  %d\n", testNum);
    if (failCount == 0) {
        Serial.println("\n  >>> ALL TESTS PASSED <<<");
    } else {
        Serial.println("\n  >>> FAILURES DETECTED <<<");
    }
    Serial.println("=============================================\n");
}

void loop() {
    buzzer.update();
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(50));
}
