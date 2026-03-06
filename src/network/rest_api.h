#pragma once
// ============================================================================
// rest_api.h — Async REST API server with OTA support
// Uses ESPAsyncWebServer for fully non-blocking HTTP + ElegantOTA for /update
// ============================================================================

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ElegantOTA.h>
#include "config.h"
#include "core/system_types.h"
#include "core/shared_state.h"
#include "core/status_json.h"

class RestApi {
public:
    explicit RestApi(SharedState& shared)
        : _shared(shared), _server(cfg::network::HTTP_PORT) {}

    void begin() {
        // --- Status endpoint ------------------------------------------------
        _server.on("/status", HTTP_GET, [this](AsyncWebServerRequest* request) {
            SystemStatus st = _shared.getStatus();
            char buf[512];
            buildStatusJson(st, buf, sizeof(buf));
            request->send(200, "application/json", buf);
        });

        // --- Command endpoints (table-driven) -------------------------------
        struct CmdRoute {
            const char* path;
            CommandType type;
            int32_t     param;
            const char* action;
        };

        static const CmdRoute routes[] = {
            {"/open",  CommandType::OPEN,        cfg::motion::SCREEN_OPEN_POS,  "opening"},
            {"/close", CommandType::CLOSE,       cfg::motion::SCREEN_CLOSE_POS, "closing"},
            {"/stop",  CommandType::STOP,        0,                             "stopped"},
            {"/home",  CommandType::HOME,        0,                             "homing"},
            {"/reset", CommandType::RESET_FAULT, 0,                             "reset_requested"},
        };

        for (const auto& r : routes) {
            _server.on(r.path, HTTP_POST, [this, r](AsyncWebServerRequest* request) {
                Command cmd{r.type, r.param};
                if (_shared.sendCommand(cmd)) {
                    sendOk(request, r.action);
                } else {
                    sendError(request, 503, "queue_full");
                }
            });
        }

        // --- 404 handler ----------------------------------------------------
        _server.onNotFound([](AsyncWebServerRequest* request) {
            char buf[64];
            snprintf(buf, sizeof(buf), "{\"status\":\"error\",\"message\":\"not_found\"}");
            request->send(404, "application/json", buf);
        });

        // --- OTA firmware update at /update ---------------------------------
        ElegantOTA.begin(&_server);

        _server.begin();
        _running = true;
    }

    void stop() {
        _server.end();
        _running = false;
    }

    bool isRunning() const { return _running; }

    // ElegantOTA needs periodic tick for progress tracking
    void loop() {
        ElegantOTA.loop();
    }

private:
    SharedState&   _shared;
    AsyncWebServer _server;
    bool           _running = false;

    static void sendOk(AsyncWebServerRequest* request, const char* action) {
        char buf[64];
        snprintf(buf, sizeof(buf), "{\"status\":\"ok\",\"action\":\"%s\"}", action);
        request->send(200, "application/json", buf);
    }

    static void sendError(AsyncWebServerRequest* request, int code, const char* msg) {
        char buf[64];
        snprintf(buf, sizeof(buf), "{\"status\":\"error\",\"message\":\"%s\"}", msg);
        request->send(code, "application/json", buf);
    }
};
