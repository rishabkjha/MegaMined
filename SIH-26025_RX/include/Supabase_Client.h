#ifndef SUPABASE_CLIENT_H
#define SUPABASE_CLIENT_H

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "LoRa_Decoder.h"

class SupabaseClient {
private:
    const char* _ssid;
    const char* _password;
    String _supabaseUrl;
    String _supabaseKey;

public:
    // Pass Wi-Fi and Supabase credentials during instantiation
    SupabaseClient(const char* ssid, const char* password, const char* supabaseProjectUrl, const char* supabaseAnonKey) 
        : _ssid(ssid), _password(password), _supabaseUrl(supabaseProjectUrl), _supabaseKey(supabaseAnonKey) {}

    // Initialize Wi-Fi Connection
    void beginWiFi() {
        Serial.print(F("[Wi-Fi] Connecting to "));
        Serial.println(_ssid);
        WiFi.begin(_ssid, _password);
        
        uint32_t startAttempt = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) {
            delay(500);
            Serial.print(F("."));
        }

        if (WiFi.status() == WL_CONNECTED) {
            Serial.println(F("\n[Wi-Fi] Connected successfully!"));
            Serial.print(F("[Wi-Fi] IP Address: "));
            Serial.println(WiFi.localIP());
        } else {
            Serial.println(F("\n[Wi-Fi Warning] Failed to connect! Retrying in background..."));
        }
    }

    // Sends/Updates packet metrics to Supabase using PostgREST Upsert logic
    bool sendTelemetryToSupabase(const DecodedTelemetry &t, double rxLat = 0.0, double rxLon = 0.0) {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println(F("[Supabase Error] Wi-Fi not connected. Cannot send request."));
            return false;
        }

        HTTPClient http;

        // Target Supabase Endpoint (Table name: "telemetry")
        // 'on_conflict=node_number' performs an UPSERT (updates row if node_number exists)
        String endpoint = _supabaseUrl + "/rest/v1/telemetry?on_conflict=node_number";

        http.begin(endpoint);

        // Supabase Headers
        http.addHeader("Content-Type", "application/json");
        http.addHeader("apikey", _supabaseKey);
        http.addHeader("Authorization", "Bearer " + _supabaseKey);
        // Instruct Supabase to UPSERT (Merge duplicated primary keys / unique constraints)
        http.addHeader("Prefer", "resolution=merge-duplicates");

        // Construct JSON Payload
        StaticJsonDocument<512> doc;

        doc["node_number"]         = t.nodeNumber;
        doc["sequence_number"]     = t.packetSequence;
        doc["latitude"]            = t.latitude;
        doc["longitude"]           = t.longitude;
        doc["altitude"]            = t.altitude;
        
        // Formatted timestamp string if valid
        if (t.isTimeValid) {
            char timeBuf[12];
            snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d:%02d", t.utcHour, t.utcMinute, t.utcSecond);
            doc["utc_time"] = timeBuf;
        } else {
            doc["utc_time"] = nullptr;
        }

        // Displacements
        doc["disp_x_m"]            = t.displacementX_m;
        doc["disp_y_m"]            = t.displacementY_m;
        doc["disp_z_m"]            = t.displacementZ_m;

        // Accelerations
        doc["acc_x_ms2"]           = t.accX_ms2;
        doc["acc_y_ms2"]           = t.accY_ms2;
        doc["acc_z_ms2"]           = t.accZ_ms2;
        doc["resultant_accel"]     = t.resultantAccel_ms2;

        // Angles
        doc["angle_roll"]          = t.angleRoll;
        doc["angle_pitch"]         = t.anglePitch;
        doc["angle_yaw"]           = t.angleYaw;

        // Rotation Rates (Degrees/Second)
        doc["rate_roll"]   = t.rateRoll;
        doc["rate_pitch"]  = t.ratePitch;
        doc["rate_yaw"]    = t.rateYaw;

        // Flags
        doc["vibration_detected"]  = t.vibrationDetected;
        doc["is_high_precision"]   = t.isHighPrecision;

        // Optional Local RX Coordinates
        if (rxLat != 0.0 || rxLon != 0.0) {
            doc["rx_latitude"]     = rxLat;
            doc["rx_longitude"]    = rxLon;
        }

        String jsonPayload;
        serializeJson(doc, jsonPayload);

        // Execute HTTP POST (PostgREST upsert)
        int httpResponseCode = http.POST(jsonPayload);

        bool success = false;
        if (httpResponseCode == 200 || httpResponseCode == 201 || httpResponseCode == 204) {
            Serial.print(F("[Supabase API] Success! Data registered/updated for Node "));
            Serial.println(t.nodeNumber);
            success = true;
        } else {
            Serial.print(F("[Supabase API Error] Response code: "));
            Serial.println(httpResponseCode);
            Serial.print(F("[Supabase API Error] Payload: "));
            Serial.println(http.getString());
        }

        http.end();
        return success;
    }
};

#endif // SUPABASE_CLIENT_H