#include "WebManagement.h"

void WebManagement::begin(const char* ssid, const char* password, const char* hostname) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  MDNS.begin(hostname);

  setupOTA(hostname);
  setupServer();
  server.begin();

  MDNS.addService("http", "tcp", 80);
}

void WebManagement::handle() {
  server.handleClient();
  ArduinoOTA.handle();
  MDNS.update();
}

void WebManagement::setupServer() {
  server.on("/", HTTP_GET, [this]() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });

  server.on("/getconfig", HTTP_GET, [this]() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", readFile(confFile));
  });

  server.on("/update", HTTP_POST, [this]() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    rp2040.restart();
  }, [this]() { handleUpdate(); });

  server.on("/config", HTTP_POST, [this]() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", "OK");
  }, [this]() { handleConfig(); });
}

void WebManagement::setupOTA(const char* hostname) {
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.setPasswordHash(OTA_PWDHASH);

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_FS
      type = "filesystem";
    }
    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });

  ArduinoOTA.begin();
}

void WebManagement::handleUpdate() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    WiFiUDP::stopAll();
    Serial.printf("Update: %s\n", upload.filename.c_str());
    FSInfo64 i;
    LittleFS.info64(i);
    uint32_t maxSketchSpace = i.totalBytes - i.usedBytes;
    if (!Update.begin(maxSketchSpace)) {  // start with max available size
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {  // true to set the size to the current progress
      Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
    } else {
      Update.printError(Serial);
    }
  }
}

void WebManagement::handleConfig() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    tempFile = LittleFS.open(confFile, "w");
    if (!tempFile) {
      Serial.println("Error while opening file.");
      return;
    }
    Serial.println("Upload: START");
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (tempFile) {
      size_t bytesWritten = tempFile.write(upload.buf, upload.currentSize);
      if (bytesWritten != upload.currentSize) {
        Serial.println("WRITE FAILED");
        return;
      }
    }
    Serial.println(String("Upload: WRITE, Bytes: ") + upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (tempFile) {
      tempFile.close();
    }
    Serial.println(String("Upload: END, Size: ") + upload.totalSize);
  }
}

String WebManagement::readFile(String path) {
  String out = "";
  File file = LittleFS.open(path, "r");
  if (file) {
    while (file.available()) out += (char) file.read();
    file.close();
  }
  return out;
}
