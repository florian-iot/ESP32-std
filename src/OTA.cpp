#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <Update.h>
#include <Stream.h>
#include <StreamString.h>

#include "OTA.h"

static const char* uploadForm =
"<script src='jquery-3.3.1.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
   "<input type='file' name='update'>"
   "<input type='submit' value='Update'>"
 "</form>"
 "<div id='prg'>progress: 0%</div>"
 "<div id='status'></div>"
 "<script>"
  "$('form').submit(function(e){"
    "console.log('Submitting...');"
    "$('#status').html('Uploading...');"
    "e.preventDefault();"
    "var form = $('#upload_form')[0];"
    "var data = new FormData(form);"
    "$('input').attr('disabled', 'true');"
    "$.ajax({"
      "url: '/update',"
      "type: 'POST',"
      "data: data,"
      "contentType: false,"
      "processData:false,"
      "xhr: function() {"
        "var xhr = new window.XMLHttpRequest();"
        "xhr.upload.addEventListener('progress', function(evt) {"
          "if (evt.lengthComputable) {"
            "var per = evt.loaded / evt.total;"
            "$('#prg').html('Progress: ' + Math.round(per*100) + '%');"
          "}"
        "}, false);"
        "return xhr;"
      "},"
      "success: function(d, s) {"
        "console.log('Submitted... Success');"
        "$('#status').html('Success: ' + s + ': ' + d);"
        "$('input').removeAttr('disabled');"
      "},"
      "error: function(xhr, s, err) {"
        "console.log('Submitted... Error: ' + xhr.responseText);"
        "$('#status').html('Error: ' + xhr.responseText);"
        "$('input').removeAttr('disabled');"
      "}"
 "});"
 "});"
 "</script>";

OTA::OTA()
:
  _uploadingRequest(nullptr),
  _updateSucceeded(false),
  _uploadCount(0)
{
}

void OTA::init(AsyncWebServer *server, LogMgr *logMgr, bool (*onUpdateStart)(), void (*onUpdateEnd)(bool succeeded))
{
  Serial.printf("Initializing OTA, upload form: GET /update, post file for update: POST /update\n");
  _server = server;
  this->logMgr = logMgr;
  this->logger = logMgr->newLogger("OTA");
  _onUpdateStart = onUpdateStart;
  _onUpdateEnd = onUpdateEnd;

  server->serveStatic("/jquery.3.3.1.min.js", SPIFFS, "/jquery-3.3.1.min.js.gz", "max-age=30758400");
  server->on("/update", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", F(uploadForm));
  });

  server->on("/update", HTTP_POST,
    [this](AsyncWebServerRequest *request) {
      logger->error("End of update, hasUpdateError: {}", _hasUpdateError);
      _onUpdateEnd(_updateSucceeded);
      _uploadingRequest = nullptr;
    },
    [this](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
      if (_uploadingRequest != nullptr && _uploadingRequest != request) {
        // we're being called for another upload, while one is ongoing
        AsyncWebServerResponse *response = request->beginResponse(400, "text/plain", "An update is already running");
        response->addHeader("Connection", "close");
        request->send(response);
        logger->error("Update request received, but an update is already running");
        return;
      }

      // Beginning of update
      if (index == 0) {
        logger->info("Update request received");
        _uploadingRequest = request;
        _hasUpdateError = false;

        logger->info("File: {}, totalSize {}, available size {}",
                filename.c_str(),
                request->contentLength(), ESP.getFreeSketchSpace());

        bool mayUpdate = _onUpdateStart();
        if (!mayUpdate) {
          updateError(request, "Update prohibited");
          return;
        }

        // try to start updating twice - maybe a leftover from an old update
        // needs an abort first
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
          logger->error("Error starting update, aborting and retrying");
          if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
            updateError(request, "Error starting update");
            return;
          }
        }
        _uploadCount = 0;
      }

      if ((_uploadCount & 0x3F) == 0) {
        logger->info("Uploaded {} bytes", index + len);
      }
      ++_uploadCount;

      // Write to flash
      if (!_hasUpdateError && !Update.hasError()) {
        size_t written;
        if ((written = Update.write(data, len)) != len) {
          updateError(request, "Error while writing block, at index %u written %u for a length of %u", index, written, len);
        }
      }

      // Last block
      if (final) {
        if (!_hasUpdateError && !Update.hasError()) {
          if (Update.end(true)) { // true to set the size to the current progress
            logger->info("Update success: {} bytes", index + len);
            AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "SUCCESS");
            response->addHeader("Connection", "close");
            request->send(response);
            _updateSucceeded = true;
          } else {
            updateError(request, "Error ending update");
            return;
          }
        } else {
          logger->error("Update terminated, an error has happened before end of upload");
          // the response was already sent (when the error happened)
        }
      }

    }
  );

}

void OTA::updateError(AsyncWebServerRequest *request, PGM_P msg...)
{
  if (_hasUpdateError) {
    return; // don't send an error twice
  }
  va_list args;
  va_start(args, msg);
  String str("");
  char data[512];

  vsnprintf(data, sizeof(data) / sizeof(data[0]), msg, args);
  data[sizeof(data) / sizeof(data[0]) - 1] = '\0';
  StreamString serialMsg;
  Update.printError(serialMsg);

  str += data;
  str += ": ";
  str += serialMsg;
  str += "\n";

  Serial.printf(str.c_str());
  logger->error("{}", LogValue(str.c_str(), LogValue::StrAction::DO_COPY));

  Update.abort();
  _hasUpdateError = true;
  AsyncWebServerResponse *response = request->beginResponse(500, "text/plain", str);
  response->addHeader("Connection", "close");
  request->send(response);

  va_end(args);
}
