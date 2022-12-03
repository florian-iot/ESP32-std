#include "Util.h"

void taskFn(void *arg)
{
  std::function<void ()> *argFn = reinterpret_cast<std::function<void ()> *>(arg);
  (*argFn)();
  delete argFn;
  vTaskDelete(nullptr); // delete the current thread
}

void Util::runAsThread(const char *taskName, std::function<void ()> fn)
{
    Util::ThreadOptions opt;
    opt.priority = 0;
    opt.stackSize = 0;
    runAsThread(taskName, opt, fn);
}

void Util::runAsThread(const char *taskName, Util::ThreadOptions &opt, std::function<void ()> fn)
{
  std::function<void ()> *argFn = new std::function<void ()>(fn);
  TaskHandle_t taskHandle;
  xTaskCreate(taskFn, taskName, opt.stackSize == 0 ? 4096 * 2 : opt.stackSize, argFn,
    opt.priority == 0 ? uxTaskPriorityGet(nullptr) : opt.priority, &taskHandle);
}

void Util::base64Encode(String *result, char *buf, size_t size)
{
    static const char* base64Chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    result->reserve((size + 2) / 3 * 4);
    unsigned int pos = 0;
    while (pos < size) {
        result->concat(base64Chars[(buf[pos + 0] & 0xfc) >> 2]);
        if (pos + 1 < size) {
            result->concat(base64Chars[((buf[pos + 0] & 0x03) << 4) + ((buf[pos + 1] & 0xf0) >> 4)]);
            if (pos+2 < size) {
                result->concat(base64Chars[((buf[pos + 1] & 0x0f) << 2) + ((buf[pos + 2] & 0xc0) >> 6)]);
                result->concat(base64Chars[buf[pos + 2] & 0x3f]);
            } else {
              result->concat(base64Chars[(buf[pos + 1] & 0x0f) << 2]);
              result->concat('=');
            }
        } else {
            result->concat(base64Chars[(buf[pos + 0] & 0x03) << 4]);
            result->concat("==");
        }
        pos += 3;
    }
}


void Util::durationToStr(String *val, int64_t millis)
{
    int d = 0;
    int h = 0;
    int m = 0;
    int s = 0;
    int ms = 0;

    if (millis > 24LL * 3600 * 1000) {
        d = (int)(millis / (24LL * 3600 * 1000));
        millis -= d * (24LL * 3600 * 1000);
    }
    if (millis > 3600LL * 1000) {
        h = (millis / (3600LL * 1000));
        millis -= h * (3600LL * 1000);
    }
    if (millis > 60LL * 1000) {
        m = (millis / (60LL * 1000));
        millis -= m * (60LL * 1000);
    }
    if (millis > 1000) {
        s = (millis / (1000));
        millis -= s * (1000);
    }
    ms = millis;

    if (d > 0) {
        *val += String(d);
        *val += "d";
    }
    if (h > 0 || d > 0) {
        if (d > 0) {
            String v("00");
            v.concat(h);
            *val += v.substring(v.length() - 2, v.length());
        } else {
            *val += h;
        }
        *val += "h";
    }
    if (m > 0 || d > 0 || h > 0) {
        if (d > 0 || h > 0) {
            String v("00");
            v.concat(m);
            *val += v.substring(v.length() - 2, v.length());
        } else {
            *val += m;
        }
        *val += "m";
    }
    if (s > 0 || d > 0 || h > 0 || m > 0) {
        if (d > 0 || h > 0 || m > 0) {
            String v("00");
            v.concat(s);
            *val += v.substring(v.length() - 2, v.length());
        } else {
            *val += s;
        }
        *val += "s";
    }
    if (ms > 0) {
        if (s > 0 || d > 0 || h > 0 || m > 0) {
            *val += ".";
            String v("000");
            v.concat((int)ms);
            *val += v.substring(v.length() - 3, v.length());
        } else {
            *val += ms; *val += "ms";
        }
    }
    if (d == 0 && h == 0 && m == 0 && s == 0 && ms == 0) {
        *val += "0s";
    }
}

#if 0


    // std::vector<char *> lines;
    // char *buff;
    // size_t buffSize;
    // char *head;
    // char *tail;

StringRing::StringRing()
{
    maxLen = 2048;
    buffSize = 20480;
    buff = new char[buffSize];
    head = buff;
    tail = buff;
}

void StringRing::setBufferSize(size_t size)
{
    TODO
}

void StringRing::setMaxLen(size_t len)
{
    TODO
}

uint32_t StringRing::push(const char *str)
{
    size_t len = strlen(str);
    if (len > maxLen) {
        len = maxLen;
    }
    if (len >= bufSize) {
        bufSize = len;
    }

    // make sure we can always put a 

    bool isSpaceOk = false;
    do {
        if (head >= tail) {
            if (buf + bufSize - head >= len) {
                // we have enough space, we're done
                isSpaceOk = true;
            } else {
                // wrap around and restart looking for space
                head = buf;
            }
        } else { // head < tail
            if (tail - head > len) { // we want at least 1 byte between head and tail
                // we have enough space, we're done
                isSpaceOk = true;
            } else {
                // remove one entry from tail and try again
                
                lines[linesTail] = -1;
                ++linesTail;

                tail += strlen(tail);
                ++tailEntryId;
            }
        }
    } while (!isSpaceOk); // eventually we'll remove all entries and we'll have enough space for the current entry

    strncpy(head, str, len);
    head[len] = '\0';
    ++headEntryId;
    char *entry = head;

    head += len;
    return headEntryId;
}

const char *StringRing::get(uint32_t id)
{
    if (id)
}

#endif

RotatingFile::RotatingFile()
{
}

RotatingFile::~RotatingFile()
{
    pFile.close();
}

void RotatingFile::openForWrite(const char *fileName, FS *fs, size_t sizeLimit)
{
    if (pFile) {
        pFile.close();
    }
    this->sizeLimit = sizeLimit;
    if (fs) {
        pFile = fs->open(fileName, "r+");
        if (!pFile) {
            pFile = fs->open(fileName, "w+");
        }
    }

    if (!pFile) {
        Serial.printf("Could not open rotating file %s for r+\n", fileName);
        return;
    }
    // read until we find the first marker, that will be marker A
    markerA = -1;
    markerB = -1;
    lastMarkerAPos = -1;
    firstMarkerBPos = -1;
    char buf[128];
    size_t pos = 0, basePos = 0, read;
    while (firstMarkerBPos == -1 && (read = pFile.readBytes(buf, sizeof(buf))) > 0) {
        for (pos = 0; firstMarkerBPos == -1 && pos < read; pos++) {
            if (markerA == -1) {
                if ((uint8_t)buf[pos] == 0x1E || (uint8_t)buf[pos] == 0x1F) {
                    markerA = buf[pos];
                    lastMarkerAPos = basePos + pos;
                    markerB = (markerA == 0x1E ? 0x1F : 0x1E);
                }
            }
            if (markerA != -1 && firstMarkerBPos == -1) {
                if (buf[pos] == markerA) {
                    lastMarkerAPos = basePos + pos;
                } else if (buf[pos] == markerB) {
                    firstMarkerBPos = basePos + pos;
                }
            }
        }
        basePos += read;
    }

    markerAB[0] = markerA;
    markerAB[1] = markerB;

    // here we know the position of first marker A (or -1 if on EOF),
    // last marker A if we had the first one, and first marker B (or -1 if on EOF)
    if (lastMarkerAPos == -1) {
        markerA = 0x1E;
        markerB = 0x1F;
        markerAB[0] = markerA;
        markerAB[1] = markerB;
        pFile.seek(0);
        pFile.write(markerAB, 2);
        pFile.seek(pFile.position() - 1);
        pFile.flush();
    } else {
        pFile.seek(lastMarkerAPos + 1);
    }

    // We start writing after last marker A in all cases.

    // If we want to read instead of write:
    // We must look from the current position for the first marker B. We read
    // from there, until we find maker A or EOF. Then we continue from the beginning of the file
    // until the first marker B.
}

size_t RotatingFile::write(uint8_t v)
{
    return pFile.write(v);
}

size_t RotatingFile::write(const uint8_t *buffer, size_t size)
{
    return pFile.write(buffer, size);
}

void RotatingFile::terminateRecord()
{
    pFile.write(markerAB, 2);
    pFile.seek(pFile.position() - 1);
    pFile.flush();
    if (pFile.position() > sizeLimit) {
        char m = markerA; markerA = markerB; markerB = m;
        markerAB[0] = markerA;
        markerAB[1] = markerB;
        pFile.seek(0);
        pFile.write(markerAB, 2);
        pFile.seek(pFile.position() - 1);
        pFile.flush();
    }
}
