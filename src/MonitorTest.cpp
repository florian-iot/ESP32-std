#include "CompilationOpts.h"
#include <HardwareSerial.h>
#include "Monitor.h"
#include "MonitorTest.h"

#ifdef USE_MONITOR_TEST


void monitorTest1()
{
    Monitor mon;
    bool rc = mon.enter();
    if (!rc) {
        Serial.printf("Error: enter returned false");
        return;
    }
    mon.leave();
}

void monitorTest2()
{

}

#endif
