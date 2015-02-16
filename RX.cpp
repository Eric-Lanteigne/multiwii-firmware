#include "Arduino.h"
#include "def.h"
#include "types.h"
#include "Serial.h"
#include "Protocol.h"
#include "MultiWii.h"

void computeRC() {
	uint8_t chan;
	for (chan = 0; chan < RC_CHANS; chan++) {
		if (chan<8 && rcSerialCount > 0) { // rcData comes from MSP and overrides RX Data until rcSerialCount reaches 0
			rcSerialCount --;
			rcData[chan] = 1500;
			if (rcSerial[chan] >900) {rcData[chan] = rcSerial[chan];} // only relevant channels are overridden
		}
	}
}
