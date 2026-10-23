#include <Arduino.h>

#include "CircuitCubeHub.h"
#include "DriverTaskDelay.h"
#include "log4MC.h"

static BLEUUID remoteControlServiceUUID(CIRCUITCUBE_REMOTECONTROL_SERVICE_UUID);
static BLEUUID remoteControlCharacteristicUUID(CIRCUITCUBE_REMOTECONTROL_CHARACTERISTIC_UUID);

CircuitCubeHub::CircuitCubeHub(BLEHubConfiguration *config)
	: BLEHub(config)
{
}

bool CircuitCubeHub::SetWatchdogTimeout(const uint8_t watchdogTimeOutInTensOfSeconds)
{
	if (!attachCharacteristic(remoteControlServiceUUID, remoteControlCharacteristicUUID)) {
		log4MC::error("BLE : CircuitCube : Unable to attach to remote control service.");
		return false;
	}

	if (!_remoteControlCharacteristic->canWriteNoResponse() && !_remoteControlCharacteristic->canWrite()) {
		log4MC::error("BLE : CircuitCube : Remote control characteristic doesn't allow writing.");
		return false;
	}

	// The Circuit Cube has no watchdog.
	log4MC::vlogf(LOG_INFO, "BLE : CircuitCube : Watchdog timeout not set");

	return true;
}

/*
The cube accepts ASCII commands over the Nordic UART TX characteristic, one per channel:
	<direction><speed><channel>
	direction | '+' forward, '-' backward
	speed     | three digits, 000 (stop) - 255 (full speed)
	channel   | 'a', 'b' or 'c'
Examples: "+150a", "-055b", "+000c"
*/
void CircuitCubeHub::DriveTaskLoop()
{
	bool channelAForward = true;
	bool channelBForward = true;
	bool channelCForward = true;

	uint8_t channelAPwr = 0;
	uint8_t channelBPwr = 0;
	uint8_t channelCPwr = 0;

	// Force sending the initial state of all channels.
	bool oldChannelAForward = false;
	bool oldChannelBForward = false;
	bool oldChannelCForward = false;

	uint8_t oldChannelAPwr = 1;
	uint8_t oldChannelBPwr = 1;
	uint8_t oldChannelCPwr = 1;

	for (;;) {
		for (BLEHubChannelController *controller : _channelControllers) {
			// Update current channel pwr, if needed.
			controller->UpdateCurrentPwrPerc();

			switch (controller->GetHubChannel()) {
			case BLEHubChannel::A:
				channelAForward = controller->IsDrivingForward();
				channelAPwr = getRawChannelPwrForController(controller);
				break;
			case BLEHubChannel::B:
				channelBForward = controller->IsDrivingForward();
				channelBPwr = getRawChannelPwrForController(controller);
				break;
			case BLEHubChannel::C:
				channelCForward = controller->IsDrivingForward();
				channelCPwr = getRawChannelPwrForController(controller);
				break;
			case BLEHubChannel::D: // the Circuit Cube only has 3 channels
			case OnboardLED:	   // not on the Circuit Cube
				break;
			}
		}

		// Only send commands for channels that changed.
		if (channelAPwr != oldChannelAPwr || channelAForward != oldChannelAForward) {
			if (sendChannelCommand('a', channelAForward, channelAPwr)) {
				oldChannelAPwr = channelAPwr;
				oldChannelAForward = channelAForward;
			}
		}
		if (channelBPwr != oldChannelBPwr || channelBForward != oldChannelBForward) {
			if (sendChannelCommand('b', channelBForward, channelBPwr)) {
				oldChannelBPwr = channelBPwr;
				oldChannelBForward = channelBForward;
			}
		}
		if (channelCPwr != oldChannelCPwr || channelCForward != oldChannelCForward) {
			if (sendChannelCommand('c', channelCForward, channelCPwr)) {
				oldChannelCPwr = channelCPwr;
				oldChannelCForward = channelCForward;
			}
		}

		// Wait 50 milliseconds.
		vTaskDelay(DRIVERTASKDELAY / portTICK_PERIOD_MS);
	}
}

bool CircuitCubeHub::sendChannelCommand(char channel, bool forward, uint8_t pwr)
{
	char cmd[6];
	snprintf(cmd, sizeof(cmd), "%c%03u%c", forward ? '+' : '-', pwr, channel);

	// log4MC::vlogf(LOG_DEBUG, "CircuitCube : %s", cmd);
	if (!_remoteControlCharacteristic->writeValue((uint8_t *)cmd, strlen(cmd), false)) {
		log4MC::vlogf(LOG_ERR, "CircuitCube : Drive failed. Unable to write to CircuitCube characteristic.");
		return false;
	}

	return true;
}

int16_t CircuitCubeHub::MapPwrPercToRaw(int pwrPerc)
{
	if (pwrPerc == 0) {
		return 0;
	}

	// Map absolute speed (no matter the direction) to raw channel speed.
	return map(abs(pwrPerc), 1, 100, CIRCUITCUBE_MIN_CHANNEL_SPEED, CIRCUITCUBE_MAX_CHANNEL_SPEED);
}

void CircuitCubeHub::NotifyCallback(NimBLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{
	// TODO: Ignored for now...
}
