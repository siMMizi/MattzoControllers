#pragma once

#include <Arduino.h>

#include "BLEHub.h"

// The Circuit Cubes Bluetooth Battery Cube (Tenka) uses the Nordic UART Service.
#define CIRCUITCUBE_REMOTECONTROL_SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
// TX characteristic (write without response), used to send commands to the cube.
#define CIRCUITCUBE_REMOTECONTROL_CHARACTERISTIC_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

// The cube accepts speed values from 0 to 255. Motors will not start below ~55, so we map 1-100% to 55-255.
#define CIRCUITCUBE_MIN_CHANNEL_SPEED 55
#define CIRCUITCUBE_MAX_CHANNEL_SPEED 255

class CircuitCubeHub : public BLEHub
{
  public:
	CircuitCubeHub(BLEHubConfiguration *config);
	bool SetWatchdogTimeout(const uint8_t watchdogTimeOutInTensOfSeconds);
	void DriveTaskLoop();
	int16_t MapPwrPercToRaw(int pwrPerc);
	/**
	 * @brief Callback function for notifications of a specific characteristic
	 * @param [in] pBLERemoteCharacteristic The pointer to the characteristic
	 * @param [in] pData The pointer to the received data
	 * @param [in] length The length of the data array
	 * @param [in] isNotify
	 */
	void NotifyCallback(NimBLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify);

  private:
	bool sendChannelCommand(char channel, bool forward, uint8_t pwr);
};
