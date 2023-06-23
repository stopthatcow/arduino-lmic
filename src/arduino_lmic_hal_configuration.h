/*

Module:  arduino_lmic_hal_configuration.h

Function:
        Arduino-LMIC C++ HAL configuration APIs

Copyright & License:
        See accompanying LICENSE file.

Author:
        Matthijs Kooijman       2015
        Terry Moore, MCCI       November 2018

*/
#pragma once

#ifndef _arduino_lmic_hal_configuration_h_
# define _arduino_lmic_hal_configuration_h_

#include <stdint.h>
#include "lmic/lmic_env.h"

namespace Arduino_LMIC {

/* these types should match the types used by the LMIC */
typedef	int32_t	ostime_t;

// this type is used when we need to represent a threee-state signal
enum class ThreeState_t : uint8_t {
	Off = 0,
	On = 1,
	HiZ = 2
};

// forward reference
class HalConfiguration_t;

//
// for legacy reasons, we need a plain-old-data C-like
// structure that defines the "pin mapping" for the
// common pins. Many clients initialize an instance of
// this structure using named-field initialization.
//
// Be careful of alignment below.
struct HalPinmap_t {
	// Use this for any unused pins.
	static constexpr uint8_t UNUSED_PIN = 0xff;
	static constexpr int NUM_DIO = 3;
	// for backward compatibility...
	static constexpr uint8_t LMIC_UNUSED_PIN = UNUSED_PIN;

	/* the contents */
	uint8_t nss = LMIC_UNUSED_PIN;   // Pin for select.
	uint8_t rxtx = LMIC_UNUSED_PIN;	 // Pin for rx/tx control.
	uint8_t rst = LMIC_UNUSED_PIN;   // Pin for reset.
	// Pins for DIO0, DOI1, DIO2.
	uint8_t dio[NUM_DIO] = {LMIC_UNUSED_PIN,
							LMIC_UNUSED_PIN,
							LMIC_UNUSED_PIN};
	uint8_t busy = LMIC_UNUSED_PIN;  // For SX126x only (BUSY / DIO4).
	// True if we must set rxtx for rx_active, false for tx_active
	bool rxtx_rx_active = false;  // Polarity of rxtx active
	int8_t rssi_cal=0;	// RSSI cal in dB -- added to RSSI
				        //   measured prior to decision.
				        //   Must include noise guardband!
    bool dio2_controls_rxtx = true;   // True if DIO2 controls rx/tx.
    bool dio3_controls_txco = false;  // True if DIO3 powers TXCO (SX126x only)..
	uint32_t spi_freq = 0;  // SPI freq in Hz.

	// optional pointer to configuration object
	HalConfiguration_t *pConfig = nullptr;
	};

class HalConfiguration_t
	{
public:
	HalConfiguration_t() {};

	// these must match the constants in radio.c
	enum class TxPowerPolicy_t : uint8_t
		{
		RFO,
		PA_BOOST,
		PA_BOOST_20dBm
		};

	virtual ostime_t setModuleActive(bool state) {
		LMIC_API_PARAMETER(state);

		// by default, if not overridden, do nothing
		// and return 0 to indicate that the caller
		// need not delay.
		return 0;
	}

	virtual void begin(void) {}
	virtual void end(void) {}
	virtual bool queryUsingTcxo(void) { return false; }

	// compute desired transmit power policy.  HopeRF needs
	// (and previous versions of this library always chose)
	// PA_BOOST mode. So that's our default. Override this
	// for the Murata module.
	virtual TxPowerPolicy_t getTxPowerPolicy(
		TxPowerPolicy_t policy,
		int8_t requestedPower,
		uint32_t frequency
		)
		{
		LMIC_API_PARAMETER(policy);
		LMIC_API_PARAMETER(requestedPower);
		LMIC_API_PARAMETER(frequency);
		// default: use PA_BOOST exclusively
		return TxPowerPolicy_t::PA_BOOST;
		}
	};

bool hal_init_with_pinmap(const HalPinmap_t *pPinmap);

}; // end namespace Arduino_LMIC

#endif
