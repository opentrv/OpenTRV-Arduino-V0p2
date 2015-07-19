/*
The OpenTRV project licenses this file to you
under the Apache Licence, Version 2.0 (the "Licence");
you may not use this file except in compliance
with the Licence. You may obtain a copy of the Licence at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing,
software distributed under the Licence is distributed on an
"AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
KIND, either express or implied. See the Licence for the
specific language governing permissions and limitations
under the Licence.

Author(s) / Copyright (s): Damon Hart-Davis 2015
*/

/*
 * OpenTRV Radio Link base class.
 */

#ifndef ARDUINO_LIB_OTRADIOLINK_OTRADIOLINK_H
#define ARDUINO_LIB_OTRADIOLINK_OTRADIOLINK_H

#include <stddef.h>

// Use namespaces to help avoid collisions.
namespace OTRadioLink
    {
    typedef class OTRadioChannelConfig
        {
        public:
            OTRadioChannelConfig(const void *_config, bool _isFull, bool _isRX, bool _isTX) :
                config(_config), isFull(_isFull), isRX(_isRX), isTX(_isTX) { }
            // Opaque configuration dependent on radio type.
            const void *config;
            // True if this is a full radio configuration, else partial/delta.
            const bool isFull:1;
            // True if this is/supports RX.  For many radios TX/RX may be exclusive.
            const bool isRX:1;
            // True if this is/supports TX.  For many radios TX/RX may be exclusive.
            const bool isTX:1;
        } OTRadioChannelConfig_t;

    class OTRadioLink
        {
        protected:
            // Number of channels; strictly positive.
            int nChannels;
            // Per-channel configuration, read-only.
            const OTRadioChannelConfig * channelConfig;

            // Configure the hardware.
            // Called from configure() once nChannels and channelConfig is set.
            // Returns false if hardware not present or config is invalid.
            // Defaults to do nothing.
            virtual bool _doconfig() { return(true); }

        public:
            // Do very minimal pre-initialisation, eg at power up, to get radio to safe low-power mode.
            // Argument is read-only pre-configuration data;
            // may be mandatory for some radio types, else can be NULL.
            // This pre-configuration data depends entirely on the radio implementation,
            // but could for example be a minimal set of register number/values pairs in ROM.
            // This routine must not lock up if radio is not actually available/fitted.
            // Defaults to do nothing.
            virtual void preinit(const void *preconfig) { }

            // Configure the hardware.
            // Must be called before begin().
            // Returns false if hardware not present or config is invalid.
            // At least one channel configuration (0) must be provided
            // and it must be a 'full' base configuration;
            // others can be reduced/partial reconfigurations that can be applied to switch channels.
            // The base/0 configuration may be neither RX nor TX, eg off/disabled.
            // The base/0 configuration will be applied at begin().
            // The supplied configuration lifetime must be at least that of this OTRadioLink instance
            // as the pointer will be retained internally.
            // Some radios will have everything hardwired
            // and can be called with (1, NULL) and will set everything internally.
            bool configure(int channels, const OTRadioChannelConfig_t * const configs)
                {
                if((channels <= 0) || (NULL == configs)) { return(false); }
                nChannels = channels;
                channelConfig = configs;
                return(_doconfig());
                }

            // Begin access to (initialise) this radio link if applicable and not already begun.
            // Returns true if it needed to be begun.
            // Allows logic to end() if required at the end of a block, etc.
            // Defaults to do nothing (and return false).
            virtual bool begin() { return(false); }

            // Switch to specified channel (must be in range).
            // Defaults to doing nothing, eg for radios that only support one channel.
            virtual void switchToChannel(int channel) { }

            // Returns true if this radio link is currently available.
            // True by default unless implementation overrides.
            // For those radios that need starting this will be false before begin().
            virtual bool isAvailable() const { return(true); }

            // Handle simple interrupt for this radio link.
            // Must be fast and ISR (Interrupt Service Routine) safe.
            // Returns true if interrupt was successfully handled and cleared
            // else another interrupt handler in the chain may be called
            // to attempt to clear the interrupt.
            // By default does nothing (and returns false).
            virtual bool handleInterruptSimple() { return(false); }

            // End access to this radio link if applicable and not already ended.
            // Returns true if it needed to be ended.
            // Defaults to do nothing (and return false).
            virtual bool end() { return(false); }

#if 0 // Defining the virtual destructor uses ~800+ bytes of Flash by forcing use of malloc()/free().
            // Ensure safe instance destruction when derived from.
            // by default attempts to shut down the sensor and otherwise free resources when done.
            // This uses ~800+ bytes of Flash by forcing use of malloc()/free().
            virtual ~OTRadioLink() { end(); }
#else
#define OTRADIOLINK_NO_VIRT_DEST // Beware, no virtual destructor so be careful of use via base pointers.
#endif
        };
    }


#endif
