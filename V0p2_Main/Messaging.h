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

Author(s) / Copyright (s): Damon Hart-Davis 2014--2016
*/

/*
 Generic messaging support for OpenTRV.

 Security, integrity, etc, see: http://www.earth.org.uk/note-on-IoT-security.html#app1

 Messages be be sent in a number of formats,
 and may be sent stand-alone alone
 or piggybacked on (appended to) another message (eg on the end of an FS20 message).

 There may be a number of efficient binary formats,
 and a general limited/compact JSON format.
 */

#ifndef MESSAGING_H
#define MESSAGING_H

#include "V0p2_Main.h"

#include <OTV0p2Base.h>
#include <OTRadioLink.h>


// Returns true if an unencrypted trailing static payload and similar (eg bare stats transmission) is permitted.
// True if the TX_ENABLE value is no higher than stTXmostUnsec.
// Some filtering may be required even if this is true.
#if defined(ALLOW_STATS_TX)
#if !defined(CONFIG_ALWAYS_TX_ALL_STATS)
// TODO: allow cacheing in RAM for speed.
inline bool enableTrailingStatsPayload() { return(OTV0P2BASE::getStatsTXLevel() <= OTV0P2BASE::stTXmostUnsec); }
#else
#define enableTrailingStatsPayload() (true) // Always allow at least some stats to be TXed.
#endif // !defined(CONFIG_ALWAYS_TX_ALL_STATS)
#else
#define enableTrailingStatsPayload() (false)
#endif


#if defined(ENABLE_RADIO_RX)
// Incrementally poll and process I/O and queued messages, including from the radio link.
// Returns true if some work was done.
// This may mean printing them to Serial (which the passed Print object usually is),
// or adjusting system parameters,
// or relaying them elsewhere, for example.
// This will write any output to the supplied Print object,
// typically the Serial output (which must be running if so).
// This will attempt to process messages in such a way
// as to avoid internal overflows or other resource exhaustion.
bool handleQueuedMessages(Print *p, bool wakeSerialIfNeeded, OTRadioLink::OTRadioLink *rl);
#else
#define handleQueuedMessages(p, wakeSerialIfNeeded, rl)
#endif


#endif
