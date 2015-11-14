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

Author(s) / Copyright (s): Damon Hart-Davis 2014--2015
*/

/*
 Security support for OpenTRV.
 */

#ifndef SECURITY_H
#define SECURITY_H


#include "V0p2_Main.h"

//// Get the current basic stats transmission level (for data outbound from this node).
//// May not exactly match enumerated levels; use inequalities.
//// Not thread-/ISR- safe.
//OTV0P2BASE::stats_TX_level getStatsTXLevel();
//
//
//#if 0 // Pairing API outline.
//struct pairInfo { bool successfullyPaired; };
//bool startPairing(bool primary, &pairInfo);
//bool continuePairing(bool primary, &pairInfo); // Incremental work.
//void clearPairing(bool primary, &pairInfo);
//#endif

#endif


