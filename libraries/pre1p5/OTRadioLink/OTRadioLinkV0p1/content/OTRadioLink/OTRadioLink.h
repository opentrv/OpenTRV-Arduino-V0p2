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

#ifndef ARDUINO_LIB_OTRADIOLINK_H
#define ARDUINO_LIB_OTRADIOLINK_H

#define ARDUINO_LIB_OTRADIOLINK_VERSION_MAJOR 0
#define ARDUINO_LIB_OTRADIOLINK_VERSION_MINOR 1


// Radio message frame types and related information.
#include "utility/OTRadioLink_FrameType.h"

// Specialist simple CRC support.
#include "utility/OTRadioLink_CRC.h"

// Radio Link base class definition.
#include "utility/OTRadioLink_OTRadioLink.h"

#endif
