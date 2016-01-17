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

Author(s) / Copyright (s): Damon Hart-Davis 2013--2016
*/

/*
 Simple schedule support for TRV.
 */

#ifndef SCHEDULE_H
#define SCHEDULE_H

#include "V0p2_Main.h"
#include "Control.h"


// Customised scheduler for the current OpenTRV application.
class SimpleValveSchedule : public SimpleValveScheduleBase
    {
    public:
        // Allow scheduled on time to dynamically depend on comfort level.
        virtual uint8_t onTime()
            {
#if LEARNED_ON_PERIOD_M == LEARNED_ON_PERIOD_COMFORT_M
            // Simplify the logic where no variation in on time is required.
            return(LEARNED_ON_PERIOD_M);
#else // LEARNED_ON_PERIOD_M != LEARNED_ON_PERIOD_COMFORT_M
            // Variable 'on' time depending on how 'eco' the settings are.
            // Three-way split based on current WARM target temperature,
            // for a relatively gentle change in behaviour along the valve dial for example.
            const uint8_t wt = getWARMTargetC();
            if(isEcoTemperature(wt)) { return(LEARNED_ON_PERIOD_M); }
            else if(isComfortTemperature(wt)) { return(LEARNED_ON_PERIOD_COMFORT_M); }
            else { return((LEARNED_ON_PERIOD_M + LEARNED_ON_PERIOD_COMFORT_M) / 2); }
#endif // LEARNED_ON_PERIOD_M == LEARNED_ON_PERIOD_COMFORT_M
            }
    };
// Singleton scheduler.
extern OTV0P2BASE::SimpleValveSchedule Scheduler;


#endif


