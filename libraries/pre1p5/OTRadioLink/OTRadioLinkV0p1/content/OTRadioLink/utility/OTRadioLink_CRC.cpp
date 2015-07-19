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

#include "OTRadioLink_CRC.h"

// Use namespaces to help avoid collisions.
namespace OTRadioLink
    {
    //int theUltimateAnswer() { return(42); }

    /**Update 7-bit CRC with next byte; result always has top bit zero.
     * Polynomial 0x5B (1011011, Koopman) = (x+1)(x^6 + x^5 + x^3 + x^2 + 1) = 0x37 (0110111, Normal)
     * <p>
     * Should maybe initialise with 0x7f.
     * <p>
     * See: http://users.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
     * <p>
     * Should detect all 3-bit errors in up to 7 bytes of payload,
     * see: http://users.ece.cmu.edu/~koopman/crc/0x5b.txt
     * <p>
     * For 2 or 3 byte payloads this should have a Hamming distance of 4 and be within a factor of 2 of optimal error detection.
     * <p>
     * TODO: provide table-driven optimised alternative,
     *     eg see http://www.tty1.net/pycrc/index_en.html
     */
    uint8_t crc7_5B_update(uint8_t crc, const uint8_t datum)
        {
        for(uint8_t i = 0x80; i != 0; i >>= 1)
            {
            bool bit = (0 != (crc & 0x40));
            if(0 != (datum & i)) { bit = !bit; }
            crc <<= 1;
            if(bit) { crc ^= 0x37; }
            }
        return(crc & 0x7f);
        }
    }
