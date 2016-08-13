Target platform: Arduino UNO (or similar)
Library format: pre-1.5-IDE


Description
===========

AES GCM crypto library based in part on on:

https://github.com/anibali/aes_gcm

(see original files and notices in extras/aes_gcm-master.zip)

in turn based on:

http://w1.fi/


Licence
=======

All the original code remains BSD licensed:

see original files and notices in extras/aes_gcm-master.zip

For the avoidance of doubt any new code and structure
is dual-licensed under BSD and Apache 2.0 as below:

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



Notes
=====

Compiled on Mac OS X 10.10 Arduino 1.0.5 for Arduino UNO gives Binary sketch size XXX bytes (of a 32,256 byte maximum):
2015/06/14T10:46Z (r4675) 16,400 bytes: Basic GCM test passes.
2015/06/14T15:57Z (r4682) 15,104 bytes: Stripped out some run-time options not needed.
2015/06/14T17:34Z (r4688) 14,448 bytes: Removed dependency on malloc()/free().


See:
https://github.com/anibali/aes_gcm
http://electronics.stackexchange.com/questions/13275/smallest-aes-implementation-for-microcontrollers
https://github.com/kokke/tiny-AES128-C
http://ccodeblog.wordpress.com/2012/05/25/aes-implementation-in-300-lines-of-code/
