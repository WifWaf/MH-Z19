﻿![Version](https://img.shields.io/badge/Version-v1.5.3-green.svg)

# MH-Z19 & MH-Z19B
A library for the MH-Z19 & MH-Z19B CO2 sensor on the Arduino platform which unlocks significant commands. Supports Hardware/Softeware serial and arduino based boads.

### Recovery for Dysfunctional Sensors:
See examples for the recovery code. *Note, Only use if your sensor is not recoverable by other means as it is somewhat untested, however from feedback and personal experience, it can often work.

### Features:
* Automatically sends "autocalibration off".
* Filter mode, to detect invalid readings when sensor is recovering from power loss / boot (see example)
* Option to print communcation between device and sensor (for debugging)
* Communication error checking
* Examples

>^*Transmittance is based upon the backwards projection of the raw value (which decreases with CO2) - see examples or [My Notes (Ravings)](https://myopenacuk-my.sharepoint.com/:x:/g/personal/jsd328_my_open_ac_uk/Ebyx4qxCBHxIk_bOBOtLkM4B40Dt9TZFd3CdI7Pv3NssMw?e=8Lr8bZ)*

### Commands
---

|             Additions               |            Existing           |
|              :---:                  |              :---:            |
| CO2 Unlimited                       | CO2 Limited                   | 
| CO2 as Raw                          | Temperature as Whole Integer  | 
| Custom Range / Span                 | Request CO2 Calibration       | 
| Reset Sensor                        | ABC On / Off                  |
| Get Temperature Adjustment          | Retrieve Accuracy             |
| Get Firmware Version                |                               |     
| Get Background CO2 Value            |                               |
| Get Range Value                     |                               |
| Get Last Response                   |                               |
| ^Get ABC Status                     |
>^* submitted by SFeli

### "Usage"
---

The library can be found in the IDE/IO library manager. Alternatively, simply clone this library to your working library folder and include "MHZ19.h" in your main sketch.

If you are having issues with specific boards, please contact me (find my details below)

### "A Bit About the Sensor"
---
**Advice:** The MH-Z19 works best in the Range of 2000ppm, outside of this accuracy begins to fall away. This is supported by documentation by also by features such as the Analog Out. I would suggest keeping to this range if you need accuracy and a maximum of 5000ppm.

**Relevant Datasheets**

* The Englisih datasheet for the MH-Z19: [MH-Z19](https://www.winsen-sensor.com/d/files/PDF/Infrared%20Gas%20Sensor/NDIR%20CO2%20SENSOR/MH-Z19%20CO2%20Ver1.0.pdf)

* The English datasheet for the non-JST MH-Z19B version: [MH-Z19B NON-JST](https://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z19b-co2-ver1_0.pdf). 

* The Chinese datasheet for the JST MH-Z19B version (more detailed): [MH-Z19B JST](https://datasheet.lcsc.com/szlcsc/1901021600_Zhengzhou-Winsen-Elec-Tech-MH-Z19_C242514.pdf).

**Auto Calibration:** 
The MH-Z19 is a sensor that is designed to be powered on and rarely turned off. The sensor calibrates over time (if autocalibration is on) tuning the Zero at the end of each "ABC" (auto calibration) period (0 - 24hrs currently). After 3 weeks, a value is stored to represent it's accuracy, this can be requested using getAccuracy(). ABC must be disabled by sending the appropriate command before the end of the ABC period to ensure it remains off - this is handled by the library.

**Calibration:**
If you plan to manually calibrate sensor (in my experience this is often be better) then it's important to be aware that Zero calibration does not refer to 0ppm (often a nitrogen environment), instead it refers to 400ppm.

**Background Calibration:** It's currently unclear how to change  this, if possible at all. The value stored on the MH-Z19 and is set to 400ppm. This is used as the zeroing point.

**Zero Calibration:** This can be made in two ways: By pulling the zero HD low (0V) for 7 Secs, or be sending command 135 (0x87). As above, the Zero refers to the background CO2 value of 400ppm, not 0ppm. Currently testing is the ability to send an adjustment with command at byte 7, however it is unclear the affects this has.

**Range:** This is essentially your highest and lowest CO2 being measured. 2000ppm is advised. Changing the value usually requires span calibration (diffiuclt), however if you intend to measure abvoe 2000 ppm this can increase accuracy.

**Span:** I highly recommend avoiding this command unless you have the equipment to do so. It requires the sensor to be at the ppm you are setting it to, e.g. 2000ppm. Roughly, it's difference between lowest and highest range points, for this sensor, it's the same value as range. From tiral and error, it's usually best sent last in the calibrations sequence.

**Alarm:** The analog output is located on the brown wire on the JST version. On the non-JST version it can be found on the far side, beside the Rx pin. It's unclear at the moment how to change the threshold and is not affected by Range. However, it is possible to attach an amplifier to the Analog Out pin and create an interrupt.

**Analog Out:** An additional feature of MH-Z19. The output in mV corresponds to ppm when using a range of 2000ppm. Alternatively, calculations can be made to adjust the value (See Examples).

### Main Priorities:
---
- [ ] Reduce memory usage

### Additional Disclaimer
---
THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

### Authors
---
Myself, you can find my contact details below.

### License
---
This project is licensed under the Lesser General Public License v3.0 License - see the LICENSE.md file for details

### Acknowledgments
----
This library was originaly inspired by Strange-V's work! https://github.com/strange-v/MHZ19;

### Feedback
---
This is one of my first pieces of code, so lots of room for imporvement, feel free to provide constructive feedback; jdwifwaf@gmail.com


>If this library was particularly helpful, and you feel like funding a replacement sensor (brutalised from testing!) [![Donate](https://img.shields.io/badge/Donate-PayPal-blue.svg?style=flat-square&logo=appveyor)](https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=9MJYH22A92LWG&source=url)
