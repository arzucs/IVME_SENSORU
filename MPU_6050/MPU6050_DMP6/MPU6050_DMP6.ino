// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2019-07-08 - Added Auto Calibration and offset generator
//		   - and altered FIFO retrieval sequence to avoid using blocking code
//      2016-04-18 - Eliminated a potential infinite loop
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"//I2C k??t??phanesini i??erir

#include "MPU6050_6Axis_MotionApps20.h"//MPU6050 k??t??phenesini i??erir
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE//#if ile ba??larsa #endif,#elif,#else ile biter ve  kaynak dosyan??n b??l??nlerinin derlenmesini denetler 
#include "Wire.h"                             
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
#define OUTPUT_TEAPOT// sabitleri tan??mlamak i??in kullan??l??r



#define INTERRUPT_PIN 2  // INT pini i??in 2 nolu pine ba??la
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;// led kapal??

// MPU kontrol/durum de??i??kenleri
bool dmpReady = false;  // DMP=IMU ??zerindeki gyro ve accelerometer verilerini birle??tirip gerekli hesaplamalar?? ve filtrelemeleri yaparak do??rudan a???? bilgisi sunar
// DMP init d??zg??n ??al????mas?? durumunda true yap??l??r.
uint8_t mpuIntStatus;   // MPU kesme durumu
uint8_t devStatus;      // Her i??lemden sonra durum de??eri geri verilir. (0 = ba??ar??l??, 1 = hatal??)
uint16_t packetSize;    // DMP paket boyutu (varsay??lan 42 bytes)
uint16_t fifoCount;     // ??uanda FIFO daki t??m bayt say??s??
uint8_t fifoBuffer[64]; // FIFO saklama bayt??

// y??nlendirme/hareket de??i??kenleri
Quaternion q;           // [w, x, y, z]         y??nlendirme koordinatlar??
VectorInt16 aa;         // [x, y, z]            h??zlanma sens??r?? ??l????mleri
VectorInt16 aaReal;     // [x, y, z]           yer??ekimi i??ermeyen h??zlanma sens??r?? ??l????mleri
VectorInt16 aaWorld;    // [x, y, z]            d??nya ??er??evesi h??zlanma sens??r?? ??l????mleri
VectorFloat gravity;    // [x, y, z]            yer??ekimi vekt??r??
float euler[3];         // [psi, theta, phi]    Euler a???? kooordinatlar??
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll kab?? ve yer??ekimi vekt??r??
// InvenSense Taepot uygulamas?? i??in paket yap??s??
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                === kesinti alg??lama d??ng??s??
// ================================================================

volatile bool mpuInterrupt = false;     // MPU kesme pininin y??ksek bo??lukta olup olmad??????n?? g??sterir,genellikle bir de??i??kenin veri tipinden ??nce, derleyicinin ve sonraki program??n de??i??keni i??leme ??eklini de??i??tirmek i??in kullan??l??r.
void dmpDataReady() { 
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    //  12C veriyoluna kat??l (12C dev k??t??phanesi bunu otomatik olarak yapmaz)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();// I2C haberle??mesini ba??latan fonksiyondur.
        Wire.setClock(400000); // 400kHz I2C clock. ??ste??e ba??l?? - I2C SCL'yi 400kHz'lik Y??ksek H??z Moduna ayarlay??n
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // seri ileti??imi ba??lat??n??z
    // (115200 se??ildi ????nk?? prossesing ile ayn?? baud h??z?? 
   // ile ileti??ime ge??mek i??in.
    
    Serial.begin(115200);// seri port ile haberle??meyi ba??latma s??resi (saniye)
    while (!Serial); /// Leonardo numaraland??rma i??in bekleyin, di??erleri devam edecek (BEKLE)

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // Cihaz?? ilk haline getirin
    Serial.println(F("I2C Kuruluyor...""));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    //  ba??lant??s??n?? do??rulay??n
    Serial.println(F("Suruculer test ediliyor..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 baglanti bassarili") : F("MPU6050 baglanti basarisiz"));

    // 
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // DMP yi y??kle ve yap??land??r
    Serial.println(F("DMP kuruluyor..."));
    devStatus = mpu.dmpInitialize();

    // / buraya gyro ofsetlerinizi tan??mlay??n , minimum hassasiyet i??in ??l??eklendirildi
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // / test ??ipim i??in 1788 ??retim varsay??lan??

    // ??al????t??????ndan emin olun (returns 0 if so)
    if (devStatus == 0) {
        // Arduino interrupt kontrol sistemi aktif edildi.
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        //  DMP yi a????n, ??imdi haz??r
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // Arduino kesinti tespitini etkinle??tir
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt0)... "));//(kesintiyi alg??laman??n etkinle??tirilmesi (Arduino haici kesinti 0))
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        //  loop() d??ng??s??ne girmek i??in DMP haz??r bayra???? ayarlan??r.
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // sonras??nda kar????la??t??rma i??in beklenen DMP paket boyutu al??n??r.
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed(ilk bellek y??klemesi ba??ar??s??z oldu)
        // 2 = DMP configuration updates failed(DMP yap??land??rma g??ncellemeleri ba??ar??s??z oldu)
        //(if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // LED'e ????k???? ver.
    pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // ba??ar??s??zl??k durumunda bir??ey yapma
    if (!dmpReady) return;
    // FIFO'dan bir paket oku, MPU kesme veya ekstra paketleri bekle
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_QUATERNION
            // Quaternion de??erlerini kolay matris formunda g??ster: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // Euler a????lar??n?? derece olarak g??ster
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // Euler a????lar??n?? derece olarak g??ster
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // yer??ekimini ortadan kald??rmak i??in ayarlanm???? ger??ek ivmeyi g??ster
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
