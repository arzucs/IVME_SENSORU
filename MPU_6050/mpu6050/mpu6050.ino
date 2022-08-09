#include <MPU6050.h>// MPU6050 kütüphanesi

#include<Wire.h>// I2C kütüphanesi
const int MPU=0x68; //MPU6050 I2C Slave adresi
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;//IMU'dan alınacak değerlerin kaydedileceği değişkenler

void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU);//I2C ile iletim sağlar.Ardından, write() işleviyle aktarım için baytları sıraya koyar ve bunları endTransmission() öğesini çağırarak iletir.
  Wire.write(0x6B); //0x6B adresindeki register'a ulaşıldı
  Wire.write(0);//MPU-6050 çalıştırıldı     
  Wire.endTransmission(true);//I2C haberleşmesi başlatıldı ve MPU-6050'nin ilk ayarları yapıldı
  Serial.begin(9600);
}
void loop(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  //0x3B adresindeki register'a ulaşıldı
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,12,true); // 12 byte'lık veri istendi 
  AcX=Wire.read()<<8|Wire.read();    
  AcY=Wire.read()<<8|Wire.read();  
  AcZ=Wire.read()<<8|Wire.read();  
  GyX=Wire.read()<<8|Wire.read();  
  GyY=Wire.read()<<8|Wire.read();  
  GyZ=Wire.read()<<8|Wire.read();  
    /* 
  * Sırası ile okunan her iki byte birleştirilerek sırası ile değişkenlere yazdırıldı
  * Böylece IMU'dan tüm değerler okunmuş oldu
  * 0x3B adresi imu değerlerinden ilk sensörün değerine denk gelmektedir.
  * IMU'dan tüm değerlerin okunabilmesi için bu adresten başlandı
  */
  
  Serial.print("Accelerometer: ");
  Serial.print("X = "); Serial.print(AcX);
  Serial.print(" | Y = "); Serial.print(AcY);
  Serial.print(" | Z = "); Serial.println(AcZ); 
  
  Serial.print("Gyroscope: ");
  Serial.print("X = "); Serial.print(GyX);
  Serial.print(" | Y = "); Serial.print(GyY);
  Serial.print(" | Z = "); Serial.println(GyZ);
  Serial.println(" ");
  delay(700);
}
