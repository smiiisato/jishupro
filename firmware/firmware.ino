/**
 * rosserial Publisher Example with ArduinoBluetoothHardware
 * Prints "hello world!"
 * This intends to be connected from bluetooth interface of host PC.
 * First pair and connect to your esp32 after burning firmware.
 *   bluetoothctl
 *   scan on # check MAC address
 *   scan off
 *   pair <MAC address>
 *   trust <MAC address>
 *   connect <MAC address>
 *   info <MAC address> # check if successfully connected
 * After connect this device from PC, bind your device to serial device.
 *   sudo rfcomm bind 1 <MAC Address of your device>
 *   sudo stty -F /dev/rfcomm1 57600 cs8
 * then you can now connect rosserial host node to serial device
 *   rosrun rosserial_python serial_node.py _port:=/dev/rfcomm1 _baud:=57600
 */

#define ROSSERIAL_ARDUINO_BLUETOOTH // set this if you use rosserial over bluetooth

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int64.h>
#include <Wire.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include <IcsHardSerialClass.h>
#include <MadgwickAHRS.h>
Madgwick MadgwickFilter;


ros::NodeHandle nh;
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw, Tmp;

//Set up the ros node and publisher
sensor_msgs::Imu imu_msg; //msg
geometry_msgs::Pose pose_msg;
ros::Publisher imu("imu", &imu_msg); //publish topic


double cr, cp, cy;
double sr, sp, sy;

void set_up_MPU6050() {
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  //Gyro初期設定
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  //加速度センサー初期設定
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  //LPF設定
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();

  MadgwickFilter.begin(100); //100Hz
}

void imu_setup()
{
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  //Serial.begin(9600);
  //nh.initNode("BluetoothHelloworld");
  //nh.advertise(imu);
  // wait for bluetooth and rosserial host connection.
    while (not nh.connected()) {
      for(int i=0; i<10; i++){
        nh.spinOnce();
        delay(10);
      }
        //nh.spinOnce();
        //delay(10);
    }    
}


void imu_loop()
{
  Wire.beginTransmission(MPU_addr);//start tranxmit data to the address
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(); //end transmit
  Wire.requestFrom(MPU_addr,14);  // request a total of 14 registers  String AX = String(mpu6050.getAccX());
  while (Wire.available() < 14);

  
  axRaw=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  ayRaw=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  azRaw=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gxRaw=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyRaw=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gzRaw=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

// 加速度値を分解能で割って加速度(G)に変換する
  float acc_x = axRaw / 16384.0;  //FS_SEL_0 16,384 LSB / g
  float acc_y = ayRaw / 16384.0;
  float acc_z = azRaw / 16384.0;

   // 角速度値を分解能で割って角速度(degrees per sec)に変換する
  float gyro_x = gxRaw / 131.0;  // (度/s)
  float gyro_y = gyRaw / 131.0;
  float gyro_z = gzRaw / 131.0;

  //Madgwickフィルターを用いて、PRY（pitch, roll, yaw）を計算
  MadgwickFilter.updateIMU(gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z);

  //PRYの計算結果を取得する
  float roll  = MadgwickFilter.getRoll();
  float pitch = MadgwickFilter.getPitch();
  float yaw   = MadgwickFilter.getYaw();


  cr = cos(roll / 2.);
  cp = cos(pitch / 2.);
  cy = cos(yaw / 2.);
  sr = sin(roll / 2.);
  sp = sin(pitch / 2.);
  sy = sin(yaw / 2.);
  pose_msg.orientation.w = cr * cp * cy + sr * sp * sy;
  pose_msg.orientation.x = -cr * sp * sy + cp * cy * sr;
  pose_msg.orientation.y = cr * cy * sp + sr * cp * sy;
  pose_msg.orientation.z = cr * cp * sy - sr * cy * sp;

  pose_msg.orientation.x = roll;
  pose_msg.orientation.y = pitch;
  pose_msg.orientation.z = yaw;

  imu_msg.orientation = pose_msg.orientation;
  imu_msg.linear_acceleration.x = acc_x * 9.8;
  imu_msg.linear_acceleration.y = acc_y * 9.8;
  imu_msg.linear_acceleration.z = acc_z * 9.8;

  imu_msg.angular_velocity.x = gyro_x / 180.0 * 3.141592653589793;
  imu_msg.angular_velocity.y = gyro_y / 180.0 * 3.141592653589793;
  imu_msg.angular_velocity.z  = gyro_z / 180.0 * 3.141592653589793;

  imu_msg.header.frame_id = "imu";
  imu_msg.header.stamp = nh.now();
  imu.publish(&imu_msg);
 
}


 
void motor_control_cb(const std_msgs::Int64& msg) ;
ros::Subscriber<std_msgs::Int64> sub("/motor/command", &motor_control_cb);
const byte EN_PIN = 2;
const long BAUDRATE = 115200;
const int TIMEOUT = 100;
IcsHardSerialClass krs(&Serial,EN_PIN,BAUDRATE,TIMEOUT);  //インスタンス＋ENピン(2番ピン)およびUARTの指定


void send_servo_angle_cb_01(const std_msgs::Int64& msg) {
   krs.setPos(1, msg.data);
}
ros::Subscriber<std_msgs::Int64> sub_servo_01("/motor/command01", &send_servo_angle_cb_01);
void send_servo_angle_cb_02(const std_msgs::Int64& msg) {
   krs.setPos(2, msg.data);
}
ros::Subscriber<std_msgs::Int64> sub_servo_02("/motor/command02", &send_servo_angle_cb_02);

void send_servo_angle_cb_03(const std_msgs::Int64& msg) {
   krs.setPos(3, msg.data);
}
ros::Subscriber<std_msgs::Int64> sub_servo_03("/motor/command03", &send_servo_angle_cb_03);

void send_servo_angle_cb_04(const std_msgs::Int64& msg) {
   krs.setPos(4, msg.data);
}
ros::Subscriber<std_msgs::Int64> sub_servo_04("/motor/command04", &send_servo_angle_cb_04);

void send_servo_angle_cb_05(const std_msgs::Int64& msg) {
   krs.setPos(7, msg.data);
}
ros::Subscriber<std_msgs::Int64> sub_servo_05("/motor/command05", &send_servo_angle_cb_05);

void send_servo_angle_cb_06(const std_msgs::Int64& msg) {
   krs.setPos(8, msg.data);
}
ros::Subscriber<std_msgs::Int64> sub_servo_06("/motor/command06", &send_servo_angle_cb_06);


int value;
//callback function
void motor_control_cb(const std_msgs::Int64& msg) {
  value = msg.data;
  Serial.println(value);
}



void servo_setup()
{
  krs.begin();
}

void servo_loop()
{
  /*if(value == 1){           
    krs.setPos(1,6900);      
    krs.setPos(7,6900);  
    nh.spinOnce();
    delay(10);  
    krs.setPos(1,9500);      
    krs.setPos(7,9500);  
    nh.spinOnce();
    delay(10);
  }
  if(value == 2){           
    krs.setPos(1,6900);      
    krs.setPos(7,6900);
nh.spinOnce(); 
    delay(10);  
    krs.setPos(1,9500);      
    krs.setPos(7,9500);  
    nh.spinOnce();
    delay(10000);
  }
  if(value == 3){           
    krs.setPos(1,6900);      
    krs.setPos(7,6900);  
    nh.spinOnce();
    delay(500);  
    krs.setPos(1,9500);      
    krs.setPos(7,9500);  
    nh.spinOnce();
    delay(10000);
  }
  if(value == 4){           
    krs.setPos(1,6900);      
    krs.setPos(7,6900);  
    nh.spinOnce();
    delay(500);  
    krs.setPos(1,9500);      
    krs.setPos(7,9500);  
    nh.spinOnce();
    delay(10000);
  }*/
  if(value == 5){           
    krs.setPos(1,6900);      
    krs.setPos(7,6900);  
    for(int i=0; i<30; i++){
      nh.spinOnce();
    delay(10);  
    }
  
    krs.setPos(1,9500);      
    krs.setPos(7,9500);  
    for(int i=0; i<100; i++){
      nh.spinOnce();
    delay(10);  
    }
    
  }
}

void setup()
{
  // setup ros functions
  nh.initNode();
  //nh.subscribe(sub);
  nh.subscribe(sub_servo_01);
  nh.subscribe(sub_servo_02);
  nh.subscribe(sub_servo_03);
  nh.subscribe(sub_servo_04);
  nh.subscribe(sub_servo_05);
  nh.subscribe(sub_servo_06);

  nh.advertise(imu);
  imu_setup();
  servo_setup();
}


void loop()
{
    nh.spinOnce();
    imu_loop();
    // servo_loop();
    delay(10);
}