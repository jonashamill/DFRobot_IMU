#include <DFRobot_WT61PC.h>
#include <SoftwareSerial.h>
#include <ros.h>
#include <std_msgs/String.h>

//Use software serial port RX：10，TX：11
SoftwareSerial mySerial(10, 11);

DFRobot_WT61PC sensor(&mySerial);

// Create ROS node handle
ros::NodeHandle nh;


// Create a publisher for the IMU data
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu_data", &imu_msg);


void setup() {
  //Use Serial as debugging serial port 
  Serial.begin(115200);
  //Use software serial port mySerial as communication seiral port 
  mySerial.begin(9600);
  //Revise the data output data frequncy of sensor FREQUENCY_0_1HZ for 0.1Hz, FREQUENCY_0_5HZ for 0.5Hz, FREQUENCY_1HZ for 1Hz, FREQUENCY_2HZ for 2Hz, 
  //                        FREQUENCY_5HZ for 5Hz, FREQUENCY_10HZ for 10Hz, FREQUENCY_20HZ for 20Hz, FREQUENCY_50HZ for 50Hz, 
  //                        FREQUENCY_100HZ for 100Hz, FREQUENCY_125HZ for 125Hz, FREQUENCY_200HZ for 200Hz.
  sensor.modifyFrequency(FREQUENCY_10HZ);

  //Init ROS node
  nh.initNode();

  //Advertise IMU publisher
  nh.advertise(imu_pub);

}

void loop() {
  if (sensor.available()) {
    //Serial.print("Acc\t"); Serial.print(sensor.Acc.X); Serial.print("\t"); Serial.print(sensor.Acc.Y); Serial.print("\t"); Serial.println(sensor.Acc.Z); //acceleration information of X,Y,Z
    //Serial.print("Gyro\t"); Serial.print(sensor.Gyro.X); Serial.print("\t"); Serial.print(sensor.Gyro.Y); Serial.print("\t"); Serial.println(sensor.Gyro.Z); //angular velocity information of X,Y,Z
    //Serial.print("Angle\t"); Serial.print(sensor.Angle.X); Serial.print("\t"); Serial.print(sensor.Angle.Y); Serial.print("\t"); Serial.println(sensor.Angle.Z); //angle information of X, Y, Z 
    //Serial.println(" ");

    imu_msg.linear_acceleration.x = sensor.Acc.X
    imu_msg.linear_acceleration.y = sensor.Acc.Y
    imu_msg.linear_acceleration.z = sensor.Acc.Z

    imu_msg.angular_velocity.x = sensor.Gyro.X
    imu_msg.angular_velocity.y = sensor.Gyro.Y
    imu_msg.angular_velocity.z = sensor.Gyro.Z


    //converts from euler to quaternion 
    tf::Quaternion q = tf::createQuaternionFromRPY(sensor.Angle.X, sensor.Angle.Y, sensor.Angle.Z);

    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.orientation.w = q.w();

   
    imu_msg.header.stamp = nh.now();
    imu_msg.header.imu_msg.header.frame_id = "base_link";


    // Publish the message
    imu_pub.publish(&imu_msg);

  }

  nh.spin0nce.();

}
