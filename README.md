## AMK Robotic Arm
This a project made for the university of baghdad, computer engineering department in 2024, named **Wireless Mobile-Controlled Robotic Arm**. As the name suggests, a Mobile App (programmed using [MIT App Inventor](https://appinventor.mit.edu/)) communicates with an **Arduino Board** through the HC-05 **Bluetooth Module**, the **Arduino** controls the Joint's **(Servo Motors)** of the Robotic Arm through the PCA9685 **Servo Driver**.  
  
## Preview :
See the Robotic arm in action in the following video :  

[![Video Thumbnail](./diagrams/thumbnail.png)](https://www.youtube.com/watch?v=02yBh6fGpnU)  

## Requirements :
**Hardware:**  
- Arduino Board.  
- HC-05 Bluetooth Module.  
- PCA9685 Servo Driver.  
- (4) Servo Motors.  
  
**Software:**  
- Arduino IDE.  
- [PCA9685 Servo Driver Library](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library) for Arduino IDE.  
- Download the [pre-built APK](https://mega.nz/file/MAMg0BLA#8Wh_-gSblxILR6ScSU8lpUBLbq5XcLNQehBtzrp5560) app for android, or use [MIT App Inventor](https://appinventor.mit.edu/) to build ```Robotic_Arm.aia```.  

## Circuit Diagram :
![Circuit Diagram](./diagrams/circuit.png)  

## Arm Assembly :
![Arm Assembly Diagram](./diagrams/assembly.png)  
  
## Acknowledgment
- Large part of inverse/forward kinematics maths used in this project were adapted from [This Github repository](https://github.com/yorkhackspace/meArm).  
- Special thanks to Dr. Asma and Brother Abdulrazaq, without them, I would never have known about robotic arms!  