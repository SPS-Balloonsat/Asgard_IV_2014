The final program should:
-Use the MPU6050's DMP
-Should run well on the T-Minus board.
-Log yaw, pitch and roll every second (at least)
-Log max acceleration on each axis (x,y,z) for the past second every second
-Be capable of coexisting with code to operate:
 +Other I2C devices
 +Bacteria collection project
 +EEPROM logging (probably over I2C)
-Device should have an appropriate system incorporated for logic level shift.
 
~Should ideally be included in the redundant package, if realised.
