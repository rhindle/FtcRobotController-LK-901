package org.firstinspires.ftc.teamcode.robot.Common;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.PartsInterface;

public class UserDrive implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;

   public double v0, v2, v1, v3;
   double driveSpeed, driveAngle, rotate;
   boolean useFieldCentricDrive = false;
   boolean useHeadingHold = true;
   boolean useHoldPosition = true;
   double maxSpeed = 1;

   /* Constructor */
   public UserDrive(Parts parts){
      construct(parts);
   }

   void construct(Parts parts){
      this.parts = parts;
   }

   public void initialize(){
      v0 = 0.0;
      v1 = 0.0;
      v2 = 0.0;
      v3 = 0.0;
   }

   public void preInit() {
   }

   public void initLoop() {
   }

   public void preRun() {
   }

   public void runLoop() {
   }

   public void stop() {
   }

   // Determine motor speeds when under driver control
   public void userDrivePower () {
//      if (idleDelay > System.currentTimeMillis()) {  // todo:bring this back
//         setTargetToCurrentPosition();
//      }

      driveSpeed = Math.pow(driveSpeed, 1);
      v0 = driveSpeed * (Math.cos(driveAngle / 180 * Math.PI) - Math.sin(driveAngle / 180 * Math.PI)) + rotate;
      v2 = driveSpeed * (Math.cos(driveAngle / 180 * Math.PI) + Math.sin(driveAngle / 180 * Math.PI)) + rotate;
      v1 = driveSpeed * (Math.cos(driveAngle / 180 * Math.PI) + Math.sin(driveAngle / 180 * Math.PI)) - rotate;
      v3 = driveSpeed * (Math.cos(driveAngle / 180 * Math.PI) - Math.sin(driveAngle / 180 * Math.PI)) - rotate;

      // scale so average motor speed is not more than maxSpeed
      // but only if maxspeed <> 1
      if (maxSpeed != 1) {
         double averageValue = JavaUtil.averageOfList(JavaUtil.createListWith(Math.abs(v0), Math.abs(v2), Math.abs(v1), Math.abs(v3)));
         averageValue = averageValue / maxSpeed;
         if (averageValue > 1) {
            v0 /= averageValue;
            v2 /= averageValue;
            v1 /= averageValue;
            v3 /= averageValue;
         }
      }

      // scale to no higher than 1
      double highValue = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(v0), Math.abs(v2), Math.abs(v1), Math.abs(v3), 1));
      v0 /= highValue;
      v2 /= highValue;
      v1 /= highValue;
      v3 /= highValue;
   }

   public void setMaxSpeed(double maxSpeed) {
      this.maxSpeed = maxSpeed;
   }

}