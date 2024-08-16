package org.firstinspires.ftc.teamcode.robot.Common;

import org.firstinspires.ftc.teamcode.robot.Common.Tools.DataTypes.DrivePowers;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.PartsInterface;
public class UserDrive implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;

   public DrivePowers driveP;
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
      driveP = new DrivePowers();
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
      driveP.v0 = driveSpeed * (Math.cos(Math.toRadians(driveAngle)) - Math.sin(Math.toRadians(driveAngle))) + rotate;
      driveP.v2 = driveSpeed * (Math.cos(Math.toRadians(driveAngle)) + Math.sin(Math.toRadians(driveAngle))) + rotate;
      driveP.v1 = driveSpeed * (Math.cos(Math.toRadians(driveAngle)) + Math.sin(Math.toRadians(driveAngle))) - rotate;
      driveP.v3 = driveSpeed * (Math.cos(Math.toRadians(driveAngle)) - Math.sin(Math.toRadians(driveAngle))) - rotate;

      // scale so average motor speed is not more than maxSpeed, but only if maxspeed <> 1
      if (maxSpeed != 1) {
         driveP.scaleAverage(maxSpeed);
      }

      // scale to no higher than 1
      driveP.scaleMax(1);
   }

   public void setMaxSpeed(double maxSpeed) {
      this.maxSpeed = maxSpeed;
   }

}