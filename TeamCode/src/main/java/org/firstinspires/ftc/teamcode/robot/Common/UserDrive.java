package org.firstinspires.ftc.teamcode.robot.Common;

import org.firstinspires.ftc.teamcode.robot.Common.Tools.DataTypes.DrivePowers;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.PartsInterface;

public class UserDrive implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;

   public DrivePowers drivePowers;
   double driveSpeed, driveAngle, rotate;
   boolean useFieldCentricDrive = false;
   boolean useHeadingHold = true;
   boolean useHoldPosition = true;
   boolean isDriving = false;
   public double storedHeading = 0;
   public double deltaHeading = 0;
   double speedMaximumWithPosition = 1;
   double speedMaximumNoPosition = 0.33;
   double speedMaximum = speedMaximumWithPosition;
   long idleDelay = System.currentTimeMillis();
   long headingDelay = System.currentTimeMillis();

   /* Constructor */
   public UserDrive(Parts parts){
      construct(parts);
   }

   void construct(Parts parts){
      this.parts = parts;
   }

   public void initialize(){
      drivePowers = new DrivePowers();
   }

   public void preInit() {
   }

   public void initLoop() {
   }

   public void preRun() {
   }

   public void runLoop() {
      userDrivePower();
   }

   public void stop() {
   }

   // Determine motor speeds when under driver control
   public void userDrivePower () {

      handleDriveIdle();  // todo: left off here
      handleRotateIdle();
      applySpeedFences();

      if (useHoldPosition) {
         if (idleDelay <= System.currentTimeMillis()) {
            // no driver input, so let autoDrive take over and do nothing else
            parts.autoDrive.setTargetToCurrentPosition();
            return;
         }
      }
      if (useHeadingHold) {
         // Correct the heading if not currently being controlled
         if (headingDelay <= System.currentTimeMillis()) {
//            this.rotate = parts.autoDrive.getHeadingError(storedHeading) / -15 * (driveSpeed + 0.2);   // base this on speed?
            // start with the same proportional in autodrive
            this.rotate = parts.autoDrive.PIDrotate.p * parts.autoDrive.getHeadingError(storedHeading);
            // and then scale it (determined experimentally, scaled like old code)
            this.rotate *= -2.5 * (driveSpeed + 0.2);
         }
      }

      drivePowers.v0 = driveSpeed * (Math.cos(Math.toRadians(driveAngle)) - Math.sin(Math.toRadians(driveAngle))) + rotate;
      drivePowers.v2 = driveSpeed * (Math.cos(Math.toRadians(driveAngle)) + Math.sin(Math.toRadians(driveAngle))) + rotate;
      drivePowers.v1 = driveSpeed * (Math.cos(Math.toRadians(driveAngle)) + Math.sin(Math.toRadians(driveAngle))) - rotate;
      drivePowers.v3 = driveSpeed * (Math.cos(Math.toRadians(driveAngle)) - Math.sin(Math.toRadians(driveAngle))) - rotate;

      // scale so average motor speed is not more than maxSpeed, but only if maxspeed <> 1
      if (speedMaximum != 1) {
         drivePowers.scaleAverage(speedMaximum);
      }
      // scale to no higher than 1
      drivePowers.scaleMax(1);
      parts.drivetrain.setDrivePowers(drivePowers);
   }

   public void setUserDriveSettings(double driveSpeed, double driveAngle, double rotate) {
      // Controls uses this method to set drive parameters
      this.driveSpeed = driveSpeed;
      this.driveAngle = driveAngle;
      this.rotate = rotate;

      isDriving = !(driveSpeed == 0 && rotate == 0);
      if (isDriving) {
         parts.autoDrive.cancelNavigation();
//         parts.autoDrive.isHolding = false;
//         parts.autoDrive.isNavigating = false;
      }

      // Skip the fancy driving if no position is available
      if (parts.positionMgr.noPosition()) return;

      // Modify for field centric Drive
      if (useFieldCentricDrive) {
         this.driveAngle = driveAngle - storedHeading + deltaHeading;  //todo: verify this (rework based on IMUmgr?)
      }
   }

   public void applySpeedFences() {
      if (parts.positionMgr.noPosition()) {
         speedMaximum = speedMaximumNoPosition;
      }
      else {
         speedMaximum = speedMaximumWithPosition * parts.dsSpeedControl.checkFences(driveAngle, useFieldCentricDrive);  //todo: verify this
      }
   }

   public void handleDriveIdle() {  //todo: finish this method
      // The purpose of this is to hold the robot position
      // delayed 500ms after any movement input
      if (parts.positionMgr.noPosition()) return;
      if (!(driveSpeed == 0 && rotate == 0)) {
         idleDelay = System.currentTimeMillis() + 500;  //was 250 to match rotate?
//         setTargetToCurrentPosition(); ?????
         //deactivate autodrive???  But may still want autoheading???
      }
      if (idleDelay < System.currentTimeMillis() && useHoldPosition) {
//         setTargetToCurrentPosition();
         //activate autodrive???
      }
   }

   public void handleRotateIdle() { //(double rotate) {
      // The purpose of this is to deal with IMU latency (and robot inertia)
      // by storing the heading delayed 250ms after any rotational input
      if (parts.positionMgr.noPosition()) return;
      if (rotate != 0) {
         storedHeading = parts.positionMgr.robotPosition.R;
         headingDelay = System.currentTimeMillis() + 250;
      }
      else if (headingDelay > System.currentTimeMillis()) {
         // keep re-reading until delay has passed
         storedHeading = parts.positionMgr.robotPosition.R;
      }
   }

   public void setSpeedMaximum(double speedMaximum) {
      this.speedMaximum = speedMaximum;
   }

}