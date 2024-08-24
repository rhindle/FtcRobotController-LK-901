package org.firstinspires.ftc.teamcode.robot.Common;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.DataTypes.DrivePowers;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.DataTypes.NavigationTarget;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.Functions;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.PartsInterface;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.DataTypes.Position;

import androidx.annotation.NonNull;

public class AutoDrive implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;

//   public Position robotPosition = new Position();
//   Position targetPos = new Position();
//   int accurate = 1;  // 0 is loose, 1 is tight, more later?

   Status status;
   public DrivePowers drivePowers;
   double speedMaximum = 1;
   double powerMotorMinimum = 0.025;
   double powerMotorMaximum = 1;
   public PIDCoefficients PIDmovement = new PIDCoefficients(0.06,0.0012,0.006); //.12 0 .035
   public PIDCoefficients PIDrotate = new PIDCoefficients(0.026,0.01,0.00025);  // .03 0 0
   PIDCoefficients PIDmovement_calculated = new PIDCoefficients(0,0,0);
   PIDCoefficients PIDrotate_calculated = new PIDCoefficients(0,0,0);
   public boolean onTargetByAccuracy = false;
   public boolean isNavigating = false;
   public boolean isHolding = false;
   public boolean isLate = false;
   public boolean abortOnTimeout = true;
   NavigationTarget navTarget;
   Error error, errorLast;
   long timeNavStart, timePIDCurrent, timePIDLast;
   double powerTranslate, powerRotate, navAngle, navAngleLast;

   /* Constructor */
   public AutoDrive(Parts parts){
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
      // only calculate the drive powers if actively navigating or holding
      if (isNavigating || isHolding) {
         status=isNavigating ? Status.DRIVING : Status.HOLDING;  //todo: is this needed?
         autoDrivePower();
      }
      else {
         if (status!=Status.SUCCESS && status!=Status.TIMEOUT && status!=Status.CANCELED && status!=Status.LOST) {
            // todo: not sure LOST belongs, because that implies navigation can resume? See also refuseNavigation().
            status=Status.IDLE;
         }
      }
      TelemetryMgr.Message(4, "AutoDrive", status);
   }

   public void stop() {
      cancelNavigation();
   }

   public void autoDrivePower () {
      /* This is used for both driving to position and holding a position */

      /* If there is no position, don't drive */
      if (parts.positionMgr.noPosition()) {
         //todo: make sure this doesn't leave motors running [now check the work]
         status=Status.LOST;
         parts.drivetrain.stopDriveMotors();
         return;
      }

      /* Check for timeout (only if navigating, and if !abortOnTimeout, keep driving anyway). */
      if (isNavigating && System.currentTimeMillis() - timeNavStart > navTarget.timeLimit) {
         isLate=true;
         status=Status.TIMEOUT;
         if (abortOnTimeout) {
            cancelNavigation();
            return;
         }
      } else isLate=false;

      /* Check for being on target */
      //todo: Do we want to switch mode from navigating to holding and thus avoid timeouts or whatever?
      //todo: Have navigate that ends in holding vs end idle?
      onTargetByAccuracy = navTarget.inToleranceByTime(parts.positionMgr.robotPosition);
      if (onTargetByAccuracy) {
         status=isNavigating ? Status.SUCCESS : Status.HOLDING;
         isNavigating=false;
         if (!isHolding) parts.drivetrain.stopDriveMotors();
      } else status=Status.DRIVING;

      if (!isNavigating && !isHolding) return;


////      double errorDist, errorX, errorY, errorRot, pDist, pRot, navAngle;
//
////      errorX = targetPos.X - parts.positionMgr.robotPosition.X;  // error in x
////      errorY = targetPos.Y - parts.positionMgr.robotPosition.Y;  // error in y
////      TelemetryMgr.Message(7, "DeltaX", JavaUtil.formatNumber(errorX, 2));
////      TelemetryMgr.Message(7, "DeltaY", JavaUtil.formatNumber(errorY, 2));
////      errorRot = getHeadingError(targetPos.R);  // error in rotation   //20221222 added deltaheading!?
////      errorDist = Math.sqrt(Math.pow(errorX,2) + Math.pow(errorY,2));  // distance (error) from xy destination
//
//      //exit criteria if destination has been adequately reached   //todo: replace all this
//      onTargetByAccuracy = false;
//      if (accurate==0 && error.dist<2) {  // no rotation component here
//         onTargetByAccuracy = true;
//      }
//      if (error.dist<0.5 && Math.abs(error.rot)<0.2) {
//         onTargetByAccuracy = true;
//      }
//      if (accurate==2 && error.dist<1 && Math.abs(error.rot)<1) {
//         onTargetByAccuracy = true;
//      }
//      if (accurate==3 && error.dist<2 && Math.abs(error.rot)<5) {  // ~ like 0 but still gets proportional
//         onTargetByAccuracy = true;
//      }
//      if (onTargetByAccuracy) {
////         navStep++;
//         return;
//      }

      updateError();
      navAngle = Math.toDegrees(Math.atan2(error.y,error.x));  // angle to xy destination (vector when combined with distance)
      timePIDCurrent = System.currentTimeMillis();

      /* Calculate translation PID and power */
      if (Math.abs(navAngle-navAngleLast)>45) PIDmovement_calculated.i = 0;  // test - zero the I if we pass through an inflection
      PIDmovement_calculated.p = PIDmovement.p * error.dist;
      PIDmovement_calculated.i += PIDmovement.i * error.dist * ((timePIDCurrent - timePIDLast) / 1000.0);
      PIDmovement_calculated.i = Math.max(Math.min(PIDmovement_calculated.i,1),-1);
      PIDmovement_calculated.d = PIDmovement.d * (error.dist - errorLast.dist) / ((timePIDCurrent - timePIDLast) / 1000.0);
      if (parts.useDrivetrainEncoders) {
         powerTranslate = PIDmovement_calculated.p;
      } else {
         powerTranslate = PIDmovement_calculated.p +  PIDmovement_calculated.i +  PIDmovement_calculated.d;
      }
      if (navTarget.noSlow) powerTranslate = 1;  // don't bother with proportional when hitting transitional destinations
      powerTranslate = Math.max(Math.min(powerTranslate, powerMotorMaximum), powerMotorMinimum);

      /* Calculate rotation PID and power */
      PIDrotate_calculated.p = PIDrotate.p * error.rot;
      PIDrotate_calculated.i += PIDrotate.i * error.rot * ((timePIDCurrent - timePIDLast) / 1000.0);
      PIDrotate_calculated.i = Math.max(Math.min(PIDrotate_calculated.i,1),-1);
      PIDrotate_calculated.d = PIDrotate.d * (error.dist - errorLast.rot) / ((timePIDCurrent - timePIDLast) / 1000.0);
      if (parts.useDrivetrainEncoders) {
         powerRotate =  PIDrotate_calculated.p;
      } else {
         powerRotate =  PIDrotate_calculated.p +  PIDrotate_calculated.i +  PIDrotate_calculated.d;
      }
      powerRotate = Math.max(Math.min(Math.abs(powerRotate), powerMotorMaximum), powerMotorMinimum)*Math.signum(powerRotate)*-1;

      /* Save data for next loop */
      timePIDLast = timePIDCurrent;
      errorLast = error.clone();
      navAngleLast = navAngle;

      TelemetryMgr.Message(7, "NavDistance", JavaUtil.formatNumber(error.dist, 2));
      TelemetryMgr.Message(7, "NavAngle", JavaUtil.formatNumber(navAngle, 2));
      TelemetryMgr.Message(7, "NavRotation", JavaUtil.formatNumber(error.rot, 2));
      TelemetryMgr.Message(7, "pDist", JavaUtil.formatNumber(powerTranslate, 2));
      TelemetryMgr.Message(7, "pRot", JavaUtil.formatNumber(powerRotate, 2));

      /* Calculate and set the drivetrain powers */
      navAngle -= parts.positionMgr.robotPosition.R;  // need to account for how the robot is oriented
      double autoSpeed = powerTranslate * 1;  // 1 here is maxspeed; could be turned into a variable
      // the following adds the mecanum X, Y, and rotation motion components for each wheel
      drivePowers.v0 = autoSpeed * (Math.cos(Math.toRadians(navAngle)) - Math.sin(Math.toRadians(navAngle))) + powerRotate;
      drivePowers.v2 = autoSpeed * (Math.cos(Math.toRadians(navAngle)) + Math.sin(Math.toRadians(navAngle))) + powerRotate;
      drivePowers.v1 = autoSpeed * (Math.cos(Math.toRadians(navAngle)) + Math.sin(Math.toRadians(navAngle))) - powerRotate;
      drivePowers.v3 = autoSpeed * (Math.cos(Math.toRadians(navAngle)) - Math.sin(Math.toRadians(navAngle))) - powerRotate;
      // scale so average motor speed is not more than maxSpeed, but only if maxspeed <> 1
      if (speedMaximum != 1) {
         drivePowers.scaleAverage(speedMaximum);
      }
      // scale to no higher than 1
      drivePowers.scaleMax(1);
      parts.drivetrain.setDrivePowers(drivePowers);
   }

   public void setSpeedMaximum(double speedMaximum) {
      this.speedMaximum = speedMaximum;
   }

   public void resetPID() {
      PIDmovement_calculated.i = 0;
      PIDrotate_calculated.i = 0;
   }

   public void updateError() {
      error.x = navTarget.targetPos.X - parts.positionMgr.robotPosition.X;  // error in x
      error.y = navTarget.targetPos.Y - parts.positionMgr.robotPosition.Y;  // error in y
      error.rot = getHeadingError(navTarget.targetPos.R);  // error in rotation   //20221222 added deltaheading!?
      error.dist = Math.sqrt(Math.pow(error.x,2) + Math.pow(error.y,2));  // distance (error) from xy destination
      TelemetryMgr.Message(7, "DeltaX", JavaUtil.formatNumber(error.x, 2));
      TelemetryMgr.Message(7, "DeltaY", JavaUtil.formatNumber(error.y, 2));
   }

   // Get heading error
   public double getHeadingError(double targetAngle) {
      if (parts.positionMgr.noPosition()) return 0;
      double robotError;
      robotError = targetAngle - parts.positionMgr.robotPosition.R;
      return Functions.normalizeAngle(robotError);
   }

   public void setTargetToCurrentPosition() {
      if (parts.positionMgr.noPosition()) {
         cancelNavigation();
         status = Status.LOST;
         return;
      }
      navTarget = new NavigationTarget(new Position (
              Math.round(parts.positionMgr.robotPosition.X),
              Math.round(parts.positionMgr.robotPosition.Y),
              Math.round(parts.positionMgr.robotPosition.R)
            ));
      isNavigating = false;
      isHolding = true;
      resetPID();
   }

   public void setNavTarget(NavigationTarget navTarget) {
      setTarget(navTarget, true);
   }

   public void setTarget(NavigationTarget target, boolean hold) {
      //if (parts.positionMgr.noPosition()) return;  //todo:Is this necessary? Or just rely on the loop's checks
      this.navTarget = target;
      timeNavStart = System.currentTimeMillis();
      isNavigating = true;
      isHolding = hold;
      resetPID();
   }

   public void cancelNavigation() {
      status = Status.CANCELED;
      isNavigating = false;
      isHolding = false;
      parts.drivetrain.stopDriveMotors();
   }

   public enum Status {
      DRIVING,
      HOLDING,
      IDLE,
      SUCCESS,
      LATE,
      TIMEOUT,
      CANCELED,
      LOST
   }

   public class Error {

      public double x, y, rot, dist;

      public Error(double x, double y, double rot, double dist) {
         this.x = x;
         this.y = y;
         this.rot = rot;
         this.dist = dist;
      }

      public Error() {
         this.x = 0;
         this.y = 0;
         this.rot = 0;
         this.dist = 0;
      }

      @NonNull
      public Error clone() {
         return new Error(x, y, rot, dist);
      }
   }
}