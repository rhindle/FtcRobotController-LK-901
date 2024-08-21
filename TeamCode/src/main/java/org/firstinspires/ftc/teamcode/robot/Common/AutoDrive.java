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
   public DrivePowers driveP;
   double maxSpeed = 1;
   double motorMinPower = 0.025;
   double motorMaxPower = 1;
   public PIDCoefficients PIDmovement = new PIDCoefficients(0.06,0.0012,0.006); //.12 0 .035
   public PIDCoefficients PIDrotate = new PIDCoefficients(0.026,0.01,0.00025);  // .03 0 0
   PIDCoefficients PIDmovement_calculated = new PIDCoefficients(0,0,0);
   PIDCoefficients PIDrotate_calculated = new PIDCoefficients(0,0,0);
   long PIDTimeCurrent;
   long PIDTimeLast;
//   double errorDistLast;
//   double errorRotLast;
   double navAngleLast;
   boolean onTargetByAccuracy = false;
   public boolean isNavigating = false;
   public boolean isHolding = false;
   NavigationTarget target;
   Error error, errorLast;
   long navStartTime;

   /* Constructor */
   public AutoDrive(Parts parts){
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
//      updateRobotPosition();
      // todo: consider... if position is null, should targetposition be reset?
      TelemetryMgr.Message(4, "AutoDrive", status);
//      if (status==Status.DRIVING) {
//
//      }
      autoDrivePower();
   }

   public void stop() {
   }

   // Determine motor speeds when under automatic control
   public void autoDrivePower () {
      double pDist, pRot, navAngle;
      //todo: check for isNavigating and isHolding
      if (parts.positionMgr.noPosition()) return;   //todo: make sure this doesn't leave motors running
      updateError();
      onTargetByAccuracy = target.inToleranceByTime(parts.positionMgr.robotPosition);
      if (onTargetByAccuracy) status=Status.SUCCESS; else status=Status.DRIVING;
      //timeout?
      if (System.currentTimeMillis() - navStartTime > target.timeLimit) {
         isNavigating = false;
         parts.drivetrain.stopDriveMotors();
         if (!onTargetByAccuracy) status=Status.TIMEOUT;
      }
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

      navAngle = Math.toDegrees(Math.atan2(error.y,error.x));  // angle to xy destination (vector when combined with distance)

      PIDTimeCurrent = System.currentTimeMillis();

      if (Math.abs(navAngle-navAngleLast)>45) PIDmovement_calculated.i = 0;  // test - zero the I if we pass through an inflection

      PIDmovement_calculated.p = PIDmovement.p * error.dist;
      PIDmovement_calculated.i += PIDmovement.i * error.dist * ((PIDTimeCurrent - PIDTimeLast) / 1000.0);
      PIDmovement_calculated.i = Math.max(Math.min(PIDmovement_calculated.i,1),-1);
      PIDmovement_calculated.d = PIDmovement.d * (error.dist - errorLast.dist) / ((PIDTimeCurrent - PIDTimeLast) / 1000.0);

      if (parts.useDriveEncoders) {
         pDist = PIDmovement_calculated.p;
      } else {
         pDist = PIDmovement_calculated.p +  PIDmovement_calculated.i +  PIDmovement_calculated.d;
      }
      pDist = Math.max(Math.min(pDist,motorMaxPower),motorMinPower);
//      if (accurate==0) pDist = 1;  // don't bother with proportional when hitting transitional destinations
      if (target.noSlow) pDist = 1;  // don't bother with proportional when hitting transitional destinations

      PIDrotate_calculated.p = PIDrotate.p * error.rot;
      PIDrotate_calculated.i += PIDrotate.i * error.rot * ((PIDTimeCurrent - PIDTimeLast) / 1000.0);
      PIDrotate_calculated.i = Math.max(Math.min(PIDrotate_calculated.i,1),-1);
      PIDrotate_calculated.d = PIDrotate.d * (error.dist - errorLast.rot) / ((PIDTimeCurrent - PIDTimeLast) / 1000.0);

      if (parts.useDriveEncoders) {
         pRot =  PIDrotate_calculated.p;
      } else {
         pRot =  PIDrotate_calculated.p +  PIDrotate_calculated.i +  PIDrotate_calculated.d;
      }
      pRot = Math.max(Math.min(Math.abs(pRot),motorMaxPower),motorMinPower)*Math.signum(pRot)*-1;

      PIDTimeLast = PIDTimeCurrent;
      errorLast = error.clone();
//      errorDistLast = errordist;
//      errorRotLast = errorRot;
      navAngleLast = navAngle;

      TelemetryMgr.Message(7, "NavDistance", JavaUtil.formatNumber(error.dist, 2));
      TelemetryMgr.Message(7, "NavAngle", JavaUtil.formatNumber(navAngle, 2));
      TelemetryMgr.Message(7, "NavRotation", JavaUtil.formatNumber(error.rot, 2));
      TelemetryMgr.Message(7, "pDist", JavaUtil.formatNumber(pDist, 2));
      TelemetryMgr.Message(7, "pRot", JavaUtil.formatNumber(pRot, 2));

      navAngle -= parts.positionMgr.robotPosition.R;  // need to account for how the robot is oriented
      double autoSpeed = pDist * 1;  // 1 here is maxspeed; could be turned into a variable
      // the following adds the mecanum X, Y, and rotation motion components for each wheel
      driveP.v0 = autoSpeed * (Math.cos(Math.toRadians(navAngle)) - Math.sin(Math.toRadians(navAngle))) + pRot;
      driveP.v2 = autoSpeed * (Math.cos(Math.toRadians(navAngle)) + Math.sin(Math.toRadians(navAngle))) + pRot;
      driveP.v1 = autoSpeed * (Math.cos(Math.toRadians(navAngle)) + Math.sin(Math.toRadians(navAngle))) - pRot;
      driveP.v3 = autoSpeed * (Math.cos(Math.toRadians(navAngle)) - Math.sin(Math.toRadians(navAngle))) - pRot;

      // scale to no higher than 1
      driveP.scaleMax(1);
      parts.drivetrain.setDrivePowers(driveP);
   }

   public void setMaxSpeed(double maxSpeed) {
      this.maxSpeed = maxSpeed;
   }

   public void resetPID() {
      PIDmovement_calculated.i = 0;
      PIDrotate_calculated.i = 0;
   }

   public void updateError() {
      error.x = target.targetPos.X - parts.positionMgr.robotPosition.X;  // error in x
      error.y = target.targetPos.Y - parts.positionMgr.robotPosition.Y;  // error in y
      error.rot = getHeadingError(target.targetPos.R);  // error in rotation   //20221222 added deltaheading!?
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

//   private void updateRobotPosition() {
//      // todo: consider... if position is null, should targetposition be reset?
//      if (parts.positionMgr.hasPosition()) robotPosition = parts.positionMgr.robotPosition.clone();
//      else robotPosition = null;
//   }

   public void setTargetToCurrentPosition() {
      if (parts.positionMgr.noPosition()) return;
//      setTarget(new NavigationTarget(new Position (
//              Math.round(parts.positionMgr.robotPosition.X),
//              Math.round(parts.positionMgr.robotPosition.Y),
//              Math.round(parts.positionMgr.robotPosition.R)
//            )));
      target = new NavigationTarget(new Position (
              Math.round(parts.positionMgr.robotPosition.X),
              Math.round(parts.positionMgr.robotPosition.Y),
              Math.round(parts.positionMgr.robotPosition.R)
            ));
      resetPID();
      isHolding = true;
   }

   public void setTarget(NavigationTarget target) {
      if (parts.positionMgr.noPosition()) return;
      this.target = target;
      resetPID();
      navStartTime = System.currentTimeMillis();
      isNavigating = true;
   }

   public void cancelNavigation () {
      status=Status.CANCELED;
      isNavigating = false;
      parts.drivetrain.stopDriveMotors();
   }

   public enum Status {
      IDLE,
      DRIVING,
      SUCCESS,
      TIMEOUT,
      CANCELED,
      POSLOST
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