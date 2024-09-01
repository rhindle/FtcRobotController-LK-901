package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr.Category;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.PartsDS;

@TeleOp(name = "AA_MentorTestAT01a", group = "")
//@Disabled
public class MentorTestAT01 extends LinearOpMode {

   private ElapsedTime elapsedTime; // = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

   private ElapsedTime timerLoop = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
   private double timeLoop;

   private final double maxSpeed = 1;//0.2;

   //    double DriveSpeed, DriveAngle, Rotate;
   double currentError = 0;

   public Parts parts;

   @Override
   public void runOpMode() {

//      parts = new PartsDS(this, Parts.robotType.GOCANUM);
      parts = new PartsDS(this);

      parts.useODO = false;
      parts.useSlamra = true; //true;
      parts.useNeoMatrix = true;
      parts.useAprilTag = true;
      parts.useDrivetrainEncoders = true;
      parts.reverseDrive = false;  // true for AndyMark
      parts.useDistanceSensors = false; //true; //false;
      //parts.fieldStartPosition = new Position (36,63,-90);
      parts.odoRobotOffset = new Position (2.25,0,0);
      parts.slamraRobotOffset = new Position(-8,-1,0);
      parts.setup();

      elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

      parts.preInit();

      TelemetryMgr.setDebugLevel(10);
//      TelemetryMgr.enableCategories(new TelemetryMgr.Category[] {TelemetryMgr.Category.AUTODRIVE});
      TelemetryMgr.enableAllCategories();
      while (!isStarted()) {
         TelemetryMgr.message(Category.MANDATORY, ">", "Press Play to start");
//         TelemetryMgr.Message(1, ">", "Robot Heading = %.1f", parts.robot.returnImuHeading(true));
         TelemetryMgr.message(Category.MANDATORY, ">", "Robot Heading = %.1f", parts.imuMgr.returnImuHeadingRaw(true));
         TelemetryMgr.message(Category.MANDATORY, "Drive Type:", parts.reverseDrive ? "AndyMark" : "GobildaBot");

         parts.initLoop();

         if (parts.buttonMgr.wasTapped(1, ButtonMgr.Buttons.x))
            parts.reverseDrive = !parts.reverseDrive;
         if (parts.buttonMgr.wasTapped(2, ButtonMgr.Buttons.x))
            parts.reverseDrive = !parts.reverseDrive;

         TelemetryMgr.Update(true);
         sleep(20);
      }

      parts.preRun();

//      parts.navigator.setMaxSpeed(maxSpeed);
      //navigator.setDeltaHeading();

      TelemetryMgr.setDebugLevel(10);
      if (opModeIsActive()) {
         // Put run blocks here.
         while (opModeIsActive()) {

            addTelemetryLoopStart();
            parts.runLoop();
            addTelemetryLoopEnd();
            TelemetryMgr.Update();
         }
      }

      parts.stop();

   }

   private void addTelemetryLoopStart() {
      TelemetryMgr.message(Category.BASIC, "Loop time (ms)", JavaUtil.formatNumber(calculateLoopTime(), 0));
//      TelemetryMgr.Message(2, "IMU raw heading", JavaUtil.formatNumber(parts.robot.returnImuHeading(),2));
      TelemetryMgr.message(Category.BASIC, "IMU raw heading", JavaUtil.formatNumber(parts.imuMgr.returnImuHeadingRaw(),2));
//      TelemetryHandler.Message(3, "rangeL", String.format("%.01f in", parts.sensors.distL));
//      TelemetryHandler.Message(3, "rangeM", String.format("%.01f in", parts.sensors.distM));
//      TelemetryHandler.Message(3, "rangeR", String.format("%.01f in", parts.sensors.distR));
//        telemetry.addData("raw__", localizer.odoRawPose.toString(2));
//        telemetry.addData("robot", localizer.odoRobotPose.toString(2));
//        telemetry.addData("final", localizer.odoFinalPose.toString(2));
      if (parts.useODO) parts.odometry.addTeleOpTelemetry();
   }

   private void addTelemetryLoopEnd() {
      TelemetryMgr.message(Category.CONTROLS, "r (magnitude)", parts.controls.driveData.driveSpeed);
      TelemetryMgr.message(Category.CONTROLS, "robotAngle", parts.controls.driveData.driveAngle);
      TelemetryMgr.message(Category.CONTROLS, "rotate", parts.controls.driveData.rotate);
      TelemetryMgr.message(Category.USERDRIVE, "storedHeading", JavaUtil.formatNumber(parts.userDrive.storedHeading, 2));
      TelemetryMgr.message(Category.USERDRIVE, "deltaHeading", JavaUtil.formatNumber(parts.userDrive.deltaHeading, 2));
//      TelemetryMgr.message(Category.USERDRIVE, "modifyHeading", JavaUtil.formatNumber(parts.autoDrive.modifyHeading, 2));
//      TelemetryMgr.Message(4, "IMU-Modified", JavaUtil.formatNumber(parts.robot.returnImuHeading()-parts.navigator.deltaHeading, 2));
//      TelemetryMgr.Message(4, "IMU-Modified", JavaUtil.formatNumber(parts.robot.returnImuHeading() - parts.navigator.modifyHeading, 2));
//      TelemetryMgr.message(Category.IMU, "IMU-Modified", JavaUtil.formatNumber(parts.imuMgr.returnImuHeadingRaw() - parts.navigator.modifyHeading, 2));
      TelemetryMgr.message(Category.IMU, "IMU-Modified", JavaUtil.formatNumber(parts.imuMgr.returnImuRobotHeading(),2));


//        telemetry.addData("error", JavaUtil.formatNumber(currentError, 2));
//        telemetry.addData("v0", JavaUtil.formatNumber(navigator.v0, 2));
//        telemetry.addData("v1", JavaUtil.formatNumber(navigator.v2, 2));
//        telemetry.addData("v2", JavaUtil.formatNumber(navigator.v1, 2));
//        telemetry.addData("v3", JavaUtil.formatNumber(navigator.v3, 2));
//        telemetry.addData("rot about Z", JavaUtil.formatNumber(robot.returnImuHeading(),2));
//        telemetry.addData("odo Heading", JavaUtil.formatNumber(localizer.returnOdoHeading(), 2));
//        telemetry.addData("Target X", navigator.targetX);
//        telemetry.addData("Target Y", navigator.targetY);
//        telemetry.addData("Target Rot", navigator.targetRot);
//        telemetry.addData ("OdoY", localizer.encoderY);
//        telemetry.addData ("OdoXL", localizer.encoderXL);
//        telemetry.addData ("OdoXR", localizer.encoderXR);
//        telemetry.addData ("X", JavaUtil.formatNumber(localizer.xPos, 2));
//        telemetry.addData ("Y", JavaUtil.formatNumber(localizer.yPos, 2));
      Position robo = parts.dsApriltag.getTagRobotPosition();
      if (robo!=null) TelemetryMgr.message(Category.APRILTAG, String.format("robotPos XYZ %6.1f %6.1f %6.1f  (inch, inch, deg)", robo.X, robo.Y, robo.R));
      else TelemetryMgr.message(Category.APRILTAG,"robotpos - no tag position");

   }

   // Calculate loop time for performance optimization
   private double calculateLoopTime() {
      timeLoop = timerLoop.milliseconds();
      timerLoop.reset();
      return timeLoop;
   }

}
