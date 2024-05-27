package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.robot.Common.ButtonMgr;
import org.firstinspires.ftc.teamcode.robot.Common.Parts;
import org.firstinspires.ftc.teamcode.robot.Common.TelemetryHandler;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.Position;
import org.firstinspires.ftc.teamcode.robot.GoCanum.PartsGC;

@TeleOp(name = "AA_MentorTestBot02", group = "")
//@Disabled
public class MentorTestBot02 extends LinearOpMode {

   private ElapsedTime elapsedTime; // = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

   private ElapsedTime timerLoop = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
   private double timeLoop;

   private final double maxSpeed = 1;//0.2;

   //    double DriveSpeed, DriveAngle, Rotate;
   double currentError = 0;

   public Parts parts;

   @Override
   public void runOpMode() {

      parts = new PartsGC(this, Parts.robotType.GOCANUM);

      parts.useODO = true;
      parts.useSlamra = true; //true;
      //robot.reverseDrive = true;  // for AndyMark test
      parts.useDistanceSensors = false; //true; //false;
      parts.fieldStartPosition = new Position (36,63,-90);
      parts.odoRobotOffset = new Position (2.25,0,0);
      parts.slamraRobotOffset = new Position(-8,-1,0);
      parts.setup();

      elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

      parts.preInit();

      TelemetryHandler.setDebugLevel(10);
      while (!isStarted()) {
         TelemetryHandler.Message(1, ">", "Press Play to start");
         TelemetryHandler.Message(1, ">", "Robot Heading = %.1f", parts.robot.returnImuHeading(true));
         TelemetryHandler.Message(1, "Drive Type:", parts.reverseDrive ? "AndyMark" : "GobildaBot");

         parts.initLoop();

         if (parts.buttonMgr.wasTapped(1, ButtonMgr.Buttons.x))
            parts.reverseDrive = !parts.reverseDrive;
         if (parts.buttonMgr.wasTapped(2, ButtonMgr.Buttons.x))
            parts.reverseDrive = !parts.reverseDrive;

         TelemetryHandler.Update(true);
         sleep(20);
      }

      parts.preRun();

      parts.navigator.setMaxSpeed(maxSpeed);
      //navigator.setDeltaHeading();

      TelemetryHandler.setDebugLevel(10);
      if (opModeIsActive()) {
         // Put run blocks here.
         while (opModeIsActive()) {

            parts.robot.loop();               // Clears bulk data and reads IMU
            parts.buttonMgr.loop();           // Processes digital controller input
            parts.odometry.loop();           // Updates odometry X, Y, Rotation
            if (parts.useSlamra) parts.slamra.loop();
            parts.sensors.loop();       // Update distance sensors, etc.

            addTelemetryLoopStart();

            parts.controls.loop();            // Acts on user controls

//                lifter.loop();
            parts.navigator.loop();           // Automatic navigation actions

            addTelemetryLoopEnd();
            //telemetry.update();
            TelemetryHandler.Update();
         }
      }

      parts.stop();

   }

   private void addTelemetryLoopStart() {
      TelemetryHandler.Message(2, "Loop time (ms)", JavaUtil.formatNumber(calculateLoopTime(), 0));
      TelemetryHandler.Message(2, "heading", JavaUtil.formatNumber(parts.robot.returnImuHeading(),2));
//      TelemetryHandler.Message(3, "rangeL", String.format("%.01f in", parts.sensors.distL));
//      TelemetryHandler.Message(3, "rangeM", String.format("%.01f in", parts.sensors.distM));
//      TelemetryHandler.Message(3, "rangeR", String.format("%.01f in", parts.sensors.distR));
//        telemetry.addData("raw__", localizer.odoRawPose.toString(2));
//        telemetry.addData("robot", localizer.odoRobotPose.toString(2));
//        telemetry.addData("final", localizer.odoFinalPose.toString(2));
      parts.odometry.addTeleOpTelemetry();
   }

   private void addTelemetryLoopEnd() {
      TelemetryHandler.Message(4, "r (magnitude)", parts.controls.DriveSpeed);
      TelemetryHandler.Message(4, "robotAngle", parts.controls.DriveAngle);
      TelemetryHandler.Message(4, "rotate", parts.controls.Rotate);
      TelemetryHandler.Message(4, "storedHeading", JavaUtil.formatNumber(parts.navigator.storedHeading, 2));
      TelemetryHandler.Message(4, "deltaHeading", JavaUtil.formatNumber(parts.navigator.deltaHeading, 2));
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

   }

   // Calculate loop time for performance optimization
   private double calculateLoopTime() {
      timeLoop = timerLoop.milliseconds();
      timerLoop.reset();
      return timeLoop;
   }

}
