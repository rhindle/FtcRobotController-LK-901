package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.robot.Universal.ButtonMgr;
import org.firstinspires.ftc.teamcode.robot.Universal.Tools.Position;
import org.firstinspires.ftc.teamcode.robot.goCanum.Parts;

@TeleOp(name = "AA_MentorTestBot01", group = "")
//@Disabled
public class MentorTestBot01 extends LinearOpMode {

    private ElapsedTime elapsedTime; // = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private ElapsedTime timerLoop = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double timeLoop;

    private final double maxSpeed = 1;//0.2;

//    double DriveSpeed, DriveAngle, Rotate;
    double currentError = 0;

    public Parts parts;

    @Override
    public void runOpMode() {

        parts = new Parts(this, Parts.robotType.GOCANUM);

        parts.useODO = true;
        parts.useSlamra = true;
        //robot.reverseDrive = true;  // for AndyMark test
        parts.useDistanceSensors = false; //true; //false;
        parts.fieldStartPosition = new Position (36,63,-90);
        parts.odoRobotOffset = new Position (2.25,0,0);
        parts.slamraRobotOffset = new Position(-8,-1,0);
        parts.setup();

        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        parts.preInit();

        while (!isStarted()) {
            parts.buttonMgr.loop();
            if (parts.buttonMgr.wasTapped(1, ButtonMgr.Buttons.x))
                parts.reverseDrive = !parts.reverseDrive;
            if (parts.buttonMgr.wasTapped(2, ButtonMgr.Buttons.x))
                parts.reverseDrive = !parts.reverseDrive;

            // Prompt user to press start button.
            telemetry.addData(">", "Press Play to start");
            telemetry.addData(">", "Robot Heading = %.1f", parts.robot.returnImuHeading(true));
            telemetry.addData("Drive Type:", parts.reverseDrive ? "AndyMark" : "GobildaBot");

            if (parts.useSlamra) parts.slamra.loop();

            telemetry.update();
            sleep(20);
        }

        parts.preRun();

        parts.navigator.setMaxSpeed(maxSpeed);
        //navigator.setDeltaHeading();

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
                telemetry.update();
            }
        }

        parts.stop();

    }

    private void addTelemetryLoopStart() {
        telemetry.addData("Loop time (ms)", JavaUtil.formatNumber(calculateLoopTime(), 0));
        telemetry.addData("heading", JavaUtil.formatNumber(parts.robot.returnImuHeading(),2));
        telemetry.addData("rangeL", String.format("%.01f in", parts.sensors.distL));
        telemetry.addData("rangeM", String.format("%.01f in", parts.sensors.distM));
        telemetry.addData("rangeR", String.format("%.01f in", parts.sensors.distR));
//        telemetry.addData("raw__", localizer.odoRawPose.toString(2));
//        telemetry.addData("robot", localizer.odoRobotPose.toString(2));
//        telemetry.addData("final", localizer.odoFinalPose.toString(2));
        parts.odometry.addTeleOpTelemetry();
    }

    private void addTelemetryLoopEnd() {
        telemetry.addData("r (magnitude)", parts.controls.DriveSpeed);
        telemetry.addData("robotAngle", parts.controls.DriveAngle);
        telemetry.addData("rotate", parts.controls.Rotate);
        telemetry.addData("storedHeading", JavaUtil.formatNumber(parts.navigator.storedHeading, 2));
        telemetry.addData("deltaHeading", JavaUtil.formatNumber(parts.navigator.deltaHeading, 2));
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

//        telemetry.addData("rangeL", String.format("%.01f in", robot.sensors.distL));
//        telemetry.addData("rangeM", String.format("%.01f in", robot.sensors.distM));
//        telemetry.addData("rangeR", String.format("%.01f in", robot.sensors.distR));
    }

    // Calculate loop time for performance optimization
    private double calculateLoopTime() {
        timeLoop = timerLoop.milliseconds();
        timerLoop.reset();
        return timeLoop;
    }

}
