package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.ButtonMgr;
import org.firstinspires.ftc.teamcode.robot.NeoMatrix;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Tools.AdafruitNeoDriver;
import org.firstinspires.ftc.teamcode.robot.Tools.QwiicLEDStick;

@TeleOp (name="AA_Adafruit_Neo_1", group="Test")
//@Disabled
public class ZZ_Adafruit_Neo_1 extends LinearOpMode {

    Robot robot;
    ButtonMgr buttonMgr;
    NeoMatrix neo;

    int loopySample = 50;
    double[] loopyTime = new double[loopySample];
    int loopyTimeCounter = 0;
    int[][] textMatrix;

    //AdafruitNeoDriver neo = null;

    @Override
    public void runOpMode() {
        robot = new Robot(this);
        buttonMgr = new ButtonMgr(this);
        neo = new NeoMatrix(robot);

        robot.init();
        neo.init();
        neo.setUpdateLimit(0);

        while (!isStarted()) {
            telemetry.addData(">", "Waiting to start...");
            telemetry.update();
            sleep(100);
        }

        ElapsedTime loopElapsedTime = new ElapsedTime();

//        neo.drawRectangle(0,7,0,7, Color.rgb(5,0,0), true, Color.rgb(0,1,0));
        neo.drawRectangle(0,3,0,3, Color.rgb(0,4,0), true, Color.rgb(0,4,0));
        neo.drawRectangle(0,3,4,7, Color.rgb(3,3,0), true, Color.rgb(3,3,0));
        neo.drawRectangle(28,31,0,3, Color.rgb(2,2,2), true, Color.rgb(2,2,2));
        neo.drawRectangle(28,31,4,7, Color.rgb(0,0,4), true, Color.rgb(0,0,4));
        textMatrix = neo.buildPixelMapFromString("14273", neo.littleLetters, Color.rgb(5,5,5), 0);
        textMatrix = neo.shiftPixelMap(textMatrix,0,1, true);
        textMatrix = neo.shiftPixelMap(textMatrix,0,1, true);
        neo.applyPixelMapToBuffer(textMatrix,7,32, 0, true);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.loop();  // this will take care of clearing out the bulk reads
            buttonMgr.updateAll();
            neo.loop();

            neo.scrollRegion(4,27,0,7, -1, 0, true);  // 19,16

            //sleep(100);

            loopyTime[loopyTimeCounter]=loopElapsedTime.milliseconds();
            loopyTimeCounter++;
            if (loopyTimeCounter >= loopySample) loopyTimeCounter = 0;
            double loopyTimeAverage = 0;
            for(int i=0; i<loopySample; i++) loopyTimeAverage+=loopyTime[i];
            loopyTimeAverage /= loopySample;

            telemetry.addData("Heading", "%.1f", robot.returnImuHeading());
            telemetry.addData("LoopTime(ms)","%.1f",loopElapsedTime.milliseconds());
            telemetry.addData("LoopTimeAvg10(ms)","%.1f",loopyTimeAverage);
            telemetry.addData("LoopSpeed(lps)","%.1f",1/(loopElapsedTime.milliseconds()/1000));
            loopElapsedTime.reset();
            telemetry.update();

        }
    }
}