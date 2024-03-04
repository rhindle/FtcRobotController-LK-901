package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Common.ButtonMgr;
import org.firstinspires.ftc.teamcode.robot.Common.NeoMatrix;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp (name="AA_Adafruit_Neo_3", group="Test")
//@Disabled
public class ZZ_Adafruit_Neo_3 extends LinearOpMode {

    Robot robot;
    ButtonMgr buttonMgr;
    NeoMatrix neo;

    int loopySample = 50;
    double[] loopyTime = new double[loopySample];
    int loopyTimeCounter = 0;
    int[][] textMatrix;
    int[][] textMatrix1;
    long lastAnimUpdate;
    int animUpdateInterval = 2500;  //250
    int timeRemaining;
    int timeColor;
    int face = 1;

    @Override
    public void runOpMode() {
        robot = new Robot(this);
        buttonMgr = new ButtonMgr(this);
        neo = new NeoMatrix(this, "neo");

        robot.init();
        neo.init();
        neo.setUpdateLimit(1);

        while (!isStarted()) {
            telemetry.addData(">", "Waiting to start...");
            telemetry.update();
            sleep(100);
        }

        ElapsedTime loopElapsedTime = new ElapsedTime();
        ElapsedTime gameTime = new ElapsedTime();
        lastAnimUpdate = System.currentTimeMillis() - animUpdateInterval;

        textMatrix = neo.buildPixelMapFromString("a", neo.specialChar, Color.rgb(0,4,0), 0);
        neo.applyPixelMapToBuffer(textMatrix,0,99, 0, true);
        textMatrix = neo.buildPixelMapFromString("b", neo.specialChar, Color.rgb(3,3,0), 0);
        neo.applyPixelMapToBuffer(textMatrix,0,99, 0, false);
        textMatrix = neo.buildPixelMapFromString("a", neo.specialChar, Color.rgb(0,0,4), 0);
        neo.applyPixelMapToBuffer(textMatrix,28,99, 0, true);
        textMatrix = neo.buildPixelMapFromString("b", neo.specialChar, Color.rgb(2,2,2), 0);
        neo.applyPixelMapToBuffer(textMatrix,28,99, 0, false);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.loop();  // this will take care of clearing out the bulk reads
            buttonMgr.updateAll();
            //neo.loop();

            if (System.currentTimeMillis() >= lastAnimUpdate + animUpdateInterval) {
                lastAnimUpdate = System.currentTimeMillis();
                textMatrix1 = neo.buildPixelMapFromString(String.valueOf(face), neo.faces, Color.rgb(10,3,0) ,0);
                if (++face>4) face=1;
                neo.applyPixelMapToBuffer(textMatrix1,19,31, 0, true);
            }

            timeRemaining = Math.max(0, 120-(int)gameTime.seconds());
            timeColor = timeRemaining < 31 ? Color.rgb(10,0,0) : timeRemaining < 46 ? Color.rgb(10,5,0) : Color.rgb(0,10,0);
            textMatrix1 = neo.buildPixelMapFromString(String.valueOf(timeRemaining),neo.littleNumbers, timeColor,0);
            textMatrix1 = neo.shiftPixelMap(textMatrix1,0,2, true);
//            textMatrix1 = neo.shiftPixelMap(textMatrix1,0,1, true);
            neo.clearCols(5, 12);
            neo.applyPixelMapToBuffer(textMatrix1,17-textMatrix1.length,16,0, true);

            neo.loop();

            sleep(8);

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