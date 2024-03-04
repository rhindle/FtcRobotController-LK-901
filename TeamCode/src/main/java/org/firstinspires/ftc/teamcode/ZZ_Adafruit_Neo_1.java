package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Common.ButtonMgr;
import org.firstinspires.ftc.teamcode.robot.Common.NeoMatrix;
import org.firstinspires.ftc.teamcode.robot.LegacyBots.Robot;

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
    int[][] textMatrix1;
    long lastAnimUpdate;
    int updateCount = 0;
    int animUpdateInterval = 200;  //250

    //AdafruitNeoDriver neo = null;

    @Override
    public void runOpMode() {
        robot = new Robot(this);
        buttonMgr = new ButtonMgr(this);
        neo = new NeoMatrix(this, "neo");

        robot.init();
        neo.init();
        neo.setUpdateLimit(1);
        neo.setPreventTearing(true);
        neo.setDimmingValue(192);

        //sleep(250);

        neo.drawLine(0,0,31,7, Color.rgb(50,0,0));
        neo.drawLine(31,0,0,7, Color.rgb(0,0,50));
        neo.drawLine(0,0,3,7, Color.rgb(0,50,0));
        neo.forceUpdateMatrix();

        while (!isStarted()) {
            telemetry.addData(">", "Waiting to start...");
            telemetry.update();
            sleep(100);
        }

        ElapsedTime loopElapsedTime = new ElapsedTime();
        lastAnimUpdate = System.currentTimeMillis();

//        neo.drawRectangle(0,7,0,7, Color.rgb(5,0,0), true, Color.rgb(0,1,0));
        neo.drawRectangle(0,3,0,3, Color.rgb(0,8,0), true, Color.rgb(0,8,0));
        neo.drawRectangle(0,3,4,7, Color.rgb(6,6,0), true, Color.rgb(6,6,0));
        neo.drawRectangle(28,31,0,3, Color.rgb(4,4,4), true, Color.rgb(4,4,4));
        neo.drawRectangle(28,31,4,7, Color.rgb(0,0,8), true, Color.rgb(0,0,8));
        textMatrix = neo.buildPixelMapFromString("  14273 ", neo.bigNumbers7HT, Color.rgb(20,20,20), 0);
        textMatrix = neo.shiftPixelMap(textMatrix,0,1, true);
        //textMatrix = neo.shiftPixelMap(textMatrix,0,1, true);
        textMatrix1 = neo.buildPixelMapFromString("SENSE", neo.littleLettersSq, Color.rgb(20,0,0), 0);
        textMatrix = neo.appendPixelMap(textMatrix, textMatrix1);
        textMatrix1 = neo.buildPixelMapFromString("+", neo.littleLettersSq, Color.rgb(10,10,10), 0);
        //textMatrix1 = neo.shiftPixelMap(textMatrix1,0,1, true);
        textMatrix1 = neo.shiftPixelMap(textMatrix1,0,1, true);
        textMatrix = neo.appendPixelMap(textMatrix, textMatrix1);
        textMatrix1 = neo.buildPixelMapFromString("SOUND ", neo.littleLettersSq, Color.rgb(0,20,0), 0);
        textMatrix1 = neo.shiftPixelMap(textMatrix1,0,3, true);
//        textMatrix1 = neo.shiftPixelMap(textMatrix1,0,1, true);
//        textMatrix1 = neo.shiftPixelMap(textMatrix1,0,1, true);
        textMatrix = neo.appendPixelMap(textMatrix, textMatrix1);
        textMatrix1 = neo.buildPixelMapFromString("ROBOTICS", neo.littleLettersSq, Color.rgb(0,0,20), 0);
        textMatrix1 = neo.shiftPixelMap(textMatrix1,0,2, true);
//        textMatrix1 = neo.shiftPixelMap(textMatrix1,0,1, true);
        textMatrix = neo.appendPixelMap(textMatrix, textMatrix1);
        neo.applyPixelMapToBuffer(textMatrix,4,27, 0, true);
//        neo.applyPixelMapToBuffer(textMatrix,7,31, 0, true);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.loop();  // this will take care of clearing out the bulk reads
            buttonMgr.updateAll();
            neo.loop();

            if (System.currentTimeMillis() >= lastAnimUpdate + animUpdateInterval) {
                lastAnimUpdate = System.currentTimeMillis();
                //neo.scrollRegion(4, 27, 0, 7, -1, 0, true);  // 19,16
                textMatrix = neo.shiftPixelMap(textMatrix,-1,0,true);
                //textMatrix = neo.shiftPixelMap(textMatrix,-1,0,true);
                if (++updateCount > 4) {
                    updateCount = 0;
                    //textMatrix = neo.shiftPixelMap(textMatrix,0,-1,true);
                }
                neo.applyPixelMapToBuffer(textMatrix,4,27, 0, true);

            }

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