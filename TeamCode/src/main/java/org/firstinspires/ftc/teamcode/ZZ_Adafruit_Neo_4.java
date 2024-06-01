package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Common.ButtonMgr;
import org.firstinspires.ftc.teamcode.robot.Common.NeoMatrix;
import org.firstinspires.ftc.teamcode.robot.LegacyBots.Robot;

@TeleOp (name="AA_Adafruit_Neo_4", group="Test")
//@Disabled
public class ZZ_Adafruit_Neo_4 extends LinearOpMode {

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
        neo = new NeoMatrix(this, "neo", 8,32);

        robot.init();
        neo.initialize();
        neo.setUpdateLimit(1);
        neo.setPreventTearing(true);
        neo.setDimmingValue(192);

        //sleep(250);

        neo.drawLine(0,0,31,0, Color.rgb(10,10,10));
        neo.drawLine(0,1,31,1, Color.rgb(20,0,0));
        neo.drawLine(0,2,31,2, Color.rgb(15,5,0));
        neo.drawLine(0,3,31,3, Color.rgb(10,10,0));
        neo.drawLine(0,4,31,4, Color.rgb(0,20,0));
        neo.drawLine(0,5,31,5, Color.rgb(0,0,20));
        neo.drawLine(0,6,31,6, Color.rgb(10,0,10));
        neo.drawLine(0,7,31,7, Color.rgb(10,10,10));
        neo.forceUpdateMatrix();

        while (!isStarted()) {
            telemetry.addData(">", "Waiting to start...");
            telemetry.update();
            sleep(100);
        }

        ElapsedTime loopElapsedTime = new ElapsedTime();
        lastAnimUpdate = System.currentTimeMillis();

        neo.clearCols(0,31);
        neo.drawRectangle(0,31,0,7,Color.rgb(10,0,0));
        textMatrix = neo.buildPixelMapFromString("  FTC Legal I2C LED Controller: Adafruit NeoDriver 5766      ", neo.compactFull, Color.rgb(20,20,20), Color.rgb(0,0,0));
        textMatrix1 = neo.buildPixelMapFromString("  FTC Legal", neo.compactFull, Color.rgb(0,5,0), Color.rgb(0,0,0));
        textMatrix = neo.underlayPixelMap(textMatrix1,textMatrix,1);
//        textMatrix = neo.shiftPixelMap(textMatrix,0,1, true);
//        textMatrix1 = neo.buildPixelMapFromString("SENSE", neo.littleLettersSq, Color.rgb(20,0,0), 0);
//        textMatrix = neo.appendPixelMap(textMatrix, textMatrix1);
//        textMatrix1 = neo.buildPixelMapFromString("+", neo.littleLettersSq, Color.rgb(10,10,10), 0);
//        textMatrix1 = neo.shiftPixelMap(textMatrix1,0,1, true);
//        textMatrix = neo.appendPixelMap(textMatrix, textMatrix1);
//        textMatrix1 = neo.buildPixelMapFromString("SOUND ", neo.littleLettersSq, Color.rgb(0,20,0), 0);
//        textMatrix1 = neo.shiftPixelMap(textMatrix1,0,3, true);
//        textMatrix = neo.appendPixelMap(textMatrix, textMatrix1);
//        textMatrix1 = neo.buildPixelMapFromString("ROBOTICS", neo.littleLettersSq, Color.rgb(0,0,20), 0);
//        textMatrix1 = neo.shiftPixelMap(textMatrix1,0,2, true);
//        textMatrix = neo.appendPixelMap(textMatrix, textMatrix1);

        neo.drawRectangle(0,31,0,7,Color.rgb(10,0,0),true,Color.rgb(0,0,0));
        neo.applyPixelMapToBuffer(textMatrix,0,31, 0, false);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.loop();  // this will take care of clearing out the bulk reads
            buttonMgr.updateAll();
            neo.runLoop();

            if (System.currentTimeMillis() >= lastAnimUpdate + animUpdateInterval) {
                lastAnimUpdate = System.currentTimeMillis();
                //neo.scrollRegion(4, 27, 0, 7, -1, 0, true);  // 19,16
                textMatrix = neo.shiftPixelMap(textMatrix,-1,0,true);
                //textMatrix = neo.shiftPixelMap(textMatrix,-1,0,true);
                if (++updateCount > 4) {
                    updateCount = 0;
                    //textMatrix = neo.shiftPixelMap(textMatrix,0,-1,true);
                }
                neo.drawRectangle(0,31,0,7,Color.rgb(10,0,0),true,Color.rgb(0,0,0));
                neo.applyPixelMapToBuffer(textMatrix,1,30, 0, false);

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