package org.firstinspires.ftc.teamcode.robot.DiscShooter;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Common.ButtonMgr;
import org.firstinspires.ftc.teamcode.robot.Common.NeoMatrix;
import org.firstinspires.ftc.teamcode.robot.Common.Parts;
import org.firstinspires.ftc.teamcode.robot.Common.Slamra;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.Position;

public class PartsDS extends Parts {
    public PartsDS(LinearOpMode opMode, robotType rType) {
        super(opMode, rType);
    }

    @Override
    public void setup(){
        // We do this after the construct because we may want to adjust some settings before creating all the sub-parts
        if (isSetup) {
            //throw new RuntimeException("Parts can only be setup once");
            return;
        }
        isSetup = true;
        robot = new RobotDS(this);
        buttonMgr = new ButtonMgr(opMode);
//        sensors = new SensorsDS(this);
//      controls = new Controls_2(this);
        controls = new ControlsDS(this);
        drivetrain = new DrivetrainDS(this);
        if (useAprilTag) apriltag = new AprilTag(this);

//        odometry = new OdometryDS(this);
//        odometry.odoFieldStart = fieldStartPosition;
//        odometry.odoRobotOffset = odoRobotOffset;

        navigator = new NavigatorDS(this);

        if (useSlamra) {
            slamra = new Slamra(this);
            slamra.slamraFieldStart = fieldStartPosition;
            slamra.slamraRobotOffset = slamraRobotOffset;
        }

        if (useNeoMatrix) neo = new NeoMatrix(opMode, "neo", 8, 16);

        switch (rType) {
            case GOCANUM:
                break;
            case ANDYMARK:
                break;
            case GENERIC:
                break;
            default:
        }
    }

    @Override
    public void preInit() {
        robot.initialize();
//        sensors.init();
        if (useSlamra) slamra.initialize();
        if (useAprilTag) apriltag.initialize();

        if (useNeoMatrix) {
            neo.initialize();
            //neo.setUpdateLimit(1);
            neo.setUpdateLimit(0);
            neo.setPreventTearing(true);
            neo.setDimmingValue(192);
            neo.drawRectangle(0, 7, 0, 7, Color.rgb(1, 1, 0));
            textMatrix = neo.buildPixelMapFromString("abcd", misc, Color.rgb(1,1,0), Color.rgb(0,0,0));
        }


    }

    @Override
    public void preRun() {
        drivetrain.initialize();
        if (useODO) odometry.initialize();
        navigator.initialize();

        if (useODO) odometry.runLoop();  // get some things squared away before the real program runs
        navigator.runLoop();
        if (useSlamra) slamra.preRun();
        if (useNeoMatrix) {
            neo.drawRectangle(0,7,0,7, Color.rgb(0,2,0));
            neo.setUpdateLimit(1);
        }
    }

    @Override
    public void initLoop() {
        buttonMgr.runLoop();
        if (useSlamra) slamra.initLoop();
        if (useAprilTag) apriltag.initLoop();
        if (useNeoMatrix) {
            neo.applyPixelMapToBuffer(textMatrix,0,7, 0, true);
            neo.applyPixelMapToBuffer(neo.reversePixelMap(textMatrix),8,15, 0, true);
//            neo.applyPixelMapToBuffer(neo.reversePixelMap(textMatrix),8,15, 0, true);
            textMatrix = neo.shiftPixelMap(textMatrix,-8,0,true);
            neo.runLoop();
        }
    }

    @Override
    public void runLoop() {
        robot.runLoop();
//        sensors.loop();
        buttonMgr.runLoop();
        if (useODO) odometry.runLoop();
        if (useSlamra) slamra.runLoop();
        if (useAprilTag) apriltag.runLoop();
        controls.runLoop();
        navigator.runLoop();
        if (useNeoMatrix) neo.runLoop();

        //experiment follows, to be moved elsewhere eventually
        if (useAprilTag) {
            Position roboTag = apriltag.getRobotTagPosition();
            if (roboTag != null) {
                slamra.setupFieldOffset(roboTag);
                navigator.deltaHeading = robot.returnImuHeading() - roboTag.R;
            }
        }
    }

    @Override
    public void stop() {
        if (useSlamra) slamra.stop();
        if (useAprilTag) apriltag.stop();
    }

    int[][] textMatrix;

    public final char[][] misc = {
    {'a', 17, 128, 0, 0, 1, 128, 0, 34},
    {'b', 34, 0, 128, 1, 0, 0, 128, 17},
    {'c', 68, 0, 1, 128, 0, 0, 1, 136},
    {'d', 136, 1, 0, 0, 128, 1, 0, 68},
    {'e', 238, 1, 129, 129, 128, 1, 129, 221},
    {'f', 221, 129, 1, 128, 129, 129, 1, 238},
    {'g', 187, 129, 128, 1, 129, 129, 128, 119},
    {'h', 119, 128, 129, 129, 1, 128, 129, 187} };

}
