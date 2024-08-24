package org.firstinspires.ftc.teamcode.robot.GoCanum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Common.ButtonMgr;
import org.firstinspires.ftc.teamcode.robot.Common.Parts;
import org.firstinspires.ftc.teamcode.robot.Common.Slamra;

public class PartsGC extends Parts {
    public PartsGC(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void setup(){
        // We do this after the construct because we may want to adjust some settings before creating all the sub-parts
        if (isSetup) {
            //throw new RuntimeException("Parts can only be setup once");
            return;
        }
        isSetup = true;
        robot = new RobotGC(this);
        buttonMgr = new ButtonMgr(opMode);
        sensors = new SensorsGC(this);
//      controls = new Controls_2(this);
        controls = new ControlsGC(this);
        drivetrain = new DrivetrainGC(this);

        odometry = new OdometryGC(this);
        odometry.odoFieldStart = fieldStartPosition;//.clone();
        odometry.odoRobotOffset = odoRobotOffset;//.clone();

        navigator = new NavigatorGC(this);

        if (useSlamra) {
            slamra = new Slamra(this);
            slamra.slamraFieldStart = fieldStartPosition;//.clone();
            slamra.slamraRobotOffset = slamraRobotOffset;//.clone();
        }

//        switch (rType) {
//            case GOCANUM:
//                break;
//            case ANDYMARK:
//                break;
//            case GENERIC:
//                break;
//            default:
//        }
    }

    @Override
    public void preInit() {
        robot.initialize();
        sensors.initialize();
        if (useSlamra) slamra.initialize();
    }

    @Override
    public void preRun() {
        drivetrain.initialize();
        odometry.initialize();
        navigator.initialize();

        odometry.runLoop();  // get some things squared away before the real program runs
        navigator.runLoop();
        if (useSlamra) slamra.preRun();
    }

    @Override
    public void initLoop() {
        buttonMgr.runLoop();
        if (useSlamra) slamra.initLoop();
    }

    @Override
    public void runLoop() {
        robot.runLoop();
        sensors.runLoop();
        buttonMgr.runLoop();
        odometry.runLoop();
        if (useSlamra) slamra.runLoop();
        controls.runLoop();
        navigator.runLoop();
        drivetrain.runLoop();
    }

    @Override
    public void stop() {
        if (useSlamra) slamra.stop();
    }

}
