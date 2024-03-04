package org.firstinspires.ftc.teamcode.robot.GoCanum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Common.ButtonMgr;
import org.firstinspires.ftc.teamcode.robot.Common.Parts;
import org.firstinspires.ftc.teamcode.robot.Common.Slamra;

public class PartsGC extends Parts {
    public PartsGC(LinearOpMode opMode, robotType rType) {
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
        robot.init();
        sensors.init();
        if (useSlamra) slamra.init();
    }

    @Override
    public void preRun() {
        drivetrain.init();
        odometry.init();
        navigator.init();

        odometry.loop();  // get some things squared away before the real program runs
        navigator.loop();
        if (useSlamra) slamra.onStart();
    }

    @Override
    public void loop() {
        robot.loop();
        sensors.loop();
        buttonMgr.loop();
        odometry.loop();
        if (useSlamra) slamra.loop();
        controls.loop();
        navigator.loop();
    }

    @Override
    public void stop() {
        if (useSlamra) slamra.onStop();
    }

}
