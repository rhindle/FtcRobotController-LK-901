package org.firstinspires.ftc.teamcode.robot.DiscShooter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Common.ButtonMgr;
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
        apriltag = new AprilTag(this);

//        odometry = new OdometryDS(this);
//        odometry.odoFieldStart = fieldStartPosition;
//        odometry.odoRobotOffset = odoRobotOffset;

        navigator = new NavigatorDS(this);

        if (useSlamra) {
            slamra = new Slamra(this);
            slamra.slamraFieldStart = fieldStartPosition;
            slamra.slamraRobotOffset = slamraRobotOffset;
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
//        sensors.init();
        if (useSlamra) slamra.init();
        apriltag.init();
    }

    @Override
    public void preRun() {
        drivetrain.init();
        if (useODO) odometry.init();
        navigator.init();

        if (useODO) odometry.loop();  // get some things squared away before the real program runs
        navigator.loop();
        if (useSlamra) slamra.onStart();
    }

    @Override
    public void initLoop() {
        buttonMgr.loop();
        if (useSlamra) slamra.initLoop();
        apriltag.initLoop();
    }

    @Override
    public void loop() {
        robot.loop();
//        sensors.loop();
        buttonMgr.loop();
        if (useODO) odometry.loop();
        if (useSlamra) slamra.loop();
        apriltag.loop();
        controls.loop();
        navigator.loop();

        //experiment follows, to be moved elsewhere eventually
        Position roboTag = apriltag.getRobotTagPosition();
        if (roboTag!=null) slamra.setupFieldOffset(roboTag);
    }

    @Override
    public void stop() {
        if (useSlamra) slamra.onStop();
        apriltag.stop();
    }

}
