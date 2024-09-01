package org.firstinspires.ftc.teamcode.RobotParts.DiscShooter;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.ImuMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.NeoMatrix;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.PositionMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Slamra;
import org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.Shooter.DSShooter;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.NavigationTarget;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;

public class PartsDS extends Parts {
    public PartsDS(LinearOpMode opMode) {
        super(opMode);
    }

    boolean firstLock = true;

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
        controls = new ControlsDS(this);
        drivetrain = new DrivetrainDS(this);

        imuMgr = new ImuMgr(this);
        positionMgr = new PositionMgr(this);
        autoDrive = new AutoDriveDS(this);
        userDrive = new UserDriveDS(this);
        dsLed = new DSLed(this);
        dsShooter = new DSShooter(this);
        dsSpeedControl = new DSSpeedControl(this);
        dsMisc = new DSMisc(this);

        if (useAprilTag) dsApriltag = new DSAprilTag(this);
        if (useODO) {
            odometry = new OdometryDS(this);
            odometry.odoFieldStart = fieldStartPosition;
            odometry.odoRobotOffset = odoRobotOffset;
        }
        if (useSlamra) {
            slamra = new Slamra(this);
            slamra.slamraFieldStart = fieldStartPosition;
            slamra.slamraRobotOffset = slamraRobotOffset;
        }

        if (useNeoMatrix) neo = new NeoMatrix(opMode, "neo", 8, 16);
    }

    @Override
    public void preInit() {
        robot.initialize();
        imuMgr.initialize();
        positionMgr.initialize();
        dsShooter.initialize();
        if (useSlamra) slamra.initialize();
        if (useAprilTag) dsApriltag.initialize();
        if (useNeoMatrix) dsLed.initialize();
    }

    @Override
    public void initLoop() {
        imuMgr.runLoop();
        buttonMgr.runLoop();
        if (useSlamra) slamra.initLoop();
        if (useAprilTag) dsApriltag.initLoop();
        positionMgr.initLoop();
        if (useNeoMatrix) dsLed.initLoop();
        dsShooter.initLoop();
    }

    @Override
    public void preRun() {
        drivetrain.initialize();
        imuMgr.runLoop();
        if (useODO) odometry.initialize();
        userDrive.initialize();
        autoDrive.initialize();
        autoDrive.setNavTarget(new NavigationTarget(new Position(-20,0,0), dsMisc.toleranceHigh));

        if (useODO) odometry.runLoop();  // get some things squared away before the regular runLoops start
        autoDrive.runLoop();

        if (useSlamra) slamra.preRun();
        if (useNeoMatrix) dsLed.preRun();
        dsShooter.preRun();
    }

    @Override
    public void runLoop() {
        robot.runLoop();
        imuMgr.runLoop();
        buttonMgr.runLoop();
        if (useODO) odometry.runLoop();
        if (useSlamra) slamra.runLoop();
        if (useAprilTag) dsApriltag.runLoop();
        positionMgr.runLoop();
        controls.runLoop();
        userDrive.runLoop();
        dsSpeedControl.runLoop();
        autoDrive.runLoop();
        drivetrain.runLoop();
        dsShooter.runLoop();
        if (useNeoMatrix) dsLed.runLoop();

        //experiment follows, to be moved elsewhere eventually  //todo:move this (including firstlock variable)
        if (useAprilTag) {
            Position roboTagPosition = dsApriltag.getTagRobotPosition();
            if (roboTagPosition != null) {
                if (useSlamra) slamra.setupFieldOffset(roboTagPosition);
                if (useODO) odometry.setupFieldOffset(roboTagPosition);
                imuMgr.setupFieldOffset(roboTagPosition);
                //  autoDrive.modifyHeading = robot.returnImuHeading() - roboTagPosition.R; // saving for reference
            }
            if (dsApriltag.tagRobotPosition!=null){
                neo.drawRectangle(3,4,3,4, Color.rgb(0,4,1));
                if (firstLock && !userDrive.isDriving) {   //todo:make this better
                    firstLock = false;
                    autoDrive.setNavTarget(new NavigationTarget(new Position(-20,0,0), dsMisc.toleranceHigh));
                }
            } else if (dsApriltag.instantTagRobotPosition!=null) {
                neo.drawRectangle(3,4,3,4, Color.rgb(2,1,0));
            } else {
                neo.drawRectangle(3,4,3,4, Color.rgb(2,0,0));
            }
        }

        if (positionMgr.robotPosition!=null) {
            if (dsApriltag.strongLocked) {
                neo.drawRectangle(2, 5, 2, 5, Color.rgb(0, 2, 0));
            } else {
                neo.drawRectangle(2, 5, 2, 5, Color.rgb(2, 1, 0));
            }
        } else {
            neo.drawRectangle(2,5,2,5, Color.rgb(2,0,0));
            dsApriltag.strongLocked=false;  // if all is lost, allow a weak lock again
        }
    }

    @Override
    public void stop() {
        if (useSlamra) slamra.stop();
        if (useAprilTag) dsApriltag.stop();
        dsShooter.stop();
        drivetrain.stop();
    }

}
