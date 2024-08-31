package org.firstinspires.ftc.teamcode.robot.Common;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.robot.Common.Tools.DataTypes.DrivePowers;
import org.firstinspires.ftc.teamcode.robot.Common.TelemetryMgr.Category;

public class Drivetrain {

    /* Public OpMode members. */
    public Parts parts;

    private DcMotorEx motorLF, motorRF, motorLR, motorRR;
    DrivePowers drivePowers, drivePowersLast;
    public boolean minimizeCycleTime = true;   // skip small power changes to improve cycle time (each motor power transaction degrades cycle time)
    public double ignoreDiff = .05;            // absolute power difference to ignore

    /* Constructor */
    public Drivetrain(Parts parts) {
        construct(parts);
    }

    void construct(Parts parts) {
        this.parts = parts;
    }

    public void initialize() {
        initMotors();
        drivePowers = new DrivePowers();
        drivePowersLast = new DrivePowers();
    }

    public void preInit() {
    }

    public void initLoop() {
    }

    public void preRun() {
    }

    public void runLoop() {
        applyDrivePowers();
    }

    public void stop() {
        stopDriveMotors(true);
    }

    public void applyDrivePowers() {
        TelemetryMgr.message(Category.DRIVETRAIN, "dt-raw", drivePowers.toString(2));
        if (minimizeCycleTime) drivePowers = adjustPowers(drivePowers, drivePowersLast);
        TelemetryMgr.message(Category.DRIVETRAIN, "dt-adj", drivePowers.toString(2));
        motorLF.setPower(drivePowers.v0);
        motorRF.setPower(drivePowers.v1);
        motorLR.setPower(drivePowers.v2);
        motorRR.setPower(drivePowers.v3);
        drivePowersLast = drivePowers.clone();
    }

    public DrivePowers getDrivePowers() {
        return drivePowers;
    }

    // By design, drive powers variables can be changed multiple times (e.g., by AutoDrive and UserDrive),
    // but are only applied once per runLoop()
    public void setDrivePowers(DrivePowers drivePowers) {
        this.drivePowers = drivePowers;
    }

    public void setDrivePowers(double[] mPow) {
        setDrivePowers(new DrivePowers(mPow[0], mPow[1], mPow[2], mPow[3]));
    }

    public void setDrivePowers(double p0, double p1, double p2, double p3) {
        setDrivePowers(new DrivePowers(p0, p1, p2, p3));
    }

    public void stopDriveMotors() {
        setDrivePowers(0, 0, 0, 0);
    }

    public void stopDriveMotors(boolean immediate) {
        setDrivePowers(0, 0, 0, 0);
        if (immediate) applyDrivePowers();
    }

    private DrivePowers adjustPowers(DrivePowers powerRequested, DrivePowers powerLast) {
        double[] newPow = {powerRequested.v0, powerRequested.v1, powerRequested.v2, powerRequested.v3};
        double[] oldPow = {powerLast.v0, powerLast.v1, powerLast.v2, powerLast.v3};
        double[] adjPow = new double[4];
        // only change motor power if it has changed more than absEff or is 0.
        for (int i = 0; i < 4; i++) {
            if (newPow[i] == 0) adjPow[i] = 0;
            else if (Math.abs(newPow[i] - oldPow[i]) < ignoreDiff) adjPow[i] = oldPow[i];
            else adjPow[i] = newPow[i];
        }
        return new DrivePowers(adjPow);
    }

    public void initMotors() {
        motorLF = parts.robot.motor0;
        motorRF = parts.robot.motor1;
        motorLR = parts.robot.motor2;
        motorRR = parts.robot.motor3;

        if (!parts.reverseDrive) {
            motorLF.setDirection(DcMotorEx.Direction.REVERSE);
            motorLR.setDirection(DcMotorEx.Direction.REVERSE);
            motorRF.setDirection(DcMotorEx.Direction.FORWARD);
            motorRR.setDirection(DcMotorEx.Direction.FORWARD);
        } else {
            motorLF.setDirection(DcMotorEx.Direction.FORWARD);
            motorLR.setDirection(DcMotorEx.Direction.FORWARD);
            motorRF.setDirection(DcMotorEx.Direction.REVERSE);
            motorRR.setDirection(DcMotorEx.Direction.REVERSE);
        }

        if (parts.useDrivetrainEncoders) {
            motorLF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motorLR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motorRF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motorRR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            motorLF.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            motorLR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            motorRF.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            motorRR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        motorLF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    // This method removes the normal restriction on RPM when using RUN_USING_ENCODER
    public void removeEncoderSpeedLimits() {
        DcMotorEx[] motors = {motorLF, motorRF, motorLR, motorRR};
        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }
    }
}
