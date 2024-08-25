package org.firstinspires.ftc.teamcode.robot.Common;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.Common.Tools.DataTypes.DrivePowers;
import org.firstinspires.ftc.teamcode.robot.Common.TelemetryMgr.Category;

public class Drivetrain {

    /* Public OpMode members. */
    public Parts parts;

    private DcMotorEx motorLF, motorRF, motorLR, motorRR;
    DrivePowers drivePowers, drivePowersLast;
//    double[] lastPow = new double[4];
    public boolean efficient = true;   // skip minor updates to improve cycle time (each motor power transaction degrades cycle time
    double absEff = .05;  // absolute difference to ignore

    /* Constructor */
    public Drivetrain(Parts parts){
        construct(parts);
    }

    void construct(Parts parts){
        this.parts = parts;
    }

    public void initialize() {
        initMotors();
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

        if (efficient) drivePowers = adjustPowers(drivePowers, drivePowersLast);
        motorLF.setPower(drivePowers.v0);
        motorRF.setPower(drivePowers.v1);
        motorLR.setPower(drivePowers.v2);
        motorRR.setPower(drivePowers.v3);
        drivePowersLast = drivePowers.clone();
    }

    public void setDrivePowers (DrivePowers drivePowers) {
        this.drivePowers = drivePowers;
    }

    public void setDrivePowers (double[] mPow) {
        setDrivePowers(new DrivePowers(mPow[0], mPow[1], mPow[2], mPow[3]));
//        if (efficient) mPow = adjustPower(mPow, lastPow);
//        motorLF.setPower(mPow[0]);
//        motorRF.setPower(mPow[1]);
//        motorLR.setPower(mPow[2]);
//        motorRR.setPower(mPow[3]);
//        System.arraycopy(mPow, 0, lastPow, 0, 4);
    }

    public void setDrivePowers (double p0, double p1, double p2, double p3) {
        setDrivePowers(new DrivePowers(p0, p1, p2, p3));
    }

    public void stopDriveMotors() {
        setDrivePowers(0, 0, 0, 0);
//        motorLF.setPower(0);
//        motorRF.setPower(0);
//        motorLR.setPower(0);
//        motorRR.setPower(0);
//        lastPow = new double[] {0, 0, 0, 0};
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
        for (int i=0; i<4; i++) {
            if (newPow[i] == 0) adjPow[i] = 0;
            else if (Math.abs(newPow[i]-oldPow[i]) < absEff) adjPow[i] = oldPow[i];
            else adjPow[i] = newPow[i];
        }
        return new DrivePowers(adjPow);
    }

//    private double[] adjustPower(double[] newPow, double[] oldPow) {
//        double[] adjPow = new double[4];
//        // only change motor power if it has changed more than absEff or is 0.
//        for (int i=0; i<4; i++) {
//            if (newPow[i] == 0) adjPow[i] = 0;
//            else if (Math.abs(newPow[i]-oldPow[i]) < absEff) adjPow[i] = oldPow[i];
//            else adjPow[i] = newPow[i];
//        }
//        return adjPow;
//    }

    public void initMotors () {
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
}
