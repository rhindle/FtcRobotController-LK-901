package org.firstinspires.ftc.teamcode.robot.Common;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.Common.Tools.DataTypes.DrivePowers;

public class Drivetrain {

    Parts parts;

    private DcMotorEx motorLF, motorRF, motorLR, motorRR;
    double[] lastPow = new double[4];
    public boolean efficient = true;   // skip minor updates to improve cycle time (each motor power transaction degrades cycle time
    //double relEff = .01;  // relative difference to ignore 1%
    double absEff = .05;  // absolute difference to ignore

    /* Constructor */
    public Drivetrain(Parts parts){
        construct(parts);
    }

    void construct(Parts parts){
        this.parts = parts;
    }

    public void initialize() {
        motorLF = parts.robot.motor0;
        motorRF = parts.robot.motor1;
        motorLR = parts.robot.motor2;
        motorRR = parts.robot.motor3;
        initMotors();
    }

    public void setDrivePowers (DrivePowers dPow) {
        setDrivePowers(new double[] {dPow.v0, dPow.v1, dPow.v2, dPow.v3});
    }

    public void setDrivePowers (double[] mPow) {
        if (efficient) mPow = adjustPower(mPow, lastPow);
        motorLF.setPower(mPow[0]);
        motorRF.setPower(mPow[1]);
        motorLR.setPower(mPow[2]);
        motorRR.setPower(mPow[3]);
        System.arraycopy(mPow, 0, lastPow, 0, 4);
    }

    public void stopDriveMotors() {
        motorLF.setPower(0);
        motorRF.setPower(0);
        motorLR.setPower(0);
        motorRR.setPower(0);
        lastPow = new double[] {0, 0, 0, 0};
    }

    private double[] adjustPower(double[] newPow, double[] oldPow) {
        double[] adjPow = new double[4];
        // only change motor power if it has changed more than absEff or is 0.
        for (int i=0; i<4; i++) {
            if (newPow[i] == 0) adjPow[i] = 0;
            else if (Math.abs(newPow[i]-oldPow[i]) < absEff) adjPow[i] = oldPow[i];
            else adjPow[i] = newPow[i];
        }
        return adjPow;
    }

    public void initMotors () {
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

        if (parts.useDriveEncoders) {
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
