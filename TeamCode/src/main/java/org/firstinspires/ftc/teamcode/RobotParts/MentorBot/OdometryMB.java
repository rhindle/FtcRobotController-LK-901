package org.firstinspires.ftc.teamcode.RobotParts.MentorBot;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Odometry;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;

public class OdometryMB extends Odometry {

    public OdometryMB(Parts parts) {
        super(parts);
    }

    @Override
    public void configureEncoders() {
        /* this is separated so it can be overridden */
        odoY = parts.robot.motor0B;
        odoXR = parts.robot.motor1B;
        odoXL = parts.robot.motor2B;
        odoY.setDirection(DcMotorEx.Direction.FORWARD);
        odoXL.setDirection(DcMotorEx.Direction.REVERSE);
        odoXR.setDirection(DcMotorEx.Direction.REVERSE);
        odoY.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        odoXL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        odoXR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
      /* dir should be -1 or 1 and is to account for the port being used elsewhere
         and needing to be set in direction opposite to what odometry needs */
        odoYdir = 1;
        odoXRdir = 1;
        odoXLdir = 1;
    }

}
