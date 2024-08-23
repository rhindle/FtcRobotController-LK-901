package org.firstinspires.ftc.teamcode.robot.DiscShooter;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.teamcode.robot.Common.Parts;
import org.firstinspires.ftc.teamcode.robot.Common.Robot;

public class RobotDS extends Robot {
    public RobotDS(Parts parts) {
        super(parts);
    }

    @Override
    public void settingOptions() {
        hubOrientation = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
    }

    @Override
    public void initOptions() {
    }
}
