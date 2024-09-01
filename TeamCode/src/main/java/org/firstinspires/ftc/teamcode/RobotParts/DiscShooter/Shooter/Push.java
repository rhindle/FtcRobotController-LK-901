package org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.Shooter;

public class Push {

    public static int statePushStep = 0;

    public static void stateMachineAutoPush() {
        if (statePushStep == -9) {
            DSShooter.retractPusher();
            statePushStep = -1;
        }
        if (statePushStep < 1 || statePushStep > 999) return;  // not running

        if (statePushStep == 1) {                 // extend pusher
            DSShooter.extendPusher();
            statePushStep++;
        }
        if (statePushStep == 2) {                 // wait for complete, retract
            if (DSShooter.isPusherExtended()) {
                DSShooter.retractPusher();
                statePushStep++;
            }
        }
        if (statePushStep == 3) {                // wait for complete
            if (DSShooter.isPusherRetracted()) {
                statePushStep=1000;
            }
        }
    }
}
