package org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.Shooter;

class Shoot1 {

    public static int stateShoot1Step = 0;

    public static void stateMachineShoot1() {
        if (stateShoot1Step == -9) {
            DSShooter.spinnerOff();
            DSShooter.retractPusher();
            // do we want to close the gate?  Ring might be in there...
            stateShoot1Step = -1;
        }
        if (stateShoot1Step < 1 || stateShoot1Step > 999) return;  // not running

        if (stateShoot1Step == 1) {                         // cancel other state machines if needed
            if (Shoot3.stateShoot3Step > 0 && Shoot3.stateShoot3Step <999) {
                DSShooter.cancelTimer = System.currentTimeMillis() + 1000;
                Shoot3.stateShoot3Step = -9;
            }
            if (System.currentTimeMillis() >= DSShooter.cancelTimer) stateShoot1Step++;
        }
        if (stateShoot1Step == 2) {                 // open gate, start spinner
            Push.statePushStep = -9;   // cancel any ongoing pusher movement
            DSShooter.openGate();
            DSShooter.spinnerOn();
            DSShooter.retractPusher();
            stateShoot1Step++;
        }
        if (stateShoot1Step == 3) {                 // wait for gate up, spinner at rpm
            if (DSShooter.isGateOpen() && DSShooter.isPusherRetracted() && DSShooter.isSpinnerInTolerance()) {
                stateShoot1Step++;
                Push.statePushStep = 1;    // start the pusher state machine
            }
        }
        if (stateShoot1Step == 4) {                 // wait for pusher machine to complete
            if (Push.statePushStep <= 0) stateShoot1Step = -9;   //cancel if problem
            if (Push.statePushStep == 1000) stateShoot1Step = 1000;
        }
    }
}
