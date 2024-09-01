package org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.Shooter;

class Shoot3 {

    public static int stateShoot3Step = 0;
    static int cycleCount = 0;

    public static void stateMachineShoot3() {
        if (stateShoot3Step == -9) {
            DSShooter.spinnerOff();
            DSShooter.retractPusher();
            // do we want to close the gate?  Ring might be in there...
            stateShoot3Step = -1;
        }
        if (stateShoot3Step < 1 || stateShoot3Step > 999) return;  // not running

        if (stateShoot3Step == 1) {                                       // cancel other state machines if needed
            if (Shoot1.stateShoot1Step > 0 && Shoot1.stateShoot1Step <999) {
                DSShooter.cancelTimer = System.currentTimeMillis() + 1000;
                Shoot1.stateShoot1Step = -9;
            }
            if (System.currentTimeMillis() >= DSShooter.cancelTimer) stateShoot3Step++;
        }
        if (stateShoot3Step == 2) {                 // open gate, start spinner
            Push.statePushStep = -9;   // cancel any ongoing pusher movement
            cycleCount = 0;
            DSShooter.openGate();
            DSShooter.spinnerOn();
            DSShooter.retractPusher();
            stateShoot3Step++;
        }
        if (stateShoot3Step == 3) {                 // wait for gate up, spinner at rpm
            if (DSShooter.isGateOpen() && DSShooter.isPusherRetracted() && DSShooter.isSpinnerInTolerance()) {
                stateShoot3Step++;
                Push.statePushStep = 1;    // start the pusher state machine
            }
        }
        if (stateShoot3Step == 4) {                 // wait for pusher machine to complete
            if (Push.statePushStep <= 0) stateShoot3Step = -9;   //cancel if problem
            if (Push.statePushStep == 1000) {
                cycleCount++;
                Push.statePushStep = 1;      //restart pusher
            }
            if (cycleCount == DSShooter.pusherAutoCycles) stateShoot3Step = 1000;
        }
    }
}
