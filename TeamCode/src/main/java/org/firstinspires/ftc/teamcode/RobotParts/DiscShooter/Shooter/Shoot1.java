package org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.Shooter;

class Shoot1 {

    private static int state = 0;
    private static boolean complete = false;
    private static long cancelTimer;
    private static final long timeLimit = 5000;


    //----State Machine Start-----
    public static void stateMachine() {
        if (complete) state = 0;
        if (state < 1) return;  // not running
        if  (System.currentTimeMillis() >= cancelTimer) stop();

        if (state == 1) {                         // cancel other state machines if needed
            if (Shoot3.isRunning()) {
                DSShooter.cancelTimer = System.currentTimeMillis() + 1000;
                Shoot3.softStop();
            }
            if (System.currentTimeMillis() >= DSShooter.cancelTimer) state++;
        }
        if (state == 2) {                 // open gate, start spinner
            Pusher.stop();   // cancel any ongoing pusher movement
            DSShooter.armShooter();
//            DSShooter.openGate();
//            DSShooter.spinnerOn();
//            DSShooter.retractPusher();
            state++;
        }
        if (state == 3) {                 // wait for gate up, spinner at rpm
            if (DSShooter.isGateOpen() && DSShooter.isPusherRetracted() && DSShooter.isSpinnerInTolerance()) {
                Pusher.start();    // start the pusher state machine
                state++;
            }
        }
        if (state == 4) {                 // wait for pusher machine to complete
            if (Pusher.isComplete()) complete = true;
            else if (!Pusher.isRunning()) stop();   //cancel if problem
        }
    }
    //----State Machine End-----

    public static void start() {
        complete = false;
        state = 1;
        cancelTimer = System.currentTimeMillis() + timeLimit;
    }

    public static void stop() {
        DSShooter.spinnerOff();
        DSShooter.retractPusher();
        DSShooter.isArmed = false;
        // do we want to close the gate?  Ring might be in there...
        state = -1;
    }

    public static void softStop() {
        // don't spin down!
        DSShooter.retractPusher();
        state = -1;
    }

    public static boolean isRunning() {
        return (state>0);
    }

    public boolean isComplete() {
        return complete;
    }

    public static int getState() {
        return state;
    }
}
