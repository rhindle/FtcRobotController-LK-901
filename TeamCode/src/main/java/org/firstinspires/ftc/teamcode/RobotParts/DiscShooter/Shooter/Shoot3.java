package org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.Shooter;

class Shoot3 {

    private static int state = 0;
    private static int cycleCount = 0;
    private static boolean complete = false;

    //----State Machine Start-----
    public static void stateMachine() {
        if (complete) state = 0;
        if (state < 1) return;  // not running

        if (state == 1) {                                       // cancel other state machines if needed
            if (Shoot1.isRunning()) {
                DSShooter.cancelTimer = System.currentTimeMillis() + 1000;
                Shoot1.stop();
            }
            if (System.currentTimeMillis() >= DSShooter.cancelTimer) state++;
        }
        if (state == 2) {                 // open gate, start spinner
            Pusher.stop();   // cancel any ongoing pusher movement
            cycleCount = 0;
            DSShooter.openGate();
            DSShooter.spinnerOn();
            DSShooter.retractPusher();
            state++;
        }
        if (state == 3) {                 // wait for gate up, spinner at rpm
            if (DSShooter.isGateOpen() && DSShooter.isPusherRetracted() && DSShooter.isSpinnerInTolerance()) {
                cycleCount++;
                Pusher.start();    // start the pusher state machine
                state++;
            }
        }
        if (state == 4) {                 // wait for pusher machine to complete
            if (Pusher.isComplete()) {
                if (cycleCount == DSShooter.pusherAutoCycles) complete = true;
                else if (DSShooter.isSpinnerInTolerance()) {
                    cycleCount++;
                    Pusher.start();      //restart pusher
                }
            }
            else if (!Pusher.isRunning()) stop();   //cancel if problem
//            if (cycleCount == DSShooter.pusherAutoCycles) complete = true;  //note:push is still running at this moment
        }
    }
    //----State Machine End-----

    public static void start() {
        complete = false;
        state = 1;
    }

    public static void stop() {
        DSShooter.spinnerOff();
        DSShooter.retractPusher();
        // do we want to close the gate?  Ring might be in there...
        state = -1;
    }

    public static boolean isRunning() {
        return (state>0);
    }

    public static boolean isComplete() {
        return complete;
    }

    public static int getState() {
        return state;
    }
}
