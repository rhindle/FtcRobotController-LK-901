package org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.Shooter;

import org.firstinspires.ftc.teamcode.Tools.DataTypes.NavigationTarget;

class FullAuto {

    public static int stateFullAuto = 0;

    public static void stateMachineFullAuto() {

        if (stateFullAuto == -9) {
            DSShooter.parts.autoDrive.setAutoDrive(false);
        }
        if (stateFullAuto < 1 || stateFullAuto > 999) return;  // not running

        if (!DSShooter.parts.positionMgr.hasPosition()) stateFullAuto = -9;    // cancel running if no navigation

        if (stateFullAuto == 1) {
            if (Shoot1.stateShoot1Step > 0 && Shoot1.stateShoot1Step <999) {
                DSShooter.cancelTimer = System.currentTimeMillis() + 1000;
                Shoot1.stateShoot1Step = -9;
            }
            if (Shoot3.stateShoot3Step > 0 && Shoot3.stateShoot3Step <999) {
                DSShooter.cancelTimer = System.currentTimeMillis() + 1000;
                Shoot3.stateShoot3Step = -9;
            }
            if (System.currentTimeMillis() >= DSShooter.cancelTimer) stateFullAuto=2;
        }

        if (stateFullAuto == 2) {                                    // navigate to launch position
            DSShooter.parts.autoDrive.setNavTarget(new NavigationTarget(DSShooter.autoLaunchPos, DSShooter.parts.dsMisc.toleranceHigh));
            stateFullAuto++;
        }
        if (stateFullAuto == 3) {                                   // wait until reach position then start shooting
            if (DSShooter.parts.autoDrive.onTargetByAccuracy) {
                Shoot3.stateShoot3Step = 1;
                stateFullAuto++;
            }
        }
        if (stateFullAuto == 4) {                                 // start blasting
            Shoot3.stateShoot3Step = 1;
            stateFullAuto++;
        }
        if (stateFullAuto == 5) {
            if (Shoot3.stateShoot3Step == 1000) {
                DSShooter.parts.autoDrive.setAutoDrive(false);
                DSShooter.closeGate();
                DSShooter.spinnerOff();
                DSShooter.retractPusher();
                stateFullAuto=1000;
            }
        }
    }
}
