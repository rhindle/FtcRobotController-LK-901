package org.firstinspires.ftc.teamcode.robot.Common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TelemetryMgr {

    static int debugLevel = 10;
    static boolean needsUpdate = false;
    static LinearOpMode opMode;

    public TelemetryMgr(LinearOpMode opMode) {
        TelemetryMgr.opMode = opMode;
    }

    public static void Message (int lvl, String cap, Object val) {
        if (lvl <= debugLevel) {
            opMode.telemetry.addData(cap, val);
            needsUpdate = true;
        }
    }

    public static void Message (int lvl, String cap, String fmt, Object... args) {
        if (lvl <= debugLevel) {
            opMode.telemetry.addData(cap, fmt, args);
            needsUpdate = true;
        }
    }

    public static void Message (int lvl, String cap) {
        if (lvl <= debugLevel) {
            opMode.telemetry.addLine(cap);
            needsUpdate = true;
        }
    }


    public static void Update () {
        if (needsUpdate) opMode.telemetry.update();
        needsUpdate = false;
    }

    public static void Update (boolean forceUpdate) {
        if (forceUpdate) opMode.telemetry.update();
        needsUpdate = false;
    }

    public static void setDebugLevel (int lvl) {
        debugLevel = lvl;
    }
}