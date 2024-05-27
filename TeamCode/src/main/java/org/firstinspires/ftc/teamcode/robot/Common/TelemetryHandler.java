package org.firstinspires.ftc.teamcode.robot.Common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryHandler {

    static int debugLevel = 10;
    static boolean needsUpdate = false;
    static LinearOpMode opMode;

    public TelemetryHandler(LinearOpMode opMode) {
        TelemetryHandler.opMode = opMode;
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