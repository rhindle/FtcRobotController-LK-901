package org.firstinspires.ftc.teamcode.robot.Common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Arrays;

public class TelemetryMgr {

    static int debugLevel = 10;
    static boolean needsUpdate = false;
    static LinearOpMode opMode;
    public static boolean[] showCategory;

    public TelemetryMgr(LinearOpMode opMode) {
        construct(opMode);
    }

    void construct(LinearOpMode opMode){
        TelemetryMgr.opMode = opMode;

        showCategory = new boolean[Category.values().length];
//        Arrays.fill(showCategory, false);
//        showCategory[Category.MANDATORY.ordinal()] = true;
//        showCategory[Category.BASIC.ordinal()] = true;
        enableDefaultCategories();
    }

    /* message with debuglevel */
    public static void message(int lvl, String cap, Object val) {
        if (lvl <= debugLevel) {
            opMode.telemetry.addData(cap, val);
            needsUpdate = true;
        }
    }

    public static void message(int lvl, String cap, String fmt, Object... args) {
        if (lvl <= debugLevel) {
            opMode.telemetry.addData(cap, fmt, args);
            needsUpdate = true;
        }
    }

    public static void message(int lvl, String cap) {
        if (lvl <= debugLevel) {
            opMode.telemetry.addLine(cap);
            needsUpdate = true;
        }
    }

    /* message with category */
    public static void message(Category category, String cap, Object val) {
        if (showCategory[category.ordinal()]) {
            opMode.telemetry.addData(cap, val);
            needsUpdate = true;
        }
    }

    public static void message(Category category, String cap, String fmt, Object... args) {
        if (showCategory[category.ordinal()]) {
            opMode.telemetry.addData(cap, fmt, args);
            needsUpdate = true;
        }
    }

    public static void message(Category category, String cap) {
        if (showCategory[category.ordinal()]) {
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

    public static void enableCategories(Category[] categories) {
        for (Category category : categories) {
            showCategory[category.ordinal()] = true;
        }
    }

    public static void disableCategories (Category[] categories) {
        for (Category category : categories) {
            showCategory[category.ordinal()] = false;
        }
    }

    public static void disableEXTCategories () {
        for (int i=0; i<showCategory.length; i++) {
//            String name = Category.values()[i].name();
//            if (name.contains("_EXT")) showCategory[i]=false;
            if (Category.values()[i].name().contains("_EXT")) showCategory[i]=false;
        }
    }

    public static void enableAllCategories() {
        Arrays.fill(showCategory, true);
    }

    public static void enableDefaultCategories() {
        Arrays.fill(showCategory, false);
        showCategory[Category.MANDATORY.ordinal()] = true;
        showCategory[Category.BASIC.ordinal()] = true;
    }

    public enum Category {
        BASIC,
        MANDATORY,
        SENSORS,
        SENSORS_EXT,
        IMU,
        IMU_EXT,
        SLAMRA,
        SLAMRA_EXT,
        ODOMETRY,
        ODOMETRY_EXT,
        AUTODRIVE,
        AUTODRIVE_EXT,
        USERDRIVE,
        USERDRIVE_EXT,
        POSITION,
        POSITION_EXT,
        DRIVETRAIN,
        CONTROLS,
        APRILTAG,
        APRILTAG_EXT,
        SPEED,
        NAVIGATOR;   //navigator is deprecated
   }
}