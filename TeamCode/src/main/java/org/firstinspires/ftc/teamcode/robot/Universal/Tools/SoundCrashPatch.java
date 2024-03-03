package org.firstinspires.ftc.teamcode.robot.Universal.Tools;

import com.qualcomm.ftcrobotcontroller.R;

import org.firstinspires.ftc.ftccommon.external.SoundPlayingRobotMonitor;
import org.firstinspires.ftc.robotcore.external.function.Consumer;

import androidx.annotation.Nullable;
import androidx.annotation.RawRes;

public final class SoundCrashPatch extends SoundPlayingRobotMonitor {
    private int[] _availableSound = {R.raw.nxtstartupsound, R.raw.errormessage, R.raw.warningmessage};

    @Override
    protected void playSound(Sound sound, @RawRes final int resourceId, @Nullable Consumer<Integer> runWhenStarted, @Nullable Runnable runWhenFinished) {
        boolean soundMatched = false;
        for (int s: _availableSound) {
            if (resourceId == s)
                soundMatched = true;
        }
        if (soundMatched)
            super.playSound(sound, resourceId, runWhenStarted, runWhenFinished);
    }
}