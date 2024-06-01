package org.firstinspires.ftc.teamcode.robot.Common.Tools;

import org.firstinspires.ftc.teamcode.robot.Common.Parts;

public interface PartsInterface {
    //void construct(Parts parts);
    public void initialize();
    public void preInit();
    public void initLoop();
    public void preRun();
    public void runLoop();
    public void stop();
}
