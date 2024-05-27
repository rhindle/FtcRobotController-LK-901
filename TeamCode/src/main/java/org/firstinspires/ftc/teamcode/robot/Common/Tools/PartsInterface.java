package org.firstinspires.ftc.teamcode.robot.Common.Tools;

import org.firstinspires.ftc.teamcode.robot.Common.Parts;

public interface PartsInterface {
    void construct(Parts parts);
    public void initialize();
    public void initLoop();
    public void loop();
    public void preInit();
    public void preStart();
    public void stop();
}
