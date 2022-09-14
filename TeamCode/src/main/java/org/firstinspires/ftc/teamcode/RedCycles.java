package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedCycles extends PlanTechOpMode {

    @Override
    protected void postInit() {
        initServos();
        calibrateGyro();
        side = 1;
    }

    @Override
    protected void run() {
        runtime.reset();
        autoCycle();
    }


    @Override
    public void end() {
        closeAllMotors();
        closeServos();
        odometry.stop();
    }


}
