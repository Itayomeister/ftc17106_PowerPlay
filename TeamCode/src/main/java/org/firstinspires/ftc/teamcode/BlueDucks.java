package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueDucks extends PlanTechOpMode {

    @Override
    public void postInit() {
        initServos();
        calibrateGyro();
        side = -1;
    }

    @Override
    protected void run() {
        runtime.reset();
        autoDucks();
    }

    @Override
    public void end() {
        closeAllMotors();
        closeServos();
        odometry.stop();
    }


}
