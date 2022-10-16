package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;

import static java.lang.Math.*;

@TeleOp
public class TestOpMode extends PlanTechOpMode {

    @Override
    protected void postInit() {
        //initCamera(); add to super post init
    }

    @Override
    protected void run() {
        runtime.reset();

        if (gamepad1.x)
            arm1.setPower(.1);
        else
            arm1.setPower(0);

    }

    @Override
    protected void end() {

    }
}
