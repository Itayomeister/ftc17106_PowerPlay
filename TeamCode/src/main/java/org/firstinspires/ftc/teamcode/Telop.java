package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp", group = "Linear Opmode")
//@Disabled
public class Telop extends PlanTechOpMode {

    @Override
    protected void postInit() {
    }

    @Override
    public void run() {
        runtime.reset();

        telemetry.addData("Say", "Hello Drive");

        waitForStart();

        while (opModeIsActive()) {


        }

    }

    @Override
    protected void end() {

    }
}