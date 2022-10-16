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
           if (gamepad2.a){
               armgound();
           }
            if (gamepad2.y){
                armHigh();
            }
            if (gamepad2.b){
                armMid();
            }
            if (gamepad2.x){
                armlow();
            }


        }

    }

    @Override
    protected void end() {

    }
}