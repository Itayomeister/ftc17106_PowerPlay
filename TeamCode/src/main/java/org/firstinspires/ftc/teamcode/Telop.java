package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp", group = "Linear Opmode")
//@Disabled
public class Telop extends PlanTechOpMode {

    @Override
    protected void postInit() {
        initCamera();
    }

    @Override
    public void run() {
        runtime.reset();

        telemetry.addData("Say", "Hello Drive");

        waitForStart();

        while (opModeIsActive()) {
           if (gamepad2.a){
               crServo1.setPower(.5);
               crServo2.setPower(.5);
               servo1.setPosition(1);
           }
            if (gamepad2.y){
                dcMotorEx1.setPower(.5);
                dcMotorEx2.setPower(.5);
            }
            if (gamepad2.b){
                crServo1.setPower(-.5);
                crServo2.setPower(-.5);
                servo1.setPosition(-1);
            }
            if (gamepad2.x){
                dcMotorEx1.setPower(-.5);
                dcMotorEx2.setPower(-.5);
            }
            crServo2.setPower(0);
            crServo1.setPower(0);
            dcMotorEx2.setPower(0);
            dcMotorEx1.setPower(0);
        }

    }

    @Override
    protected void end() {

    }
}