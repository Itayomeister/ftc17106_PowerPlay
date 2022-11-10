package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static java.lang.Math.PI;
import static java.lang.Math.asin;
import static java.lang.Math.atan;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;
import static java.lang.Math.toRadians;
import static java.lang.StrictMath.toDegrees;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


public abstract class PlanTechOpMode extends LinearOpMode {

    DcMotorEx arm1, arm2;
    int countsPreRevolution = 28;
    double armGearBox = 143.1188; // 5:5:5:1
    final int armCountsPerRev = (int) (armGearBox * countsPreRevolution);
    final int FINALARMCOUNT = (int) (.25 * armCountsPerRev);

    Gamepad gamepad1, gamepad2;

    protected ElapsedTime runtime = new ElapsedTime();

    void initialize() {

    }

    @Override
    public void runOpMode() {
        initialize();

        postInit();

        waitForStart();


        if (opModeIsActive()) {
            run();
        }

        end();
    }

    protected void postInit() {

    }

    protected abstract void run();

    protected abstract void end();


    public void armHigh() {
        double degreees = toDegrees(asin(70 / 80));
        //B = beta,degreees = alpha,T=ticks
        int B = (int) (90 - degreees);
        int BtoY = 90 / B;
        int YtoT = FINALARMCOUNT / BtoY;
        arm1.setTargetPosition(YtoT);
        arm2.setTargetPosition((int) .5 * FINALARMCOUNT);
    }

    public void armMid() {
        double degreees = toDegrees(asin(40 / 80));//30 deg
        //B = beta,degreees = alpha,T=ticks
        int B = (int) (90 - degreees);//60
        int BtoY = 90 / B;//1.5
        int YtoT = FINALARMCOUNT / BtoY;
        arm1.setTargetPosition(YtoT);
        arm2.setTargetPosition((int) .5 * FINALARMCOUNT);
    }

    public void armlow() {
        double degreees = toDegrees(asin(20 / 80));
        //B = beta,degreees = alpha,T=ticks
        int B = (int) (90 - degreees);
        int BtoY = 90 / B;
        int YtoT = FINALARMCOUNT / BtoY;
        arm1.setTargetPosition(YtoT);
        arm2.setTargetPosition((int) .5 * FINALARMCOUNT);
    }

    public void armgound() {
        int alpha = (int) toDegrees(asin(30 / 40));
        int beta = 90 - alpha;
        int alphaToTicks = alpha / 360 * FINALARMCOUNT;
        int betaToTicks = beta / 360 * FINALARMCOUNT;
        arm1.setTargetPosition(alphaToTicks);
        arm2.setTargetPosition(betaToTicks);
    }

    public void armHigh2() {
        int degreesToTicks = FINALARMCOUNT *135/360; // sets to 135 deg angle
        arm2.setTargetPosition(degreesToTicks);
    }
}


