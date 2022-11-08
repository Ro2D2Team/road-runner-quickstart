package org.firstinspires.ftc.teamcode.drive.OGCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="TestServo", group="Linear Opmode")

// This is the main programm
public class TestServo extends LinearOpMode {

    //variables for registering how long a button is pressed
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timeservoClaw = new ElapsedTime();
    private ElapsedTime timeservoBrat = new ElapsedTime();
    int poz = 0,poz2=0;
    //a variable that controls the power received by the drive motors
    double joyScale = 1;



    @Override
    public void runOpMode() {


        int CUENCODERE = 1;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        Servo Claw = hardwareMap.get(Servo.class,"ServoClaw");

        while (opModeIsActive()) {
            if (gamepad1.b && timeservoClaw.seconds()>0.2)
            {
                poz=1-poz;
                timeservoClaw.reset();
            }
            if (poz==1)
            {
                Claw.setPosition(0.7); /// deschis
            }
            else
            {
                Claw.setPosition(0.4); /// colectat
            }
        }
    }
}