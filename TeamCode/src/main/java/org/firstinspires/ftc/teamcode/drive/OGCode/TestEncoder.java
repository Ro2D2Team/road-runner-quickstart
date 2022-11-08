package org.firstinspires.ftc.teamcode.drive.OGCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="TestEncoder", group="Linear Opmode")

// This is the main programm
public class TestEncoder extends LinearOpMode {

    //variables for registering how long a button is pressed
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timeservoClaw = new ElapsedTime();
    private ElapsedTime timeservoBrat = new ElapsedTime();
    int poz = 0,poz2=0;
    //a variable that controls the power received by the drive motors
    double joyScale = 1;



    @Override
    public void runOpMode() {


        int CUENCODERE = 0;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        DcMotor dreaptaLift = hardwareMap.get(DcMotor.class,"dreaptaLift");
        DcMotor stangaLift = hardwareMap.get(DcMotor.class,"stangaLift");
        stangaLift.setDirection(DcMotorSimple.Direction.REVERSE);
        dreaptaLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangaLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stangaLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive()) {
            if (CUENCODERE==1)
            {
                if (gamepad1.x)
                {
                    stangaLift.setTargetPosition(-2200);
                    dreaptaLift.setTargetPosition(-2200);
                    stangaLift.setPower(0.7);
                    dreaptaLift.setPower(0.7);
                    stangaLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    dreaptaLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                if (gamepad1.b)
                {
                    stangaLift.setTargetPosition(0);
                    dreaptaLift.setTargetPosition(0);
                    stangaLift.setPower(-0.7);
                    dreaptaLift.setPower(-0.7);
                    stangaLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    dreaptaLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }
            else
            {
                if (gamepad1.x) {

                    stangaLift.setPower(0.3);
                    dreaptaLift.setPower(0.3);
                }
                else
                if (gamepad1.b)
                {
                    stangaLift.setPower(-0.3);
                    dreaptaLift.setPower(-0.3);
                }
                else
                {
                    stangaLift.setPower(0);
                    dreaptaLift.setPower(0);
                }
            }
            telemetry.addData("EncoderStangaLift",stangaLift.getCurrentPosition());
            telemetry.addData("EncoderDreaptaLift",dreaptaLift.getCurrentPosition());
            telemetry.update();
        }
    }
}