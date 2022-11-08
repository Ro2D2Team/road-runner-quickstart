package org.firstinspires.ftc.teamcode.drive.OGCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="AlexDOpMode", group="Linear Opmode")

// This is the main programm
public class SampleOpMode extends LinearOpMode {

    //variables for registering how long a button is pressed
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timeservoClaw = new ElapsedTime();
    private ElapsedTime timeservoBrat = new ElapsedTime();
    private ElapsedTime timeDR4B = new ElapsedTime();
    private ElapsedTime timeSlowDown = new ElapsedTime();
    private ElapsedTime timeStartPosition = new ElapsedTime();
    private ElapsedTime timeLowPosition = new ElapsedTime();
    private ElapsedTime timeMidPosition = new ElapsedTime();
    private ElapsedTime timeHighPosition = new ElapsedTime();
    private ElapsedTime dpadUpTimer = new ElapsedTime();
    private ElapsedTime dpadDownTimer = new ElapsedTime();
    private ElapsedTime timeIntoarcere90 = new ElapsedTime();
    private ElapsedTime timpPrecisionMovement = new ElapsedTime();
    int poz = 0,poz2=0,pozDR4B = 0,SlowDownMode=0, okPosition=1, polePosition=-1800;
    boolean okHigh = true, okMid=true, okLow=true;
    int  PrecisionDenominator=1;
    //a variable that controls the power received by the drive motors
    double joyScale = 1;


    double Clip(double Speed,double lim)
    {
        return Math.max(Math.min(Speed,lim),-lim);
    }
    @Override
    public void runOpMode() {

        RobotMap robot=new RobotMap(hardwareMap);
        DriveTeleOp drive = new DriveTeleOp(0,0,0,0);
        double x1=0,y1=0,x2=0;
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        double lim = 0.50;
        while (opModeIsActive()) {


            /// DRIVE
            double y = gamepad1.right_stick_y; // Remember, this is reversed!
            double x = -gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.left_stick_x;

            rx/=PrecisionDenominator;
            x/=PrecisionDenominator;
            y/=PrecisionDenominator;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftPower = Clip(frontLeftPower,lim);
            backLeftPower = Clip(backLeftPower,lim);
            frontRightPower = Clip(frontRightPower,lim);
            backRightPower = Clip(backRightPower,lim);

            robot.leftFront.setPower(frontLeftPower);
            robot.leftBack.setPower(backLeftPower);
            robot.rightFront.setPower(frontRightPower);
            robot.rightBack.setPower(backRightPower);

            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            robot.imu.getPosition();
           /* if ((gamepad1.dpad_down ) && timeIntoarcere90.seconds()>0.2)
            {
                double targetHeading = 90;
                robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robot.imu.getPosition();
                robot.currentHeading = robot.angles.firstAngle;
                targetHeading = targetHeading + robot.currentHeading;
                while(Math.abs(targetHeading-robot.currentHeading)>10)
                {
                    robot.leftBack.setPower(0.6);
                    robot.leftFront.setPower(0.6);
                    robot.rightBack.setPower(-0.6);
                    robot.rightFront.setPower(-0.6);
                    robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    robot.imu.getPosition();
                    robot.currentHeading = robot.angles.firstAngle;
                }
                robot.leftBack.setPower(0);
                robot.leftFront.setPower(0);
                robot.rightBack.setPower(0);
                robot.rightFront.setPower(0);
                timeIntoarcere90.reset();
                telemetry.addLine("PressedDPADDOWN");
                telemetry.update();
            }*/
            telemetry.addData("x:",x);
            telemetry.addData("y",y);
            telemetry.addData("rx:",rx);
            /// DRIVE

            if ( ( gamepad2.a || gamepad1.a) && timeHighPosition.seconds()>0.2)
            {
                if (okHigh==true)
                {

                    robot.stangaLift.setTargetPosition(-2200);
                    robot.dreaptaLift.setTargetPosition(-2200);
                    robot.stangaLift.setPower(0.7);
                    robot.dreaptaLift.setPower(0.7);
                    robot.stangaLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.dreaptaLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    timeHighPosition.reset();
                    okHigh=false;
                }
                else
                {
                    robot.stangaLift.setTargetPosition(0);
                    robot.dreaptaLift.setTargetPosition(0);
                    robot.stangaLift.setPower(0.7);
                    robot.dreaptaLift.setPower(0.7);
                    robot.stangaLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.dreaptaLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    timeHighPosition.reset();
                    okHigh=true;

                }
            }
            if ( ( gamepad2.x || gamepad1.x) && timeMidPosition.seconds()>0.2)
            {
                if (okMid==true)
                {

                    robot.stangaLift.setTargetPosition(-1450);
                    robot.dreaptaLift.setTargetPosition(-1450);
                    robot.stangaLift.setPower(0.7);
                    robot.dreaptaLift.setPower(0.7);
                    robot.stangaLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.dreaptaLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    timeMidPosition.reset();
                    okMid=false;
                }
                else
                {
                    robot.stangaLift.setTargetPosition(0);
                    robot.dreaptaLift.setTargetPosition(0);
                    robot.stangaLift.setPower(0.7);
                    robot.dreaptaLift.setPower(0.7);
                    robot.stangaLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.dreaptaLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    timeMidPosition.reset();

                    okMid=true;


                }
            }
            if ( ( gamepad2.b || gamepad1.b) && timeLowPosition.seconds()>0.2)
            {
                if (okLow==true)
                {

                    robot.stangaLift.setTargetPosition(-800);
                    robot.dreaptaLift.setTargetPosition(-800);
                    robot.stangaLift.setPower(0.7);
                    robot.dreaptaLift.setPower(0.7);
                    robot.stangaLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.dreaptaLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    timeLowPosition.reset();
                    okLow=false;
                }
                else
                {
                    robot.stangaLift.setTargetPosition(0);
                    robot.dreaptaLift.setTargetPosition(0);
                    robot.stangaLift.setPower(0.7);
                    robot.dreaptaLift.setPower(0.7);
                    robot.stangaLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.dreaptaLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    timeLowPosition.reset();

                    okLow=true;
                }
            }
            if ((gamepad1.left_trigger>0 || gamepad2.left_trigger>0))
            {
                PrecisionDenominator=3;
            }
            else
            {
                PrecisionDenominator=1;
            }

            if ((gamepad1.left_bumper || gamepad2.left_bumper) && timeservoClaw.seconds()>0.2)
            {
                poz = 1 - poz;
                timeservoClaw.reset();
            }
            if (poz == 0)
            {
                robot.servoClaw.setPosition(0.7);
            }
            else
            {
                robot.servoClaw.setPosition(0.4);
            }
            if ((gamepad1.right_bumper || gamepad2.right_bumper) && timeservoBrat.seconds()>0.2)
            {
                poz2 = 1-poz2;
                timeservoBrat.reset();
            }
            if (poz2==0)
            {
                robot.servoBrat.setPosition(0.47);
            }
            else
            {
                robot.servoBrat.setPosition(0.325);
            }
            telemetry.addData("LeftLiftEncoder" , robot.stangaLift.getCurrentPosition());
            telemetry.addData("RightLiftEncoder",robot.dreaptaLift.getCurrentPosition());
            telemetry.addData("Z",robot.angles.firstAngle);
            telemetry.addData("Y",robot.angles.secondAngle);
            telemetry.addData("X",robot.angles.thirdAngle);
            telemetry.addData("PrecisionDenominator", PrecisionDenominator);
            telemetry.update();
        }
    }
}