package org.firstinspires.ftc.teamcode.drive.OGCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class OpModeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        /*RobotMap robot = new RobotMap(hardwareMap);
        DriveTeleOp drive = new DriveTeleOp(0,0,0,0);
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        //motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double x = -gamepad1.right_stick_x;
            double y = gamepad1.right_stick_y *1.1;
            double rot = gamepad1.left_stick_x;
            drive.calculatePower(x,y,rot);
            drive.ScaleDownSpeed(0.5);
            drive.setPwr(drive.getLB(),drive.getLF(),drive.getRB(),drive.getRF(),robot);
            telemetry.addData("Encoder Center" , robot.leftBack.getCurrentPosition());
            telemetry.addData("Encoder Left" , robot.rightBack.getCurrentPosition());
            telemetry.addData("Encoder Right" , robot.rightFront.getCurrentPosition());
            telemetry.update();
           /* // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rot), 1);
            double frontLeftPower = (rotY + rotX + rot) / denominator;
            double backLeftPower = (rotY - rotX + rot) / denominator;
            double frontRightPower = (rotY - rotX - rot) / denominator;
            double backRightPower = (rotY + rotX - rot) / denominator;
            drive.setPwr(backLeftPower,frontLeftPower,backRightPower,frontRightPower,robot);
        }*/
    }
}