package org.firstinspires.ftc.teamcode.drive.OGCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotMap {

    /*
    Variabilele - > Motoare/Servouri etc
     */

    // Control Hub

    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor dreaptaLift = null;
    public DcMotor stangaLift = null;
    public Servo servoClaw = null;
    public Servo servoBrat = null;
    BNO055IMU imu = null;

    Orientation angles;
    Acceleration gravity;
    public double currentHeading=0;
    public RobotMap(HardwareMap Init)
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = Init.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        stangaLift = Init.get(DcMotor.class,"stangaLift");
        dreaptaLift = Init.get(DcMotor.class,"dreaptaLift");
        servoClaw = Init.get(Servo.class,"servoClaw");
        servoBrat = Init.get(Servo.class,"servoBrat");
        leftFront = Init.get(DcMotor.class,"leftFront");
        rightFront = Init.get(DcMotor.class,"rightFront");
        leftBack = Init.get(DcMotor.class,"leftBack");
        rightBack = Init.get(DcMotor.class,"rightBack");


        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        stangaLift.setDirection(DcMotorSimple.Direction.REVERSE);
        stangaLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dreaptaLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dreaptaLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangaLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}