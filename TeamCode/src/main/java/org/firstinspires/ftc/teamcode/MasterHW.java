package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class MasterHW {
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public DcMotor backrightMotor = null;
    public DcMotor backleftMotor = null;
    public Servo rClaw = null;
   public Servo lClaw = null;
    public Servo xDrone = null;
    //public Servo xSlide = null;
   // public Servo xRotate = null;
    public DcMotor inTake = null;
    public DcMotor liftMotor2 = null;
    public DcMotor liftMotor1 = null;
   // public NormalizedColorSensor colorR1;
    public WebcamName webCam1 = null;
    //public DistanceSensor sensorRange;



    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    public BNO055IMU imu1 = null;
  //public BHI260IMU imu1 = null;


    /* Initialize standard Hardware interfaces */

    public void init(HardwareMap ahwMap) {
        RobotLog.ii("CAL", "Enter - init");

        leftMotor = ahwMap.get(DcMotor.class, "MFrontLeft");
        rightMotor = ahwMap.get(DcMotor.class, "MFrontRight");
        backleftMotor = ahwMap.get(DcMotor.class, "MBackLeft");
        backrightMotor = ahwMap.get(DcMotor.class, "MBackRight");
        inTake = ahwMap.get(DcMotor.class,  "MInTake");
        liftMotor2 = ahwMap.get(DcMotor.class,  "liftMotor2");
        liftMotor1 = ahwMap.get(DcMotor.class,  "liftMotor1");
        rClaw = ahwMap.get(Servo.class, "rClaw");
        lClaw = ahwMap.get(Servo.class, "lClaw");
        xDrone = ahwMap.get(Servo.class, "Drone");
       /* xClaw = ahwMap.get(Servo.class, "rClaw");
        xSlide = ahwMap.get(Servo.class, "lClaw");
        xRotate = ahwMap.get(Servo.class, "xRotate");
        colorR1 =ahwMap.get(NormalizedColorSensor.class, "colorR1");
        sensorRange =ahwMap.get(DistanceSensor.class, "d1");*/

        // webCam1 = ahwMap.get(WebcamName.class,"webcam 1");


        imu1 = ahwMap.get(BNO055IMU.class, "imu");

        //initialize the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu1.initialize(parameters);

        //Invert direction for left motors
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backleftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to zero power
        stopAllMotors();

        //Set zero power behavior to braking
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void stopAllMotors() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        backleftMotor.setPower(0);
        backrightMotor.setPower(0);
        inTake.setPower(0);
        //  liftMotor2.setPower(0);
    }

    public void stopLiftMotors() {
        liftMotor1.setPower(0);
        liftMotor2.setPower(0);
       // liftMotor3.setPower(0);
            }

    public void moveHolonomic(double x, double y , double z)
    {
        double max_power = 0.7;
        double min_power = -1*max_power;

        double fl_power = Range.clip(y + x - z, min_power, max_power);
        double fr_power = Range.clip(y - x + z, min_power, max_power);
        double br_power = Range.clip(y + x + z, min_power, max_power);
        double bl_power = Range.clip(y - x - z, min_power, max_power);
        RobotLog.ii("CAL", "moveHolonomic - Enter x(%f), y(%f), z(%f)", x, y, z);
        RobotLog.ii("CAL", "moveHolonomic - Enter fl(%f), fr(%f), bl(%f), br(%f)", fl_power,fr_power, bl_power, br_power );

        // Sets the power of the motors to the power defined above

        setPowerAll(fl_power, fr_power, bl_power, br_power);
        RobotLog.ii("CAL", "moveHolonomic - Exit ");

    }

    public void setPowerAll(double fl_power, double fr_power, double bl_power, double br_power){
        leftMotor.setPower(fl_power);
        rightMotor.setPower(fr_power);
        backleftMotor.setPower(bl_power);
        backrightMotor.setPower(br_power);


    }

    public void moveForward(double power)
    {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        backleftMotor.setPower(power);
        backrightMotor.setPower(power);
    }

    public void moveBackwards(double power)
    {
        leftMotor.setPower(-1 * power);
        rightMotor.setPower(-1 * power);
        backleftMotor.setPower(-1 * power);
        backrightMotor.setPower(-1 * power);
    }

    public void turnRight(double power)
    {
        leftMotor.setPower(power);
        rightMotor.setPower(-1 * power);
        backleftMotor.setPower(power);
        backrightMotor.setPower(-1 * power);
    }

    public void turnLeft(double power)
    {
        leftMotor.setPower(-1 * power);
        rightMotor.setPower(power);
        backleftMotor.setPower(-1 * power);
        backrightMotor.setPower(power);
    }

    public void strafeRight(double power)
    {
        leftMotor.setPower(power);
        rightMotor.setPower(-1 * power);
        backleftMotor.setPower(-1 * power);
        backrightMotor.setPower(power);
    }

    public void strafeleft(double power)
    {
        leftMotor.setPower(-1 * power);
        rightMotor.setPower(power);
        backleftMotor.setPower(power);
        backrightMotor.setPower(-1 * power);
    }

    public void diagonalforwardRight(double power)
    {
        leftMotor.setPower(power);
        backrightMotor.setPower(power);
    }

    public void diagonalforwardLeft(double power)
    {
        rightMotor.setPower(power);
        backleftMotor.setPower(power);
    }

    public void diagonalbackwardsRight(double power)
    {
        rightMotor.setPower(-1 * power);
        backleftMotor.setPower(-1 * power);
    }

    public void diagonalbackwardsLeft(double power)
    {
        leftMotor.setPower(-1 * power);
        backrightMotor.setPower(-1 * power);
    }

    public void forwardSlow()
    {
        leftMotor.setPower(Range.clip(leftMotor.getPower() + 0.01, 0.3, 1.0));
        rightMotor.setPower(Range.clip(rightMotor.getPower() + 0.01, 0.3, 1.0));
        backleftMotor.setPower(Range.clip(backleftMotor.getPower() + 0.01, 0.3, 1.0));
        backrightMotor.setPower(Range.clip(backrightMotor.getPower() + 0.01, 0.3, 1.0));

    }

    public void backwardSlow()
    {
        leftMotor.setPower(Range.clip(leftMotor.getPower() - 0.01, -0.3, -1.0));
        rightMotor.setPower(Range.clip(rightMotor.getPower() - 0.01, -0.3, -1.0));
        backleftMotor.setPower(Range.clip(backleftMotor.getPower() - 0.01, -0.3, -1.0));
        backrightMotor.setPower(Range.clip(backrightMotor.getPower() - 0.01, -0.3, -1.0));
    }

  /*  public void clawOpen() {
        rClaw.setPosition(0.8);
        lClaw.setPosition(0.8);

    }
    public void clawClose() {
        rClaw.setPosition(0);
        lClaw.setPosition(0);

    }
*/
}
