package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AttachHw   {

    public Servo rRotate = null;
    public Servo Claw = null;
    public Servo rClaw = null;
    public Servo servoArm = null;
    public Servo Drone2 = null;
    //public Servo xSlide = null;
    // public Servo xRotate = null;
    public DcMotor inTake = null;
    public DcMotor liftMotor2 = null;
    public DcMotor liftMotor1 = null;
    public DcMotor mHook = null;
    // public NormalizedColorSensor colorR1;
    public WebcamName webCam1 = null;
    public DistanceSensor sensorD1;
    public DistanceSensor sensorD2;
    public void init(HardwareMap ahwMap) {
        RobotLog.ii("CAL", "Enter - init");

        inTake = ahwMap.get(DcMotor.class, "MInTake");
        liftMotor2 = ahwMap.get(DcMotor.class, "liftMotor2");
        liftMotor1 = ahwMap.get(DcMotor.class, "liftMotor1");
        mHook = ahwMap.get(DcMotor.class, "Mhook");
        rRotate = ahwMap.get(Servo.class, "rotate");
        Claw = ahwMap.get(Servo.class, "lClaw");
        rClaw = ahwMap.get(Servo.class, "rClaw");
        servoArm = ahwMap.get(Servo.class, "Drone");
        Drone2 = ahwMap.get(Servo.class, "Drone2");
       /* xClaw = ahwMap.get(Servo.class, "rClaw");
        xSlide = ahwMap.get(Servo.class, "lClaw");
        xRotate = ahwMap.get(Servo.class, "xRotate");
        colorR1 =ahwMap.get(NormalizedColorSensor.class, "colorR1");*/
        sensorD1 = ahwMap.get(DistanceSensor.class, "d1");
        sensorD2 = ahwMap.get(DistanceSensor.class, "d2");
        // webCam1 = ahwMap.get(WebcamName.class,"webcam 1");



    }



}