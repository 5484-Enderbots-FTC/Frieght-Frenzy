package org.firstinspires.ftc.teamcode.frieght_frenzy_code;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class hardwareFF {
    //common hardware imports are stored here so that its crazy easy to import them and make changes to ALL CODE
    //documents at the same time instead of doing it manually

    /**

    * the variables here that are only used in this doc can be private but everything else like motors and servos need to be
    * public so they can be called in the other programs.

    * the methods and classes are public so you can call them and have them reference the other things in here

    //So far this season we just have motors, so I've done the work to initialize them here:
     */
    public DcMotorEx mtrBL, mtrBR, mtrFL, mtrFR;
    public Servo carouselServo;

    private VoltageSensor batteryVoltage;
    private HardwareMap hw = null;


    public hardwareFF() {
        //nothing goes in this- its just a way to call the program
    }

    public void init(HardwareMap thisHwMap){
        hw = thisHwMap;

        mtrBL = hw.get(DcMotorEx.class, "mtrBL");
        mtrBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrBL.setDirection(DcMotorEx.Direction.FORWARD);

        mtrBR = hw.get(DcMotorEx.class, "mtrBR");
        mtrBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrBR.setDirection(DcMotorEx.Direction.REVERSE);

        mtrFL = hw.get(DcMotorEx.class, "mtrFL");
        mtrFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrFL.setDirection(DcMotorEx.Direction.FORWARD);

        mtrFR = hw.get(DcMotorEx.class, "mtrFR");
        mtrFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrFR.setDirection(DcMotorEx.Direction.REVERSE);

        batteryVoltage = hw.voltageSensor.iterator().next();

        carouselServo = hw.get(Servo.class, "carouselServo");

    }

    public void updateDrive(double fwdStick, double turnStick, double strStick){
        mtrBL.setPower((fwdStick - turnStick + strStick));
        mtrBR.setPower((fwdStick + turnStick - strStick));
        mtrFL.setPower((fwdStick - turnStick - strStick));
        mtrFR.setPower((fwdStick + turnStick + strStick));
    }

    public void updateDrive(double fwdStick, double turnStick, double strStick, boolean reversed){
        mtrBL.setPower((-fwdStick + turnStick - strStick));
        mtrBR.setPower((-fwdStick - turnStick + strStick));
        mtrFL.setPower((-fwdStick + turnStick + strStick));
        mtrFR.setPower((-fwdStick - turnStick - strStick));
    }
    public void brake(){
        mtrBR.setPower(0);
        mtrBL.setPower(0);
        mtrFR.setPower(0);
        mtrFL.setPower(0);
    }
    public void reset(){
        mtrBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void forward (double power, int position){
        reset();
        int adjustment = 1;
        mtrBR.setPower(power);
        mtrBR.setTargetPosition(position*adjustment);
        mtrBL.setPower(power);
        mtrBL.setTargetPosition(position*adjustment);
        mtrFR.setPower(power);
        mtrFR.setTargetPosition(position*adjustment);
        mtrFL.setPower(power);
        mtrFL.setTargetPosition(position*adjustment);
        mtrBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(mtrFR.isBusy() | mtrFL.isBusy() | mtrBR.isBusy() | mtrBL.isBusy()){

        }
        brake();
    }
    public void strafe (double power, int position){
        reset();
        int adjustment = 1;
        mtrBR.setPower(power);
        mtrBR.setTargetPosition(position*adjustment);
        mtrBL.setPower(-power);
        mtrBL.setTargetPosition(position*adjustment);
        mtrFR.setPower(-power);
        mtrFR.setTargetPosition(position*adjustment);
        mtrFL.setPower(power);
        mtrFL.setTargetPosition(position*adjustment);
        mtrBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(mtrFR.isBusy() | mtrFL.isBusy() | mtrBR.isBusy() | mtrBL.isBusy()){

        }
        brake();
    }
    public void turn (double power, int position){
        reset();
        int adjustment = 1;
        mtrBR.setPower(power);
        mtrBR.setTargetPosition(position*adjustment);
        mtrBL.setPower(-power);
        mtrBL.setTargetPosition(position*adjustment);
        mtrFR.setPower(power);
        mtrFR.setTargetPosition(position*adjustment);
        mtrFL.setPower(-power);
        mtrFL.setTargetPosition(position*adjustment);
        mtrBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(mtrFR.isBusy() | mtrFL.isBusy() | mtrBR.isBusy() | mtrBL.isBusy()){

        }
        brake();
    }


}
