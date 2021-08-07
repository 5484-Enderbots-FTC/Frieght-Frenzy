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

    private VoltageSensor batteryVoltage;
    private HardwareMap hw = null;


    public hardwareFF() {
        //nothing goes in this- its just a way to call the program
    }

    public void init(HardwareMap thisHwMap){
        hw = thisHwMap;

        mtrBL = hw.get(DcMotorEx.class, "mtrBL");
        mtrBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrBL.setDirection(DcMotorEx.Direction.REVERSE);

        mtrBR = hw.get(DcMotorEx.class, "mtrBR");
        mtrBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrBR.setDirection(DcMotorEx.Direction.FORWARD);

        mtrFL = hw.get(DcMotorEx.class, "mtrFL");
        mtrFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrFL.setDirection(DcMotorEx.Direction.REVERSE);

        mtrFR = hw.get(DcMotorEx.class, "mtrFR");
        mtrFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrFR.setDirection(DcMotorEx.Direction.FORWARD);

        batteryVoltage = hw.voltageSensor.iterator().next();

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


}
