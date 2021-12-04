package org.firstinspires.ftc.teamcode.frieght_frenzy_code;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

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
    public DcMotorEx mtrBL, mtrBR, mtrFL, mtrFR; //control hub ports , , ,
    public DcMotorEx mtrArm, mtrTurret; //expansion hub ports ,
    public CRServo svoCarousel, svoIntake; //servo port 0, 1
    public Servo svoIntakeTilt, LEDstrip; //servo port

    public TouchSensor leftLimit, rightLimit, topLimit, bottomLimit; //digital ports . . .

    public DigitalChannel alliance_switch, position_switch;//digital port
    public VoltageSensor batteryVoltage;
    public HardwareMap hw = null;

    public double alliance = 0;
    public final double red = 1;
    public final double blue = -1;

    public double position = 0;
    public final double carousel = 1;
    public final double warehouse = -1;

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

        mtrArm = hw.get(DcMotorEx.class, "mtrArm");
        mtrArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrArm.setDirection(DcMotorEx.Direction.FORWARD);

        mtrTurret = hw.get(DcMotorEx.class, "mtrTurret");
        mtrTurret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrTurret.setDirection(DcMotorEx.Direction.REVERSE);

        svoCarousel = hw.get(CRServo.class, "svoCarousel");
        svoCarousel.setDirection(CRServo.Direction.REVERSE);

        svoIntake = hw.get(CRServo.class, "svoIntake");
        svoIntake.setDirection(CRServo.Direction.FORWARD);

        svoIntakeTilt = hw.get(Servo.class, "svoIntakeTilt");

        LEDstrip = hw.get(Servo.class, "LEDstrip");

        leftLimit = hw.get(TouchSensor.class, "leftLimit");
        rightLimit = hw.get(TouchSensor.class, "rightLimit");
        topLimit = hw.get(TouchSensor.class, "topLimit");
        bottomLimit = hw.get(TouchSensor.class, "bottomLimit");

        alliance_switch = hw.get(DigitalChannel.class, "alliance_switch");
        position_switch = hw.get(DigitalChannel.class, "position_switch");
        batteryVoltage = hw.voltageSensor.iterator().next();

        LEDstrip.setPosition(0.7545);

        if (alliance_switch.getState() == true) {
            alliance = red;
        }
        if (alliance_switch.getState() == false) {
            alliance = blue;
        }
        if (position_switch.getState() == true) {
            position = carousel;
        }
        if (position_switch.getState() == false) {
            position = warehouse;
        }

    }

    public void initWebcam (){

    }

    public void initPID (){


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
