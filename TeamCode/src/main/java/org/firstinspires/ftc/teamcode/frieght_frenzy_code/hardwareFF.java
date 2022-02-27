package org.firstinspires.ftc.teamcode.frieght_frenzy_code;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class hardwareFF {
    //common hardware imports are stored here so that its crazy easy to import them and make changes to ALL CODE
    //documents at the same time instead of doing it manually

    /**
     * the variables here that are only used in this doc can be private but everything else like motors and servos need to be
     * public so they can be called in the other programs.
     *
     * the methods and classes are public so you can call them and have them reference the other things in here
     *
     */
    public ElementAnalysisPipelineFF pipeline;

    public DcMotorEx mtrBL, mtrBR, mtrFL, mtrFR; //control hub ports , , ,
    public DcMotorEx mtrArm, mtrTurret, mtrTape; //expansion hub ports ,
    public CRServo svoCarousel, svoIntake; //servo port 0, 1
    public Servo svoIntakeTilt, LEDstrip; //servo port

    public TouchSensor frontLimit,backLimit, midLimit, topLimit, bottomLimit, intakeLimit; //digital ports . . .

    public DigitalChannel alliance_switch, position_switch;//digital port

    public ModernRoboticsI2cRangeSensor frontRange, backRange; //front should be configured as the one under the battery holder :)
    public DistanceSensor rightDistance, leftDistance; //right distance should be configured as the one under the battery holder too lol

    public VoltageSensor batteryVoltage;
    public HardwareMap hw = null;

    public OpenCvWebcam webcam;

    public double alliance = 0;
    public final double red = 1;
    public final double blue = -1;

    public double position = 0;
    public final double carousel = 1;
    public final double warehouse = -1;

    ElapsedTime deinitWait = new ElapsedTime();

    public hardwareFF() {
        //nothing goes in this- its just a way to call the program
    }

    public void init(HardwareMap thisHwMap) {
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

        mtrTape = hw.get(DcMotorEx.class, "mtrTape");
        mtrTurret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrTurret.setDirection(DcMotorEx.Direction.FORWARD);

        svoCarousel = hw.get(CRServo.class, "svoCarousel");
        svoCarousel.setDirection(CRServo.Direction.REVERSE);

        svoIntake = hw.get(CRServo.class, "svoIntake");
        svoIntake.setDirection(CRServo.Direction.FORWARD);

        svoIntakeTilt = hw.get(Servo.class, "svoIntakeTilt");

        LEDstrip = hw.get(Servo.class, "LEDstrip");

        midLimit = hw.get(TouchSensor.class, "midLimit");
        bottomLimit = hw.get(TouchSensor.class, "bottomLimit");
        intakeLimit = hw.get(TouchSensor.class, "intakeLimit");

        frontLimit = hw.get(TouchSensor.class, "frontLimit");
        backLimit = hw.get(TouchSensor.class, "backLimit");

        alliance_switch = hw.get(DigitalChannel.class, "alliance_switch");
        position_switch = hw.get(DigitalChannel.class, "position_switch");

        frontRange = hw.get(ModernRoboticsI2cRangeSensor.class, "frontRange");
        backRange = hw.get(ModernRoboticsI2cRangeSensor.class, "backRange");
        rightDistance = hw.get(DistanceSensor.class, "rightDistance");
        leftDistance = hw.get(DistanceSensor.class, "leftDistance");

        batteryVoltage = hw.voltageSensor.iterator().next();

        LEDstrip.setPosition(var.ocean);

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

    public void initWebcam() {
        // Create camera instance
        int cameraMonitorViewId = hw.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hw.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hw.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        // Open async and start streaming inside opened callback
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

                pipeline = new ElementAnalysisPipelineFF();
                webcam.setPipeline(pipeline);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

    public void updateDrive(double fwdStick, double turnStick, double strStick) {
        mtrBL.setPower((fwdStick - turnStick + strStick));
        mtrBR.setPower((fwdStick + turnStick - strStick));
        mtrFL.setPower((fwdStick - turnStick - strStick));
        mtrFR.setPower((fwdStick + turnStick + strStick));
    }

    public void updateDrive(double fwdStick, double turnStick, double strStick, boolean reversed) {
        mtrBL.setPower((-fwdStick + turnStick - strStick));
        mtrBR.setPower((-fwdStick - turnStick + strStick));
        mtrFL.setPower((-fwdStick + turnStick + strStick));
        mtrFR.setPower((-fwdStick - turnStick - strStick));
    }

    public void brake() {
        mtrBR.setPower(0);
        mtrBL.setPower(0);
        mtrFR.setPower(0);
        mtrFL.setPower(0);
    }

    public void reset() {
        mtrBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void forward(double power, int position) {
        reset();
        int adjustment = 1;
        mtrBR.setPower(power);
        mtrBR.setTargetPosition(position * adjustment);
        mtrBL.setPower(power);
        mtrBL.setTargetPosition(position * adjustment);
        mtrFR.setPower(power);
        mtrFR.setTargetPosition(position * adjustment);
        mtrFL.setPower(power);
        mtrFL.setTargetPosition(position * adjustment);
        mtrBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (mtrFR.isBusy()) {

        }
        brake();
    }

    public void strafe(double power, int position) {
        reset();
        int adjustment = 1;
        mtrBR.setPower(power);
        mtrBR.setTargetPosition(position * adjustment);
        mtrBL.setPower(-power);
        mtrBL.setTargetPosition(-position * adjustment);
        mtrFR.setPower(-power);
        mtrFR.setTargetPosition(-position * adjustment);
        mtrFL.setPower(power);
        mtrFL.setTargetPosition(position * adjustment);
        mtrBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (mtrFR.isBusy()) {

        }
        brake();
    }

    public void turn(double power, int position) {
        reset();
        int adjustment = 1;
        mtrBR.setPower(power);
        mtrBR.setTargetPosition(position * adjustment);
        mtrBL.setPower(power);
        mtrBL.setTargetPosition(position * adjustment);
        mtrFR.setPower(-power);
        mtrFR.setTargetPosition(-position * adjustment);
        mtrFL.setPower(-power);
        mtrFL.setTargetPosition(-position * adjustment);
        mtrBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (mtrFR.isBusy()) {

        }

        brake();
    }

    public void powerTurn(double power) {
        mtrBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBR.setPower(power);
        mtrBL.setPower(power);
        mtrFR.setPower(-power);
        mtrFL.setPower(-power);
    }

    public void movearm(double power, int position) {
        int adjustment = 1;
        mtrArm.setPower(-power);
        mtrArm.setTargetPosition(-position * adjustment);
        //mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void moveturret(double power, int position) {
        int adjustment = 1;
        mtrTurret.setPower(power);
        mtrTurret.setTargetPosition(position * adjustment);
        //mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void deinit() {
        mtrArm.setPower(-0.7);
        mtrArm.setTargetPosition(-var.deinitArm);
        mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (mtrArm.isBusy()) {

        }
        mtrArm.setPower(0);
        deinitWait.reset();
        //svoIntakeTilt.setPosition(var.intakeTiltMid);
        while (deinitWait.seconds() < 0.25) {

        }
        //svoIntakeTilt.setPosition(var.intakeTiltCollect);
    }
    public void armToPosition (double runningOpMode){
        mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (runningOpMode == 3) {
            movearm(0.7, var.thirdLvl);
        } else if (runningOpMode == 2) {
            movearm(0.7, var.secondLvl);
        } else if (runningOpMode == 1) {
            movearm(0.7, var.firstLvl);
        }
        mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
