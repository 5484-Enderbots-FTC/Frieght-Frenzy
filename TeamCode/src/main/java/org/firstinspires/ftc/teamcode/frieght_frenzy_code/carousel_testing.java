package org.firstinspires.ftc.teamcode.frieght_frenzy_code;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@TeleOp(name = "carousel test", group = "teleop")
public class carousel_testing extends LinearOpMode {

    DcMotorEx mtrCarousel;
    VoltageSensor batteryVoltageSensor;

    private static double stop = 0;
    private static double largeIncrement = 100;
    private static double smallIncrement = 20;
    private double currentVelocity = 0;

    private static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0, 0);

    public void runOpMode() {

        //VelocityPID
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        //PID motor config
        mtrCarousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorConfigurationType motorConfigurationType = mtrCarousel.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        mtrCarousel.setMotorType(motorConfigurationType);

        setPIDFCoefficients(mtrCarousel, MOTOR_VELO_PID);

        //init motor and battery
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        mtrCarousel = hardwareMap.get(DcMotorEx.class, "mtrBL");
        mtrCarousel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrCarousel.setDirection(DcMotorEx.Direction.FORWARD);
        mtrCarousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        currentVelocity = stop;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            //actual controls lmao
            mtrCarousel.setVelocity(currentVelocity);

            if(gamepad1.b){
                currentVelocity = stop;
            }
            if(gamepad1.dpad_up){
                currentVelocity += largeIncrement;
            }
            if(gamepad1.dpad_down){
                currentVelocity -= largeIncrement;
            }
            if(gamepad1.right_bumper){
                currentVelocity += smallIncrement;
            }
            if(gamepad1.left_bumper){
                currentVelocity -= smallIncrement;
            }

            telemetry.addData("current motor velocity: ", mtrCarousel.getVelocity());
            telemetry.addData("current velocity variable: ", currentVelocity);
            telemetry.update();
        }
    }
    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        ));
    }
}
