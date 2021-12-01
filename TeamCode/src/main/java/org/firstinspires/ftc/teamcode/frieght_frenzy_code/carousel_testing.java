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

    VoltageSensor batteryVoltageSensor;

    private static double stop = 0;
    private static double largeIncrement = 100;
    private static double smallIncrement = 20;
    private double currentVelocity = 0;

    private static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(12, 1.5, 5, 12.3);

    public void runOpMode() {
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        //VelocityPID
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        //PID motor config
        DcMotorEx myMotor = hardwareMap.get(DcMotorEx.class, "mtrCarousel");

        myMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorConfigurationType motorConfigurationType = myMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        myMotor.setMotorType(motorConfigurationType);
        myMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        setPIDFCoefficients(myMotor, MOTOR_VELO_PID);

        //init motor and battery



        currentVelocity = stop;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            //actual controls lmao
            myMotor.setVelocity(currentVelocity);

            if(gamepad1.b){
                currentVelocity = stop;
            }
            if(gamepad1.dpad_up){
                currentVelocity = 1200;
            }
            if(gamepad1.dpad_down){
                currentVelocity = 1000;
            }
            if(gamepad1.right_bumper){
                currentVelocity = 1600;
            }
            if(gamepad1.left_bumper){
                currentVelocity = 1400;
            }
            if(gamepad1.x){
                currentVelocity = 1800;
            }
if(gamepad1.y){
    currentVelocity = 2000;
}
if(gamepad1.a){
    currentVelocity = 2200;
}
            telemetry.addData("current motor velocity: ", myMotor.getVelocity());
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
