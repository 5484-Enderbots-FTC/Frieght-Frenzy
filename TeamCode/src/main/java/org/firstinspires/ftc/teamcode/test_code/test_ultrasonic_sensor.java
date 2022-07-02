package org.firstinspires.ftc.teamcode.test_code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ultimate_goal_code.hardwareUltimateGoal;
import org.firstinspires.ftc.teamcode.ultimate_goal_code.var;

//@Disabled
@TeleOp(name = "ultrasonic babey", group = "testing")

public class test_ultrasonic_sensor extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();

    hardwareUltimateGoal robot = new hardwareUltimateGoal();
    AnalogInput rangeSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        //robot.init(hardwareMap);
        //robot.initShooterPID(hardwareMap);
//hewwo
        /**
         * - I2C port
         * - needs level shifter
         * - ultrasonic used for accurate far distances up to 255cm
         * - switches to optical distance sensor for distances accurate between 1cm and 7cm
         */

        rangeSensor = hardwareMap.get(AnalogInput.class, "rangeSensor");

        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            telemetry.addData("raw ultrasonic", rangeSensor.getVoltage());
            telemetry.addData("real distance", (rangeSensor.getVoltage() - .6050) / .0175);

            telemetry.update();
        }

    }

}


















