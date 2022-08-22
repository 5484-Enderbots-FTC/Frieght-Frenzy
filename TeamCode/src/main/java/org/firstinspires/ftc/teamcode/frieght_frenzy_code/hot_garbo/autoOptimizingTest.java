package org.firstinspires.ftc.teamcode.frieght_frenzy_code.hot_garbo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.frieght_frenzy_code.hardwareFF;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.vari;

@Disabled
@Autonomous(name = "auto optimizing woo")
public class autoOptimizingTest extends LinearOpMode {
    hardwareFF robot = new hardwareFF();
    turretState currentTurretState = turretState.NOTHING;

    ElapsedTime duckTimer = new ElapsedTime();
    double duckTime = 2.5;

    private enum turretState {
        NOTHING,
        FINDMID
    }

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();
        robot.svoCarousel.setPower(1);
        robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.movearm(0.7, vari.secondLvl);
        robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        duckTimer.reset();
        while(duckTimer.seconds() <= duckTime && robot.mtrArm.isBusy()){
            telemetry.addData("is servo running? ", robot.svoCarousel.getPower());
            telemetry.update();
        }
        robot.svoCarousel.setPower(0);
        robot.mtrArm.setPower(0);
        robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}

