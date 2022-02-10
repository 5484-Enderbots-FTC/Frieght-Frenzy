package org.firstinspires.ftc.teamcode.frieght_frenzy_code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.FFMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.FFMecanumDriveCancelable;

@Disabled
@Autonomous(name = "testTurretEncoder", group = "auto")
public class testTurretEncoder extends LinearOpMode {
    hardwareFF robot = new hardwareFF();
    ElapsedTime armTime = new ElapsedTime();

    public void runOpMode() {
        robot.init(hardwareMap);
        FFMecanumDriveCancelable drive = new FFMecanumDriveCancelable(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        Trajectory traj = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(35, 0))
                .build();

        robot.svoIntakeTilt.setPosition(var.intakeCollect);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            drive.followTrajectoryAsync(traj);
            robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.movearm(0.7, var.thirdLvl);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(robot.mtrArm.getCurrentPosition() >= -1000){
            telemetry.addData("haha", robot.mtrArm.getCurrentPosition());
                telemetry.addData("pose estimate: ", drive.getPoseEstimate());
            telemetry.update();
            drive.update();
            drive.updatePoseEstimate();
            }
            while (!robot.midLimit.isPressed()) {
                telemetry.addData("pose estimate: ", drive.getPoseEstimate());
                telemetry.update();
                robot.mtrTurret.setPower(-0.3);
                drive.update();
                drive.updatePoseEstimate();
            }
            robot.mtrTurret.setPower(0);
            while (robot.mtrArm.isBusy()) {
                telemetry.addLine("weeeee arm finish");
                telemetry.addData("pose estimate: ", drive.getPoseEstimate());
                drive.update();
                drive.updatePoseEstimate();
            }
            robot.mtrArm.setPower(0);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while(drive.isBusy()){
                telemetry.addLine("drive is busy");
                telemetry.addData("pose estimate: ", drive.getPoseEstimate());
                telemetry.update();
                drive.update();
                drive.updatePoseEstimate();
            }
            Trajectory trajFinish = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(35, 0))
                    .build();

            drive.followTrajectory(trajFinish);

            /*
            robot.mtrTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.mtrTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.moveturret(0.3, 1450);
            robot.mtrTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (robot.mtrTurret.isBusy()) {
                telemetry.addLine("turret go brrrrr");
                telemetry.update();
            }
            robot.mtrTurret.setPower(0);
            robot.mtrTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

             */
            telemetry.update();
            break;
        }

    }
}
