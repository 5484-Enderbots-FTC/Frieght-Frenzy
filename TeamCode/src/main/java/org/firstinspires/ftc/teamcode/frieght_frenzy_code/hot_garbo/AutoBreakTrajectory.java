package org.firstinspires.ftc.teamcode.frieght_frenzy_code.hot_garbo;

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
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.hardwareFF;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.var;

@Disabled
@Autonomous(group = "advanced")
public class AutoBreakTrajectory extends LinearOpMode {
    hardwareFF robot = new hardwareFF();

    Pose2d intakeEnd = new Pose2d();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        FFMecanumDriveCancelable drive = new FFMecanumDriveCancelable(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d startPose2 = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(150, 0), FFMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), FFMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(startPose2)
                .lineToConstantHeading(new Vector2d(150, 0), FFMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), FFMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        /**
         * drive forward for consumption
         */
        robot.svoIntake.setPower(var.lessPower);
        drive.followTrajectoryAsync(traj);
        while (robot.intakeLimit.isPressed()) {
            telemetry.addLine("consuming");
            telemetry.update();
            drive.update();
            drive.updatePoseEstimate();
        }
        drive.cancelFollowing();
        intakeEnd = drive.getPoseEstimate();
        drive.setDrivePower(new Pose2d());
        drive.update();
        telemetry.addLine("consumed");
        telemetry.addData("intake end: ", intakeEnd);
        telemetry.update();
        robot.svoIntake.setPower(0);

        /**
         * has been consumed, now go to hub
         */
        Trajectory goBack = drive.trajectoryBuilder(intakeEnd, true)
                .splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(180))
                .build();
        drive.followTrajectory(goBack);
        //arm
        robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.movearm(0.7, var.thirdLvl);

        robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(robot.mtrArm.getCurrentPosition() >= -1000) {
            telemetry.addData("haha", robot.mtrArm.getCurrentPosition());
            telemetry.addData("pose estimate: ", drive.getPoseEstimate());
            telemetry.update();
        }
        while (!robot.midLimit.isPressed()) {
            telemetry.addData("pose estimate: ", drive.getPoseEstimate());
            telemetry.update();
            robot.mtrTurret.setPower(-0.3);
        }
        robot.mtrTurret.setPower(0);
        while (robot.mtrArm.isBusy()) {
            telemetry.addLine("weeeee arm finish");
            telemetry.addData("pose estimate: ", drive.getPoseEstimate());
        }
        robot.mtrArm.setPower(0);
        robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spitOutBlock();

        /**
         * send the turret back and arm down to collect again
         */
        robot.mtrTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.moveturret(0.3,1480);
        robot.mtrTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(100);
        while (robot.mtrTurret.isBusy()) {
            telemetry.addLine("turret go brrrrr");
            telemetry.update();
        }
        robot.mtrTurret.setPower(0);
        robot.mtrTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addLine("arm go brrrrrrrrrrrrrrrrrrrrrrrrrr");
        telemetry.update();
        robot.svoIntakeTilt.setPosition(var.intakeCollect);
        while(!robot.bottomLimit.isPressed()){
            robot.mtrArm.setPower(0.7);
        }
        robot.mtrArm.setPower(0);

        /**
         * begin consumption AGAIN
         */
        robot.svoIntake.setPower(var.lessPower);
        drive.followTrajectoryAsync(traj2);
        telemetry.addData("intake limit?", robot.intakeLimit.isPressed());
        telemetry.update();
        sleep(1000);
        while (robot.intakeLimit.isPressed()) {
            telemetry.addLine("consuming");
            telemetry.update();
            drive.update();
            drive.updatePoseEstimate();
        }
        drive.cancelFollowing();
        intakeEnd = drive.getPoseEstimate();
        drive.setDrivePower(new Pose2d());
        drive.update();
        telemetry.addLine("consumed");
        telemetry.addData("intake end: ", intakeEnd);
        telemetry.update();
        robot.svoIntake.setPower(0);
        Trajectory goBack2 = drive.trajectoryBuilder(intakeEnd, true)
                .splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(180))
                .build();
        drive.followTrajectory(goBack2);
        //arm
        robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.movearm(0.7, var.thirdLvl);

        robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(robot.mtrArm.getCurrentPosition() >= -1000){
            telemetry.addData("haha", robot.mtrArm.getCurrentPosition());
            telemetry.addData("pose estimate: ", drive.getPoseEstimate());
            telemetry.update();
        }
        while (!robot.midLimit.isPressed()) {
            telemetry.addData("pose estimate: ", drive.getPoseEstimate());
            telemetry.update();
            robot.mtrTurret.setPower(-0.3);
        }
        robot.mtrTurret.setPower(0);
        while (robot.mtrArm.isBusy()) {
            telemetry.addLine("weeeee arm finish");
            telemetry.addData("pose estimate: ", drive.getPoseEstimate());
        }
        robot.mtrArm.setPower(0);
        robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spitOutBlock();
    }

    public void spitOutBlock() {
        robot.svoIntakeTilt.setPosition(var.intakeHigh);
        sleep(1000);
        robot.svoIntake.setPower(-var.lessPower);
        sleep(1500);
        robot.svoIntake.setPower(0);
    }
}
