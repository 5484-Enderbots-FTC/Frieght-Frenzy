package org.firstinspires.ftc.teamcode.frieght_frenzy_code;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.FFMecanumDriveCancelable;

@Disabled
@Autonomous(name = "armturretduring traj", group = "auto")
public class armTurretMoveInTraj extends LinearOpMode {
    hardwareFF robot = new hardwareFF();
    ElapsedTime armTime = new ElapsedTime();
    autoTrajectories traj = new autoTrajectories();

    public void runOpMode() {
        robot.init(hardwareMap);
        FFMecanumDriveCancelable drive = new FFMecanumDriveCancelable(hardwareMap);

        drive.setPoseEstimate(traj.startPoseRC);

        Trajectory toRedCarousel = drive.trajectoryBuilder(traj.startPoseRC, true)
                .splineToConstantHeading(new Vector2d(-63, -58), Math.toRadians(180))
                .build();

        robot.svoIntakeTilt.setPosition(var.intakeInit);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            drive.followTrajectoryAsync(toRedCarousel);
            robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.movearm(0.7, var.thirdLvl);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!robot.midLimit.isPressed()) {
                telemetry.addData("pose estimate: ", drive.getPoseEstimate());
                telemetry.update();
                robot.mtrTurret.setPower(-0.4);
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

            drive.updatePoseEstimate();
            Trajectory toRedCarouselFinish = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                    .splineToConstantHeading(new Vector2d(-63, -58), Math.toRadians(180))
                    .build();

            drive.followTrajectory(toRedCarouselFinish);

            telemetry.update();
            break;
        }

    }
}
