/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.frieght_frenzy_code;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.FFMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.FFMecanumDriveCancelable;

@Autonomous(name = "autoRedCarousel no webcam w freight")
public class AutoRedCarouselNoWebWFreight extends LinearOpMode {
    hardwareFF robot = new hardwareFF();
    autoTrajectories traj = new autoTrajectories();

    ElapsedTime duckTimer = new ElapsedTime();

    double runningOpMode = 3;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        /**
         * This uses the cancelable drive 0.0
         */
        FFMecanumDriveCancelable drive = new FFMecanumDriveCancelable(hardwareMap);

        drive.setPoseEstimate(traj.startPoseRC);

        Trajectory toRedCarousel = drive.trajectoryBuilder(traj.startPoseRC, true)
                .splineToConstantHeading(new Vector2d(-63, -58), Math.toRadians(180))
                /*
                .addTemporalMarker(0.9, 0, () -> {
                    robot.svoCarousel.setPower(1);
                    duckTimer.reset();
                })

                 */
                .build();

        Trajectory toRedHub3 = drive.trajectoryBuilder(toRedCarousel.end())
                .splineTo(new Vector2d(-12, -47), Math.toRadians(0))
                .build();

        Trajectory toRedHub2 = drive.trajectoryBuilder(toRedCarousel.end())
                .splineTo(new Vector2d(-12, -52), Math.toRadians(0))
                .build();

        Trajectory toRedHub1 = drive.trajectoryBuilder(toRedCarousel.end())
                .splineTo(new Vector2d(-12, -50), Math.toRadians(0))
                .build();

        Trajectory toPark1_3 = drive.trajectoryBuilder(toRedHub3.end())
                .lineTo(traj.toParkRedPos1)
                .build();
        Trajectory toPark1_2 = drive.trajectoryBuilder(toRedHub2.end())
                .lineTo(traj.toParkRedPos1)
                .build();
        Trajectory toPark1_1 = drive.trajectoryBuilder(toRedHub1.end())
                .lineTo(traj.toParkRedPos1)
                .build();

        Trajectory toPark2 = drive.trajectoryBuilder(toPark1_3.end())
                .lineTo(traj.toParkRedPos2)
                .build();

        Trajectory intakeForward = drive.trajectoryBuilder(toPark2.end())
                .forward(20,  FFMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        FFMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0, () -> {robot.svoIntake.setPower(var.intakeCollect);})
                .build();

        //TODO: change the starting pose to be changeable??
        Trajectory backToRedHub3 = drive.trajectoryBuilder(intakeForward.end())
                .splineToConstantHeading(new Vector2d(-12, -47), Math.toRadians(90))
                .build();

        telemetry.addLine("Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            /**
             * move turret after start & head to carousel B)
             */
            while (!robot.midLimit.isPressed()) {
                robot.mtrTurret.setPower(-0.4);
            }
            robot.mtrTurret.setPower(0);
            drive.followTrajectory(toRedCarousel);

            /**
             * spin carousel & move arm at the same time :)
             */
            robot.svoCarousel.setPower(1);
            robot.armToPosition(runningOpMode);
            duckTimer.reset();
            while(duckTimer.seconds() <= var.duckTime && robot.mtrArm.isBusy()){
                telemetry.addData("is servo running? ", robot.svoCarousel.getPower());
                telemetry.update();
            }
            robot.svoCarousel.setPower(0);
            robot.mtrArm.setPower(0);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            /**
             * drive to red hub & spit out block then park :3
             */
            if (runningOpMode == 3) {
                robot.svoIntakeTilt.setPosition(var.intakeHigh);
            } else if (runningOpMode == 2) {
                robot.svoIntakeTilt.setPosition(var.intakeMid);
            } else if (runningOpMode == 1) {
                robot.svoIntakeTilt.setPosition(var.intakeLow);
            }
            if (runningOpMode == 3) {
                drive.followTrajectory(toRedHub3);
                spitOutBlock();
                drive.followTrajectory(toPark1_3);
            } else if (runningOpMode == 2) {
                drive.followTrajectory(toRedHub2);
                spitOutBlock();
                drive.followTrajectory(toPark1_2);
            } else if (runningOpMode == 1) {
                drive.followTrajectory(toRedHub1);
                spitOutBlock();
                drive.followTrajectory(toPark1_1);
            }

            //TODO: utilize left/right limits to start moving the turret perhaps while it's in the park1 trajectory
            //      then move the arm down until it hits bottom limit
            //      THEN go grab more freight
            robot.svoIntakeTilt.setPosition(var.intakeHigh);
            drive.followTrajectory(toPark2);

            /**
             * Extra freight time??
             */
            break;
        }
    }

    public void spitOutBlock() {
        robot.svoIntake.setPower(-var.lessPower);
        sleep(1500);
        robot.svoIntake.setPower(0);
    }
}