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

package org.firstinspires.ftc.teamcode.frieght_frenzy_code.hot_garbo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.FFMecanumDrive;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.autoTrajectories;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.hardwareFF;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.var;

@Disabled
@Autonomous(name = "autoCarousel no webcam")
public class AutoCarouselNoWeb extends LinearOpMode {
    hardwareFF robot = new hardwareFF();
    autoTrajectories traj = new autoTrajectories();

    ElapsedTime duckTimer = new ElapsedTime();

    double runningOpMode = 1;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        FFMecanumDrive drive = new FFMecanumDrive(hardwareMap);

        /**
         * Red trajectories
         */
        drive.setPoseEstimate(traj.startPoseRC);

        Trajectory toRedCarousel = drive.trajectoryBuilder(traj.startPoseRC, true)
                .splineToConstantHeading(new Vector2d(-63, -58), Math.toRadians(180))
                //TODO: do some testing w/ temporal markers and the carousel to help get rid of that delay time
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

        /**
         * Blue trajectories
         */
        drive.setPoseEstimate(traj.startPoseBC);

        Trajectory toBlueCarousel = drive.trajectoryBuilder(traj.startPoseRC)
                .splineToConstantHeading(new Vector2d(-63, 58), Math.toRadians(180))
                .build();

        Trajectory toBlueHub3 = drive.trajectoryBuilder(toRedCarousel.end(), true)
                .splineTo(new Vector2d(-12, 47), Math.toRadians(0))
                .build();

        Trajectory toBlueHub2 = drive.trajectoryBuilder(toRedCarousel.end(), true)
                .splineTo(new Vector2d(-12, 52), Math.toRadians(0))
                .build();

        Trajectory toBlueHub1 = drive.trajectoryBuilder(toRedCarousel.end(), true)
                .splineTo(new Vector2d(-12, 50), Math.toRadians(0))
                .build();

        Trajectory toParkBlue1_3 = drive.trajectoryBuilder(toBlueHub3.end(), true)
                .lineTo(traj.toParkBluePos1)
                .build();
        Trajectory toParkBlue1_2 = drive.trajectoryBuilder(toBlueHub2.end(), true)
                .lineTo(traj.toParkBluePos1)
                .build();
        Trajectory toParkBlue1_1 = drive.trajectoryBuilder(toBlueHub1.end(), true)
                .lineTo(traj.toParkBluePos1)
                .build();

        Trajectory toParkBlue2 = drive.trajectoryBuilder(toParkBlue1_3.end(), true)
                .lineTo(traj.toParkBluePos2)
                .build();


        telemetry.addLine("Initialized");
        telemetry.update();
        waitForStart();
        /**
         * move turret after start & head to carousel B)
         */
        while (!robot.midLimit.isPressed()) {
            robot.mtrTurret.setPower(-0.4);
        }
        robot.mtrTurret.setPower(0);
        if (robot.alliance == robot.red) {
            drive.followTrajectory(toRedCarousel);

            /**
             * spin carousel & move arm at the same time :)
             */
            robot.svoCarousel.setPower(1);
        } else {
            drive.followTrajectory(toBlueCarousel);
            robot.svoCarousel.setPower(-1);
        }

        robot.armToPosition(runningOpMode);
        duckTimer.reset();
        while (duckTimer.seconds() <= var.duckTime && robot.mtrArm.isBusy()) {
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

        if (robot.alliance == robot.red) {
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
            robot.svoIntakeTilt.setPosition(var.intakeHigh);
            drive.followTrajectory(toPark2);
        } else {
            if (runningOpMode == 3) {
                drive.followTrajectory(toBlueHub3);
                spitOutBlock();
                drive.followTrajectory(toParkBlue1_3);
            } else if (runningOpMode == 2) {
                drive.followTrajectory(toBlueHub2);
                spitOutBlock();
                drive.followTrajectory(toParkBlue1_2);
            } else if (runningOpMode == 1) {
                drive.followTrajectory(toBlueHub1);
                spitOutBlock();
                drive.followTrajectory(toParkBlue1_1);
            }
            robot.svoIntakeTilt.setPosition(var.intakeHigh);
            drive.followTrajectory(toParkBlue2);
        }
    }

    public void spitOutBlock() {
        robot.svoIntake.setPower(-var.lessPower);
        sleep(1500);
        robot.svoIntake.setPower(0);
    }
}