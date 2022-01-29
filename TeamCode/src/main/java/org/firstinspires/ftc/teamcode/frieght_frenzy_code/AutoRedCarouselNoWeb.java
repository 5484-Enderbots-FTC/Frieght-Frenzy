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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.FFMecanumDrive;

@Autonomous(name = "autoRedCarousel no webcam")
public class AutoRedCarouselNoWeb extends LinearOpMode {
    hardwareFF robot = new hardwareFF();
    autoTrajectories traj = new autoTrajectories();

    ElapsedTime duckTimer = new ElapsedTime();
    ElapsedTime dispenseTimer = new ElapsedTime();

    double duckTime = 2.5;
    double dispenseTime = 1.5;

    double runningOpMode = 3;

    Pose2d endDepPos = new Pose2d(0,0,Math.toRadians(90));

    turretState currentTurretState = turretState.NOTHING;
    armState currentArmState = armState.NOTHING;

    private enum turretState {
        NOTHING,
        FINDMID
    }

    private enum armState {
        NOTHING,
        RAISE,
        DISPENSE,
        WAIT
    }

    @Override
    public void runOpMode() {
        //this will init EVERYTHING on the robot
        robot.init(hardwareMap);
        //FFMecanum must be called AFTER the robot init bc just the motors need to be overridden.
        FFMecanumDrive drive = new FFMecanumDrive(hardwareMap);

        drive.setPoseEstimate(traj.startPoseRC);

        Trajectory toRedCarousel = drive.trajectoryBuilder(traj.startPoseRC, true)
                .splineToConstantHeading(new Vector2d(-63, -58), Math.toRadians(180))
                .addTemporalMarker(0.9, 0, () -> {
                    robot.svoCarousel.setPower(1);
                    duckTimer.reset();
                })
                .addTemporalMarker(0, () -> {
                    robot.movearm(0.7, var.thirdLvl);
                    robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if (robot.mtrArm.isBusy()) {

                    } else {
                        robot.mtrArm.setPower(0);
                        robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    }

                })
                .build();

        Trajectory toRedHub3 = drive.trajectoryBuilder(toRedCarousel.end())
                .splineTo(new Vector2d(-12, -47), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    robot.svoIntakeTilt.setPosition(var.intakeHigh);
                })
                .build();

        Trajectory toRedHub2 = drive.trajectoryBuilder(toRedCarousel.end())
                .splineTo(new Vector2d(-12, -52), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    robot.svoIntakeTilt.setPosition(var.intakeMid);
                })
                .build();

        Trajectory toRedHub1 = drive.trajectoryBuilder(toRedCarousel.end())
                .splineTo(new Vector2d(-12, -47), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    robot.svoIntakeTilt.setPosition(var.intakeLow);
                })
                .build();

        Trajectory toPark1 = drive.trajectoryBuilder(endDepPos)
                .lineTo(traj.toParkPos1)
                .build();
        Trajectory toPark2 = drive.trajectoryBuilder(toPark1.end())
                .lineTo(traj.toParkPos2)
                .build();


        /***
        Trajectory toRedCarousel = drive.trajectoryBuilder(traj.startPoseRC, true)
                .splineToConstantHeading(new Vector2d(-63.5, -58), Math.toRadians(180))
                .addTemporalMarker(0.9, 0, () -> {
                    robot.svoCarousel.setPower(1);
                    duckTimer.reset();
                })
                .build();

        Trajectory toRedHub3 = drive.trajectoryBuilder(toRedCarousel.end())
                .splineToConstantHeading(new Vector2d(-12, -47), Math.toRadians(135))
                .addTemporalMarker(0, () -> {
                    robot.svoIntakeTilt.setPosition(var.intakeHigh);
                })
                .build();

        Trajectory toRedHub2 = drive.trajectoryBuilder(toRedCarousel.end())
                .splineTo(new Vector2d(-12, -52), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    robot.svoIntakeTilt.setPosition(var.intakeMid);
                })
                .build();

        Trajectory toRedHub1 = drive.trajectoryBuilder(toRedCarousel.end())
                .splineTo(new Vector2d(-12, -47), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    robot.svoIntakeTilt.setPosition(var.intakeLow);
                })
                .build();

        /**
        Trajectory toPark1 = drive.trajectoryBuilder(endDepPos)
                .splineToConstantHeading(traj.toParkPos1, Math.toRadians(-45))

        Trajectory toPark1 = drive.trajectoryBuilder(toRedCarousel.end())
                .lineTo(traj.toParkPos1)


        Trajectory toPark1 = drive.trajectoryBuilder(endDepPos)
                .splineToConstantHeading(traj.toParkPos1, Math.toRadians(-45))
                .addTemporalMarker(0.5, () -> {
                    if(runningOpMode == 1) {
                        robot.svoIntakeTilt.setPosition(0.9);
                    }
                })
                .build();
        Trajectory toPark2 = drive.trajectoryBuilder(toPark1.end())
                .lineTo(traj.toParkPos2)
                .build();
        */

        telemetry.addLine("Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.followTrajectory(toRedCarousel);
            robot.mtrTurret.setPower(-0.2);
            currentTurretState = turretState.FINDMID;
        /***
            while (duckTimer.seconds() < duckTime && !robot.midLimit.isPressed()){
                if (robot.midLimit.isPressed()){
                    robot.mtrTurret.setPower(0);
                    telemetry.addData("mid limit", robot.midLimit.isPressed());
                }
                if (duckTimer.seconds() > duckTime){
                    robot.svoCarousel.setPower(var.stop);
                }
                telemetry.update();
            }

            switch (currentTurretState) {
                case NOTHING:
                    break;
                case FINDMID:
                    telemetry.addData("duck timer", duckTimer.seconds());
                    telemetry.addData("mid limit:", robot.midLimit.isPressed());
                    telemetry.update();
                    if (duckTimer.seconds() < duckTime && !robot.midLimit.isPressed()) {
                        if (robot.midLimit.isPressed()) {
                            robot.mtrTurret.setPower(0);
                        }
                    } else {
                        robot.svoCarousel.setPower(var.stop);
                        currentTurretState = turretState.NOTHING;
                    }
                    break;
            }
             **/
            telemetry.addLine("while loop done");
            telemetry.update();
            robot.mtrTurret.setPower(0);
            runningOpMode = 3;
            if (runningOpMode == 3) {
                telemetry.addLine("Running mode 3");
                telemetry.update();
                drive.followTrajectoryAsync(toRedHub3);
                endDepPos = toRedHub3.end();
            } else if (runningOpMode == 2) {
                drive.followTrajectoryAsync(toRedHub2);
                endDepPos = toRedHub2.end();
            } else if (runningOpMode == 1) {
                drive.followTrajectoryAsync(toRedHub1);
                endDepPos = toRedHub1.end();
            }
            //robot.movearm(0.7, var.thirdLvl);
            //robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            currentArmState = armState.RAISE;

            switch (currentArmState) {
                case NOTHING:
                    break;
                case RAISE:
                    if (robot.mtrArm.isBusy()) {

                    } else {
                        robot.mtrArm.setPower(0);
                        robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        currentArmState = armState.DISPENSE;
                    }
                    break;
                case DISPENSE:
                    if (!drive.isBusy()) {
                        robot.svoIntake.setPower(-var.lessPower);
                        dispenseTimer.reset();
                        currentArmState = armState.WAIT;
                    } else {
                    }
                    break;
                case WAIT:
                    if (dispenseTimer.seconds() > dispenseTime) {
                        robot.svoIntake.setPower(0);
                        currentArmState = armState.NOTHING;
                    } else {
                    }
                    break;
            }

            drive.followTrajectory(toPark1);
            drive.followTrajectory(toPark2);
            break;
        }
    }
}
