
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.FFMecanumDrive;

@Disabled
@Autonomous(name = "old autoRedCarousel no webcam")
public class AutoRedCarouselNoWebOld extends LinearOpMode {
    hardwareFF robot = new hardwareFF();
    autoTrajectories traj = new autoTrajectories();
    ElapsedTime duckTimer = new ElapsedTime();

    boolean zeroPosSet = false;
    double duckTime = 2.5;
    double runningOpMode = 3;

    turretState currentState = turretState.NOTHING;

    private enum turretState{
    NOTHING,
    SET0
    }

    Pose2d endDepPos;

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
                robot.movearm(0.7, variable.thirdLvl);
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
                robot.svoIntakeTilt.setPosition(variable.intakeHigh);
            })
            .build();

    Trajectory toRedHub2 = drive.trajectoryBuilder(toRedCarousel.end())
            .splineTo(new Vector2d(-12, -52), Math.toRadians(0))
            .addTemporalMarker(0, () -> {
                robot.svoIntakeTilt.setPosition(variable.intakeMid);
            })
            .build();

    Trajectory toRedHub1 = drive.trajectoryBuilder(toRedCarousel.end())
            .splineTo(new Vector2d(-12, -47), Math.toRadians(0))
            .addTemporalMarker(0, () -> {
                robot.svoIntakeTilt.setPosition(variable.intakeLow);
            })
            .build();

    Trajectory toPark1 = drive.trajectoryBuilder(endDepPos)
            .lineTo(traj.toParkRedPos1)
            .build();
    Trajectory toPark2 = drive.trajectoryBuilder(toPark1.end())
            .lineTo(traj.toParkRedPos2)
            .build();


    telemetry.addLine("Initialized");
    telemetry.update();
    waitForStart();
    while (opModeIsActive()) {

        robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.followTrajectory(toRedCarousel);
        /**
        robot.mtrTurret.setPower(-0.3);
        currentState = turretState.SET0;
        switch (currentState){
            case NOTHING:
                break;
            case SET0:
                if(duckTimer.seconds() < duckTime && !robot.midLimit.isPressed()){
                    if (robot.midLimit.isPressed()) {
                        robot.mtrTurret.setPower(0);
                    }
                }else{
                    robot.svoCarousel.setPower(var.stop);
                    currentState = turretState.NOTHING;
                }
                break;
        }
         */
        if (runningOpMode == 3) {
            drive.followTrajectory(toRedHub3);
            endDepPos = toRedHub3.end();
        } else if (runningOpMode == 2) {
            drive.followTrajectory(toRedHub2);
            endDepPos = toRedHub2.end();
        } else if (runningOpMode == 1) {
            drive.followTrajectory(toRedHub1);
            endDepPos = toRedHub1.end();
        }
        robot.svoIntakeTilt.setPosition(variable.intakeHigh);
        robot.svoIntake.setPower(-variable.lessPower);
        sleep(1500);
        robot.svoIntake.setPower(0);
        if (runningOpMode == 1) {
            robot.svoIntakeTilt.setPosition(0.9);
        }
        drive.followTrajectory(toPark1);
        drive.followTrajectory(toPark2);

        break;
    }
    }
} 