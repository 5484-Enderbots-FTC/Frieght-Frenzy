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

package org.firstinspires.ftc.teamcode.frieght_frenzy_code.auto.experimental_autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.FFMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.FFMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.ElementAnalysisPipelineFF;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.autoTrajectories;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.hardwareFF;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.var;

import java.util.ArrayList;

@Autonomous(name = "red warehouse front", group = "red")
public class AutoRedWarehouseFrontWTurretMoveInitial extends LinearOpMode {
    hardwareFF robot = new hardwareFF();
    autoTrajectories traj = new autoTrajectories();

    double runningOpMode = 3;
    Pose2d intakeEnd;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.initWebcam();
        FFMecanumDriveCancelable drive = new FFMecanumDriveCancelable(hardwareMap);

        drive.setPoseEstimate(traj.startPoseRW);

        Trajectory toRedHub3 = drive.trajectoryBuilder(traj.startPoseRW, true)
                .splineToConstantHeading(traj.redHub3, Math.toRadians(90))
                .build();

        Trajectory toRedHub2 = drive.trajectoryBuilder(traj.startPoseRW, true)
                .splineToConstantHeading(traj.redHub2, Math.toRadians(90))
                .build();

        Trajectory toRedHub1 = drive.trajectoryBuilder(traj.startPoseRW, true)
                .splineToConstantHeading(traj.redHub1, Math.toRadians(90))
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

        Trajectory goCollect = drive.trajectoryBuilder(toPark2.end())
                .forward(25, FFMecanumDrive.getVelocityConstraint(2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), FFMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        // Tell telemetry to update faster than the default 250ms period :)
        telemetry.setMsTransmissionInterval(20);
        robot.svoIntakeTilt.setPosition(var.intakeInit);
        sleep(5000);
        while (!isStarted()) {
            //what did u detect
            ArrayList<ElementAnalysisPipelineFF.AnalyzedElement> elements = robot.pipeline.getDetectedElements();
            sleep(250);

            if (elements.isEmpty()) {
                telemetry.addLine("No objects detected");
            } else {
                for (ElementAnalysisPipelineFF.AnalyzedElement element : elements) {
                    //telemetry.addLine(String.format("%s: Width=%f, Height=%f, Angle=%f", element.object.toString(), element.rectWidth, element.rectHeight, element.angle));
                    telemetry.addLine("Ratio of W/H: " + element.rectWidth / element.rectHeight);
                    telemetry.addLine("Section: " + element.section);
                    telemetry.addData("OpMode: ", runningOpMode);
                    if (element.section == ElementAnalysisPipelineFF.Section.LEFT) {
                        runningOpMode = 1;
                    } else if (element.section == ElementAnalysisPipelineFF.Section.MID) {
                        runningOpMode = 2;
                    } else if (element.section == ElementAnalysisPipelineFF.Section.RIGHT) {
                        runningOpMode = 3;
                    }

                }
            }
            telemetry.update();
        }

        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            /**
             * move arm and turret at same time
             */
            robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (runningOpMode == 3) {
                robot.movearm(var.armInitPower, var.thirdLvl);
            } else if (runningOpMode == 2) {
                robot.movearm(var.armInitPower, var.secondLvl);
            } else if (runningOpMode == 1) {
                robot.movearm(var.armInitPower, var.firstLvl);
            }
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!robot.midLimit.isPressed()) {

                robot.mtrTurret.setPower(-0.4);
            }
            robot.mtrTurret.setPower(0);


            /**
             * go to red hub and spit out bloque
             * then go to wall
             */

            telemetry.addLine("GOOOOOOOOO");
            telemetry.update();
            if (runningOpMode == 3) {
                robot.svoIntakeTilt.setPosition(var.intakeHigh);
                drive.followTrajectory(toRedHub3);
                spitOutBlock(false);
                drive.followTrajectory(toPark1_3);
            } else if (runningOpMode == 2) {
                robot.svoIntakeTilt.setPosition(var.intakeMid);
                drive.followTrajectory(toRedHub2);
                spitOutBlock(false);
                drive.followTrajectory(toPark1_2);
            } else if (runningOpMode == 1) {
                robot.svoIntakeTilt.setPosition(var.intakeLow);
                drive.followTrajectory(toRedHub1);
                spitOutBlock(false);
                drive.followTrajectory(toPark1_1);
                robot.svoIntakeTilt.setPosition(var.intakeInit);
            }

            telemetry.addLine("1st part done");
            telemetry.update();
            robot.svoIntakeTilt.setPosition(var.intakeCollect);
            robot.mtrTurret.setPower(0.7);
            drive.update();
            while (drive.isBusy()){
                drive.update();
                if (robot.frontLimit.isPressed()){
                    robot.mtrTurret.setPower(0);
                    robot.mtrArm.setPower(0.5);
                }
                if (robot.bottomLimit.isPressed()){
                    robot.mtrArm.setPower(0);
                }
            }
            while (!robot.bottomLimit.isPressed()){
                robot.mtrArm.setPower(0.5);
            }
            robot.mtrArm.setPower(0);
            /**
             * set turret to go collect pos and arm go down
             */

            //TODO: change this to be waiting for limit siwtch >:)
            //TODO: fine tune this number (900) to optimize turret and arm go down

            /*
            robot.movearm(0.7, var.collect);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(robot.mtrArm.isBusy()){
            }

            robot.mtrArm.setPower(0);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.svoIntakeTilt.setPosition(var.intakeExtraFreight);
             */

            drive.followTrajectory(toPark2);

            /*            robot.mtrTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.movearm(0.5, 150);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (robot.mtrArm.isBusy()) {
            }
            robot.mtrArm.setPower(0);
             */

            /**
             * drive into warehouse for consumption
             */
            robot.svoIntake.setPower(var.lessPower * 1.5);
            drive.followTrajectoryAsync(goCollect);
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
             * has been consumed, now go to hub (and move arm/turret)
             */
            Trajectory goBack = drive.trajectoryBuilder(intakeEnd, true)
                    .splineToConstantHeading(traj.redHub3, Math.toRadians(90))
                    .build();
            drive.followTrajectory(goBack);

            //TODO: update later to be during trajectory on way to hub :)
            robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.movearm(var.armInitPower, var.thirdLvl);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (robot.mtrArm.getCurrentPosition() >= -1000) {
                telemetry.addData("haha", robot.mtrArm.getCurrentPosition());
                telemetry.update();
                //drive.update();
            }
            while (!robot.midLimit.isPressed()) {
                robot.mtrTurret.setPower(-0.5);
                //drive.update();
            }
            robot.mtrTurret.setPower(0);
            while (robot.mtrArm.isBusy()) {
                telemetry.addLine("weeeee arm finish");
                telemetry.update();
                //drive.update();
            }
            //drive.updatePoseEstimate();
            robot.mtrArm.setPower(0);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //TODO: make this spline correct lmao


            spitOutBlock(true);

            /**
             el parque
             */
            drive.followTrajectory(toPark1_3);
            drive.followTrajectory(toPark2);

            robot.mtrTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while (!robot.frontLimit.isPressed()) {
                robot.mtrTurret.setPower(0.7);
            }
            robot.mtrTurret.setPower(0);
            break;
        }
    }

    public void spitOutBlock(boolean warehouse_block) {
        if (warehouse_block) {
            robot.svoIntakeTilt.setPosition(var.intakeHigh);
        } else if (runningOpMode == 3) {
            robot.svoIntakeTilt.setPosition(var.intakeHigh);
        } else if (runningOpMode == 2) {
            robot.svoIntakeTilt.setPosition(var.intakeMid);
        } else if (runningOpMode == 1) {
            robot.svoIntakeTilt.setPosition(var.intakeLow);
        }
        sleep(1000);
        robot.svoIntake.setPower(-var.lessPower);
        sleep(1500);
        robot.svoIntake.setPower(0);
    }
}
