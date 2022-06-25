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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.FFMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.FFMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.ElementAnalysisPipelineFF;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.autoTrajectories;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.hardwareFF;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.var;

import java.util.ArrayList;

@Autonomous(name = "blue warehouse front three blocks (not tested)", group = "blue")
public class AutoBlueWarehouseFrontWThirdBlock extends LinearOpMode {
    hardwareFF robot = new hardwareFF();
    autoTrajectories traj = new autoTrajectories();
    ElapsedTime spitTime = new ElapsedTime();
    ElapsedTime limitTime = new ElapsedTime();

    double runningOpMode = 3;
    Pose2d intakeEnd;
    ElapsedTime intakeTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.initWebcam();
        FFMecanumDriveCancelable drive = new FFMecanumDriveCancelable(hardwareMap);

        drive.setPoseEstimate(traj.startPoseBW);

        Trajectory toBlueHub3 = drive.trajectoryBuilder(traj.startPoseBW)
                .splineToConstantHeading(traj.blueHub3, Math.toRadians(180))
                .addDisplacementMarker(0.95, 0, () -> {
                    robot.svoIntake.setPower(-var.lessPower);
                    spitTime.reset();
                })
                .build();

        Trajectory toBlueHub2 = drive.trajectoryBuilder(traj.startPoseBW)
                .splineToConstantHeading(traj.blueHub2, Math.toRadians(180))
                .addDisplacementMarker(0.95, 0, () -> {
                    robot.svoIntake.setPower(-var.lessPower);
                    spitTime.reset();
                })
                .build();

        Trajectory toBlueHub1 = drive.trajectoryBuilder(traj.startPoseBW)
                .splineToConstantHeading(traj.blueHub1, Math.toRadians(180))
                .addDisplacementMarker(0.95, 0, () -> {
                    robot.svoIntake.setPower(-var.lessPower);
                    spitTime.reset();
                })
                .build();

        Trajectory toPark1_3 = drive.trajectoryBuilder(toBlueHub3.end(), true)
                .lineTo(traj.toParkBluePos1)
                .build();
        Trajectory toPark1_2 = drive.trajectoryBuilder(toBlueHub2.end(), true)
                .lineTo(traj.toParkBluePos1)
                .build();
        Trajectory toPark1_1 = drive.trajectoryBuilder(toBlueHub1.end(), true)
                .lineTo(traj.toParkBluePos1)
                .build();
        Trajectory toCollect = drive.trajectoryBuilder(toPark1_3.end(), true)
                .lineTo(traj.toParkBluePos2)
                .build();

        Trajectory goCollect = drive.trajectoryBuilder(toCollect.end(), true)
                .forward(-25, FFMecanumDrive.getVelocityConstraint(3.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), FFMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory toPark2 = drive.trajectoryBuilder(toPark1_3.end(), true)
                .lineTo(traj.toParkBluePosWarehouseEnd)
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
            if (runningOpMode == 3) {
                robot.svoIntakeTilt.setPosition(var.intakeHighInitial);
                drive.followTrajectory(toBlueHub3);
                while (spitTime.seconds() < 1.5) {

                }
                robot.svoIntake.setPower(0);
                var.intakeCollectHalfway = ((var.intakeHigh - var.intakeCollect) / 2) + var.intakeCollect;
                drive.followTrajectoryAsync(toPark1_3);
            } else if (runningOpMode == 2) {
                robot.svoIntakeTilt.setPosition(var.intakeMid);
                drive.followTrajectory(toBlueHub2);
                while (spitTime.seconds() < 1.5) {

                }
                robot.svoIntake.setPower(0);
                var.intakeCollectHalfway = ((var.intakeMid - var.intakeCollect) / 2) + var.intakeCollect;
                drive.followTrajectoryAsync(toPark1_2);
            } else if (runningOpMode == 1) {
                robot.svoIntakeTilt.setPosition(var.intakeLow);
                drive.followTrajectory(toBlueHub1);
                while (spitTime.seconds() < 1.5) {

                }
                robot.svoIntake.setPower(0);
                drive.followTrajectoryAsync(toPark1_1);
            }
            robot.svoIntakeTilt.setPosition(var.intakeInit);

            /**
             * set turret to go collect pos and arm go down
             */

            robot.mtrTurret.setPower(-0.7);
            drive.update();
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.mtrArm.setPower(0);
            while (drive.isBusy()) {
                drive.update();
                telemetry.update();
                if (robot.backLimit.isPressed()) {
                    robot.mtrTurret.setPower(0);
                    robot.mtrArm.setPower(0.65);
                    telemetry.addLine("front limit pressed");
                }
                if (robot.bottomLimit.isPressed()) {
                    robot.mtrArm.setPower(0);
                    telemetry.addLine("arm down all the way");
                }
            }
            telemetry.addLine("drive done");
            telemetry.update();
            while (!robot.bottomLimit.isPressed()) {
                robot.mtrArm.setPower(0.65);
                telemetry.addLine("arm moving");
                telemetry.update();
            }
            telemetry.addLine("arm down after move");
            telemetry.update();
            robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.mtrArm.setPower(0);
            robot.svoIntakeTilt.setPosition(var.intakeCollect);
            telemetry.addLine("optimize done");
            telemetry.update();

            drive.followTrajectory(toCollect);
            /**
             * drive into warehouse for consumption
             */
            robot.mtrArm.setPower(var.holdDownArmPower);
            robot.svoIntake.setPower(var.almostFullPower);
            drive.followTrajectoryAsync(goCollect);
            intakeTime.reset();
            while (robot.intakeLimit.isPressed() && intakeTime.seconds() <= var.intakeStopTime) {
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

            robot.svoIntake.setPower(0.1);
            sleep(500);
            robot.svoIntake.setPower(0.03);

            /**
             * has been consumed, now go to hub (and move arm/turret)
             */
            Trajectory goBack = drive.trajectoryBuilder(intakeEnd)
                    .splineToConstantHeading(traj.blueHub3, Math.toRadians(-90))
                    .addDisplacementMarker(0.2, 0, () -> {
                        robot.mtrTurret.setPower(0.4);
                    })
                    .addDisplacementMarker(0.96, 0, () -> {
                        robot.svoIntakeTilt.setPosition(var.intakeHigh);
                        robot.svoIntake.setPower(-var.lessPower);
                        spitTime.reset();
                    })
                    .build();
            robot.movearm(var.armInitPower, var.thirdLvl);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.followTrajectoryAsync(goBack);
            drive.update();

            while (drive.isBusy() | !robot.midLimit.isPressed()) {
                drive.update();
                if (robot.midLimit.isPressed()) {
                    robot.mtrTurret.setPower(0);
                    telemetry.addLine("midlimit hit");
                }
            }
            robot.mtrTurret.setPower(0);
            while (spitTime.seconds() < 2){

            }
            robot.svoIntake.setPower(0);
            /**
             el parque
             */
            drive.followTrajectoryAsync(toPark1_3);





            robot.svoIntakeTilt.setPosition(var.intakeInit);

            /**
             * set turret to go collect pos and arm go down
             */

            robot.mtrTurret.setPower(-0.7);
            drive.update();
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.mtrArm.setPower(0);
            while (drive.isBusy()) {
                drive.update();
                telemetry.update();
                if (robot.backLimit.isPressed()) {
                    robot.mtrTurret.setPower(0);
                    robot.mtrArm.setPower(0.65);
                    telemetry.addLine("front limit pressed");
                }
                if (robot.bottomLimit.isPressed()) {
                    robot.mtrArm.setPower(0);
                    telemetry.addLine("arm down all the way");
                }
            }
            telemetry.addLine("drive done");
            telemetry.update();
            while (!robot.bottomLimit.isPressed()) {
                robot.mtrArm.setPower(0.65);
                telemetry.addLine("arm moving");
                telemetry.update();
            }
            telemetry.addLine("arm down after move");
            telemetry.update();
            robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.mtrArm.setPower(0);
            robot.svoIntakeTilt.setPosition(var.intakeCollect);
            telemetry.addLine("optimize done");
            telemetry.update();

            drive.followTrajectory(toCollect);
            /**
             * drive into warehouse for consumption
             */
            robot.mtrArm.setPower(var.holdDownArmPower);
            robot.svoIntake.setPower(var.almostFullPower);
            drive.followTrajectoryAsync(goCollect);
            intakeTime.reset();
            while (robot.intakeLimit.isPressed() && intakeTime.seconds() <= var.intakeStopTime) {
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

            robot.svoIntake.setPower(0.1);
            sleep(500);
            robot.svoIntake.setPower(0.03);

            /**
             * has been consumed, now go to hub (and move arm/turret)
             */
            Trajectory goBack2 = drive.trajectoryBuilder(intakeEnd)
                    .splineToConstantHeading(traj.blueHub3Displaced, Math.toRadians(-90))
                    .addDisplacementMarker(0.2, 0, () -> {
                        robot.mtrTurret.setPower(0.4);
                    })
                    .addDisplacementMarker(0.96, 0, () -> {
                        robot.svoIntakeTilt.setPosition(var.intakeHigh);
                        robot.svoIntake.setPower(-var.lessPower);
                        spitTime.reset();
                    })
                    .build();
            robot.movearm(var.armInitPower, var.thirdLvl);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.followTrajectoryAsync(goBack2);
            drive.update();

            while (drive.isBusy() | !robot.midLimit.isPressed()) {
                drive.update();
                if (robot.midLimit.isPressed()) {
                    robot.mtrTurret.setPower(0);
                    telemetry.addLine("midlimit hit");
                }
            }
            robot.mtrTurret.setPower(0);
            while (spitTime.seconds() < 2){

            }
            robot.svoIntake.setPower(0);
            /**
             el parque
             */
            drive.followTrajectory(toPark1_3);
            drive.followTrajectory(toPark2);

            robot.mtrTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.mtrTurret.setPower(-0.7);
            robot.mtrTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            limitTime.reset();
            while (!robot.backLimit.isPressed() | limitTime.seconds() < 0.75) {

            }
            robot.mtrTurret.setPower(0);

            drive.cancelFollowing();
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

