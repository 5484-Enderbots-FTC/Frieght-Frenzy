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

package org.firstinspires.ftc.teamcode.frieght_frenzy_code.auto;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.FFMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.ElementAnalysisPipelineFF;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.autoTrajectories;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.hardwareFF;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.var;

import java.util.ArrayList;

@Autonomous(name = "red carousel storage flank", group = "red")
public class AutoRedCarouselStorageFlank extends LinearOpMode {
    hardwareFF robot = new hardwareFF();
    autoTrajectories traj = new autoTrajectories();

    double runningOpMode = 3;
    ElapsedTime duckTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.initWebcam();
        FFMecanumDriveCancelable drive = new FFMecanumDriveCancelable(hardwareMap);

        drive.setPoseEstimate(traj.startPoseRC);

        Trajectory toRedCarousel = drive.trajectoryBuilder(traj.startPoseRC, true)
                .splineToConstantHeading(traj.redCarousel, Math.toRadians(180))
                .addDisplacementMarker(0.95,0, () -> {
                            robot.svoCarousel.setPower(1);
                            robot.mtrTurret.setPower(-0.25);
                        }
                )
                .build();

        Trajectory toFlank = drive.trajectoryBuilder(toRedCarousel.end(), Math.toRadians(90))
                .splineToLinearHeading(traj.toFlankRed, Math.toRadians(0))
                .build();

        Trajectory toRedHub3 = drive.trajectoryBuilder(toFlank.end())
                .lineTo(traj.redHub3Flank)
                .build();

        Trajectory toRedHub2 = drive.trajectoryBuilder(toFlank.end())
                .lineTo(traj.redHub2Flank)
                .build();

        Trajectory toRedHub1 = drive.trajectoryBuilder(toFlank.end())
                .lineTo(traj.redHub1Flank)
                .build();

        Trajectory toPark1_3 = drive.trajectoryBuilder(toRedHub3.end(), Math.toRadians(135))
                .splineToConstantHeading(traj.toParkRedStorageFlank, Math.toRadians(-90))
                .build();

        Trajectory toPark1_2 = drive.trajectoryBuilder(toRedHub2.end(), Math.toRadians(135))
                .splineToConstantHeading(traj.toParkRedStorageFlank, Math.toRadians(-90))
                .build();

        Trajectory toPark1_1 = drive.trajectoryBuilder(toRedHub1.end(), Math.toRadians(135))
                .splineToConstantHeading(traj.toParkRedStorageFlank, Math.toRadians(-90))
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

            /**
             * shmove on to carousel and spain without the a
             */
            drive.followTrajectory(toRedCarousel);
            drive.setPoseEstimate(traj.redCarouselReset);
            drive.updatePoseEstimate();
            duckTime.reset();
            while (!robot.midLimit.isPressed() | duckTime.seconds() < 3 && !isStopRequested()){
                if (robot.midLimit.isPressed()){
                    robot.mtrTurret.setPower(0);
                }
                if (duckTime.seconds() > 3){
                    robot.svoCarousel.setPower(0);
                }
            }

            robot.mtrTurret.setPower(0);
            robot.svoCarousel.setPower(0);

            /**
             * go to red hub and spit out bloque
             * then go to wall
             */
            drive.followTrajectory(toFlank);

            if (runningOpMode == 3) {
                robot.svoIntakeTilt.setPosition(var.intakeHigh);
                drive.followTrajectory(toRedHub3);
                spitOutBlock();
                drive.followTrajectory(toPark1_3);
            } else if (runningOpMode == 2) {
                robot.svoIntakeTilt.setPosition(var.intakeMid);
                drive.followTrajectory(toRedHub2);
                spitOutBlock();
                drive.followTrajectory(toPark1_2);
            } else if (runningOpMode == 1) {
                robot.svoIntakeTilt.setPosition(var.intakeLow);
                drive.followTrajectory(toRedHub1);
                spitOutBlock();
                drive.followTrajectory(toPark1_1);
            }

            robot.svoIntakeTilt.setPosition(var.intakeInit);

            /**
             * set turret to go collect pos and arm go down
             */
            robot.mtrArm.setPower(0);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while (!robot.bottomLimit.isPressed()) {
                robot.mtrArm.setPower(0.3);
            }
            robot.mtrArm.setPower(0);
            break;
        }
    }

    public void spitOutBlock() {
        if (runningOpMode == 3) {
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
