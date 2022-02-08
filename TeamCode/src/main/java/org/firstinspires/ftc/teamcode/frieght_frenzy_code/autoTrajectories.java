package org.firstinspires.ftc.teamcode.frieght_frenzy_code;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.FFMecanumDrive;
public class autoTrajectories {
//some var has been changed (like -67 -> -69 in park poses)
    Pose2d startPoseRC = new Pose2d(-31, -65.75);
    Pose2d startPoseRW = new Pose2d(14, -67.75);
    Pose2d startPoseBC = new Pose2d(-41, 67.75);
    Pose2d startPoseBW = new Pose2d(4, -65.75);
    Pose2d redHub3Exit = new Pose2d(-12,-47,Math.toRadians(-90));
    Pose2d redHub2Exit = new Pose2d(-12,-52,Math.toRadians(-90));
    Pose2d redHub1Exit = new Pose2d(-12,-50,Math.toRadians(-90));

    Vector2d redHub3 = new Vector2d(-12,-47);
    Vector2d redHub2 = new Vector2d(-12,-52);
    Vector2d redHub1 = new Vector2d(-12,-49);
    Vector2d blueHub3 = new Vector2d(-12,47);
    Vector2d blueHub2 = new Vector2d(-12,52);
    Vector2d blueHub1 = new Vector2d(-12,49);
    Vector2d toParkPos1 = new Vector2d(12, -69);
    Vector2d toParkPos2 = new Vector2d(40, -69);
    Vector2d toParkBarrierPos = new Vector2d(50,-35);
    Vector2d toParkBarrierPosBlue = new Vector2d(50,35);
    Vector2d toParkRedPos1 = new Vector2d(12, -69);
    Vector2d toParkRedPos2 = new Vector2d(40, -69);
    Vector2d toParkBluePos1 = new Vector2d(12, 69);
    Vector2d toParkBluePos2 = new Vector2d(40, 69);
    Pose2d toParkRedStorage = new Pose2d(-69, -36, Math.toRadians(-90));
    Vector2d toParkBlueStorage = new Vector2d(60, 36);
}
