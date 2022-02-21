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
    public Pose2d startPoseRC = new Pose2d(-31, -65.75, 0);
    public Pose2d startPoseRW = new Pose2d(14, -65.75, 0);
    public Pose2d startPoseBC = new Pose2d(-41, 65.75, Math.toRadians(180));
    public Pose2d startPoseBW = new Pose2d(10, 65.75, Math.toRadians(180));

    public Vector2d redCarousel = new Vector2d(-63, -58);
    public Vector2d blueCarousel = new Vector2d(-63, 58);

    public Pose2d redCarouselReset = new Pose2d(-61,-56,0);
    public Pose2d blueCarouselReset = new Pose2d(-61,58,0);

    public Vector2d redHub3 = new Vector2d(-12, -47);
    public Vector2d redHub2 = new Vector2d(-12, -50);
    public Vector2d redHub1 = new Vector2d(-12, -48);

    public Vector2d blueHub3 = new Vector2d(-15, 47);
    public Vector2d blueHub2 = new Vector2d(-14.5, 50);
    public Vector2d blueHub1 = new Vector2d(-13, 48);

    public Vector2d toParkPos1 = new Vector2d(12, -69+2);
    public Vector2d toParkPos2 = new Vector2d(40, -69+2);

    public Vector2d toParkRedPos1 = new Vector2d(12, -69+2);
    public Vector2d toParkRedPos2 = new Vector2d(40, -69+2);

    public Vector2d toParkBluePos1 = new Vector2d(12,69-2);
    public Vector2d toParkBluePos2 = new Vector2d(40,69-2);

    public Vector2d toParkBarrierPos = new Vector2d(50, -35);
    public Vector2d toParkBarrierPosBlue = new Vector2d(50, 35);

    public Vector2d toParkBarrierPosRedHalf = new Vector2d(0, -35);
    public Vector2d toParkBarrierPosBlueHalf = new Vector2d(0, 35);

    public Pose2d toParkRedStorage = new Pose2d(-69+1, -36, Math.toRadians(-90));
    public Pose2d toParkBlueStorage = new Pose2d(-69+1, 36, Math.toRadians(-90));
    //new Pose2d(60, 36, Math.toRadians(-90));

}
