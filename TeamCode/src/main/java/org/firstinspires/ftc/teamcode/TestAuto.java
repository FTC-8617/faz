package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.WebcamExample;
import org.openftc.easyopencv.OpenCvWebcam;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous(name = "TestAuto", preselectTeleOp = "MECHANUM")
public class TestAuto extends LinearOpMode
{
    private DcMotor BackrightAsDcMotor = null;
    private DcMotor FrontrightAsDcMotor = null;
    private DcMotor FrontleftAsDcMotor = null;
    private DcMotor BackleftAsDcMotor = null;
    private CRServo leftServo = null;
    private CRServo rightServo = null;
    private DcMotor elevator = null;

    @Override
    public void runOpMode() {
        BackrightAsDcMotor = hardwareMap.get(DcMotor.class, "back right");
        BackleftAsDcMotor = hardwareMap.get(DcMotor.class, "back left");
        FrontrightAsDcMotor = hardwareMap.get(DcMotor.class, "front right");
        FrontleftAsDcMotor = hardwareMap.get(DcMotor.class, "front left");
        leftServo = hardwareMap.crservo.get("left");
        rightServo = hardwareMap.crservo.get("right");
        elevator = hardwareMap.dcMotor.get("elevator");

        leftServo.setDirection(CRServo.Direction.FORWARD);
        rightServo.setDirection(CRServo.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()) {
            while (!isStopRequested()) {
                FrontrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                FrontrightAsDcMotor.setTargetPosition(100);
                FrontrightAsDcMotor.setPower(0.1);
                FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(3000);
            }
        }
    }
    /*private void elevatorUp(int distance, double power){
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elevator.setDirection(DcMotorSimple.Direction.REVERSE);

        elevator.setTargetPosition(distance);

        elevator.setPower(power);

        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void elevatorDown(int distance, double power){
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elevator.setDirection(DcMotorSimple.Direction.FORWARD);

        elevator.setTargetPosition(distance);

        elevator.setPower(power);

        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void drop(double power){
        rightServo.setPower(power);
        leftServo.setPower(power);
    }
    private void stopServos(){
        rightServo.setPower(0);
        leftServo.setPower(0);
    }
    private void in(double power){
        rightServo.setPower(-power);
        leftServo.setPower(-power);
    }
    private void crabRight(int distance, double power){
        BackrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontleftAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackleftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontrightAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackrightAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        BackrightAsDcMotor.setTargetPosition(distance);
        BackleftAsDcMotor.setTargetPosition(distance);
        FrontrightAsDcMotor.setTargetPosition(distance);
        FrontleftAsDcMotor.setTargetPosition(distance);

        BackrightAsDcMotor.setPower(power);
        BackleftAsDcMotor.setPower(power);
        FrontrightAsDcMotor.setPower(power);
        FrontleftAsDcMotor.setPower(power);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void crabLeft(int distance, double power){
        BackrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontleftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackleftAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontrightAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackrightAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        BackrightAsDcMotor.setTargetPosition(distance);
        BackleftAsDcMotor.setTargetPosition(distance);
        FrontrightAsDcMotor.setTargetPosition(distance);
        FrontleftAsDcMotor.setTargetPosition(distance);

        BackrightAsDcMotor.setPower(power);
        BackleftAsDcMotor.setPower(power);
        FrontrightAsDcMotor.setPower(power);
        FrontleftAsDcMotor.setPower(power);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void turnleft(int distance, double power) {
        BackrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontleftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackleftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontrightAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackrightAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontleftAsDcMotor.setTargetPosition(distance);
        BackleftAsDcMotor.setTargetPosition(distance);
        FrontrightAsDcMotor.setTargetPosition(distance);
        BackrightAsDcMotor.setTargetPosition(distance);

        FrontrightAsDcMotor.setPower(power);
        BackrightAsDcMotor.setPower(power);
        BackleftAsDcMotor.setPower(power);
        FrontleftAsDcMotor.setPower(power);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }*/
    private void turnright(int distance, double power) {
        BackrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontleftAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackleftAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontrightAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackrightAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        FrontleftAsDcMotor.setTargetPosition(distance);
        BackleftAsDcMotor.setTargetPosition(distance);
        FrontrightAsDcMotor.setTargetPosition(distance);
        BackrightAsDcMotor.setTargetPosition(distance);

        FrontrightAsDcMotor.setPower(power);
        BackrightAsDcMotor.setPower(power);
        BackleftAsDcMotor.setPower(power);
        FrontleftAsDcMotor.setPower(power);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }/*
    private void forward(int distance, double power) {
        BackrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontleftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackleftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontrightAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackrightAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        BackrightAsDcMotor.setTargetPosition(distance);
        BackleftAsDcMotor.setTargetPosition(distance);
        FrontrightAsDcMotor.setTargetPosition(distance);
        FrontleftAsDcMotor.setTargetPosition(distance);

        BackrightAsDcMotor.setPower(power);
        BackleftAsDcMotor.setPower(power);
        FrontrightAsDcMotor.setPower(power);
        FrontleftAsDcMotor.setPower(power);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void backward(int distance, double power) {
        BackrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontrightAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontleftAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackrightAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackleftAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        BackrightAsDcMotor.setTargetPosition(distance);
        BackleftAsDcMotor.setTargetPosition(distance);
        FrontrightAsDcMotor.setTargetPosition(distance);
        FrontleftAsDcMotor.setTargetPosition(distance);

        BackrightAsDcMotor.setPower(power);
        BackleftAsDcMotor.setPower(power);
        FrontrightAsDcMotor.setPower(power);
        FrontleftAsDcMotor.setPower(power);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void driveMotorStop(){
        BackrightAsDcMotor.setPower(0);
        BackleftAsDcMotor.setPower(0);
        FrontrightAsDcMotor.setPower(0);
        FrontleftAsDcMotor.setPower(0);
    }*/
}


