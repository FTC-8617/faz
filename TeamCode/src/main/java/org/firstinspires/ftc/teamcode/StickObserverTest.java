package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.StickObserverPipeline;

@Autonomous(name = "StickObserverTest")

public class StickObserverTest extends LinearOpMode {

    private DcMotor BackrightAsDcMotor = null;
    private DcMotor FrontrightAsDcMotor = null;
    private DcMotor FrontleftAsDcMotor = null;
    private DcMotor BackleftAsDcMotor = null;
    private CRServo leftServo = null;
    private CRServo rightServo = null;
    private DcMotor elevator = null;


    int move;

    @Override
    public void runOpMode() {
//        initialize camera and pipeline
        CVMaster cv = new CVMaster(this);
        BackrightAsDcMotor = hardwareMap.get(DcMotor.class, "back right");
        BackleftAsDcMotor = hardwareMap.get(DcMotor.class, "back left");
        FrontrightAsDcMotor = hardwareMap.get(DcMotor.class, "front right");
        FrontleftAsDcMotor = hardwareMap.get(DcMotor.class, "front left");
        leftServo = hardwareMap.crservo.get("left");
        rightServo = hardwareMap.crservo.get("right");
        elevator = hardwareMap.dcMotor.get("elevator");

        leftServo.setDirection(CRServo.Direction.FORWARD);
        rightServo.setDirection(CRServo.Direction.REVERSE);

//      call the function to startStreaming

        //move = cv.observeStick();


        waitForStart();
        while (opModeIsActive()) {
            if(move==-1){
                turnleft(1,0.1);

            }
            else if(move==1){
                turnright(1,0.1);

            }
            else if(move==0) {
                telemetry.addLine("GOOD TO GO");
                telemetry.update();
            }
            else{
                telemetry.addLine("BIG ERROR. THIS IS SO BAD");
                telemetry.update();
            }



        }
//        stopStreaming
        cv.stopCamera();
    }
    private void elevatorUp(int distance, double power){
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

    private void in(double power){
        rightServo.setPower(-power);
        leftServo.setPower(-power);
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

    private void crabRight(int distance, double power){
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
    private void turnright(int distance, double power) {
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
    }

    /**
     * Describe this function...
     */
    private void turnleft(int distance, double power) {
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
    }

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

    /**
     * Describe this function...
     */
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
}

