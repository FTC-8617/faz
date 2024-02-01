package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name = "Blitzkrieg_BattleOfTheBulge", preselectTeleOp = "MRHIM")
public class Blitzkrieg_BattleOfBritain extends LinearOpMode {

  private DcMotor BackrightAsDcMotor = null;
  private DcMotor FrontrightAsDcMotor = null;
  private DcMotor FrontleftAsDcMotor = null;
  private DcMotor BackleftAsDcMotor = null;
  private CRServo leftServo = null;
  private CRServo rightServo = null;
  private DcMotor elevator = null;
  BNO055IMU imu;


  Orientation angles;
  
  @Override
  public void runOpMode() {
    
    BackrightAsDcMotor = hardwareMap.get(DcMotor.class, "back right");
    BackleftAsDcMotor = hardwareMap.get(DcMotor.class, "back left");
    FrontrightAsDcMotor = hardwareMap.get(DcMotor.class, "front right");
    FrontleftAsDcMotor = hardwareMap.get(DcMotor.class, "front left");
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    imu=hardwareMap.get(BNO055IMU.class,"imu");
    imu.initialize(parameters);

    leftServo.setDirection(CRServo.Direction.FORWARD);
    rightServo.setDirection(CRServo.Direction.REVERSE);



    waitForStart();
    if (opModeIsActive()) {
      while (!isStopRequested()) {
        forward(12, 1.0);
        backward(12, 1.0);
        sleep(30000);
      }
    }
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
  private void backward(int distance, double power) {
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
  private void forward(int distance, double power) {
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
