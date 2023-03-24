package bad_code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



@Autonomous(name = "FazLeft", preselectTeleOp = "bad_code.FazLeft")
public class FazLeft extends LinearOpMode {

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
    leftServo = hardwareMap.crservo.get("left");
    rightServo = hardwareMap.crservo.get("right");
    elevator = hardwareMap.dcMotor.get("elevator");
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    imu=hardwareMap.get(BNO055IMU.class,"imu");
    imu.initialize(parameters);

    leftServo.setDirection(CRServo.Direction.FORWARD);
    rightServo.setDirection(CRServo.Direction.REVERSE);



    waitForStart();
    if (opModeIsActive()) {
      while (!isStopRequested()) {
        /*angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading: ",angles.firstAngle);
        telemetry.addData("Roll: ",angles.secondAngle);
        telemetry.addData("Pitch: ",angles.thirdAngle);
        telemetry.update();
        in(0.5);
        elevatorUp(750, 1);
        forward(2500, 1);
        sleep(2000);
        backward(350,1);
        sleep(2000);
        centerOn(0);
        sleep(2000);
        crabLeft(720,0.3);
        sleep(1000);
        centerOn(0);
        sleep(1000);
        forward(220,1);
        elevatorUp(4250,1);
        sleep(3000);
        elevatorDown(1000,1);
        sleep(500);
        drop(1);
        sleep(1000);
        backward(300,1);
        //forward(200,1);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        dashboardTelemetry.addData("Heading: ",angles.firstAngle);
        dashboardTelemetry.update();*/

        sleep(30000);
      }
      telemetry.update();
    } ;
  }
  private void centerOn(double angle){
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double daAngle=angles.firstAngle;
    while(opModeIsActive() && (daAngle>(2) || daAngle<(-2))) {
      angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      daAngle=angles.firstAngle;
      if (daAngle > 2) {
        turnright(1, 0.3);
      } else if (daAngle < -2) {
        turnleft(1, 0.3);
      } else {
        telemetry.addLine("We got a problem");
        telemetry.update();
      }
    }
  }
  private void centerLeft(double angle){
    while(opModeIsActive() && ((((angles.firstAngle - angle) > 2) )||((angles.firstAngle -angle) < -2))) {
      angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      double daAngle=angles.firstAngle;
      if (((daAngle- angle) <= 2) && ((daAngle - angle) >= -2)) {
        telemetry.addLine("Angle all good"+(daAngle-angle));
        telemetry.update();
        break;
      }
      else if ((angles.firstAngle - angle) > 2) {
        turnright(1,0.1);
        telemetry.addLine("hitting a right"+(daAngle-angle));
        telemetry.update();
      }
      else if((angles.firstAngle -angle) < -2){
        turnleft(1,0.1);
        telemetry.addLine("hitting a left"+(daAngle-angle));
        telemetry.update();
      }
      else{
        telemetry.addLine("yo faz we got a prob dawg");
        telemetry.update();
      }

    }
    telemetry.addLine("while loop ended");
    telemetry.update();
  }
  private void centerRight(double angle){
    while(opModeIsActive() && ((((angles.firstAngle - angle) > 2) )||((angles.firstAngle -angle) < -2))) {
      angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      double daAngle=-angles.firstAngle;
      if (((daAngle-angle) <= 2) && ((daAngle - angle) >= -2)) {
        telemetry.addLine("Angle all good"+(daAngle-angle));
        telemetry.update();
        break;
      }
      else if ((angles.firstAngle - angle) > 2) {
        turnleft(1,0.1);
        telemetry.addLine("hitting a right"+(daAngle-angle));
        telemetry.update();
      }
      else if((angles.firstAngle -angle) < -2){
        turnright(1,0.1);
        telemetry.addLine("hitting a left"+(daAngle-angle));
        telemetry.update();
      }
      else{
        telemetry.addLine("yo faz we got a prob dawg");
        telemetry.update();
      }

    }
    telemetry.addLine("while loop ended");
    telemetry.update();
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
  private void stopServos(){
    rightServo.setPower(0);
    leftServo.setPower(0);
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
