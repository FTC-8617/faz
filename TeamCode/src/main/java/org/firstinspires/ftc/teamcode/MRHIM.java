package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="MRHIM", group="TeleOp")
public class MRHIM extends LinearOpMode {
    //importing config; CH = control hub & EH = expansion hub
    private DcMotor frontLeft = null; //CH
    private DcMotor frontRight = null; //CH
    private DcMotor backLeft = null; //CH
    private DcMotor backRight = null; //CH
    private CRServo wristServo = null; //CH
    private CRServo clawServo = null; //CH
    private CRServo slopeServo = null; //CH
    private DcMotor elevator = null; //EH
    private DcMotor intake = null; //EH
    //private DcMotor actuator = null; [EH]
    //private CRServo drone = null; [CH]
    private final double driveAdjuster = 1;
    double  elevPower = 0;
    @Override
    public void runOpMode()  throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //mapping motor/servo interfaces to driver station config names
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");
        wristServo = hardwareMap.crservo.get("wristServo");
        clawServo = hardwareMap.crservo.get("clawServo");
        slopeServo = hardwareMap.crservo.get("slopeServo");
        elevator = hardwareMap.dcMotor.get("elevator");
        intake = hardwareMap.dcMotor.get("intake");
        //actuator = hardwareMap.dcMotor.get("actuator");
        //drone = hardwareMap.crservo.get("drone");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        elevator.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);

        wristServo.setDirection(CRServo.Direction.REVERSE);
        clawServo.setDirection(CRServo.Direction.FORWARD);
        slopeServo.setDirection(CRServo.Direction.FORWARD);

        waitForStart();

        boolean goingOut = false;
        boolean goingIn = false;

        while (opModeIsActive()) {

            //Finds the hypotenuse of the triangle created by the two joystick values. Used to find the absoulte direction to go in.
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            //Finds the robot's angle from the raw values of the joystick
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            if(gamepad1.left_stick_y > 0.15 || gamepad1.left_stick_y < -0.15 || gamepad1.left_stick_x > 0.15 || gamepad1.left_stick_x < -0.15) {
                frontLeft.setPower(0.75*v2);
                frontRight.setPower(0.75*v1);
                backLeft.setPower(0.75*v4);
                backRight.setPower(0.75*v3);
            }

            else if(gamepad1.left_bumper) {
                frontLeft.setPower(0.5);
                backLeft.setPower(0.5);
                frontRight.setPower(-0.5);
                backRight.setPower(-0.5);
            }

            else if(gamepad1.right_bumper) {
                frontLeft.setPower(-0.5);
                backLeft.setPower(-0.5);
                frontRight.setPower(0.5);
                backRight.setPower(0.5);
            }
            else if(gamepad1.right_trigger > 0.15) {
                double power = 0.5*gamepad1.right_trigger;
                frontLeft.setPower(-power);
                backLeft.setPower(-power);
                frontRight.setPower(power);
                backRight.setPower(power);
            }
            else if(gamepad1.left_trigger > 0.15) {
                double power = 0.5*gamepad1.left_trigger;
                frontLeft.setPower(power);
                backLeft.setPower(power);
                frontRight.setPower(-power);
                backRight.setPower(-power);
            }
            else {
                frontLeft.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
            }
            // Wrist backwards
            if(gamepad2.right_trigger > 0.15){
                double servoPower = 0.5*gamepad2.right_trigger;
                wristServo.setPower(servoPower);
            }
            // Wrist forwards
            else if(gamepad2.left_trigger > 0.15){
                double servoPower = 0.5*gamepad2.left_trigger;
                wristServo.setPower(-servoPower);
            }
            else{
                wristServo.setPower(0);
            }

            //elevator up or down
            if(gamepad2.right_stick_y > 0.15 || gamepad2.right_stick_y < -0.15){
                elevPower=(gamepad2.right_stick_y);
            }
            else{
                elevPower=0;
            }
            elevator.setPower(elevPower);

            //intake forward/backward
            if(gamepad2.left_stick_y > 0.15 || gamepad2.left_stick_y < -0.15){
                intake.setPower(gamepad2.left_stick_y);
            }
            else{
                intake.setPower(0);
            }
            if(gamepad2.left_stick_x > 0.15 || gamepad2.left_stick_x < -0.15){
                slopeServo.setPower(gamepad2.left_stick_x);
            }
            else{
                slopeServo.setPower(0);
            }

            //Claw in
            if(gamepad2.left_bumper){
                clawServo.setPower(100);
            }
            //Claw out
            if(gamepad2.right_bumper){
                clawServo.setPower(-100);
            }

            //Intake 100% forward
            if(gamepad2.square){
                intake.setPower(100);
                slopeServo.setPower(100);
            }
            //Intake 100% backward
            if(gamepad2.x){
                intake.setPower(-100);
                slopeServo.setPower(-100);
            }

            //Claw in
            if(gamepad2.triangle){
                goingOut = !goingOut;
                if(goingOut){
                    clawServo.setPower(100);
                }
                else{
                    clawServo.setPower(0);
                }
            }
            //Claw out
            if(gamepad2.circle){
                goingIn = !goingIn;
                if(goingIn){
                    clawServo.setPower(-100);
                }
                else{
                    clawServo.setPower(0);
                }
            }
        }
    }

}