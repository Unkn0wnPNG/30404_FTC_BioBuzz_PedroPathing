/*
This is 30404's Autonomous Teleop Code
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
/*
This program can drive the meccanum chassis and run intake, outake and pollen pusher.
*/
@TeleOp(name = "MainTeleOpTest", group = "Robot")
public class MainTeleOpTest extends OpMode {
    // This declares the four motors needed
    DcMotorEx frontLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx backRightMotor;
    DcMotorEx intakeMotor;
    DcMotorEx pollenFlywheelMotor;
    DcMotorEx nectarFlywheelMotor;
    Servo pollenServo;
    CRServo throughputServo;
    
    //Global Variables
    public double pollenMotorVelocity = 1700.0;
    public double nectarMotorVelocity = 2000.0;
    public double intakeMotorVelocity = 2500.0;
    public double driveMaxVelocity = 2240.0;  // make this slower for outreaches
    public double pollenServoTop = 0.2;
    public double pollenServoBottom = 0.96;    
    
    boolean pollenMotorOn = false;
    boolean nectarMotorOn = false;
    boolean intakeMotorOn = false;
    
    boolean pollenServoPosition = false;

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    @Override
    public void init() {
        // Drive
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRight");
        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        PIDFCoefficients frontLeft_pidfCoefficients = new PIDFCoefficients(11.7000, 0, 0, 14.0600);
        PIDFCoefficients frontRight_pidfCoefficients = new PIDFCoefficients(11.7000, 0, 0, 14.0600);
        PIDFCoefficients backLeft_pidfCoefficients = new PIDFCoefficients(11.7000, 0, 0, 14.0600);
        PIDFCoefficients backRight_pidfCoefficients = new PIDFCoefficients(11.7000, 0, 0, 14.0600);
        frontLeftMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, frontLeft_pidfCoefficients);
        frontRightMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, frontRight_pidfCoefficients);
        backLeftMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, backLeft_pidfCoefficients);
        backRightMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, backRight_pidfCoefficients);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Intake
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        PIDFCoefficients intake_pidfCoefficients = new PIDFCoefficients(11.7000, 0, 0, 14.0600);
        intakeMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, intake_pidfCoefficients);
        // Pollen Outtake
        pollenFlywheelMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        pollenFlywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pollenFlywheelMotor.setDirection(DcMotorEx.Direction.FORWARD);
        PIDFCoefficients pollen_pidfCoefficients = new PIDFCoefficients(11.7000, 0, 0, 15.1600);
        pollenFlywheelMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pollen_pidfCoefficients);
        // Nectar Outtake
        // nectarFlywheelMotor = hardwareMap.get(DcMotorEx.class,"outtakeMotor");
        // nectarFlywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        // nectarFlywheelMotor.setDirection(DcMotorEx.Direction.FORWARD);
        // PIDFCoefficients nectar_pidfCoefficients = new PIDFCoefficients(11.7000, 0, 0, 15.1600);
        // nectarFlywheelMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, nectar_pidfCoefficients);
        // Servo
        pollenServo = hardwareMap.get(Servo.class, "pollenServo");
        throughputServo = hardwareMap.get(CRServo.class, "throughputServo");
        
        telemetry.addLine("Init complete");

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    @Override
    public void loop() {
        telemetry.addLine("Press Cross to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");

        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if(gamepad1.aWasPressed()){
            pollenServo(pollenServoPosition);
        }
        if(gamepad1.xWasPressed()){
            imu.resetYaw();
        }
        if(gamepad1.yWasPressed()){
            intakeMotorOn = !intakeMotorOn;
        }
        if(gamepad2.bWasPressed()){
            pollenMotorOn = !pollenMotorOn;
        }
        if(gamepad2.dpadUpWasPressed()){
            pollenMotorVelocity += 10;
        }
        else if(gamepad2.dpadDownWasPressed()){
            pollenMotorVelocity -= 10;
        }
        if(gamepad1.dpadRightWasPressed()){
            intakeMotorVelocity += 50;
        }
        else if(gamepad1.dpadLeftWasPressed()){
            intakeMotorVelocity -= 50;
        }
        // If you press the left bumper, you get a drive from the point of view of the robot
        // (much like driving an RC vehicle)
        driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        
        intake(intakeMotorOn, intakeMotorVelocity);
        pollenOuttake(pollenMotorOn, pollenMotorVelocity);
        // nectarOuttake(nectarMotorOn, nectarMotorVelocity);
        
        telemetry.addData("pollenMotorVelocity dpad up/down: ", pollenMotorVelocity);
        telemetry.addData("nectarMotorVelocity: ", nectarMotorVelocity);
        telemetry.addData("intakeMotorVelocity dpad right/left: ", intakeMotorVelocity);
        telemetry.addData("frontLeftMotor velocity: ", frontLeftMotor.getVelocity());
        telemetry.addData("backLeftMotor velocity: ", backLeftMotor.getVelocity());
        telemetry.addData("frontRightMotor velocity: ", frontRightMotor.getVelocity());
        telemetry.addData("backRightMotor velocity: ", backRightMotor.getVelocity());
    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        
        double frontLeftVelocity = driveMaxVelocity * (frontLeftPower / maxPower);
        double frontRightVelocity = driveMaxVelocity * (frontRightPower / maxPower);
        double backRightVelocity = driveMaxVelocity * (backRightPower / maxPower);
        double backLeftVelocity = driveMaxVelocity * (backLeftPower / maxPower);
        
        frontLeftMotor.setVelocity(frontLeftVelocity);
        frontRightMotor.setVelocity(frontRightVelocity);
        backRightMotor.setVelocity(backRightVelocity);
        backLeftMotor.setVelocity(backLeftVelocity);
        
        telemetry.addData("forward (called): ", forward);
        telemetry.addData("right (called): ", right);
        telemetry.addData("rotate (called): ", rotate);
        telemetry.addData("frontLeftSpeed (called): ", frontLeftVelocity);
        telemetry.addData("frontRightSpeed (called): ", frontRightVelocity);
        telemetry.addData("backRightSpeed (called): ", backRightVelocity);
        telemetry.addData("backLeftSpeed (called): ", backLeftVelocity);
    }
    
    public void intake(boolean on, double velocity){
        if(on){
            intakeMotor.setVelocity(velocity);
        }
        else{
            intakeMotor.setVelocity(0);
        }
    }
    
    public void pollenOuttake(boolean on, double velocity){
        if (on){
            throughputServo.setPower(1.0);
            pollenFlywheelMotor.setVelocity(velocity);
        }
        else{
            throughputServo.setPower(0);
            pollenFlywheelMotor.setVelocity(0);
        }
    }
    
    // public void nectarOuttake(boolean on, double veloctiy){
        
    // }
    
    public void pollenServo(boolean position){
        pollenServoPosition = !pollenServoPosition;
        if(position){
            pollenServo.setPosition(pollenServoTop);
        }
        else{
            pollenServo.setPosition(pollenServoBottom);
        }
    }
}
