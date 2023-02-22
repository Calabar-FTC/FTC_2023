
package org.firstinspires.ftc.teamcode.teleop;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */



@TeleOp(name="PlanB", group="Linear Opmode")

public class Original extends LinearOpMode {

    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor leftDrive2 = null;
    public DcMotor rightDrive2 = null;
    public DcMotor linslide_left = null;
    public DcMotor  linslide_right = null;
    public Servo servo1;
    public Servo servo2;
    public NormalizedColorSensor colorSensor;
    public DistanceSensor sensorRange;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive2 = hardwareMap.get(DcMotor.class, "left_drive2");
        rightDrive2 = hardwareMap.get(DcMotor.class, "right_drive2");

        linslide_left = hardwareMap.get(DcMotor.class, "arm_right");
        linslide_right = hardwareMap.get(DcMotor.class, "arm_left");

        servo1 = hardwareMap.get(Servo.class, "claw_right");
        servo2 = hardwareMap.get(Servo.class, "claw_left");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);

        linslide_left.setDirection(DcMotor.Direction.FORWARD);
        linslide_right.setDirection(DcMotor.Direction.FORWARD);

        servo1.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            Mecanum_Drive();
            Intake();
            Linear_Slide();
            DistanceSensor();
            ColourSensor();


            telemetry.update();
        }
    }

    public void Mecanum_Drive() {
        double right_power = -gamepad1.right_stick_y;
        double left_power = -gamepad1.left_stick_y;
        boolean mecanumright = gamepad1.dpad_left;//makes robot move to the right side
        boolean mecanumleft = gamepad1.dpad_right;//makes robot move to the right side

        double power = 0.75;

        if (mecanumleft) {
            leftDrive2.setPower(-power);
            rightDrive2.setPower(power);
            leftDrive.setPower(power);
            rightDrive.setPower(-power);

        } else {
            leftDrive2.setPower(0);
            rightDrive2.setPower(0);
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }
        if (mecanumright) {
            leftDrive2.setPower(power);
            rightDrive2.setPower(-power);
            leftDrive.setPower(-power);
            rightDrive.setPower(power);

        } else {
            leftDrive2.setPower(0);
            rightDrive2.setPower(0);
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }

        if ((!mecanumleft) && (!mecanumright)) {
            leftDrive2.setPower(left_power);
            rightDrive2.setPower(right_power);
            leftDrive.setPower(left_power);
            rightDrive.setPower(right_power);

        }

//        int l_pos = leftDrive.getCurrentPosition();
//        telemetry.addData("Motors", "left_motor - %d", l_pos);
    }

    public void Linear_Slide() {
        float right_trig = gamepad2.right_trigger;//makes linear slide go up
        float left_trig = gamepad2.left_trigger;//makes linear slide go down
        double speedy=1;

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        if (right_trig > 0.5) {
            linslide_left.setPower(speedy);
            linslide_right.setPower(-speedy);
        } else {
            linslide_left.setPower(0);
            linslide_right.setPower(0);
        }

        if (left_trig > 0.5) {
            linslide_left.setPower(-speedy);
            linslide_right.setPower(speedy);
        } else {
            linslide_left.setPower(0);
            linslide_right.setPower(0);

        }


        telemetry.addData("Power of linear slide",speedy);
        int slide = linslide_left.getCurrentPosition();
        telemetry.addData("Motors for linear slide", "left_motor - %d", slide);
    }

    public void Intake() {
        boolean bump_right = gamepad2.a;//open claws
        boolean bump_left = gamepad2.x;//close claws

        if (bump_right) {
            servo1.setPosition(0.1);
            servo2.setPosition(0.1);
        }

        if (bump_left) {
            servo1.setPosition(0);
            servo2.setPosition(0);
        }
    }

    public void ColourSensor() {
        final float[] hsvValues = new float[3];
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        if((colors.red>colors.green) && (colors.red >colors.blue)){
            servo1.setPosition(0.3);
            servo2.setPosition(0.3);

            sleep(3000);

            servo1.setPosition(0);
            servo2.setPosition(0);
        }

        Color.colorToHSV(colors.toColor(), hsvValues);
        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
        telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2]);

        telemetry.addData("Alpha", "%.3f", colors.alpha);

    }
    public void DistanceSensor() {
        telemetry.addData("deviceName",sensorRange.getDeviceName() );
        telemetry.addData("range", String.format("%.01f mm", sensorRange.getDistance(DistanceUnit.MM)));
        telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
        telemetry.addData("range", String.format("%.01f m", sensorRange.getDistance(DistanceUnit.METER)));
        telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));

    }

    public void MswitchSensor()
    {

    }





}


// Show the elapsed game time and wheel power.
//telemetry.addData("Status", "Run Time: " + runtime.toString());