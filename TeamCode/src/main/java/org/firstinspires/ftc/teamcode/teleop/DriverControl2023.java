/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.MainConfig2023;


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



@TeleOp(name="Champion_DC", group="FTC-2023-Calabar")
public class DriverControl2023 extends LinearOpMode {

    // Declare the main configurations for the code
    private MainConfig2023 config = new MainConfig2023();

    @Override
    public void runOpMode() {
        // Initialize all the robot configurations
        config.TotalHardwareMap(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // reset the runtime to track the game time
        config.runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            // check for any commands for the chassis and driving
            Mecanum_Drive();

            //check for any lift commands
            Linear_Slide();

            //check for any intake commands
            Intake();
            ColourSensor();

            MLS();

            // update the screen with the new data
            telemetry.addData("Status", "Run Time: " + config.runtime.toString());
            telemetry.update();
        }
    }

    public void Mecanum_Drive() {

        /*
        * ========================
        * DEFAULT
        * TANK DRIVE
        * ========================
        * */

        config.right_power_1 = -gamepad1.right_stick_y;
        config.right_power_2 = -gamepad1.right_stick_y;
        config.left_power_1 = -gamepad1.left_stick_y;
        config.left_power_2 = -gamepad1.left_stick_y;


        /*
         * ========================
         * DEFAULT
         * Mecanum DRIVE Horizontal
         * ========================
         * */

        if(gamepad1.right_stick_y == 0 && gamepad1.left_stick_y == 0){

            config.mecanumright = gamepad1.dpad_left;//makes robot move to the left side
            config.mecanumleft = gamepad1.dpad_right;//makes robot move to the right side

            if (config.mecanumright) {
                // Slide the robot to the right
                config.right_power_1 = config.mecanum_power;
                config.right_power_2 = -config.mecanum_power;
                config.left_power_1 = -config.mecanum_power;
                config.left_power_2 = config.mecanum_power;

            } else if (config.mecanumleft) {
                // Slide the robot to the left
                config.right_power_1 = -config.mecanum_power;
                config.right_power_2 = config.mecanum_power;
                config.left_power_1 = config.mecanum_power;
                config.left_power_2 = -config.mecanum_power;
            }
        }

        /*
         * ========================
         * Racing Game
         * DRIVE - GT
         * ========================
         * */
        // coming soon

        /*
         * ========================
         * Speed Limiter
         * ========================
         * */

        config.rightDrive.setPower(config.right_power_1);
        config.rightDrive2.setPower(config.right_power_2);
        config.leftDrive.setPower(config.left_power_1);
        config.leftDrive2.setPower(config.left_power_2);

        // Get the wheel data and show it
        config.right_1_wheel_position = config.rightDrive.getCurrentPosition();
        config.right_2_wheel_position = config.rightDrive2.getCurrentPosition();
        config.left_1_wheel_position = config.leftDrive.getCurrentPosition();
        config.left_2_wheel_position = config.leftDrive2.getCurrentPosition();

        telemetry.addData("Mecanums Power","R1 %f%% | R2 %f%% | L1 %f%% | L2 %f%%",
                config.right_power_1*100, config.right_power_2*100, config.left_power_1*100,config.left_power_2*100);

        telemetry.addData("Mecanums Positions","R1 %d | R2 %d | L1 %d| L2 %d",
                config.right_1_wheel_position, config.right_2_wheel_position, config.left_1_wheel_position, config.left_2_wheel_position);

    }

    public void  Linear_Slide() {
        config.right_trig = gamepad2.right_trigger;//makes linear slide go up
        config.left_trig = gamepad2.left_trigger;//makes linear slide go down

        config.m_switch=config.MagneticLimitSwitch.getState();
        config.claw=config.isopen();


        if (config.right_trig > 0.5) {
            //if((config.m_switch=false) && (config.claw=false)) {
                //slide up
                config.slide_power_1 = config.slide_speedy;
                config.slide_power_2 = config.slide_speedy;


        } else if (config.left_trig > 0.5) {
            //slide down
            config.slide_power_1 = -config.slide_speedy;
            config.slide_power_2 = -config.slide_speedy;

        } else {
            // stop and brake
            config.slide_power_1 = 0;
            config.slide_power_2 = 0;
        }


        config.linslide_left.setPower(-config.slide_power_1);
        config.linslide_right.setPower(-config.slide_power_2);

        // Get the slide data and show it
        config.slide_1_position =config.linslide_left.getCurrentPosition();
        config.slide_2_position =config.linslide_right.getCurrentPosition();
        telemetry.addData("Linear Slide","Power: %f%% | Position: %d", config.slide_power_1*100, config.slide_1_position);
    }

   public void Intake()
   {
       config.bump_right = gamepad2.a;//open claws
       config.bump_left = gamepad2.x;//close claws

       if ( config.bump_right) {
           // open intake
          config.claw_open();

       }else if (config.bump_left) {
           // close intake
           config.claw_close();
       }
   }

    public void ColourSensor() {

        final float[] hsvValues = new float[3];
        if (config.colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)config.colorSensor).enableLight(true);
        }

        NormalizedRGBA colors = config.colorSensor.getNormalizedColors();

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

    public void MLS() {

        // set the digital channel to input.
        if (config.MagneticLimitSwitch.getState() == true) {

            telemetry.addData("Digital Touch", "Linear slide is up");
        } else {
            telemetry.addData("Digital Touch", "Linear slide is down");
        }
        telemetry.update();
    }

}

