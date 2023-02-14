package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MainConfig2023
{

    // Declare OpMode members.
     ElapsedTime runtime = new ElapsedTime();
     DcMotor leftDrive = null;
     DcMotor rightDrive = null;
     DcMotor leftDrive2 = null;
     DcMotor rightDrive2 = null;
     DcMotor linearslide = null;
     Servo servo;

     public HardwareMap hardMap;
     NormalizedColorSensor colorSensor;

    static double wheel_diameter = 9.8;


    public void TotalHardeware ()
    {

        colorSensor =hardMap.get(NormalizedColorSensor.class, "sensor_color");
        //colorSensor = hardMap.get(ColorSensor.class, "color_distance");


        leftDrive =hardMap.get(DcMotor.class, "left_drive");
        rightDrive = hardMap.get(DcMotor.class, "right_drive");
        leftDrive2 = hardMap.get(DcMotor.class, "left_drive2");
        rightDrive2 = hardMap.get(DcMotor.class, "right_drive2");
        linearslide = hardMap.get(DcMotor.class, "linear");
        servo = hardMap.get(Servo.class, "lin-motor");


        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);
        linearslide.setDirection(DcMotor.Direction.FORWARD);
        servo.setDirection(Servo.Direction.FORWARD);


        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
}
