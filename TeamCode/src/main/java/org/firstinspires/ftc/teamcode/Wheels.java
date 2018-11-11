package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Wheels {

    /**
     * Motor 3- front left
     * Motor2 - front right
     * Motor 1 - back left
     * Motor 4 - back right
     */
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    public void init(HardwareMap hardwareMap){
        frontLeftDrive = hardwareMap.get(DcMotor.class, "Motor3");
        frontRightDrive = hardwareMap.get(DcMotor.class, "Motor2");
        backLeftDrive = hardwareMap.get(DcMotor.class, "Motor1");
        backRightDrive = hardwareMap.get(DcMotor.class, "Motor0");

        frontLeftDrive.setPower(0.0);
        frontRightDrive.setPower(0.0);
        backLeftDrive.setPower(0.0);
        backRightDrive.setPower(0.0);

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);






    }

    public void drive(double drive, double strafe, double rotate, Telemetry telemetry){

        double frontLeftPower = drive + strafe + rotate;
        double backLeftPower = drive - strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backRightPower = drive + strafe - rotate;

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);

        telemetry.addData("Wheel","FL %.2f FR %.2f BL %.2f BR %.2f", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    public void stop(){
        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        backLeftDrive.setPower(0);
    }
}
