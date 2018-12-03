package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {

    private DcMotor liftDrive = null;
    private TouchSensor liftTouch;
    private int targetEncoderPos;

    public void init(HardwareMap hardwareMap){
        liftDrive = hardwareMap.get(DcMotor.class, "Motor4");
        liftDrive.setPower(0.0);
        liftDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        liftTouch = hardwareMap.get(TouchSensor.class, "Touch0");
    }

    public void lift(double power, Telemetry telemetry){
        boolean touch = liftTouch.isPressed();
        if((touch == true) && (power < 0)){
            power = 0;
        }
        liftDrive.setPower(power);
        double liftPosition = liftDrive.getCurrentPosition();
        telemetry.addData("Lift","Power %.2f Touch %b Position %.2f", power, touch, liftPosition);
    }

    public void liftToPosition(Telemetry telemetry, int pos){
        liftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        targetEncoderPos = pos;
        liftDrive.setTargetPosition(targetEncoderPos);
        liftDrive.setPower(1.0);

    }

    public boolean isBusy(Telemetry telemetry){
        return liftDrive.isBusy();
    }





    public void stop(){
        liftDrive.setPower(0);
    }
}
