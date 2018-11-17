package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {

    private DcMotor liftDrive = null;

    public void init(HardwareMap hardwareMap){
        liftDrive = hardwareMap.get(DcMotor.class, "Motor4");
        liftDrive.setPower(0.0);
        liftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void lift(double power, Telemetry telemetry){
        liftDrive.setPower(power);
        telemetry.addData("Lift","Power %.2f", power);
    }

    public void stop(){
        liftDrive.setPower(0);
    }
}
