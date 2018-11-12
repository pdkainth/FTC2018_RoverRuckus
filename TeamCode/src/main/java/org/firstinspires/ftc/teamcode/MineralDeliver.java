package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MineralDeliver {

  private DcMotor mineralMotor;
  double motorPosition;
  double mineralPower;

  private Servo mineralServo;
  private double servoPosition;
  //private static final double COLLECT_POSITION;
  //private static final double DUMP_POSITION;

  public void init(HardwareMap hardwareMap){
    mineralMotor = hardwareMap.get(DcMotor.class, "Motor5");
    mineralMotor.setPower(0.0);
    mineralMotor.setDirection(DcMotorSimple.Direction.FORWARD);

    mineralServo = hardwareMap.get(Servo.class, "Servo2");
    collect();
  }


  public void test(double motorPower){
    mineralPower = motorPower;
    mineralMotor.setPower(mineralPower);

  }



  public void stop(){
    mineralMotor.setPower(0.0);
  }

  public void dump(){
    servoPosition = 0.8;
    mineralServo.setPosition(servoPosition);
  }

  public void collect(){
    servoPosition = 0.05;
    mineralServo.setPosition(servoPosition);
  }
  public void update(Telemetry telemetry){
    motorPosition = mineralMotor.getCurrentPosition();
    telemetry.addData("MineralMotor", "Power %.2f CurrentPosition %.2f", mineralPower, motorPosition);
    telemetry.addData("MineralServo", "Position %.2f", servoPosition);
  }
}
