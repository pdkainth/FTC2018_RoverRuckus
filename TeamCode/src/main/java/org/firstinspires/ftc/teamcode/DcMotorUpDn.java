package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DcMotorUpDn {

  public enum MotorOpMode {
    IDLE, MANUAL, ENCODER
  }

  public static final double MAX_POWER = 1.0;

  private DcMotor motor = null;
  private TouchSensor touchSensor = null;
  private double power;
  private int targetEncoderPos;
  private MotorOpMode opMode = MotorOpMode.IDLE;
  private String moduleName = null;

  public void init(HardwareMap hardwareMap, String moduleName, String motorName, String touchSensorName) {
    motor = hardwareMap.get(DcMotor.class, motorName);
    motor.setPower(0.0);
    motor.setDirection(DcMotorSimple.Direction.FORWARD);
    resetEncoder();
    stop();

    touchSensor = hardwareMap.get(TouchSensor.class, touchSensorName);

    this.moduleName = moduleName;
  }

  public void lift(double power) {
    // reached lowest point. STOP and don't drive further
    boolean touch = touchSensor.isPressed();
    if ((touch == true) && (power < 0.0)) {
      stop();
      return;
    }

    // In encoder or idle mode and no manual power, don't disturb
    if (((opMode == MotorOpMode.ENCODER) || (opMode == MotorOpMode.IDLE)) && (power == 0.0)) {
      return;
    }

    // Manual mode and no power, stop and goto IDLE
    if ((opMode == MotorOpMode.MANUAL) && (power == 0.0)) {
      stop();
      return;
    }

    // There is non zero manual power, override encoder mode and take manual control
    if (opMode != MotorOpMode.MANUAL) {
      opMode = MotorOpMode.MANUAL;
      motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    motor.setPower(power);
    this.power = power;
  }

  public void resetEncoder() {
    opMode = MotorOpMode.IDLE;
    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  }

  public void liftToPosition(int pos) {
    if (opMode != MotorOpMode.ENCODER) {
      opMode = MotorOpMode.ENCODER;
      motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    targetEncoderPos = pos;
    motor.setTargetPosition(targetEncoderPos);
    motor.setPower(MAX_POWER);
    this.power = MAX_POWER;
  }

  public boolean isBusy() {
    return motor.isBusy();
  }

  public void stop() {
    opMode = MotorOpMode.IDLE;
    power = 0.0;
    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor.setPower(0.0);
  }

  public void update(Telemetry telemetry) {
    int pos = motor.getCurrentPosition();
    boolean touch = touchSensor.isPressed();

    // OpMode is encoder and is going down. If touch detected, stop and reset encoder
    if ((opMode == MotorOpMode.ENCODER) &&
      (targetEncoderPos < pos) &&
      (touch == true)) {
      stop();
      resetEncoder();
    }

    // refresh position
    pos = motor.getCurrentPosition();

    telemetry.addData(moduleName, "%s P %.2f T %b B %b pos c %d t %d",
      opMode, power, touch, isBusy(), pos, targetEncoderPos);
  }
}
