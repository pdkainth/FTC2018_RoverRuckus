package org.firstinspires.ftc.teamcode;


public class Position
{
  public float posX;
  public float posY;
  public float angle;

  public Position() {
    posX = posY = angle = 0.0f;
  }

  public Position(float posX, float posY) {
    this.posX = posX;
    this.posY = posY;
    angle = 0.0f;
  }

  public Position(float posX, float posY, float angle) {
    this.posX = posX;
    this.posY = posY;
    this.angle = angle;
  }

  public void setPos(float posX, float posY) {
    this.posX = posX;
    this.posY = posY;
  }

  public void setPos(float posX, float posY, float angle) {
    this.posX = posX;
    this.posY = posY;
    this.angle = angle;
  }

  public void setPos(Position pos) {
    this.posX = pos.posX;
    this.posY = pos.posY;
    this.angle = pos.angle;
  }

  public void addAngle(float delta){
    angle += delta;
    angle = Gyro.convertAngle180(angle);
  }


  public float slopeTo(Position toPos) {
    double sloeRad =  Math.atan2(
        (double)(toPos.posY - posY),
        (double)(toPos.posX - posX));
    return Gyro.convertAngle180((float)Math.toDegrees(sloeRad));
  }

  public float distance(Position toPos) {
    double distanceSq =
        Math.pow((double) (toPos.posX - posX), 2.0) + Math.pow((double) (toPos.posY - posY), 2.0);
    return (float) Math.sqrt(distanceSq);
  }

  public void translate(float distance) {
    posX = posX + distance * (float)Math.cos(Math.toRadians(angle));
    posY = posY + distance * (float)Math.sin(Math.toRadians(angle));
  }

  public String toString() {
    return String.format("X %.1f Y %.1f A %.1f", posX, posY, angle);
  }
}
