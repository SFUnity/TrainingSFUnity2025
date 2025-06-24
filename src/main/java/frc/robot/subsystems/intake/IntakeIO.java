package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double pivotPositionDeg = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotCurrentAmps = 0.0;
    public double rollersAppliedVolts = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setPivot(double setpoint) {}

  public default void runRollers(double volts) {}


  public default void configurePID(double kP, double kI, double kD) {}
}
