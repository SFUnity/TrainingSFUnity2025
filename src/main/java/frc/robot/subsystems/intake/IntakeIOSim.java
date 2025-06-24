package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constantsGlobal.*;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import frc.robot.util.LoggedTunableNumber;

public class IntakeIOSim implements IntakeIO {
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          gearing,
          weight,
          armLengthMeters,
          minAngleRad,
          maxAngleRad,
          false,
          minAngleRad);

  private final PIDController controller;
  private double pivotAppliedVolts = 0.0;
  private double rollersAppliedVolts = 0.0;

  public IntakeIOSim() {
    controller = new PIDController(kP.get(), 0.0, 0.0);
    sim.setState(minAngleRad, 0.0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    sim.update(Constants.loopPeriodSecs);

    inputs.pivotPositionDeg = getAngle();
    inputs.pivotAppliedVolts = pivotAppliedVolts;
    inputs.pivotCurrentAmps = sim.getCurrentDrawAmps();

    inputs.rollersAppliedVolts = rollersAppliedVolts;

    LoggedTunableNumber.ifChanged(hashCode(), () -> controller.setP(kP.get()), kP);
    sim.setInputVoltage(0.0);
  }

  @Override
  public void runRollers(double volts) {
    rollersAppliedVolts = volts;
  }

  @Override
  public void setPivot(double angle) {
    double volts = controller.calculate(getAngle(), angle);
    pivotAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(pivotAppliedVolts);
  }
  @Override
  public boolean 


  private double getAngle() {
    return Units.radiansToDegrees(sim.getAngleRads() - minAngleRad);
  }
}
