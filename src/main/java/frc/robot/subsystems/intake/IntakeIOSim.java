package frc.robot.subsystems.intake;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.LoggedTunableNumber;

public class IntakeIOSim {
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
}
