// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.flywheel;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class FlywheelIOSparkMax implements FlywheelIO {
  private static final double GEAR_RATIO = 1.5;

  private final SparkMax leader = new SparkMax(0, MotorType.kBrushless);
  private final SparkMax follower = new SparkMax(1, MotorType.kBrushless);
  private final RelativeEncoder encoder = leader.getEncoder();
  private final SparkClosedLoopController pid = leader.getClosedLoopController();

  public FlywheelIOSparkMax() {
    var leaderConfig = new SparkMaxConfig();
    var followerConfig = new SparkMaxConfig();

    leaderConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).voltageCompensation(12.0);

    followerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).voltageCompensation(12.0);
    followerConfig.follow(leader);

    tryUntilOk(
        leader,
        () ->
            leader.configure(
                leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        follower,
        () ->
            follower.configure(
                followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pid.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig.closedLoop.pidf(kP, 0, kD, 0);
    tryUntilOk(
        leader,
        () ->
            leader.configure(
                leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }
}
