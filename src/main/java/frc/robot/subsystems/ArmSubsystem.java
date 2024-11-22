// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  private final WPI_TalonSRX motor = new WPI_TalonSRX(Constants.ArmConstants.motorPort);

  /** Creates a new Arm. */
  public ArmSubsystem() {
    motor.setSelectedSensorPosition(0);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configFactoryDefault();
    // encoder.setDistancePerPulse(Constants.EncoderConstants.distancePerPulse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeedPercent(double speed) {
    motor.set(MathUtil.clamp(speed, -1, 1) * 0.5);
  }

  public void setSpeedRPM(double RPM) {
    motor.set(TalonSRXControlMode.Velocity, RPM / 600 * Constants.EncoderConstants.ticksPerRev);
  }

  public void rotateToAngle(double angle) {
    motor.set(TalonSRXControlMode.Position, angle / 360 * Constants.EncoderConstants.ticksPerRev);
  }

  public void rotateByAngle(double angle) {
    motor.set(TalonSRXControlMode.Position,
            //clamp between the physical min and max value allowed for the arm
            MathUtil.clamp(getEncoderAngle() + angle, Constants.EncoderConstants.minAngle, Constants.EncoderConstants.maxAngle)
            //convert angle to ticks
            / 360 * Constants.EncoderConstants.ticksPerRev);
  }

  public double getEncoderDistance() {
    return motor.getSelectedSensorPosition();
  }

  public double getEncoderVelocity() {
    return motor.getSelectedSensorVelocity();
  }

  public double getEncoderAngle() {
    return getEncoderDistance() * 360 / Constants.EncoderConstants.ticksPerRev;
  }

}