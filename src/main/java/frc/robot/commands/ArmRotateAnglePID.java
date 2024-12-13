// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmRotateAnglePID extends Command {
  private ArmSubsystem armSub;
  private PIDController controller;
  private DoubleSupplier setpoint;
  private DoubleSupplier tolerance;

  /** Creates a new ArmRotateTo. */
  public ArmRotateAnglePID(ArmSubsystem armsub, DoubleSupplier setpoint, DoubleSupplier tolerance) {
    this.armSub = armsub;
    this.setpoint = setpoint;
    this.tolerance = tolerance;
    controller = armsub.getPosController();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setSetpoint(setpoint.getAsDouble());
    controller.setTolerance(tolerance.getAsDouble());
    controller.enableContinuousInput(0, 360);
    controller.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSub.setSpeedPercent(-controller.calculate(armSub.getEncoderAngle() % 360));

    SmartDashboard.putNumber("Arm PID Output", controller.calculate(armSub.getEncoderAngle() % 360));
    SmartDashboard.putNumber("Arm PID Setpoint", setpoint.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSub.setSpeedPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
