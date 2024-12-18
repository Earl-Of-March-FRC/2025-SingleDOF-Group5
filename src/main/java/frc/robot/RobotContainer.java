// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmRotate;
import frc.robot.commands.ArmRotateAnglePID;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.commands.ForwardAndBack;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ArmSubsystem armSub = new ArmSubsystem();

  private final XboxController xboxController = new XboxController(OperatorConstants.driverControllerPort);
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.driverControllerPort);
  private final Command m_simpleAuto = new ForwardAndBack(armSub);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_chooser.setDefaultOption("Simple Auto", m_simpleAuto);
    SmartDashboard.putData(m_chooser);

    armSub.setDefaultCommand(
      new ArmRotate( 
        armSub,() -> MathUtil.applyDeadband(xboxController.getLeftY(), 0.1)
      )
    );
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new Trigger(xboxController::getXButtonPressed).onTrue(
      new ArmRotateAnglePID(armSub, 
                      () -> 90,
                      () -> 3));
    new Trigger(xboxController::getYButtonPressed).onTrue(
      new ArmRotateAnglePID(armSub, 
                      () -> 180,
                      () -> 3));
    new Trigger(xboxController::getAButtonPressed).whileTrue(
      new ArmRotateAnglePID(armSub, 
                      () -> 270,
                      () -> 3));
    new Trigger(xboxController::getBButtonPressed).whileTrue(
      new ArmRotateAnglePID(armSub, 
                      () -> 359,
                      () -> 3));
    
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
