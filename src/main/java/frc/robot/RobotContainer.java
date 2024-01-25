// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import SOTAlib.Config.ConfigUtils;
import SOTAlib.Control.SOTA_Xboxcontroller;
import SOTAlib.Factories.CompositeMotorFactory;
import SOTAlib.Factories.IllegalMotorModel;
import SOTAlib.Factories.MotorControllerFactory;
import SOTAlib.Gyro.NavX;
import SOTAlib.Gyro.SOTA_Gyro;
import SOTAlib.MotorController.SOTA_CompositeMotor;
import SOTAlib.MotorController.SOTA_MotorController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.swerve.DriveCommand;
import frc.robot.subsystems.SOTA_SwerveDrive;
import frc.robot.subsystems.SOTA_SwerveModule;
import frc.robot.subsystems.configs.SOTA_SwerveDriveConfig;
import frc.robot.subsystems.configs.SOTA_SwerveModuleConfig;

public class RobotContainer {
  private ConfigUtils mConfigUtils;

  private SOTA_Xboxcontroller dController;
  private SOTA_Gyro mGyro;

  private SOTA_SwerveDrive mSwerveDrive;

  public RobotContainer() {
    this.mConfigUtils = new ConfigUtils();

    this.dController = new SOTA_Xboxcontroller(0);
    this.mGyro = new NavX(new AHRS(Port.kMXP));

    try {
      CompositeMotorFactory mCompositeMotorFactory = new CompositeMotorFactory();

      SOTA_SwerveDriveConfig driveConfig = mConfigUtils.readFromClassPath(SOTA_SwerveDriveConfig.class, "swerve/drive");

      SwerveDriveKinematics kinematics = new SwerveDriveKinematics(driveConfig.generateModuleTranslations());

      SOTA_SwerveModule[] modules = {
          initModule(mConfigUtils, mCompositeMotorFactory, "swerve/frontright", driveConfig),
          initModule(mConfigUtils, mCompositeMotorFactory, "swerve/frontleft", driveConfig),
          initModule(mConfigUtils, mCompositeMotorFactory, "swerve/backleft", driveConfig),
          initModule(mConfigUtils, mCompositeMotorFactory, "swerve/backright", driveConfig)
      };

      this.mSwerveDrive = new SOTA_SwerveDrive(modules, kinematics, mGyro, driveConfig);
      configureDefaultCommands();

    } catch (Exception e) {
      // TODO: handle exception
    }

    configureBindings();
  }

  private void configureDefaultCommands() {
    mSwerveDrive.setDefaultCommand(
        new DriveCommand(mSwerveDrive, dController::getLeftY, dController::getLeftX, dController::getRightY));
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private SOTA_SwerveModule initModule(ConfigUtils lConfigUtils, CompositeMotorFactory lCompositeMotorFactory,
      String swerveModuleConfigPath, SOTA_SwerveDriveConfig driveConfig) throws IllegalMotorModel, Exception {

    SOTA_SwerveModuleConfig moduleConfig = lConfigUtils.readFromClassPath(SOTA_SwerveModuleConfig.class,
        swerveModuleConfigPath);
    SOTA_CompositeMotor angleSystem = lCompositeMotorFactory.generateCompositeMotor(moduleConfig.getAngleSystem());
    SOTA_MotorController speedMotor = MotorControllerFactory.generateMotorController(moduleConfig.getSpeedConfig());
    return new SOTA_SwerveModule(driveConfig, moduleConfig, speedMotor, angleSystem.getAbsEncoder(),
        angleSystem.getMotor());

  }
}
