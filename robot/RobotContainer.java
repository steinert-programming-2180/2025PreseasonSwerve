// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {

  private final DriveSubsystem driveSubsystem=new DriveSubsystem();
  private final PS5Controller driveController=new PS5Controller(OperatorConstants.driveControllerPort);

  public RobotContainer() {
    
    //I hate using this notation for commands but it is simply more convenient here.
    //This just gets the needed controller values and puts it into a format the Computer can use
    //Avoid the () -> command notation if possible
    //To note, this automatically operates field oriented. I put circle button here for turning it off but idk if it works
    //Just try to stick with one so I dont have to do any more code

    driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem,
    () -> driveController.getLeftX(),
    () -> driveController.getLeftY(),
    () -> driveController.getRightX(),
    () -> driveController.getCircleButtonReleased()));
    //Also to note for any new programmers: this is the code that is run to make the thing drive.

    configureBindings();
  }


  private void configureBindings() {
    //put code here so you can push a button and run driveSubsystem.zeroHeading();
    //needs to be in the form of a Command I think tho and I dont feel like figuring that out
    //its 2:30 AM on a Saturday. Im going to sleep and pushing this to Github on Monday.
  }

  public Command getAutonomousCommand() {
    
    return null;
  }
}
