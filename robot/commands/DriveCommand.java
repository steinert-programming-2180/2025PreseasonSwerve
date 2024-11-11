package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends Command{
    private final DriveSubsystem driveSubsystem;
    private final Supplier<Double> xSpdFunc, ySpdFunc, turnSpdFunc;
    private final Supplier<Boolean> fieldOrientedFunc;
    private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;

    //standard requirements. suppliers are the values from the joystick
    public DriveCommand(
    DriveSubsystem driveSubsystem,
    Supplier<Double> xSpdFunc,
    Supplier<Double> ySpdFunc,
    Supplier<Double> turnSpdFunc,
    Supplier<Boolean> fieldOrientedFunc){

        this.driveSubsystem=driveSubsystem;
        this.xSpdFunc=xSpdFunc;
        this.ySpdFunc=ySpdFunc;
        this.turnSpdFunc=turnSpdFunc;
        this.fieldOrientedFunc=fieldOrientedFunc;
        this.xLimiter=new SlewRateLimiter(DriveConstants.driveAccMax);
        this.yLimiter=new SlewRateLimiter(DriveConstants.driveAccMax);
        this.turnLimiter=new SlewRateLimiter(DriveConstants.turnAccMax);

        addRequirements(driveSubsystem);
        }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double xSpeed=xSpdFunc.get();
        double ySpeed=ySpdFunc.get();
        double turnSpeed=turnSpdFunc.get();

        //apparently this applies a deadband. I dont like this notation but if it works it works
        xSpeed=Math.abs(xSpeed) > OperatorConstants.Deadband ? xSpeed : 0.0;
        ySpeed=Math.abs(ySpeed) > OperatorConstants.Deadband ? ySpeed : 0.0;
        turnSpeed=Math.abs(turnSpeed) > OperatorConstants.Deadband ? turnSpeed : 0.0;

        //the input value is always gonna be btwn -1 and 1, so this just converts that into a percent of the max speed
        xSpeed=xLimiter.calculate(xSpeed) * DriveConstants.maxSpeed;
        ySpeed=yLimiter.calculate(ySpeed) * DriveConstants.maxSpeed;
        turnSpeed=turnLimiter.calculate(turnSpeed) * DriveConstants.maxAngularSpeed;
        
        //Converts values into a ChassisSpeeds object depending on if the user wants the drive to be field oriented or not
        ChassisSpeeds speeds;
        if(fieldOrientedFunc.get()){
            speeds=ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, driveSubsystem.getRotation2d());
        } else{
            speeds=new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
        }

        //turn that ChassisSpeeds object into an array of swerve module states using the info from the kinematics object
        //I hate how much Swerve makes you switch between object types. Why do it this way FRC?
        SwerveModuleState[] moduleStates=DriveConstants.driveKinematics.toSwerveModuleStates(speeds);

        //this one line is the culmination of everything. Look in previous files for info on how this works
        driveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted){
        driveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
