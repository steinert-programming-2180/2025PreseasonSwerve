package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;
    private final PIDController turnPIDController;
    private final AnalogInput absEncoder;
    private final boolean absEncoderReversed;
    private final double absEncoderOffset;

//just defines everything. Seperated each requirement into its own line to make it easier to read.
    public SwerveModule(
        int driveMotorID,
        int turnMotorID,
        int absEncoderID,
        double absEncoderOffset,
        boolean driveMotorReversed,
        boolean turnMotorReversed,
        boolean absEncoderReversed){
            driveMotor=new CANSparkMax(driveMotorID, MotorType.kBrushless);
            turnMotor=new CANSparkMax(turnMotorID, MotorType.kBrushless);
            driveMotor.setInverted(driveMotorReversed);
            turnMotor.setInverted(turnMotorReversed);

            driveEncoder=driveMotor.getEncoder();
            turnEncoder=turnMotor.getEncoder();
            absEncoder=new AnalogInput(absEncoderID);

            this.absEncoderOffset=absEncoderOffset;
            this.absEncoderReversed=absEncoderReversed;
            driveEncoder.setPositionConversionFactor(ModuleConstants.driveEncoderConv);
            driveEncoder.setVelocityConversionFactor(ModuleConstants.driveEncoderConvROC);
            turnEncoder.setPositionConversionFactor(ModuleConstants.turnEncoderConv);
            turnEncoder.setVelocityConversionFactor(ModuleConstants.turnEncoderConvROC);

            turnPIDController=new PIDController(ModuleConstants.turningP,ModuleConstants.turningI,ModuleConstants.turningD);
            turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

            resetEncoders();
    }

    //basic functions to get encoder values
    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }
    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }
    public double getTurnPosition(){
        return turnEncoder.getPosition();
    }
    public double getTurnVelocity(){
        return turnEncoder.getVelocity();
    }
    public double getAbsEncoderRad(){
        if(absEncoderReversed){
            return (((absEncoder.getVoltage()/RobotController.getVoltage5V())*2.0*Math.PI)-absEncoderOffset)*-1;
        }
            return ((absEncoder.getVoltage()/RobotController.getVoltage5V())*2.0*Math.PI)-absEncoderOffset;
    }

    public void resetEncoders(){
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsEncoderRad());
    }

    //creates a SwerveModuleState object out of the information in the file
    public SwerveModuleState getSwerveModuleState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    }

    //takes a SwerveModuleState object and uses it to update the current motor values
    public void setDesiredState(SwerveModuleState newState){
        if(Math.abs(newState.speedMetersPerSecond)<0.001){
            STOPRIGHTNOWRAAAAAH();
            return;
        }
        newState=SwerveModuleState.optimize(newState, getSwerveModuleState().angle);
        driveMotor.set(newState.speedMetersPerSecond/DriveConstants.maxSpeed);
        turnMotor.set(turnPIDController.calculate(getTurnPosition(), newState.angle.getRadians()));
        SmartDashboard.putString("Swerve Module["+absEncoder.getChannel()+"] state",newState.toString());
    }

    public void STOPRIGHTNOWRAAAAAH(){
        driveMotor.set(0);
        turnMotor.set(0);
    }

}
