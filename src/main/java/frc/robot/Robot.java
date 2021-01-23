/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//imports
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class Robot extends TimedRobot {
  // JOYSTICKS
  private final Joystick driveStick = new Joystick(0);
  private final Joystick shootStick = new Joystick(1);
  // MOTORS
  //private final VictorSP m_leftMotor = new VictorSP(0);
  //private final VictorSP m_rightMotor = new VictorSP(1);
  private CANSparkMax leftMotorF = new CANSparkMax(5, MotorType.kBrushless);
  private CANSparkMax leftMotorB = new CANSparkMax(6, MotorType.kBrushless);
  private CANSparkMax rightMotorF = new CANSparkMax(7, MotorType.kBrushless);
  private CANSparkMax rightMotorB = new CANSparkMax(8, MotorType.kBrushless);


  //private final Spark leftShooter = new Spark(8);
  //private final Spark rightShooter = new Spark(9);
  //private final Victor collector = new Victor(3);
  private final Talon flopper = new Talon(2);
  private final Talon aimer = new Talon(5);
  private CANSparkMax indexer = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax leftShooter = new CANSparkMax(2, MotorType.kBrushed);
  private CANSparkMax rightShooter = new CANSparkMax(3, MotorType.kBrushed);
  private CANSparkMax collector = new CANSparkMax(4, MotorType.kBrushless);

  // ENCODERS
  private final Encoder aimerEncoder = new Encoder(0, 1);
  private CANEncoder indexEncoder = indexer.getEncoder(EncoderType.kHallSensor, 1);
  private CANEncoder leftEncoder = leftMotorF.getEncoder(EncoderType.kHallSensor, 1);
  private CANEncoder rightEncoder = rightMotorF.getEncoder(EncoderType.kHallSensor, 1);

  //PID CONTROLLERS 
  private CANPIDController leftMotorFPID = leftMotorF.getPIDController();
  private CANPIDController leftMotorBPID = leftMotorB.getPIDController(); 
  private CANPIDController rightMotorFPID = rightMotorF.getPIDController();
  private CANPIDController rightMotorBPID = rightMotorB.getPIDController();
  public double kP = 0.1; 
  public double kI = 1e-4;
  public double kD = 1; 
  public double kIz = 0; 
  public double kFF = 0; 
  public double kMaxOutput = 1; 
  public double kMinOutput = -1;

  // LIMIT SWITCHES
  DigitalInput upSwitch, downSwitch;

  // DRIVETRAIN
  //private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftMotorF, leftMotorB);
  private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightMotorF, rightMotorB);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(leftMotors, rightMotors);
  // NETWORK TABLES
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  //ADDITIONAL SENSORS
  /*private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private Rev2mDistanceSensor distOnboard; 
  Gyro euro = new AnalogGyro(0);*/
  // VARIABLES
  public boolean isCollectorOn;
  public boolean isIndexerOn;
  public boolean isCollectorBackwards;
  public boolean isIndexerBackwards;
  public boolean readyToIndex;
  public double wantedIndex;
  public double shootIndex;
  public double ballCount;

  final Timer t = new Timer();

  @Override
  public void robotInit() {
    //SENSORS
    CameraServer.getInstance().startAutomaticCapture();
    //m_colorMatcher.addColorMatch(kYellowTarget);
    /*distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
    distOnboard.setRangeProfile(RangeProfile.kHighSpeed);
    distOnboard.setRangeProfile(RangeProfile.kHighAccuracy);
    distOnboard.setRangeProfile(RangeProfile.kLongRange);
    distOnboard.setRangeProfile(RangeProfile.kDefault);
    //enable the Distance Sensor background Thread
    distOnboard.setAutomaticMode(true);*/
    //CAN AND ENCODERS

    aimerEncoder.setDistancePerPulse(1.0/650);
    resetIndexer();
    ballCount = 0;
    //LIMIT SWITCHES
    upSwitch = new DigitalInput(2);
    downSwitch = new DigitalInput(3);

    leftMotorFPID.setP(kP);
    leftMotorFPID.setI(kI);
    leftMotorFPID.setD(kD);
    leftMotorFPID.setIZone(kIz);
    leftMotorFPID.setFF(kFF);
    leftMotorFPID.setOutputRange(kMinOutput, kMaxOutput);

    leftMotorBPID.setP(kP);
    leftMotorBPID.setI(kI);
    leftMotorBPID.setD(kD);
    leftMotorBPID.setIZone(kIz);
    leftMotorBPID.setFF(kFF);
    leftMotorBPID.setOutputRange(kMinOutput, kMaxOutput);

    rightMotorFPID.setP(kP);
    rightMotorFPID.setI(kI);
    rightMotorFPID.setD(kD);
    rightMotorFPID.setIZone(kIz);
    rightMotorFPID.setFF(kFF);
    rightMotorFPID.setOutputRange(kMinOutput, kMaxOutput);

    rightMotorBPID.setP(kP);
    rightMotorBPID.setI(kI);
    rightMotorBPID.setD(kD);
    rightMotorBPID.setIZone(kIz);
    rightMotorBPID.setFF(kFF);
    rightMotorBPID.setOutputRange(kMinOutput, kMaxOutput);

    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);

    t.start();
  }

public void autonomousPeriodic() {

}

  public void teleopPeriodic() {
    //REPEATEDLY GET THESE VALUES
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double distanceAngle = Math.toRadians(y);
    double distance = 74.25 / Math.tan(0.401426 + distanceAngle);
    double aimerEncoderValue = (aimerEncoder.getDistance());
    double indexPosition = indexEncoder.getPosition();

    //More Sensing
   // rangeSensor();
    //colorSensor();

    // SMART DASHBOARD
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("distance", distance);
    SmartDashboard.putNumber("Aimer", aimerEncoder.getDistance());
    SmartDashboard.putNumber("IndexCoder", indexPosition);
    SmartDashboard.putNumber("wanted index", wantedIndex);
    SmartDashboard.putNumber("ball count", ballCount);
    SmartDashboard.putNumber("Left Enoder", leftEncoder.getPosition());

  //RUN PARTS OF THE ROBOT WHEN BUTTONS PRESSED
  countBalls();
  manualAim();
  heShootsHeScores(indexPosition);


  if(collectorRun())
  runCollector(.4);
  if(collectorBack())
  runCollector(-.75);
  if(collectorStop() || collectorBackStop())
  runCollector(0);
  if (pleaseStop())
  stopThePress();
  if (shooterPrimedPressed()){
    resetIndexer();
    runShooter(-1);}
  if (fwoooopTimePressed()){
    resetIndexer();
    ballCount = ballCount + 1;
  }
  fwoooop(indexPosition);
  if (fire()){
    runIndexer(.75);
    runCollector(1);}
  if(driveStick.getTriggerReleased()){
    runIndexer(0);
    runShooter(0);
    runCollector(0);
    ballCount = 0;
  }
  if (shootStick.getRawButton(12))
  resetAimer();
  if(stopShooter()){
    runShooter(0);
  }
  if (stopIndexer()){
    runIndexer(0);
  }

  flopIt();
  if (aimingEngaged()) {
    aim(x, aimerEncoderValue, distance);
  } 
  else drive(-driveStick.getY(), driveStick.getX(), driveStick.getRawButton(2));
}
  //FUNCTIONS

  public void heShootsHeScores(double indexPosition) {
    if(shooterPrimed()){
      wantedIndex = -.11;
    }
    IndexBackwards(wantedIndex, 1, indexPosition);}

  public void fwoooop(double indexPosition) {
    if(fwoooopTime()) {
      wantedIndex = .40+((ballCount-1)*.25);}
    indexForward(wantedIndex, 1, indexPosition);
  }


  public void flopIt() {
    if(shootStick.getRawButton(6) && (upSwitch.get()))
      flopper.set(-1);
    else if(shootStick.getRawButton(4) && (downSwitch.get()))
      flopper.set(.25);
    else flopper.set(0);
  }

  public void aim(double h, double aimerEncoderValue, double distance) {
    // Heading Error
    double x = h;
    double steering_adjust = 0;
    if (x > .32)
      steering_adjust = .235 + (.015) * Math.log(x);
    if (x < -.32)
      steering_adjust = -(.235 + (.015) * Math.log(-x));
    m_robotDrive.tankDrive(steering_adjust, -steering_adjust);
    if ((x < -.32) && (x > .32))
    {
      //adjust(distance);
    }
  }

  public void adjust(double distance) {
    double currentWormDistance = (aimerEncoder.getDistance()/ 2.62);
    //needs Math
    double currentAngle = currentWormDistance;
    double wantedAngle = (.1733 * ((distance*distance)/144) - 4.8*(distance/12) + 58.667);
    if(currentAngle < wantedAngle)
    aimUp(wantedAngle, currentAngle);
    if(currentAngle > wantedAngle)
    aimDown(wantedAngle, currentAngle);
  }

  public void aimDown(double wantedAngle, double currentAngle) {
    //needs Math
  double aimerSpeed = wantedAngle - currentAngle;
  aimer.set(aimerSpeed);
  }

  public void aimUp(double wantedAngle, double currentAngle) {
    //needs Math
    double aimerSpeed = wantedAngle - currentAngle;
    aimer.set(-aimerSpeed);
  }

  public void drive(double speed, double turn, boolean quickTurn) {
    m_robotDrive.curvatureDrive(speed, 0, false);
  }

  /*public void rangeSensor(){
    if(distOnboard.isRangeValid()) {
      SmartDashboard.putNumber("Range Onboard", distOnboard.getRange());
      SmartDashboard.putNumber("Timestamp Onboard", distOnboard.getTimestamp());
    }
  }*/

  /*public void colorSensor(){
    Color detectedColor = m_colorSensor.getColor();
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchColor(detectedColor);
  
    if (match.color == kYellowTarget) {
      colorString = "Yes";
    } else {
      colorString = "No";
    }
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Index Now", colorString);
  }*/

  public void indexForward(double wantedIndexIn, double indexSpeedMagnitude, double indexPosition){
    double wantedIndexForward = wantedIndexIn;
    if(wantedIndexForward > 0){
    double indexSpeed = indexSpeedMagnitude;
    if (indexPosition < wantedIndexForward){
      indexer.set(indexSpeed);
  }
   if(indexPosition >= wantedIndexForward){ 
     indexer.set(0);
     wantedIndexForward = 0;
    }
  }
  }

  public void IndexBackwards(double wantedIndexIn, double indexSpeedMagnitude, double indexPosition){
    double wantedIndexBackwards = (wantedIndexIn);
    if(wantedIndexBackwards < 0){
    double indexSpeed = (-indexSpeedMagnitude);
    if(indexPosition > wantedIndexBackwards){
      indexer.set(indexSpeed);
  }
    if(indexPosition < wantedIndexBackwards && wantedIndexBackwards != 0){ 
      wantedIndexBackwards = 0;
      indexer.set(0);
    }
  }
    SmartDashboard.putNumber("wanted Index back", wantedIndex);
  }

  public void runIndexer(double indexerSpeed){
    indexer.set(indexerSpeed);
  }

  public void runShooter(double shootSpeed){
    leftShooter.set(shootSpeed);
    rightShooter.set(-shootSpeed);
  }

  public void runCollector(double collectorSpeed){
    collector.set(collectorSpeed);
  }

  public void resetIndexer(){
    indexEncoder.setPosition(0);
  }

  public void resetAimer(){
    aimerEncoder.reset();
  }

  public void checkButtons(){
    aimingEngaged();
    shooterPrimed();
    shooterPrimedPressed();
    fwoooopTime();
    fwoooopTimePressed();
    fire();
    stopShooter();
    goingUp();
    goingDown();
    stopIndexer();
    pleaseStop();
  }

  public boolean aimingEngaged() {
    return shootStick.getRawButton(2);
  }

  public boolean shooterPrimed() {
    return shootStick.getRawButton(7);
  }
  public boolean shooterPrimedPressed() {
    return shootStick.getRawButtonPressed(7);
  }
  public boolean fwoooopTime() {
    return shootStick.getTrigger();
  }
  public boolean fwoooopTimePressed() {
    return shootStick.getTriggerPressed();
  }
  public boolean fire() {
    return driveStick.getTrigger();
  }
  public boolean stopShooter() {
    return shootStick.getRawButtonPressed(8);
  }
  public boolean goingUp(){
   return driveStick.getRawButtonPressed(5);
  }
  public boolean goingDown(){
    return driveStick.getRawButtonPressed(3);
  }
  public boolean stopIndexer() {
    return shootStick.getRawButtonPressed(10);
  }
  public boolean pleaseStop(){
    return driveStick.getRawButton(7);
  }
  public void stopThePress(){
    indexer.set(0);
    collector.set(0);
    runShooter(0);
    drive(0, 0, false);
    flopper.set(0);
    aimer.set(0);
  }

  public void reverseIndex(){
    if(shootStick.getRawButton(9)){
      runIndexer(.5);
    }
  }

  public void manualAim(){
    if(shootStick.getY() >= .5 || shootStick.getY() <= -.5)
    aimer.set(-shootStick.getY());
    else aimer.set(0);
  }

  public void countBalls(){
    if(ballCount >= 5) ballCount = 5;
    if(ballCount < 0) ballCount = 0;
    if(shootStick.getRawButtonPressed(11)) ballCount = ballCount -1;
  } 

  public void turnDegrees(double turnDegrees, double moveSpeedMagnitude){
    double turnState = 0;
    double turnState1 = 0; 
    double  leftMoveSpeed = 0;
    double  rightMoveSpeed = 0;
    double ticksPerDegree = 0;
    double leftMotorValue = leftEncoder.getPosition();
    double leftFinalTicks = ticksPerDegree * turnDegrees + leftMotorValue;
    double rightMotorValue = rightEncoder.getPosition();
    double rightFinalTicks = ticksPerDegree * turnDegrees + rightMotorValue;
    if(leftFinalTicks != leftMotorValue){
      if(leftFinalTicks > leftMotorValue){
        leftMoveSpeed = moveSpeedMagnitude;
      }
      else { leftMoveSpeed = -moveSpeedMagnitude;}
    }
    if(rightFinalTicks != rightMotorValue){
      if(rightFinalTicks > rightMotorValue){
        rightMoveSpeed = -moveSpeedMagnitude;
      }
      else { rightMoveSpeed = moveSpeedMagnitude;}
    }
    if(false){
    //kill me
    leftMoveSpeed = moveSpeedMagnitude;
    }
  }

  public void driveDistance(double driveDistance){
    double rotations = driveDistance / 7;
    leftMotorFPID.setReference(rotations, ControlType.kPosition);
    leftMotorBPID.setReference(rotations, ControlType.kPosition);
    rightMotorFPID.setReference(rotations, ControlType.kPosition);
    rightMotorBPID.setReference(rotations, ControlType.kPosition);

  }

  public boolean collectorRun(){
    return shootStick.getRawButton(3);
  }
  public boolean collectorStop(){
    return shootStick.getRawButtonReleased(3);
  }
  public boolean collectorBack(){
    return shootStick.getRawButton(5);
  }
  public boolean collectorBackStop(){
    return shootStick.getRawButtonReleased(5);
  }


}