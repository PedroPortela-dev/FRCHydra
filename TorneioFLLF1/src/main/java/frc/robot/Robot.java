package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.P_Joystick;

public class Robot extends TimedRobot {
    
  //Motores
  VictorSPX M_Right = new VictorSPX(1);
  VictorSPX M_Right2 = new VictorSPX(2);
  VictorSPX M_Left = new VictorSPX(3);
  VictorSPX M_Left2 = new VictorSPX(4);
  VictorSPX M_Armazém = new VictorSPX(7);
  VictorSPX M_Lançamento = new VictorSPX(6);
  
  //Joystick
  P_Joystick S_Left = new P_Joystick("Esquerdo");
  P_Joystick S_Right = new P_Joystick("Direito");
  P_Joystick G_Left = new P_Joystick("G_Esquerdo");
  P_Joystick G_Right = new P_Joystick("G_Direito");
  Joystick Stick = new Joystick(0);
  Joystick Stick2 = new Joystick(1);

  boolean[] b = new boolean[4];

  double[] p = new double[4];
  final int T_ROTINA = 30;
  static final double DELAY = 0.05;
  double[][] vetPower = new double[2][(int)(T_ROTINA/DELAY)];
  boolean gravador;
  int i;

  public Robot(){
    super(DELAY);
  }
  @Override
  public void robotInit(){
    
    //Motores Servos
    M_Right2.follow(M_Right);
    M_Left2.follow(M_Left);

    //Configura Modo Neutro para 4%
    M_Left.configNeutralDeadband(0.04);
    M_Left2.configNeutralDeadband(0.04);
    M_Right.configNeutralDeadband(0.04);
    M_Right2.configNeutralDeadband(0.04);

    //Travar motores caso força seja 0
    M_Left.setNeutralMode(NeutralMode.Brake);
    M_Left2.setNeutralMode(NeutralMode.Brake);
    M_Right.setNeutralMode(NeutralMode.Brake);
    M_Right2.setNeutralMode(NeutralMode.Brake);
    M_Armazém.setNeutralMode(NeutralMode.Brake);
    M_Lançamento.setNeutralMode(NeutralMode.Brake);
    
  }
  @Override
  public void teleopInit() {
    java.util.Arrays.fill(vetPower, 0);
    gravador=false;
    S_Left.reset();
    S_Right.reset();
  }
  @Override
  public void teleopPeriodic(){

      S_Left.t_ResetNeutral();
      S_Right.t_ResetNeutral();

      // calculo das forças
      if(G_Left.j_GetPowerTrigger()!=0){
        G_Left.p_setPowerTrigger();
      }
      else if(G_Right.j_GetPowerTrigger()!=0){
        G_Right.p_setPowerTrigger();
      }
      else if(S_Left.j_SetPower()!=0){
        S_Left.p_setPowerStick();
      }
      else if(S_Right.j_SetPower()!=0){
        S_Right.p_setPowerStick();
      }else{
        SmartDashboard.putString("Condição","Nenhuma");
        p[0]=0;
        p[1]=0;
      }

      if(P_Joystick.pov()!=-1){
        int[][][] POV = 
        {
          {
            {0,270, 315},
            {90, 180, 225}
          },
          {
            {0,45, 90},
            {135, 180, 270}
          }
        };
        p[0]=p_getPowerPOV(POV[0][0], POV[0][1]);
        p[1]=p_getPowerPOV(POV[1][0], POV[1][1]);
        SmartDashboard.putString("Condição", "POV");
      }else{
        p[0]=P_Joystick.p[0];
        p[1]=P_Joystick.p[1];
      }

      // execução dos motores da locomoção

      SmartDashboard.putNumber("Power Right", p[0]);
      SmartDashboard.putNumber("Power Left", p[1]);
      M_Left.set(ControlMode.PercentOutput, p[0]);
      M_Right.set(ControlMode.PercentOutput, -p[1]);
      

      //gravação

      if(Stick.getRawButtonPressed(4)){
        gravador=!gravador;
        i=0;
      }
      if(gravador){
          SmartDashboard.putBoolean("Gravador:", true);
          vetPower[0][i]=p[0];
          vetPower[1][i]=p[1];
          i++;
      }else{
        SmartDashboard.putBoolean("Gravador:", false);
      }
      SmartDashboard.putNumberArray("motor esquerdo", vetPower[0]);
      SmartDashboard.putNumberArray("motor direito", vetPower[1]);

      //Lançamento

      b[0] = Stick2.getRawButton(1);
      b[1] = Stick2.getRawButton(3);
      b[2] = Stick2.getRawButton(2);

      p[2] = (b[0])?1:(b[1])?0.75:(b[2])?0.5:0;
      p[3]= (b[0] || b[1] || b[2])?1:0;

      M_Armazém.set(ControlMode.PercentOutput, p[3]);
      M_Lançamento.set(ControlMode.PercentOutput, -p[2]);
      SmartDashboard.putNumber("ForcaLancamento", p[2]);
  }
  @Override
  public void autonomousInit() {
    i=0;
  }
  @Override
  public void autonomousPeriodic() {
    
    M_Left.set(ControlMode.PercentOutput, vetPower[0][i]);
    M_Right.set(ControlMode.PercentOutput, -vetPower[1][i]);
    SmartDashboard.putNumber("motor esquerdo", vetPower[0][i]);
    SmartDashboard.putNumber("motor direito", vetPower[1][i]);
    i++;
  }

  public double p_getPowerPOV(int[] condiçãoForcaDireta, int[] condiçãoForcaInversa){
    return
    (P_Joystick.pov()==condiçãoForcaDireta[0] || P_Joystick.pov()==condiçãoForcaDireta[1] || P_Joystick.pov()==condiçãoForcaDireta[2])?
    P_Joystick.setBt():
    (P_Joystick.pov()==condiçãoForcaInversa[0] || P_Joystick.pov()==condiçãoForcaInversa[1] || P_Joystick.pov()==condiçãoForcaInversa[2])?
    -P_Joystick.setBt():
    0;
  }
}