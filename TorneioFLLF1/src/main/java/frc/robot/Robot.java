package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  
  //Motores
  VictorSPX M_right = new VictorSPX(1);
  VictorSPX M_right2 = new VictorSPX(2);
  VictorSPX M_left = new VictorSPX(3);
  VictorSPX M_left2 = new VictorSPX(4);
  VictorSPX M_armazem = new VictorSPX(7);
  VictorSPX M_lancamento = new VictorSPX(6);
  
  //Joystick
  Joystick J_stick = new Joystick(0);
  Joystick J_stick2 = new Joystick(1);

  Timer T_Timer[] = new Timer[2];

  int tempoRotina = 30;
  int pov;
  int i;
  double limMin=0.2, medio=0.4, limMax=1, bt=limMin;
  double[] p = new double[4];
  boolean a, b, x, y, davi, daviA, daviX, daviB, gravador, b_Minimo, b_Medio, b_Maximo;
  double[][] vetpower = new double[2][(int)(tempoRotina/0.05)];

  public Robot(){
    super(0.05);
  }
  @Override
  public void robotInit(){
    
    //Motores Servos
    M_right2.follow(M_right);
    M_left2.follow(M_left);

    //Configura Modo Neutro para 4%
    M_left.configNeutralDeadband(0.04);
    M_left2.configNeutralDeadband(0.04);
    M_right.configNeutralDeadband(0.04);
    M_right2.configNeutralDeadband(0.04);

    //Travar motores caso força seja 0
    M_left.setNeutralMode(NeutralMode.Brake);
    M_left2.setNeutralMode(NeutralMode.Brake);
    M_right.setNeutralMode(NeutralMode.Brake);
    M_right2.setNeutralMode(NeutralMode.Brake);
    M_armazem.setNeutralMode(NeutralMode.Brake);
    M_lancamento.setNeutralMode(NeutralMode.Brake);
    
  }
  @Override
  public void teleopInit() {
    java.util.Arrays.fill(vetpower, 0);
    gravador=false;
    b_Minimo=false;
    b_Medio=false;
    b_Maximo=false;
    bt=limMin;
    t_Reset(0);
    t_Reset(1);
  }
 @Override
 public void teleopPeriodic(){

  // Limite da força
  a=J_stick.getRawButton(1);
  b=J_stick.getRawButton(2);
  x=J_stick.getRawButton(3);

  if(b){
    b_Minimo=true;
    b_Medio=false;
    b_Maximo=false;
    bt=limMin;
  }else if(a){
    b_Minimo=false;
    b_Medio=true;
    b_Maximo=false;
    bt=medio;
  }else if(x){
    b_Minimo=false;
    b_Medio=false;
    b_Maximo=true;
    bt=limMax;
  }
  SmartDashboard.putBoolean("Minimo", b_Minimo);
  SmartDashboard.putBoolean("Medio", b_Medio);
  SmartDashboard.putBoolean("Maximo", b_Maximo);

  // atribuido valores a variaveis

  pov=J_stick.getPOV();
  // zerando as variaveis em caso de neutro

  t_ResetNeutral(0, 1, 0);
  t_ResetNeutral(4, 5, 1);

  // calculo das forças
  if(j_GetPowerTrigger(3, 0)!=0 || j_GetPowerTrigger(2, 1)!=0)
  {
    p_setPowerTrigger(0, 1, 3, 0, 1, "GatilhoDireito");
    p_setPowerTrigger(1, 0, 2, 1,-1, "GatilhoEsquerdo");
  }
  else if(j_SetPower(0, 1, 0)!=0 || j_SetPower(3, 4, 1)!=0){
    p_setPowerStick(0, 1, 0, 1, 0, 1, "Joystick Esquerdo");
    p_setPowerStick(1, 0, 3, 4, 1,-1, "Joystick Direito");
  }
  else if(pov!=-1){
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
    SmartDashboard.putString("Condicao", "POV");
  }else{
    SmartDashboard.putString("Condicao","Nenhuma");
    p[0]=0;
    p[1]=0;
  }

  // execução dos motores da locomoção

  SmartDashboard.putNumber("Power Right", p[0]);
  SmartDashboard.putNumber("Power Left", p[1]);
  M_left.set(ControlMode.PercentOutput, p[0]);
  M_right.set(ControlMode.PercentOutput, -p[1]);
  

  //gravação

  if(J_stick.getRawButtonPressed(4)){
    gravador=!gravador;
    i=0;
  }
  if(gravador){
      SmartDashboard.putBoolean("Gravador:", true);
      vetpower[0][i]=p[0];
      vetpower[1][i]=p[1];
      i++;
  }else{
    SmartDashboard.putBoolean("Gravador:", false);
  }
  SmartDashboard.putNumberArray("motor esquerdo", vetpower[0]);
  SmartDashboard.putNumberArray("motor direito", vetpower[1]);

  //Lançamento

  daviA = J_stick2.getRawButton(1);
  daviB = J_stick2.getRawButton(2);
  daviX = J_stick2.getRawButton(3);

  p[2] = (daviA)?1:(daviX)?0.75:(daviB)?0.5:0;
  p[3]= (daviA || daviB || daviX)?1:0;

  M_armazem.set(ControlMode.PercentOutput, p[3]);
  M_lancamento.set(ControlMode.PercentOutput, -p[2]);
  SmartDashboard.putNumber("ForcaLancamento", p[2]);
 }
 @Override
 public void autonomousInit() {
   i=0;
 }
 @Override
 public void autonomousPeriodic() {
  
  M_left.set(ControlMode.PercentOutput, vetpower[0][i]);
  M_right.set(ControlMode.PercentOutput, -vetpower[1][i]);
  SmartDashboard.putNumber("motor esquerdo", vetpower[0][i]);
  SmartDashboard.putNumber("motor direito", vetpower[1][i]);
  i++;
 }

 public void t_Reset(int t){

  T_Timer[t].reset();
  T_Timer[t].start();
 }

 public void t_ResetNeutral(int x, int y, int t){

  if(j_SetPower(x, y, t) <0.1 && j_GetPowerTrigger(3, 0)==0 && j_GetPowerTrigger(2, 1)==0) t_Reset(t);

 }

 public double j_SetEixo(int b){

  return (Math.abs(J_stick.getRawAxis(b))<0.1)?0:J_stick.getRawAxis(b);
 }

 public double t_SetTimer(int c){

  return (T_Timer[c].get()>=0.5)?1:T_Timer[c].get()*2;
 }

 public double j_SetSen(int x, int y){

  return j_SetEixo(y)/Math.hypot(j_SetEixo(x), j_SetEixo(y));
 }

 public double j_SetPower(int x, int y, int t){

  return Math.hypot(j_SetEixo(x), j_SetEixo(y))*t_SetTimer(t)*bt;
 }

 public double j_GetPowerTrigger(int trigger, int t){

  return J_stick.getRawAxis(trigger)*bt*t_SetTimer(t);
 }

 public double p_PercentualForca(double x, double limMinX, double limMaxX, double limMinY, double limMaxY){

  double a = (limMaxY-limMinY)/(limMaxX-limMinX);

  return a*(x - limMinX)+limMinY;
 }

 public void p_setPowerTrigger(int a, int b, int c, int d, int e, String condicao){
  if(j_GetPowerTrigger(c, d)!=0){
    p[(j_SetEixo(0)>=0)?a:b]=e*p_PercentualForca(j_SetEixo(0), 0, (j_SetEixo(0)>=0)?1:-1, 1, 0)*j_GetPowerTrigger(c, d);
    p[(j_SetEixo(0)>=0)?b:a]=e*j_GetPowerTrigger(c, d);
    SmartDashboard.putString("Condicao", condicao);
  }
 }

 public void p_setPowerStick(int a, int b, int c, int d, int e, int f, String condicao){
  if(j_SetPower(c, d, e)!=0){
    p[(j_SetEixo(0)/j_SetEixo(1)>=0)?0:1]=f*((j_SetEixo(1)>=0)?1:-1)*p_PercentualForca(j_SetSen(c, d), 0, (j_SetEixo(1)>=0)?1:-1, 0, 1)*j_SetPower(c, d, e);
    p[(j_SetEixo(0)/j_SetEixo(1)>=0)?1:0]=f*((j_SetEixo(1)>=0)?1:-1)*j_SetPower(c, d, e);
    SmartDashboard.putString("Condicao", condicao);
  }
 }

 public double p_getPowerPOV(int[] condicaoForcaDireta, int[] condicaoForcaInversa){
   return (pov==condicaoForcaDireta[0] || pov==condicaoForcaDireta[1] || pov==condicaoForcaDireta[2])?bt:(pov==condicaoForcaInversa[0] || pov==condicaoForcaInversa[1] || pov==condicaoForcaInversa[2])?-bt:0;
 }
}