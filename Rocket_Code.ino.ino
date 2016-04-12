#include <MatrixMath.h>

float accx, accy, accz, roll, pitch, yaw, p, Cd, m, A, T,tk=0,tk1=0;
float  phi[6][6];
float P[6][6];
float Pbar[6][6];
float G[6][6];
float  xhat[6][1];
float xbar[6][1];
float  Rv[6][6];
float Rw[6][6];
float R[3][3];
float R0[3][3];
float xintg[6][1];
float Rvinv[6][6];
float y[6][1];
int counter = 1;
enum Loopstate{PRELAUNCH,LAUNCH,LANDED};
Loopstate loopstate = PRELAUNCH;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
}

void loop() {
  // put your main code here, to run repeatedly:
String msg = "";
String msg1 = "";
  switch(loopstate){
        case PRELAUNCH:
  if(Serial.available()>0)
  {
    while(Serial.available())
    {
      msg += char(Serial.read());
      delay(25);
    }
  if(msg[0] == 'm')
  {
    Serial.println("this value corresponds to mass");
    for(int i = 1;i < msg.length(); i++){
      msg1 += msg[i];
      delay(25);
    }
    m = msg1.toFloat();
    Serial.println(m,DEC);
  }
  else if(msg[0] == 'C')
  {
    Serial.println("this value corresponds to drag coefficient");
    for(int i = 1;i < msg.length(); i++){
      msg1 += msg[i];
      delay(25);
    }
    Cd = msg1.toFloat();
    Serial.println(Cd,DEC);
  }
  else if(msg[0] == 'A')
  {
    Serial.println("This value corresponds to Area");
    for(int i = 1;i < msg.length(); i++){
      msg1 += msg[i];
      delay(25);
    }
    A = msg1.toFloat();
    Serial.println(A,DEC);
  }
  else if(msg[0] == 'T')
  {
    Serial.println("this value corresponds to Thrust");
    for(int i = 1;i < msg.length(); i++){
      msg1 += msg[i];
      delay(25);
    }
    T = msg1.toFloat();
    Serial.println(T,DEC);
  }
    else if(msg[0] == 'L')
  {
    Serial.println("Launching");
    loopstate = LAUNCH;
  }
  else
  {
    Serial.println("not a valid entry");
  }
  }
        break;
        case LAUNCH:
          if(counter == 1){
            tk1 = 0;
          }
          else
          {
            tk1 = millis();
          }
          Serial.println("I am in launch mode");
          phicalc();
          xbarcalc();
          Pbarcalc();
          Gcalc();
          xhatcalc();
          Pcalc();
          counter += 1;
          tk = tk1;
        break;
        case LANDED:
          
          
        break;
    }
}





// used functions:
void phicalc()
{
  // enter phi equation here
  double dt = tk1 - tk;
  phi[0][0] = 1;
  phi[0][1] = 0;
  phi[0][2] = 0;
  phi[1][0] = 0;
  phi[1][1] = 1;
  phi[1][2] = 0;
  phi[2][0] = 0;
  phi[2][1] = 0;
  phi[2][2] = 1;
  phi[3][0] = 0;
  phi[3][1] = 0;
  phi[3][2] = 0;
  phi[4][0] = 0;
  phi[4][1] = 0;
  phi[4][2] = 0;
  phi[5][0] = 0;
  phi[5][1] = 0;
  phi[5][2] = 0;
  // enter in equations
  phi[0][3] = (20*R[0][2]*m*xhat[3][0] + sq(R[1][2])*dt*p*sq(xhat[4][0]) + sq(R[2][2])*dt*p*sq(xhat[5][0]) - 20*R[0][2]*m*xhat[3][0]*exp(-(dt*p*(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]))/(20*m)) + R[0][2]*R[1][2]*dt*p*xhat[3][0]*xhat[4][0] + R[0][2]*R[2][2]*dt*p*xhat[3][0]*xhat[5][0] + 2*R[1][2]*R[2][2]*dt*p*xhat[4][0]*xhat[5][0])/(p*sq(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]));
  phi[0][4] = -(20*R[0][2]*m*xhat[4][0]*exp(-(dt*p*(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]))/(20*m)) - 20*R[0][2]*m*xhat[4][0] + R[0][2]*R[1][2]*dt*p*sq(xhat[4][0]) + sq(R[0][2])*dt*p*xhat[3][0]*xhat[4][0] + R[0][2]*R[2][2]*dt*p*xhat[4][0]*xhat[5][0])/(p*sq(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]));
  phi[0][5] = -(20*R[0][2]*m*xhat[5][0]*exp(-(dt*p*(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]))/(20*m)) - 20*R[0][2]*m*xhat[5][0] + R[0][2]*R[2][2]*dt*p*sq(xhat[5][0]) + sq(R[0][2])*dt*p*xhat[3][0]*xhat[5][0] + R[0][2]*R[1][2]*dt*p*xhat[4][0]*xhat[5][0])/(p*sq(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]));
  phi[1][3] = -(20*R[1][2]*m*xhat[3][0]*exp(-(dt*p*(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]))/(20*m)) - 20*R[1][2]*m*xhat[5][0] + R[0][2]*R[1][2]*dt*p*sq(xhat[3][0]) + sq(R[1][2])*dt*p*xhat[3][0]*xhat[4][0] + R[1][2]*R[2][2]*dt*p*xhat[4][0]*xhat[5][0])/(p*sq(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]));
  phi[1][4] =  (20*R[1][2]*m*xhat[4][0] + sq(R[0][2])*dt*p*sq(xhat[3][0]) + sq(R[2][2])*dt*p*sq(xhat[5][0]) - 20*R[1][2]*m*xhat[4][0]*exp(-(dt*p*(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]))/(20*m)) + R[0][2]*R[1][2]*dt*p*xhat[3][0]*xhat[4][0] + 2*R[0][2]*R[2][2]*dt*p*xhat[3][0]*xhat[5][0] + R[1][2]*R[2][2]*dt*p*xhat[4][0]*xhat[5][0])/(p*sq(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]));
  phi[1][5] = -(R[1][2]*(20*m*xhat[5][0]*exp(-(dt*p*(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]))/(20*m)) - 20*m*xhat[5][0] + R[2][2]*dt*p*sq(xhat[5][0]) + R[0][2]*dt*p*xhat[3][0]*xhat[5][0] + R[1][2]*dt*p*xhat[4][0]*xhat[5][0]))/(p*sq(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]));
  phi[2][3] = -(20*R[2][2]*m*xhat[3][0]*exp(-(dt*p*(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]))/(20*m)) - 20*R[2][2]*m*xhat[3][0] + R[0][2]*R[2][2]*dt*p*sq(xhat[3][0]) + sq(R[2][2])*dt*p*xhat[3][0]*xhat[5][0] + R[1][2]*R[2][2]*dt*p*xhat[3][0]*xhat[4][0])/(p*sq(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]));
  phi[2][4] = -(20*R[2][2]*m*xhat[4][0]*exp(-(dt*p*(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]))/(20*m)) - 20*R[2][2]*m*xhat[4][0] + R[1][2]*R[2][2]*dt*p*sq(xhat[4][0]) + sq(R[2][2])*dt*p*xhat[4][0]*xhat[5][0] + R[0][2]*R[2][2]*dt*p*xhat[3][0]*xhat[4][0])/(p*sq(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]));
  phi[2][5] = (20*R[2][2]*m*xhat[5][0] + sq(R[0][2])*dt*p*sq(xhat[3][0]) + sq(R[1][2])*dt*p*sq(xhat[4][0]) - 20*R[2][2]*m*xhat[5][0]*exp(-(dt*p*(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]))/(20*m)) + 2*R[0][2]*R[1][2]*dt*p*xhat[3][0]*xhat[4][0] + R[0][2]*R[2][2]*dt*p*xhat[3][0]*xhat[5][0] + R[1][2]*R[2][2]*dt*p*xhat[4][0]*xhat[5][0])/(p*sq(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]));
  phi[3][3] = (R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0] + R[0][2]*xhat[3][0]*exp(-(dt*p*(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]))/(20*m)))/(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]);
  phi[3][4] = -(xhat[4][0]*(R[0][2] - R[0][2]*exp(-(dt*p*(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]))/(20*m))))/(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]);
  phi[3][5] = -(R[0][2]*xhat[5][0] - R[0][2]*xhat[5][0]*exp(-(dt*p*(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]))/(20*m)))/(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]);
  phi[4][3] = -(R[1][2]*xhat[3][0] - R[1][2]*xhat[3][0]*exp(-(dt*p*(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]))/(20*m)))/(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]);
  phi[4][4] = (R[0][2]*xhat[3][0] + R[2][2]*xhat[5][0] + R[1][2]*xhat[4][0]*exp(-(dt*p*(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]))/(20*m)))/(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]);
  phi[4][5] = -(R[1][2]*(xhat[5][0] - xhat[5][0]*exp(-(dt*p*(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]))/(20*m))))/(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]);
  phi[5][3] = -(R[2][2]*xhat[3][0] - R[2][2]*xhat[3][0]*exp(-(dt*p*(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]))/(20*m)))/(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]);
  phi[5][4] = -(R[2][2]*xhat[4][0] - R[2][2]*xhat[4][0]*exp(-(dt*p*(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]))/(20*m)))/(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]);
  phi[5][5] = (R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]*exp(-(dt*p*(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]))/(20*m)))/(R[0][2]*xhat[3][0] + R[1][2]*xhat[4][0] + R[2][2]*xhat[5][0]);
}
void integralcalc()
{
  float dt = tk1-tk;
  float term1[6][1];
  float term2[6][1];
  float B[6][1];
  float Bnot[6][1];
  float ut[1][1] = {T};
  float scale[1][1] = {dt/2};
  B[0][0] = 0;
  B[1][0] = 0;
  B[2][0] = 0;
  B[3][0] = R[0][2]/m;
  B[4][0] = R[1][2]/m;
  B[5][0] = R[2][2]/m;
  Bnot[0][0] = 0;
  Bnot[1][0] = 0;
  Bnot[2][0] = 0;
  Bnot[3][0] = R0[0][2]/m;
  Bnot[4][0] = R0[1][2]/m;
  Bnot[5][0] = R0[2][2]/m;
  Matrix.Multiply((float*)B,(float*)ut,6,1,1,(float*)term1);
  Matrix.Multiply((float*)phi,(float*)term1,6,6,1,(float*)term1);
  Matrix.Multiply((float*)Bnot,(float*)ut,6,1,1,(float*)term2);
  Matrix.Add((float*)term1,(float*)term2,6,1,(float*)xintg);
  Matrix.Multiply((float*)xintg,(float*)scale,6,1,1,(float*)xintg);
}
void xbarcalc()
{
  float xbar1[6][1];
  Matrix.Multiply((float*)phi,(float*)xhat,6,6,1,(float*)xbar1);
  Matrix.Add((float*)xbar1,(float*)xintg,6,1,(float*)xbar);
}
void Pbarcalc()
{
  // C is just identity so it is omitted
  float phisq[6][6];
  Matrix.Multiply((float*)phi,(float*)phi,6,6,6,(float*)phisq);
  Matrix.Multiply((float*)P,(float*)phisq,6,6,6,(float*)Pbar);
  Matrix.Add((float*)Pbar,(float*)Rv,6,6,(float*)Pbar);
}
void Gcalc()
{
  Matrix.Multiply((float*)Pbar,(float*)Rvinv,6,6,6,(float*)G);
}
void xhatcalc()
{
  // C is just identity so to minimize calculations it is omitted 
  Matrix.Subtract((float*)y,(float*)xbar,6,1,(float*)xhat);
  Matrix.Multiply((float*)G,(float*)xhat,6,6,1,(float*)xhat);
  Matrix.Add((float*)xbar,(float*)xhat,6,1,(float*)xhat); 
}
void Pcalc()
{
  float term1[6][6];
  float term2[6][6];
  float I[6][6] = {{1, 0, 0, 0, 0, 0},{0, 1, 0, 0, 0, 0},{0, 0, 1, 0, 0, 0},{0, 0, 0, 1, 0, 0},{0, 0, 0, 0, 1, 0},{0, 0, 0, 0, 0, 1}};
  Matrix.Subtract((float*)I,(float*)G,6,6,(float*)term1);
  Matrix.Multiply((float*)term1,(float*)term1,6,6,6,(float*)term1);
  Matrix.Multiply((float*)Pbar,(float*)term1,6,6,6,(float*)term1);
  Matrix.Multiply((float*)G,(float*)G,6,6,6,(float*)term2);
  Matrix.Multiply((float*)Rv,(float*)term2,6,6,6,(float*)term2);
  Matrix.Add((float*)term1,(float*)term2,6,6,(float*)P);
  
}




