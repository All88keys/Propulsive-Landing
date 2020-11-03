//includes


//mission constants
double specificI={.0178, 0, 0, 0, .0178, 0, 0, .000722}; //rocket's specific inertia tensor
double dryMass=.875; //drymass
double propellantMass=.06; //propellant mass
double momentArm=.18; //distance from cm to engine location
double tb=.625; //mission initial ballistic time
double fireDelayEst=.1; //average delay between pyro signal and engine fire
double g=9.81; //local gravitation acceleration
double pi=3.14159;

//pins
int IMUS=18;
int G_LED_1=4;
int B_LED_1=6;
int G_LED_2=8;
int B_LED_2=9;
int PYRO_1=22;
int PYRO_2=23;
int TVC_X=20;
int TVC_Y=21;
int TVC_TOP=5;
//mosi, miso , sck, cs

//stage switch buffers
double tn1buff=.25; //acceleration value less than 0 to trigger fall flag
double tnt1buff=.05; //time within condition to trigger fall flag (n1)
//double tn3buff=.25; //acceleration value greater than 9.81 to trigger fire flag
//double tnt3buff=.025; //time within condition to trigger fire flag (n3)
double tn6buff=.25; //acceleration buffer to trigger landed flag
double tnt6buff=.5; //time within condition to trigger landed flag (n6)

//thrust curve constants
double T={0, 0, .25, 30, .38, 12.5, .725, 10.2, 2.7, 9.8, 2.8, 0}; //thrust curve (even indexes are time, odd are thrust values)
const int sa=4;
const int sb=4;
double Tmaxtime=.38;
double Tavgtime=.725;
double Tendtime=2.8;
double Tavg=12;
double Tmax=12.5;
double Timpulse=30;

//observer constants
int vqMin=3; //number of imu velocity values needed to raise yFlag (along with wqMin)
int wqMin=3; //number of imu angular values needed to raise yFlag (along with vqMin)
int vqMax=9; //yFlag raised raised when vqMax is reached regardless of wqCount
int wqMax=9; //yFlag raised raised when wqMax is reached regardless of vqCount

//control constants
double K={};
double A={};
double B={};
double C={1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0};
double L={};
double uMax=5/180*pi; //max tvc angle
double landingTime=.5; //time at the end of the burn where divert is not allowed to operate
double initialDivertSpan=.2; //initial divert length
double divertWaitMult=1.5; //how much longer is the divert wait than the divert span
double divertZgoal=.25; //+-0 when we don't divert

//other constants
int blinkPatterns={}; //blink patterns


//controller variables
double x [6]; //state variable
double u [2]; //control variable
int n=0; //mode
double h=0; //loop timestep
double clk=0; //flight clk
double startclk=0; //used to calculate h
double endclk=0; //used to calculate h
double k [12]; //controller gain
int currentk; //K iteration
double a [36]; //current A
double b [12]; //current B
double tfire; //fire time
double tfall; //fall time
double nextDivertTime;

//observer variables
double vbSum [3]; //rolling sum of incoming velocity signals
double wbSum [3]; //rolling sum of incoming angular rate signals
int vbCount=0; //how many velocity signals have been summed
int wbCount=0; //how many angular rate signals have been summed
double gamma;
double ddz;
double lastDz;
double z;
float R [9];
float R1 [9];
float R2 [9];
float R3 [9];
float dcm [9];

//status variables
int controllerOn=0; //run the controller?
int observerOn=1; //run the observer?

//interrupt flags
int yFlag=0;        //when suitable number of signals have come in
int divertFlag=0;   //when time to divert
int blinkFlag=0;    //when blink timer runs out
int iterKFlag=0;    //when time to iter K
int incomingFlag=0; //when signal comes in

//other variables
double lastBlink=0; //time when LEDS blinked last
int blinkBin=0;     //blink on or off
double data [25];   //data to record

//function prototypes
void processMeasure(double v, double w); //transform body measurements to earth/euler
void divert(double u[2]); //divert u
void checkDivert();
double * trimTurnInput(double u[2]); //trim input to maxU
void sendInput(double u[2]); //send input to tvc
void ledBlink(int n);
void record(double data[25]); //record data to flash
//double * rungeKutta(double x[6], double dx[6], double h);
//double * rungeKuttaOmega(double x[3], double dx[3], double h);
double * mAdd(double * A, double * B, int elements);
double * sMult(double * A, double rho, int elements);
double * mMult(double * A, double * B, int m, int n, int p);
double * mInv(double * A, int n);

//interrupt handlers
void divertIntr();
void blinkIntr();
void iterKIntr();
void incomingIntr();
void yIntr();
//void intrN2(...
//void intrN3(...
//void intrN4(...
//void intrN5(...
//void intrN6(...

void setup() {
  //define A, B, K, L(?)

}

void loop() {
  // put your main code here, to run repeatedly:
  if (yFlag){
    if(observerOn){
      data[0]=clk;
      y=processMeasure(vbSum, vbCount, wbSum, wbCount, x); //data[5]=z; data[6]=lastDz; data[7]=ddz; data[8]=gamma;
      //reset sums, counts & handle z, dz, etc
      y=y[0:3]; data[1:4]=y; 
      if(controllerOn){
        if(iterKFlag){
          currentk++;
          k=K[13*currentk+1:13*currentk+12];
          a=A[37*currentk+1:37*currentk+36];
          b=B[13*currentk+1:13*currentk+12];
          iterKFlag=0;
        }
        u=sMult(mMult(k, x, 2, 6, 1), -1, 2); data[9:10]=u;
        if(divertFlag){
          checkDivert(); //add input variables
        }
        if(divertNow){
          u=divert(u, lastDz, ddz, clk, tavgtime);
        } data[11:12]=u;
        u=inputProcess(u); data[13:14]=u;
        sendInput(u);
      }
      else{
        data[9:13]={0, 0, 0, 0, 0, 0};
      }
      if(n<3){
        u={0, 0}; //so it doesn't affect kalman filter (Bu term)
      }
      endclk=micros();
      h=endclk-startclk;
      clk+=h;
      startclk=endclk;
      dx=mMinus(y, mMult(C, x, 4, 6, 1), 4); dx=mMult(L, dx, 6, 4, 1);
      dx=mAdd(dx, mAdd(mMult(a, x, 6, 6, 1), mMult(b, u, 6, 2, 1), 6), 6); data[14:19]=dx;
      x=dx*h+x; data[20:25]=x;
    }
    yFlag=0;
  }
  if (blinkFlag){
    ledBlink(n);
    blinkFlag=0;
  }
}

void processMeasure(){ //transform body measurements to earth/euler
  euler1=x[4];
  euler2=x[1];
  euler3=gamma;
  R={1, 0,  -sin(euler1), 0, cos(euler2), sin(euler2)*cos(euler1), 0, -sin(euler2), cos(euler(2))*cos(euler(1)};
  R=mInv(R, 3);
  R1={1, 0, 0, 0, cos(euler1), sin(euler1), 0, -sin(euler1), cos(euler1)}; 
  R2={cos(euler2), 0, -sin(euler2), 0, 1, 0, sin(euler2), 0, cos(euler2)}; 
  R3={cos(euler3), sin(euler3), 0, -sin(euler3), cos(euler3), 0, 0, 0, 1}; 
  dcm=mMult(R1, R2, 3, 3, 3); dcm=mMult(dcm, R3, 3, 3, 3); dcm=mInv(dcm, 3);
  double * ve;
  ve=mMult(dcm, vbSum/vbCount, 3, 3, 1);
  vbCount=0; vbCount=0;
  y[0]=ve[0]; y[1]=ve[1];
  ddz=(ve[1]-lastDz)/h; lastDz=ve[1];
  z=z+h*lastDz;
  double * de;
  de=mMult(R, wbSum/wbCount, 3, 3, 1);
  wbCount=0; wbSum=0;
  y[2]=de[0]; y[3]=de[1];
  gamma=gamma+h*de[2];
}

void divert(double u[2]); 

double * mMult(double * A, double * B, int m, int n, int p) {
  double * C;
  C = new double[m*p];
  for (int i = 0;  i < m; i++)
  {
    for (int j = 0; j < p; j++)
    {
      C[i*p + j] = 0;
      for (int k = 0; k < n; k++) 
      {
        C[i*p + j] += A[i*n + k] * B[j + p * k];
      }
    }
  }
  return C;
}

double * mAdd(double * A, double * B, int elements) {
  double * C;
  C = new double[elements];
  for (int i = 0; i < elements; i++) {
    C[i] = A[i] + B[i];
  }
  return C;
}

double * sMult(double * A, double rho, int elements) {
  double * C;
  C = new double[elements];
  for (int i = 0; i < elements; i++) {
    C[i] = A[i] * rho;
  }
  return C;
}
