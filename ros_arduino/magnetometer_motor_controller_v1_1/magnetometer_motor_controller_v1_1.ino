#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>

// Magnetometer code
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Ros code
#define LMOTOR_PWM 2
#define LMOTOR_IN1 42
#define LMOTOR_IN2 46
#define LMOTOR_STATUS 50
#define LMOTOR_SENSE 3

#define RMOTOR_PWM 4
#define RMOTOR_IN1 43    
#define RMOTOR_IN2 47
#define RMOTOR_STATUS 51
#define RMOTOR_SENSE 5

ros::NodeHandle nh;

double loop_rate = 20.0;  // In Hz

//pid control class
class pid_control
{
  private:
    //setpoint parameters
    double setpt;    //regulated level to maintain
    //gain parameters
    double kprop;    //proportionality coefficient
    double ki;      //integrator coefficient
    double kd;      //differentiator coefficient
    double kgain;      //loop gain coefficient
    double lowlim;
    double highlim;
    double anti_windup;
    //controller state parameters
    double integ;      //integral of setpoint errors
    double deriv;      //previous setpoint error
    double oldderiv;    //previous to previous setpoint error
    double time;      //previous time

    //default parameters
    static const double csetpt = -135.0;
    static const double ckprop = 10.0;  // correct a the end
    static const double cki = 0.001;  // correct at the end
    static const double ckd = 0.0;
    static const double ckgain =1.0;
    static const double clowlim = -255.0;
    static const double chighlim = 255.0;
    static const double canti_windup = 0.0;
    static const double cinteg = 0.0;
    static const double cderiv = 0.0;
    static const double coldderiv = 0.0;

    double derivative(double err, double delt)
    {
      double pid_deriv = (kd*(-1.0)*(err-oldderiv)/2.0)/(delt*loop_rate);
      oldderiv = deriv;
      deriv = err;
      return pid_deriv;
    }
    double integral(double err, double delt)
    {
      integ += (-1.0)*err*delt*loop_rate;
      return ki*integ;
    }
    double proportional(double err)
    {
      return (-1.0)*kprop*err;
    }
  public:
    pid_control(double setpt=csetpt, double kprop=ckprop, double ki=cki, double kd=ckd, double kgain=ckgain, double lowlim=clowlim, double highlim=chighlim, double anti_windup=canti_windup, double integ=cinteg, double deriv=cderiv, double oldderiv=coldderiv, double time=millis()/1000.0-1/loop_rate)
    {
      this->setpt = setpt;
      this->kprop = kprop;
      this->ki = ki;
      this->kd = kd;
      this->kgain = kgain;
      this->lowlim = clowlim;
      this->highlim = chighlim;
      this->anti_windup = canti_windup;
      this->integ = integ;
      this->deriv = deriv;
      this->oldderiv = coldderiv;
      this->time = time;
    }
    
    double pid_out(double curr_feedback, double curr_time)
    {
      double err = curr_feedback-setpt;
      if(err > 180.0)
      {
        err -= 360.0;
      }
      else if (err < -180.0)
      {
        err += 360.0;
      }
      double delt = curr_time-time;
      if(delt < 1/(10*loop_rate))  // for exceptions in loop rates because of any event derivative protector
      {
        delt = 1/loop_rate;
        Serial.println("Ooops, delt went down by order 1, loop is suffering");  // remove at end
      }
      if(delt < 10.0/loop_rate)  // for exceptions in loop rates because of any event integral protector
      {
        delt = 1/loop_rate;
        Serial.println("Ooops, delt went up by order 1, loop is suffering");  // remove at end
      }
      double time = curr_time;
      double pidout = kgain*(derivative(err, delt)+integral(err, delt)+proportional(err));
      if(pidout > highlim)
      {
        pidout = highlim;
        integ -= (1.0-anti_windup)*(-1.0)*err*delt*loop_rate;
      }
      if(pidout < lowlim)
      {
        pidout = lowlim;
        integ -= (1.0-anti_windup)*(-1.0)*err*delt*loop_rate;
      }
      return pidout;
    }

    void set_setpt(double head)
    {
      this->deriv+=this->setpt;
      this->oldderiv+=this->setpt;
      this->setpt=head;
      this->deriv-=this->setpt;
      this->oldderiv-=this->setpt;
    }
}ground_bot;

// pwm write function
void vector_callback(const geometry_msgs::Vector3& vec)
{
  if(vec.z == 1)
  {
    if(vec.x >= 0)
    {
      digitalWrite(LMOTOR_IN1, LOW);
      digitalWrite(LMOTOR_IN2, HIGH);
      analogWrite(LMOTOR_PWM, 255-vec.x);
    }
    else
    {
      digitalWrite(LMOTOR_IN1, HIGH);
      digitalWrite(LMOTOR_IN2, LOW);
      analogWrite(LMOTOR_PWM, 255+vec.x);
    }
    
    if(vec.y >= 0)
    {
      digitalWrite(RMOTOR_IN1, LOW);
      digitalWrite(RMOTOR_IN2, HIGH);
      analogWrite(RMOTOR_PWM, 255-vec.y);
    }
    else
    {
      digitalWrite(RMOTOR_IN1, HIGH);
      digitalWrite(RMOTOR_IN2, LOW);
      analogWrite(RMOTOR_PWM, 255+vec.y);
    }
  }
  else
  {
    digitalWrite(LMOTOR_IN1, HIGH);
    digitalWrite(LMOTOR_IN2, HIGH);
    analogWrite(LMOTOR_PWM, 255);
    
    digitalWrite(RMOTOR_IN1, HIGH);
    digitalWrite(RMOTOR_IN2, HIGH);  
    analogWrite(RMOTOR_PWM, 255);
  }
}

void setpoint_callback(const std_msgs::Float64& head)
{
  double setpoint;
  if(head.data > 180.0)
  {
    setpoint = 360.0-head.data;
  }
  else
  {
    setpoint = -head.data;
  }
  ground_bot.set_setpt(setpoint);
}

ros::Subscriber<std_msgs::Float64>sub("/arduino/heading", &setpoint_callback);

void setup(void) 
{
  // Magnetometer code
  Serial.begin(57600);
  Serial.println("HMC5883 Magnetometer Test");  // remove at end
  Serial.println("""");  // remove at end
  if(!mag.begin())
  {
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");  // remove at end
    while(1);
  }
  
  // Ros code
  pinMode(LMOTOR_PWM, OUTPUT);
  pinMode(LMOTOR_IN1, OUTPUT);
  pinMode(LMOTOR_IN2, OUTPUT);
  pinMode(LMOTOR_STATUS, INPUT);
  pinMode(LMOTOR_SENSE, INPUT);
    
  pinMode(RMOTOR_PWM, OUTPUT);
  pinMode(RMOTOR_IN1, OUTPUT);
  pinMode(RMOTOR_IN2, OUTPUT);
  pinMode(RMOTOR_STATUS, INPUT);
  pinMode(RMOTOR_SENSE, INPUT);
//  std_msgs::Float64 head;
//  head.data = 225.0;
//  setpoint_callback(head);
  
  nh.initNode();
  nh.subscribe(sub);
}

void loop(void) 
{
  // Magnetometer code
  sensors_event_t event;
  mag.getEvent(&event);
  double heading = atan2(event.magnetic.y, event.magnetic.x);
  double declinationAngle = 0.22;
  heading += declinationAngle;
  if(heading < 0)
    heading += 2*PI;
  if(heading > 2*PI)
    heading -= 2*PI;
  double headingDegrees = heading*180/M_PI;

  // Ros code
  double curr_feedback;
  if(headingDegrees > 180.0)
  {
    curr_feedback = 360.0-headingDegrees;
  }
  else
  {
    curr_feedback = -headingDegrees;
  }

  Serial.print(curr_feedback);  // remove at end
  Serial.print("  ");  // remove at end
  Serial.print(headingDegrees);  // remove at end
  double omega = ground_bot.pid_out(curr_feedback, millis()/1000.0);
  Serial. print("  ");  // remove at end
  Serial.print(omega);  // remove at end
  geometry_msgs::Vector3 pwm_vector;
  pwm_vector.x = -omega;
  pwm_vector.y = omega;
  pwm_vector.z = 1;
  Serial.print("  ");  // remove at end
  Serial.print((double)pwm_vector.x);  // remove at end
  Serial.print("  ");  // remove at end
  Serial.println((double)pwm_vector.y);  // remove at end

  vector_callback(pwm_vector);
  nh.spinOnce();
  delay(1000.0/loop_rate);
}
