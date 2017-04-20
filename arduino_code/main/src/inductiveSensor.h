#include <std_msgs/Bool.h>

#define inductivePin 25
int inductiveState = 0;
extern const int INDUCTIVE_LOOP_PERIOD;
void handleInductive(ros::NodeHandle& nh);

std_msgs::Bool inductive_msg;
ros::Publisher pub_inductive( "/Inductive", &inductive_msg);

void setupInductiveSensor(ros::NodeHandle& nh){ 
  pinMode(inductivePin, INPUT);  
  nh.advertise(pub_inductive);
}

void loopInductiveSensor(ros::NodeHandle& nh){
  static unsigned long lastUpdateTime = millis();
  unsigned long now = millis();
    if (now - lastUpdateTime >= INDUCTIVE_LOOP_PERIOD) {
      lastUpdateTime = now;
      handleInductive(nh);
    } 
}

void handleInductive(ros::NodeHandle& nh){
  // sensor is normally closed
  inductiveState = !digitalRead(inductivePin);
  
  inductive_msg.data = inductiveState;
  pub_inductive.publish(&inductive_msg);
}

