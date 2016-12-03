#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <linux/input.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <termios.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"


void reset(geometry_msgs::Vector3& v){
	v.x = v.y = v.z = 0.0;
}

enum {RELEASED, PRESSED, REPEATED};

class Controller{
	private:
		int kb; // --> keyboard identifier
		input_event kb_ev;
		ros::Publisher vel_pub;
		geometry_msgs::Twist vel_msg;
		ros::NodeHandle nh;
		struct termios cooked, raw;

		ros::Subscriber joy_sub;
		bool useJoy;
		double last_published;

	public:
		Controller(const char* kbDev = NULL){
			vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);

			std::string joy;
			nh.getParam("joy", joy);
			std::cout << "JOY " << joy << std::endl;

			useJoy = true; //(joy == "true" || joy == "True");

			if(useJoy){
				joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 1000, &Controller::joyread, this);
			}else{
				kb = open(kbDev,O_RDONLY);
				if(kb == -1){
					fprintf(stderr,"Cannot open device %s: %s.\n", kbDev,strerror(errno));
					close(kb);
				}else{
					//suppress echo
					// get the console in raw mode
					tcgetattr(kb, &cooked);
					memcpy(&raw, &cooked, sizeof(struct termios));
					raw.c_lflag &=~ (ICANON | ECHO);
					// Setting a new line, then end of file                         
					raw.c_cc[VEOL] = 1;
					raw.c_cc[VEOF] = 2;
					tcsetattr(kb, TCSANOW, &raw);
				}

			}
		}

		~Controller(){
			if(!useJoy){
				tcsetattr(kb, TCSANOW, &cooked);
				close(kb);
			}
		}

		bool kbread(){

			ssize_t bytes = read(kb,&kb_ev,sizeof(kb_ev));

			if(bytes == (ssize_t)-1){
				if(errno == EINTR){
					return true;
				}
				else{
					return false;
				}
			}else if(bytes != sizeof(kb_ev)){
				errno = EIO;
				return false;
			}


			if(kb_ev.type == EV_KEY){
				geometry_msgs::Vector3& a = vel_msg.angular;
				geometry_msgs::Vector3& l = vel_msg.linear;

				reset(a);
				reset(l);

				switch(kb_ev.value){
					case REPEATED:
					case PRESSED:
						switch(kb_ev.code){
							case 0x0067: //UP
								l.x = 1.0;
								break;
							case 0x0069: //LEFT
								a.z = 1.0;
								break;
							case 0x006a: //RIGHT
								a.z = -1.0;
								break;
							case 0x006c: //DOWN
								l.x = -1.0;
								break;
							default:
								// all vel. 0
								break;
						}
						break;
					case RELEASED:
						break;
				}
			}
			return true;
		}
		void joyread(const sensor_msgs::JoyConstPtr& msg){
			last_published = ros::Time::now().toSec();

			double turn = msg->axes[0];
			double thrust = msg->axes[1];

			vel_msg.linear.x = thrust;
			vel_msg.angular.z = turn;
		}

		bool joyread(){
			double now = ros::Time::now().toSec();
			if(now - last_published > 1.0){
				reset(vel_msg.linear);
				reset(vel_msg.angular);
			}
			return true;
		}

		bool cmdread(){
			return useJoy? joyread():kbread();
		}

		void run(){
			ros::Rate rate(10.0);
			while(ros::ok() && cmdread()){
				vel_pub.publish(vel_msg);
				ros::spinOnce();
				rate.sleep();
			}
		}

};


int main(int argc, char* argv[]) {
	ros::init(argc,argv,"controller");

	Controller controller("/dev/input/event4");

	controller.run();

	return 0;
}
