#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

// Linux headers
#include <fcntl.h>	 // Contains file controls like O_RDWR
#include <errno.h>	 // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>	 // write(), read(), close()

using namespace std;

// Emotion code
#define NEUTRAL 0
#define SAD 1
#define HAPPY 2
#define ANGRY 3
#define ASTONISHED 4
#define EVIL 5
#define VOICE 10

#define REQ_TEMPERATURE 100

#define VOICE_INTENSITY_NULL 0
#define VOICE_INTENSITY_LOW 1
#define VOICE_INTENSITY_MED 2
#define VOICE_INTENSITY_HIGH 3

float emoticon_idx = NEUTRAL;
float voice_intensity = VOICE_INTENSITY_NULL;
bool changed_v_intensity = false;
bool last_cmd = false;
bool powerbooster = false;
ros::Time last_cmd_time;

/*---------------------------------------------------------------------*
 *                                                *
 *                                                                      *
 *----------------------------------------------------------------------*/
void emoticonread__Callback(const std_msgs::Float64::ConstPtr &msg)
{
	emoticon_idx = msg->data;
	last_cmd_time = ros::Time::now();
	last_cmd = false;
}

void voiceintensityread__Callback(const std_msgs::Float64::ConstPtr &msg)
{
	voice_intensity = msg->data;

	if (voice_intensity < VOICE_INTENSITY_NULL)
	{
		voice_intensity = VOICE_INTENSITY_NULL;
	}

	if (voice_intensity > VOICE_INTENSITY_HIGH)
	{
		voice_intensity = VOICE_INTENSITY_HIGH;
	}

	changed_v_intensity = true;
}

void powerbooster__Callback(const std_msgs::Bool::ConstPtr &msg)
{

	static bool old_powerbooster = false;
	powerbooster = msg->data;

	if (powerbooster != old_powerbooster && !powerbooster)
	{
		emoticon_idx = NEUTRAL;
		last_cmd_time = ros::Time::now();
		last_cmd = false;
	}

	old_powerbooster = powerbooster;
}

int openPort(int &serial_port, string port)
{

	// Open serial port
	serial_port = open(port.c_str(), O_RDWR);

	// Create new termios struc, we call it 'tty' for convention
	struct termios tty;

	// Read in existing settings, and handle any error
	if (tcgetattr(serial_port, &tty) != 0)
	{
		printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
		return 1;
	}

	tty.c_cflag &= ~PARENB;		   // Clear parity bit, disabling parity (most common)
	tty.c_cflag &= ~CSTOPB;		   // Clear stop field, only one stop bit used in communication (most common)
	tty.c_cflag &= ~CSIZE;		   // Clear all bits that set the data size
	tty.c_cflag |= CS8;			   // 8 bits per byte (most common)
	tty.c_cflag &= ~CRTSCTS;	   // Disable RTS/CTS hardware flow control (most common)
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO;														 // Disable echo
	tty.c_lflag &= ~ECHOE;														 // Disable erasure
	tty.c_lflag &= ~ECHONL;														 // Disable new-line echo
	tty.c_lflag &= ~ISIG;														 // Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);										 // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
	// tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
	// tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

	tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
	tty.c_cc[VMIN] = 0;

	// Set in/out baud rate to match 115200 on pico
	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);

	// Save tty settings, also checking for error
	if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
	{
		printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
		return 1;
	}

	return 0;
}

int main(int argc, char **argv)
{
	string port;
	string emoticon_topic;
	string voice_intensity_topic;
	string temperature_topic;
	string power_booster_topic;
	double run_freq(50);	// Desired main rate in Hz [50 Hz -> 20 ms]
	double temp_freq(0.25); // [0.25 Hz -> 4 s]
	int loop_cnt = 0;
	char pkg[20];
	int serial_port;
	bool can_read_temp = false;
	bool has_face_expression = true;
	bool has_powerboost_ = false;
	ros::init(argc, argv, "face_expressions"); // Initiate new ROS node

	// NodeHandle is the main access point to communications with the ROS system. The first NodeHandle constructed will fully initialize this node, and the last NodeHandle destructed will close down the node.
	ros::NodeHandle n;

	n.getParam("face_expressions/port", port);
	n.getParam("face_expressions/emoticon_topic", emoticon_topic);
	n.getParam("face_expressions/voice_intensity_topic", voice_intensity_topic);
	n.getParam("face_expressions/has_temperature", can_read_temp);
	n.getParam("power_booster_topic", power_booster_topic);

	n.getParam("face_expressions/has_face_expression", has_face_expression);
	n.getParam("has_powerboost", has_powerboost_);

	ros::Subscriber emoticon_sub = n.subscribe(emoticon_topic, 1, emoticonread__Callback);
	ros::Subscriber voice_intensity_sub = n.subscribe(voice_intensity_topic, 1, voiceintensityread__Callback);
	ros::Subscriber powerbooster_sub;
	if (has_powerboost_)
		powerbooster_sub = n.subscribe("power_booster", 1, powerbooster__Callback);

	ros::Publisher pub_temperature;
	std_msgs::Float64 msg_temperature;
	n.getParam("face_expressions/temperature_topic", temperature_topic);

	ros::Rate loop_rate(run_freq);

	// Device preparation
	cout << "\nROS NODE FOR ALTEREGO FACE EXPRESSIONS\n";
	cout << "\n\nOpening serial COM port to communicate with the face controller.\n";

	if ((!openPort(serial_port, port)) && has_face_expression)
	{
		pub_temperature = n.advertise<std_msgs::Float64>(temperature_topic, 1);

		cout << "Node running" << endl;

		int old_emo_idx = -1;
		int emo_idx = NEUTRAL;
		ros::Duration max_cmd_latency = ros::Duration(5);

		while (ros::ok())
		{
			if (!last_cmd && ros::Time::now() - last_cmd_time > max_cmd_latency)
			{
				if (powerbooster)
				{
					emoticon_idx = EVIL;
				}
				else
				{
					emoticon_idx = NEUTRAL;
				}
				last_cmd = true;
			}

			if (powerbooster)
			{
				char msg = (char)((int)EVIL);
				// Write to serial COM port the expression number
				write(serial_port, &msg, sizeof(msg));
				old_emo_idx = EVIL;
			}
			else
			{
				emo_idx = emoticon_idx;

				if (emo_idx != old_emo_idx || changed_v_intensity)
				{

					if (emo_idx == VOICE)
					{

						// Add intensity to emoticon_idx
						emo_idx += voice_intensity;
					}

					char msg = (char)((int)emo_idx);

					// Write to serial COM port the expression number
					write(serial_port, &msg, sizeof(msg));

					old_emo_idx = emo_idx;
					changed_v_intensity = false;
				}
			}

			if ((loop_cnt % (int)(run_freq / temp_freq) == 0) && can_read_temp)
			{ // This part is executed at temp_freq frequency

				// Raspberry Pico ask for updating temperature
				char msg = (char)((int)REQ_TEMPERATURE);

				// Write to serial COM port the message number
				write(serial_port, &msg, sizeof(msg));

				char buf[2];
				int nb = read(serial_port, buf, sizeof(buf));

				while (nb != 2)
				{
					nb = read(serial_port, buf, sizeof(buf));
					if (nb == 1)
						break;
				}

				if (nb == 2)
				{
					short int aux_si;
					((char *)&aux_si)[0] = buf[1];
					((char *)&aux_si)[1] = buf[0];
					msg_temperature.data = (float)(aux_si / 100.0);

					// cout << "Temperature: " << msg_temperature.data << endl;

					// if (msg_temperature.data == 0.0){
					// 	cout << "Restarting Serial Port" << endl;
					// 	close(serial_port);
					// 	usleep(1000000);
					// 	openPort(serial_port, port);
					// }

					pub_temperature.publish(msg_temperature);
				}
			}

			ros::spinOnce();   // Need to call this function often to allow ROS to process incoming messages
			loop_rate.sleep(); // Sleep for the rest of the cycle, to enforce the loop rate

			// Update loop counter
			loop_cnt++;
			if (loop_cnt > (int)(run_freq / temp_freq))
			{
				loop_cnt -= (int)(run_freq / temp_freq);
			}
		}

		cout << "Port closing" << endl;
		close(serial_port);
	}

	return 0;
}