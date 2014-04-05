#include "Galil.h"   
#include <iostream>  
#include <sstream>   
#include <fstream>
#include <unistd.h>
#include <math.h>
#include <string>
#include <time.h>

typedef struct _axis_readings {
	double a;
	double b;
	double c;
} axis_readings;

using namespace std;

//global pointer to Galil connection
Galil * galil_connection = NULL;
//global debug
bool OFFLINE = true;

Galil* setup();
double read_axis1();
double read_axis2();
double read_axis3();
void write_axis1(double val);
void write_axes(double a, double b, double c);
int stringToInt (std::string s);
void write_axes(double a, double b, double c);
void write_axis1(double val);
bool returnToHome();
axis_readings get_readings();
axis_readings get_decoy_readings();
double read_axis(int n);
int control_loop(double targ_x, double targ_y, double* sig_x, double* sig_y, double* err_x, double* err_y);
void update_hist(double * x,double new_val);

int main (int argc, char * const argv[]) {
	Galil *g = NULL;
	double sig_x[4] = {0}, sig_y[4] = {0}, err_x[4]={0}, err_y[4]={0};
	int ctrl_cnt = 0;
	
	try {
		if (!OFFLINE) {
			std::cout << "Attempting setup...\n";
			g= setup();
			std::cout << "Controller: " << g->connection() << "\n";
			write_axis1(30);
			cout << "Testing home function...\n";
			returnToHome();
			axis_readings test_read;
			test_read = get_readings();
			cout << test_read.a << "\n";
			cout << test_read.b << "\n";
			cout << test_read.c << "\n";
		}
		
		cout << "Begininning to read input file...\n";
		ifstream ifile("nodes.txt");
		if (ifile.fail()) {
			cout << "Couldn't open file.\n";
		}
		
		time_t t0,t1;
		double start_time=0, curr_t = 0, t_lapsed = 0;
		double x_spd, y_spd, leg_time;
		int leg_ct = 0;
		
		while (ifile >> x_spd >> y_spd >> leg_time) {
			leg_ct++;
			cout << "Beginning leg: " << leg_ct << "\n";
			
			//target x and y speeds have been read
			
			cout << "x: "  << x_spd << " y: " << y_spd << "\n";
			
			//Initialize timer
			time(&t0);
			start_time = t0*1000;
		
			
			while (!control_loop(x_spd, y_spd, sig_x,sig_y,err_x,err_y) && !(t_lapsed > leg_time*1000)) { //this could make an infinite loop if it never reaches curr_x, curr_y
				time(&t1);
				curr_t = t1*1000;
				t_lapsed = curr_t-start_time;
				//cout << "Time elapsed: " << curr_t-start_time << "\n";
				ctrl_cnt++;
				if (!ctrl_cnt&10) {
					cout << "Controller loop, count: " << ctrl_cnt << "\n";
				}
			}
			t_lapsed = 0;
		
		}
		
	} catch ( std::string exc ) {
		std::cout << exc;
	}
	
	if ( g != NULL )
		delete g;
	
	std::cout << "Exiting!\n";
	
    return 0;
}

int control_loop(double t_x, double t_y, double* sig_x, double* sig_y, double* err_x, double* err_y) {
	double curr_err_x = 0, curr_err_y = 0; //units of error signal are mm
	double fdbk_x, fdbk_y; //units of feedback are steps/s
	double curr_veloc_x = 0, curr_veloc_y =0;
	
	//read current value, in encoder counts 
	axis_readings ar;
	ar = get_decoy_readings();
	
	//convert encoder counts into mm


	//calculate velocity and update velocity history
	
	
	//generate error signal
	curr_err_x = t_x - ar.a;
	curr_err_y = t_y - ar.b;
	
	update_hist(err_x,curr_err_x);
	update_hist(err_y, curr_err_y);
	
	//CONVERT ERROR SIGNAL INTO FEEDBACK.  CONTROL CODE GOES HERE.
	fdbk_x=1/0.755*(sig_x[3]-2.739*sig_x[2]+2.494*sig_x[1]-0.009282*err_x[3]+0.006123*err_x[2]+0.009093*err_x[1]-0.006311*err_x[0]);
	fdbk_y=1/0.755*(sig_y[3]-2.739*sig_y[2]+2.494*sig_y[1]-0.009282*err_y[3]+0.006123*err_y[2]+0.009093*err_y[1]-0.006311*err_y[0]);
	
	//add new control signal to state variables
	update_hist(sig_x,fdbk_x);
	update_hist(sig_y,fdbk_y);
		
	//send updated speed
	if (!OFFLINE) {
		cout << "Sending speed.  x=" << fdbk_x << ", y=" << fdbk_y << "\n";
		write_axes(fdbk_x,fdbk_y,0);
	}
	
	return false;
	
}

void update_hist(double * x,double new_val) {
	
	int array_size = sizeof (x) / sizeof (x[0]);
	
	for (int j = array_size - 1; j > 0; j--)
	{
		x[j] = x[j - 1];
	}
	
	x[0] = new_val;
}

Galil* setup() {
	galil_connection = new Galil("/dev/tty.usbserial-00001004");
	galil_connection->command("ST; SH; MT2,2,2; ST; AC 90000,,90000; DC 90000,,90000; JG0,0,0; BG ABC;");
	return galil_connection;
}

bool homeSwitchOff() {
	if(!galil_connection)
		return 0;
	return galil_connection->commandValue("MG_HMC");
}

bool returnToHome() {
	if(!galil_connection)
		return false;
	
	//Home C axis (left-right)
	galil_connection->command("ST; LD 3,3,3; CN,1; SPC=22000; HM C; BG AC;");
	sleep(5);
	//galil_connection->command("DEA=0; DEC=0; LD0,0,0; JG0,0,0; BG ABC;");
	
	//Home A axis (in/out)
	galil_connection->command("ST; LD 3,3,3; CN,-1; SPA=22000; HM A; BG A;");
	sleep(45);
	galil_connection->command("ST; DEC=0; JG0,0,0; BG ABC;");
	
	galil_connection->command("JG60000,0,0;");
	sleep(3);
	galil_connection->command("ST; JG0,0,0;");
	sleep(1);
	galil_connection->command("DEA=0"); //Zero A
	
	galil_connection->command("BG C; JG0,0,-6000;");
	sleep(1);
	galil_connection->command("ST; JG0,0,0;");
	
	galil_connection->command("JG0,0,0; LD0,0,0;");
	
	//Don't scare people
	sleep(3);
	
	galil_connection->command("BG ABC;");
	
	return true;
}

void write_axis1(double val)
{
	if(!galil_connection)
		return;
	
	int num = floor(val);
	ostringstream convert;
	convert<<num;
	
	string out = convert.str();
	galil_connection->command("JG,,"+out+";");
}

void write_axes(double a, double b, double c)
{
	if(!galil_connection)
		return;
	
	ostringstream convert;
	convert<<"BG ABC; JG"
	<<((int) floor(a))<<","
	<<((int) floor(b))<<","
	<<((int) floor(c));
	
	string out = convert.str();
	
	galil_connection->command(out);
}

int stringToInt (std::string s) {
	int d;
	std::stringstream ss(s);
	ss >> d;
	return d;
}

axis_readings get_readings() {
	axis_readings out;
	out.a = read_axis(2);
	out.b = read_axis(3);
	out.c = read_axis(1);
	return out;
}

axis_readings get_decoy_readings() {
	axis_readings out;
	out.a = 100;
	out.b = 200;
	out.c = 300;
	return out;
}

double read_axis(int n)
{
	if(!galil_connection)
		return 0;
	
	switch (n) {
		case 1:
			return galil_connection->commandValue("MG_TPC");
			break;
		case 2:
			return galil_connection->commandValue("MG_TPA");
			break;
		case 3:
			return galil_connection->commandValue("MG_TPB");
			break;
		default:
			return 0;
			break;
	}
	
}

