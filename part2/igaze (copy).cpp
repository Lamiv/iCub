#include <cstdio>
#include <mutex>
#include <cmath>
#include <deque>

#include <stdio.h>
#include <unistd.h>
#include <time.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/math/Math.h>

#define EXIT_STATE 		_leave
#define ENTRY_STATE 		_entry
#define THREAD_RUN_FREQ     	1       // Freq in Secs when Run will be called
#define STATUS_PRINT_FREQ    	1.0        // Freq of Status output
#define STATE_CHANGE_FREQ   	2.0        // Freq of State change
#define CENTRE_GAZE_TIMEOUT   	30.0        // Freq of State change


using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

/****************************************************************/
/*		Class Definition				*/
/****************************************************************/
class iCubMotionControl: public PeriodicThread,
                  public GazeEvent
{

protected:
    PolyDriver        clientGaze;
    PolyDriver       cubTorsoDrv;
    PolyDriver	     cubRightArmDrv;
    IGazeControl     *igaze;
    IEncoders        *iencTorso;
    IEncoders        *iencRightArm;	
    IPositionControl *iposCtrlTorso;
    IPositionControl *iposCtrlRightArm;

    int state;
    int startup_context_id;
    
    Vector fp;

    deque<Vector> poiList;
    mutex myMutex;

    double t;
    double t0;
    double t1;
    double t2;
    double t3;

    int selectedCamera;
    yarp::sig::Vector lookAtPixel; 

    std::string outputXY = "/lap/out/lookhere";
    std::string inputXY = "/lap/in/lookhere";
    BufferedPort<Vector> XYPort;

    bool faceFound, objectFound;
    // the motion-done callback
    virtual void gazeEventCallback()
    {
        lock_guard<mutex> lg(myMutex);

/*        Vector ang;
        igaze->getAngles(ang);

        fprintf(stdout,"Actual gaze configuration: (%s) [deg]\n",
                ang.toString(3,3).c_str());

        fprintf(stdout,"Moving the torso; check if the gaze is compensated ...\n");

        // move the torso yaw
        double val;
        iencTorso->getEncoder(0,&val);
        iposCtrlTorso->positionMove(0,val>0.0?-30.0:30.0);
	getFixationPoint();
        //detach the callback
        igaze->unregisterEvent(*this); */

    }

public:
enum state_codes: int {	_entry,
			_left, 
			_centre, 
			_right, 
			_leave};					//states of the machine

string state_code_names[5]={"ENTRY STATE","LEFT STATE", "CENTRE STATE", "RIGHT_STATE", "EXIT_STATE"};

enum ret_codes: int {_ok, _fail, _repeat };				//return code from each state operation
struct transition {							//structure of a transition
    enum state_codes frm_state;
    enum ret_codes   ret_code;
    enum state_codes to_state;
};


struct transition state_transitions[9] = {
{_entry,	_ok, 	_left},
{_entry,	_fail,	_leave},
{_left,		_ok, 	_centre},
{_left, 	_fail,	_centre},
{_centre,	_repeat,_centre},
{_centre, 	_ok,	_right},
{_centre, 	_fail,	_leave},
{_right,	_ok,	_centre},
{_right,	_fail,	_left}
};
typedef int (iCubMotionControl::*IntStateFunction)(void);
IntStateFunction states[5] = {   &iCubMotionControl::entry_state, 
				&iCubMotionControl::left_state , 
				&iCubMotionControl::centre_state, 
				&iCubMotionControl::right_state, 
				&iCubMotionControl::exit_state };	
IntStateFunction function_to_call;
int cur_state, pre_state;

    iCubMotionControl(const double period) : PeriodicThread(period)
    {
        // here we specify that the event we are interested in is
        // of type "motion-done"
        gazeEventParameters.type="motion-done";
    }


    virtual bool threadInit()
    {			
	

/*	//connect to ikingaze control
	Property optGaze("(device gazecontrollerclient)");
        optGaze.put("remote","/iKinGazeCtrl");
        optGaze.put("local","/gaze_client");
	if (!clientGaze.open(optGaze))
            return false;
        // open the view
        clientGaze.view(igaze);
	
	//actuator motor settings
	igaze->setNeckTrajTime(0.8);
        igaze->setEyesTrajTime(0.4);
	igaze->setTrackingMode(true);

	//get control properties
	Bottle bottle;
        igaze->getInfo(bottle);
        printf("------------->Bottle info : %s\n",bottle.toString().c_str());
	
	//get the starting vector of gaze
	getFixationPoint(); */
	
	cur_state = ENTRY_STATE;
 	
	//Try enter the State Machine
	function_to_call = states[cur_state];
	int retCode = (this->*function_to_call)();
	pre_state = cur_state;//save last state
	cur_state = lookup_transitions(cur_state, retCode);//get new state
	fp.resize(3);
	lookAtPixel.resize(2);
	selectedCamera = 0; //left camera

	//connect the Yarp port for XY axis of gaze
	XYPort.open(inputXY);
	Network::connect(outputXY, inputXY);

	Property optTorso("(device remote_controlboard)");
        optTorso.put("remote","/icubSim/torso");
        optTorso.put("local","/torso_client");


        if (!cubTorsoDrv.open(optTorso))
            return false;

        // open the view
        cubTorsoDrv.view(iencTorso);
        cubTorsoDrv.view(iposCtrlTorso);
        iposCtrlTorso->setRefSpeed(0,10.0);

	Property optRightArm("(device remote_controlboard)");
	optRightArm.put("remote","/icubSim/right_arm");
        optRightArm.put("local","/right_arm_client");

	if (!cubRightArmDrv.open(optRightArm))
            return false;

	cubRightArmDrv.view(iencRightArm);
        cubRightArmDrv.view(iposCtrlRightArm);
        iposCtrlRightArm->setRefSpeed(0,10.0);

	t=t0=t1=t2=t3=Time::now();


	return true; 
    }

    void getFixationPoint(){
	Vector x;
        igaze->getFixationPoint(x);
	printf("------------->Gaze vector at: [%s]\n",x.toString(3,3).c_str());       
	}


    virtual void afterStart(bool s)
    {
	printf("start()   >>>>>>>\n");
        if (s)
	    printf("------------->Thread started successfully\n");
        else
            printf("------------->Thread did not start\n");      
	printf("start()   <<<<<<<\n");
    }

    virtual void run()
    {
	bool over; 

        lock_guard<mutex> lg(myMutex);

        t=Time::now();
	
	printStatus();

	if (EXIT_STATE == cur_state)
	       	   askToStop();	//exit_state will be called when thread stops
	//else, just call the current state function and decide next state
	int retCode;
	function_to_call = states[cur_state];
	//printf("\n function call = %d", function_to_call);
    	retCode = (this->*function_to_call)();
	pre_state = cur_state;//save last state so we can track
	if (cur_state == _centre) { //we expect some events are not completed
		iposCtrlRightArm->checkMotionDone(&over);
		if (!over){ return; } //wait till manouever is completed
	}
	//in other cases check if Torso Movement is completed
	iposCtrlTorso->checkMotionDone(&over);
		if (!over){ return; } //wait till manouever is completed
	//otherwise
	if (t-t2>=STATE_CHANGE_FREQ){ //switch state every few seconds
	       	cur_state = lookup_transitions(cur_state, retCode);
		t2 = t = Time::now();
	}  
    }

    virtual void threadRelease()
    {
    	int leave =	exit_state();
	/*igaze->stopControl();
	clientGaze.close();*/
	cubTorsoDrv.close();
	cubRightArmDrv.close();
    }

    void findTarget()
    {   
        Vector *view = XYPort.read(); // read a target
	objectFound = false; faceFound = false;
	if (view!=NULL) { 
			if ((*view)[2] == 5) objectFound = true; 
			if ((*view)[2] == 10) faceFound = true; 
	}
	printf("findTarget()  >>>>>>> \n");
	printf("-------------->Incoming Feed m[]= %s \n", (*view).toString(3,3).c_str());
	if (objectFound or faceFound) { 
		
		fp[0] = (*view)[0];
		fp[0] -= 320/2 ; fp[0] = fp[0] * 0.1; //fp[0]/-62.81; //x axis
		fp[1] = (*view)[1]; ///-34; //y axis
		fp[1] -= 240/2; fp[1] = -fp[1] * 0.1;
		fp[2] = 1; //fixed z axis 
/*		printf("findTarget()  >>>>>>> \n");
		printf("-------------->Object %g at %g , %g \n", (*view)[2], fp[0], fp[1]);
		printf("findTarget()  <<<<<<< \n");
*/
		lookAtPixel[0]=(*view)[0];
		lookAtPixel[1]=(*view)[1];
		printf("-------------->Object on Left Image Pane at X= %d ,Y= %d \n", lookAtPixel[0], lookAtPixel[1]);
	}
	printf("findTarget()  <<<<<<< \n");
	
    }

    void storeInterestingPoint()
    {
       
    }

    void printStatus()
    {        
        if (t-t1>=STATUS_PRINT_FREQ)
        {
            printf("printStatus() >>>>>>>\n");
	    printf("------------->Current State = (%s)\n", state_code_names[cur_state].c_str()); 
            printf("printStatus() <<<<<<<\n");

            t1=t;
        }
    }

int lookup_transitions(int cur_state, int retCode){
 	int i = 0;
	enum state_codes temp = EXIT_STATE;
	for (i = 0;; ++i) {
	  if (state_transitions[i].frm_state == cur_state && state_transitions[i].ret_code == retCode) {
		temp = state_transitions[i].to_state;
		break;
	  }
	}
	return temp;
}

int entry_state(void){
	printf("\n Entering the State Machine \n");
	return _ok;
}

int exit_state(void){
	printf("\n Exiting the State Machine \n");	
	return _ok;
}

void move_torso(double i, double j)
{	if(pre_state != cur_state){

	bool over = false; 
	int jnts = 0;
	iposCtrlTorso->setRefSpeed(jnts,10.0);
	iposCtrlTorso->getAxes(&jnts);
 	iposCtrlTorso->positionMove(i, j);  }
	
}


void wave_hand( )
{	printf("\n wave_hand() \n");
	int jnts = 0;
	bool over = false; 
	iposCtrlRightArm->getAxes(&jnts);
	iposCtrlRightArm->setRefSpeed(0,10.0);
 	iposCtrlRightArm->positionMove(0, -45);
	
	jnts = 2;
	iposCtrlRightArm->getAxes(&jnts);
	iposCtrlRightArm->setRefSpeed(2,10.0);
	iposCtrlRightArm->positionMove(2, -35);

	jnts = 3;
	iposCtrlRightArm->getAxes(&jnts);
	iposCtrlRightArm->setRefSpeed(3,10.0);
	iposCtrlRightArm->positionMove(3, 90);
	
	
	jnts = 6;
	iposCtrlRightArm->getAxes(&jnts);
	iposCtrlRightArm->setRefSpeed(6,10.0);
	iposCtrlRightArm->positionMove(6, 20);
	//while(1) { iposCtrlRightArm->checkMotionDone(&over); if(over) break; }
	iposCtrlRightArm->positionMove(6, -20);
	//while(1) { iposCtrlRightArm->checkMotionDone(&over); if(over) break; }
	iposCtrlRightArm->positionMove(6, 20);
	//while(1) { iposCtrlRightArm->checkMotionDone(&over); if(over) break; }
	iposCtrlRightArm->positionMove(6, -20);
	/*double st = Time::now();
	while(!over) iposCtrlRightArm->checkMotionDone(&over); 
	st = Time::now() - st;
	printf("\n Time taken to finish the Wave manoveur : %g", st) 	;*/
	
}

void reset_hand( )
{	printf("\n reset_hand() \n");
	int jnts;
	jnts = 6;
	iposCtrlRightArm->getAxes(&jnts);
 	iposCtrlRightArm->positionMove(6, 0);
	jnts = 3;
	iposCtrlRightArm->getAxes(&jnts);
	iposCtrlRightArm->positionMove(3, 50);
	jnts = 2;
	iposCtrlRightArm->getAxes(&jnts);
	iposCtrlRightArm->positionMove(2, 0);
	jnts = 0;
	iposCtrlRightArm->getAxes(&jnts);
	iposCtrlRightArm->positionMove(0, 0);
}



int left_state(void){
	//look left	
	//printf("\n left_state() \n");
	move_torso(0,-15);
	return _ok;
}

int centre_state(void){
	//printf("\n center_state() \n");
	move_torso(0, 0);
	if(cur_state != pre_state) t0 = Time::now(); //initialize time in case this status is new
	//process the image stream
	//analyzeWebCamFeed((float)1); //look for 1 second
	//find the fp from analysis above.
	findTarget();
	if (t-t0>=CENTRE_GAZE_TIMEOUT) return _ok;
	if(faceFound or objectFound){ //if any object or face found, look there
		double z=1.0;   // distance [m] of the object from the image plane (extended to infinity)
		//igaze->registerEvent(*this);
		//igaze->lookAtMonoPixel(selectedCamera,lookAtPixel,z);    // look!
		if (faceFound){ //say hi!
				wave_hand( );
				Time::delay(1);
				reset_hand( );
			
		}
		return _repeat ; 
	}else return _ok; //go to next state

	

}


int right_state(void){
	//printf("\n right_state() \n");
	//look right
	move_torso(0,15);
	findTarget();
	if(faceFound or objectFound)	return _ok;
	else return _fail; 
}

};

/*************************************************************************/
/*             The Master Class defintion				 */
/*************************************************************************/
//this is inheritence of yarp::os::RFModule Class, so we can use the ResourceFinder, 
//then to start the Controller Thread (https://www.yarp.it/yarp_resource_finder_basic.html)
class MotionMaster: public RFModule
{
 protected:
    iCubMotionControl *controllerThread;

 public:
    virtual bool configure(ResourceFinder &resFinder)
    {
        controllerThread=new iCubMotionControl(THREAD_RUN_FREQ);
        if (!controllerThread->start())
        {
            delete controllerThread;
            return false;
        }

        return true;
    }

    virtual bool close()
    {
        controllerThread->stop();
        delete controllerThread;

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};



int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        fprintf(stdout,"Error: yarp server does not seem available\n");
        return 1;
    }

    MotionMaster master;

    ResourceFinder resFinder;
    return master.runModule(resFinder);
}


