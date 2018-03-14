// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// A tutorial on how to use the Cartesian Interface to control a limb
// in the operational space.
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

//NOTE FLIPPED Y AND X! Y = HORIZONTAL X = VERTICAL

#include <cstdio>
#include <cmath>
#include <iostream>
#include <string>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <iCub/iKin/iKinVocabs.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/iKin/iKinIpOpt.h>

#define CTRL_THREAD_PER     0.02    // [s]
#define PRINT_STATUS_PER    1.0     // [s]
#define MAX_TORSO_PITCH     30.0    // [deg]

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iKin;

class CtrlThread: public RateThread,
                  public CartesianEvent
{
protected:
    PolyDriver         client;
    ICartesianControl *icart;

    PolyDriver positionRight;
    IPositionControl *posRight;

    //motor movement
    PolyDriver robotDevice;
    IPositionControl *movePos;
    IEncoders *moveEncs;

    Vector xd;
    Vector od;

    Vector home;
    Vector home_od;
    Vector x_notes;
    Vector y_notes;

    int startup_context_id;

    double t;
    double t0;
    double t1;

    //key spaces x
    double white_white_y;
    double small_white_black_y;
    double big_white_black_y;
    double g_to_a_y; //half white_white?
    double black_white_x;

    char ack;
    int index;

    //to fill in later on physical robot?
    double robotOffset;
    double tableHeight;

    // the event callback attached to the "motion-ongoing"
    virtual void cartesianEventCallback()
    {
        fprintf(stdout,"20%% of trajectory attained\n");
    }

public:
    CtrlThread(const double period) : RateThread(int(period*1000.0))
    {
        // we wanna raise an event each time the arm is at 20%
        // of the trajectory (or 80% far from the target)
        cartesianEventParameters.type="motion-ongoing";
        cartesianEventParameters.motionOngoingCheckPoint=0.2;
    }

    virtual bool threadInit()
    {
        // open a client interface to connect to the cartesian server of the simulator
        // we suppose that:
        //
        // 1 - the iCub simulator is running
        //     (launch: iCub_SIM)
        //
        // 2 - the cartesian server is running
        //     (launch: yarprobotinterface --context simCartesianControl)
        //
        // 3 - the cartesian solver for the right arm is running too
        //     (launch: iKinCartesianSolver --context simCartesianControl --part right_arm)
        //
        Property option("(device cartesiancontrollerclient)");
        option.put("remote","/icub/cartesianController/right_arm");
        option.put("local","/cartesian_client/right_arm");

        std::string robotName="icub";
        std::string remotePorts="/";
        remotePorts+=robotName;
        remotePorts+="/right_arm";

        std::string localPorts="/test/client";

        Property options;
        options.put("device", "remote_controlboard");
        options.put("local", localPorts.c_str());   //local port names
        options.put("remote", remotePorts.c_str());         //where we connect to

        // create a device
        if (!robotDevice.open(options)) {
            printf("Device not available.  Here are the known devices:\n");
            printf("%s", Drivers::factory().toString().c_str());
            return 0;
        }

        bool ok;
        ok = robotDevice.view(movePos);
        ok = ok && robotDevice.view(moveEncs);

        if (!ok) {
            printf("Problems acquiring interfaces\n");
            return 0;
        }

        if (!client.open(option))
            return false;

        int nj=0;
        movePos->getAxes(&nj);
        Vector encoders;
        Vector command;
        Vector tmp;
        encoders.resize(nj);
        tmp.resize(nj);
        command.resize(nj);
        
        int i;

        for (i = 0; i < nj; i++) {
            tmp[i] = 1.0;
            movePos->setRefSpeed(i, tmp[i]);
        }

        //pos->setRefSpeeds(tmp.data()))
        
        //fisrst read all encoders
        //
        printf("waiting for encoders");
        while(!moveEncs->getEncoders(encoders.data()))
        {
            Time::delay(0.1);
            printf(".");
        }
        printf("\n");

        cout << "movement one" << endl;

        command=encoders;

        command[0]=-10;
        command[1]=80;
        command[2]=0;
        command[3]=75;
        command[4]=20;
        command[5]=0;
        command[6]=0;
        //hand down position?
        command[7]=60;
        command[8]=19;
        command[9]=40;
        command[10]=1;
        command[11]=2;
        command[12]=10;
        command[13]=48;
        command[14]=0;
        command[15]=14;
        movePos->positionMove(command.data());
        
        bool done=false;

        while(!done)
        {
            movePos->checkMotionDone(&done);
            Time::delay(0.1);
        }

        robotDevice.close();

        // open the view
        client.view(icart);

        // latch the controller context in order to preserve
        // it after closing the module
        // the context contains the dofs status, the tracking mode,
        // the resting positions, the limits and so on.
        icart->storeContext(&startup_context_id);

        // set trajectory time
        icart->setTrajTime(1.0);

        // get the torso dofs
        Vector newDof, curDof;
        icart->getDOF(curDof);

        newDof=curDof;

        // enable the torso yaw and pitch
        // disable the torso roll
        newDof[0]=0;
        newDof[1]=0;
        newDof[2]=0;

        // send the request for dofs reconfiguration
        icart->setDOF(newDof,curDof); 
        icart->getDOF(curDof);
        fprintf(stdout,"curDof = %s\n",curDof.toString().c_str());  

        Vector xdhat, odhat, armPos;
        icart->askForPose(xd,od, xdhat, odhat, armPos);
        fprintf(stdout,"armPos = %s\n",armPos.toString().c_str());

        // print out some info about the controller
        Bottle info;
        icart->getInfo(info);
        fprintf(stdout,"info = %s\n",info.toString().c_str());

        // register the event, attaching the callback
        icart->registerEvent(*this);

        iCubFinger finger("right_middle");
        int nEncs;
        moveEncs->getAxes(&nEncs);
        // Vector encs(nEncs);
        // moveEncs->getEncoders(encs.data());

        Vector joints;
        finger.getChainJoints(command, joints);
        Matrix tipFrame=finger.getH((M_PI/180.0)*joints);

        Vector tip_x=tipFrame.getCol(3);
        Vector tip_o=iCub::ctrl::dcm2axis(tipFrame);
        icart->attachTipFrame(tip_x,tip_o);

        xd.resize(3);
        od.resize(4);
        home.resize(3);
        home_od.resize(4);
        x_notes.resize(12);
        y_notes.resize(12);

        icart->getPose(home, home_od);

        // home[0]=-0.25;
        // home[1]=0.2;
        // home[2]=+0.2;

        tableHeight = 0.1;

        // goHome();
        // icart->goToPoseSync(xd,od);
        // icart->waitMotionDone(0.04);

        //middle c caused last two notes to be unreachable?
        cout << "(Wait until finger movement is finished)" << endl;
        cout << "Line up F with middle finger in this position. Enter any character to continue." << endl;
        cin >> ack; 
        
        white_white_y=0.0225;
        small_white_black_y=0.00825; 
        big_white_black_y=0.01425;
        g_to_a_y=0.01125;
        black_white_x=0.035;

        //NOTE "y" is horizontal due to the setup. +y = move right from robot POV
        //-x = move forward from robot POV
        // -z = move down from robot POV
        //C
        x_notes[0]=home[0];
        y_notes[0]=home[1] - white_white_y * 3;

        //C#/Db
        x_notes[1]=home[0] - black_white_x;
        y_notes[1]=home[1] - white_white_y * 2 - big_white_black_y;

        //D
        x_notes[2]=home[0];
        y_notes[2]=home[1] - white_white_y * 2;

        //D#/Eb
        x_notes[3]=home[0] - black_white_x;
        y_notes[3]=home[1] - small_white_black_y - white_white_y;

        //E
        x_notes[4]=home[0];
        y_notes[4]=home[1] - white_white_y;

        //F
        x_notes[5]=home[0];
        y_notes[5]=home[1];

        //F#/Gb
        x_notes[6]=home[0] - black_white_x;
        y_notes[6]=y_notes[5] + small_white_black_y;

        //G
        x_notes[7]=home[0];
        y_notes[7]=y_notes[6] + big_white_black_y;

        //G#/Ab
        x_notes[8]=home[0] - black_white_x;
        y_notes[8]=y_notes[7] + g_to_a_y;

        //A
        x_notes[9]=home[0];
        y_notes[9]=y_notes[8] + g_to_a_y;

        //A#/Bb
        x_notes[10]=home[0] - black_white_x;
        y_notes[10]=y_notes[9] + big_white_black_y;

        //B
        x_notes[11]=home[0];
        y_notes[11]=y_notes[10] + small_white_black_y;

        index = 0;
        return true;
    }

    virtual void afterStart(bool s)
    {
        if (s)
            fprintf(stdout,"Thread started successfully\n");
        else
            fprintf(stdout,"Thread did not start\n");

        t=t0=t1=Time::now();
    }

    virtual void run()
    {
        Vector xdhat,odhat,armPos;
        Vector test;
        test.resize(12);
        test[0]=0;
        test[1]=2;
        test[2]=4;
        test[3]=5;
        test[4]=7;
        test[5]=9;
        test[6]=11;
        test[7]=10;
        test[8]=8;
        test[9]=6;
        test[10]=3;
        test[11]=1;
        t=Time::now();

        generateTarget(test[index]);

        // go to the target 
        cout << "Going to this note: " << test[index] << endl;
		icart->askForPose(xd,od, xdhat, odhat, armPos);
        fprintf(stdout,"armPos = %s\n",armPos.toString().c_str());
        cout << "Continue to move?" <<endl;
        cin >> ack;
        icart->goToPoseSync(xd,od);
        icart->waitMotionDone(0.04);

        cout << "Continue?" << endl;
        cin >> ack;

        //go down
        xd[2] = tableHeight;
		icart->askForPose(xd,od, xdhat, odhat, armPos);
        fprintf(stdout,"armPos = %s\n",armPos.toString().c_str());
        cout << "Continue?" << endl;
        cin >> ack;
        icart->goToPoseSync(xd,od);
        icart->waitMotionDone(0.04);

        cout << "Continue?" << endl;
        cin >> ack;

        //back up
        xd[2] = home[2];
		icart->askForPose(xd,od, xdhat, odhat, armPos);
        fprintf(stdout,"armPos = %s\n",armPos.toString().c_str());
        cout << "Continue?" << endl;
        cin >> ack;
        icart->goToPoseSync(xd,od);
        icart->waitMotionDone(0.04);

        cout << "Continue?" << endl;
        cin >> ack;

        index++;
        if(index == 12)
            index = 0;

        // some verbosity
        //printStatus();
    }

    virtual void threadRelease()
    {
        // we require an immediate stop
        // before closing the client for safety reason
        icart->stopControl();

        // it's a good rule to restore the controller
        // context as it was before opening the module
        icart->restoreContext(startup_context_id);

        client.close();
    }

    void generateTarget(int i)
    {
        // translational target part: a circular trajectory
        // in the yz plane centered in [-0.3,-0.1,0.1] with radius=0.1 m
        // and frequency 0.1 Hz
        xd[0]=x_notes[i];
        xd[1]=y_notes[i];
        xd[2]=home[2];

        // we keep the orientation of the left arm constant:
        // we want the middle finger to point forward (end-effector x-axis)
        // with the palm turned down (end-effector y-axis points leftward);
        // to achieve that it is enough to rotate the root frame of pi around z-axis
        //od[0]=0.0; od[1]=1.0; od[2]=0.0; od[3]=M_PI;
        od = home_od;
    }

    void goHome()
    {
        xd = home;
        od = home_od;
    }

    void printStatus()
    {
        if (t-t1>=PRINT_STATUS_PER)
        {
            Vector x,o,xdhat,odhat,qdhat;

            // we get the current arm pose in the
            // operational space
            icart->getPose(x,o);

            // we get the final destination of the arm
            // as found by the solver: it differs a bit
            // from the desired pose according to the tolerances
            icart->getDesired(xdhat,odhat,qdhat);

            double e_x=norm(xdhat-x);
            double e_o=norm(odhat-o);

            fprintf(stdout,"+++++++++\n");
            fprintf(stdout,"xd          [m] = %s\n",xd.toString().c_str());
            fprintf(stdout,"xdhat       [m] = %s\n",xdhat.toString().c_str());
            fprintf(stdout,"x           [m] = %s\n",x.toString().c_str());
            fprintf(stdout,"od        [rad] = %s\n",od.toString().c_str());
            fprintf(stdout,"odhat     [rad] = %s\n",odhat.toString().c_str());
            fprintf(stdout,"o         [rad] = %s\n",o.toString().c_str());
            fprintf(stdout,"norm(e_x)   [m] = %g\n",e_x);
            fprintf(stdout,"norm(e_o) [rad] = %g\n",e_o);
            fprintf(stdout,"---------\n\n");

            t1=t;
        }
    }
};



class CtrlModule: public RFModule
{
protected:
    CtrlThread *thr;

public:
    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        thr=new CtrlThread(CTRL_THREAD_PER);
        if (!thr->start())
        {
            delete thr;
            return false;
        }

        return true;
    }

    virtual bool close()
    {
        thr->stop();
        delete thr;

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};



YARP_DECLARE_DEVICES(icubmod) int main()
{
    YARP_REGISTER_DEVICES(icubmod);
    Network yarp;
    if (!yarp.checkNetwork())
    {
        fprintf(stdout,"Error: yarp server does not seem available\n");
        return 1;
    }

    CtrlModule mod;

    ResourceFinder rf;
    return mod.runModule(rf);
}
