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

    PolyDriver positionLeft;
    IPositionControl *posLeft;

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

    Vector command;

    int startup_context_id;
    int run_mode;

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
        if (!positionRight.open(options)) {
            printf("Device not available.  Here are the known devices:\n");
            printf("%s", Drivers::factory().toString().c_str());
            return 0;
        }

        bool ok;
        ok = positionRight.view(posRight);
        ok = ok && positionRight.view(moveEncs);

        if (!ok) {
            printf("Problems acquiring interfaces\n");
            return 0;
        }

        if (!client.open(option))
            return false;

        int nj=0;
        posRight->getAxes(&nj);
        Vector encoders;
        
        Vector tmp;
        encoders.resize(nj);
        tmp.resize(nj);
        command.resize(nj);
        
        int i;
        for (i = 0; i < nj; i++) {
             tmp[i] = 50.0;
        }
        posRight->setRefAccelerations(tmp.data());

        for (i = 0; i < nj; i++) {
            tmp[i] = 10.0;
            posRight->setRefSpeed(i, tmp[i]);
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

        command[0]=-9;
        command[1]=80;
        command[2]=0;
        command[3]=75;
        command[4]=20;
        command[5]=0;
        command[6]=0;
        //hand down position?
        command[7]=38;
        command[8]=4;
        command[9]=48;
        command[10]=55;
        command[11]=2;
        command[12]=10;
        command[13]=48;
        command[14]=0;
        command[15]=14;
        posRight->positionMove(command.data());
        
        bool done=false;

        while(!done)
        {
            posRight->checkMotionDone(&done);
            Time::delay(0.1);
        }

        //positionRight.close();

        //move left hand out of the way
        std::string remotePorts2="/";
        remotePorts2+=robotName;
        remotePorts2+="/left_arm";

        std::string localPorts2="/test/clientLeft";

        Property options2;
        options2.put("device", "remote_controlboard");
        options2.put("local", localPorts2.c_str());   //local port names
        options2.put("remote", remotePorts2.c_str());         //where we connect to

        // create a device
        if (!positionLeft.open(options2)) {
            printf("Device not available.  Here are the known devices:\n");
            printf("%s", Drivers::factory().toString().c_str());
            return 0;
        }

        ok = positionLeft.view(posLeft);
        ok = ok && positionLeft.view(moveEncs);

        if (!ok) {
            printf("Problems acquiring interfaces\n");
            return 0;
        }
        
        int nj2=0;
        posLeft->getAxes(&nj2);
        Vector encoders2;
        
        Vector tmp2;
        encoders2.resize(nj2);
        tmp2.resize(nj2);
        command.resize(nj2);

        for (i = 0; i < nj2; i++) {
             tmp2[i] = 50.0;
        }
        posLeft->setRefAccelerations(tmp2.data());

        for (i = 0; i < nj2; i++) {
            tmp2[i] = 10.0;
            posLeft->setRefSpeed(i, tmp2[i]);
        }

        //pos->setRefSpeeds(tmp.data()))
        
        //fisrst read all encoders
        //
        printf("waiting for encoders");
        while(!moveEncs->getEncoders(encoders2.data()))
        {
            Time::delay(0.1);
            printf(".");
        }
        printf("\n");

		cout << "Right hand moved. Continue?" << endl;
        cin >> ack;
        cout << "moving left hand out of the way..." << endl;

        command=encoders2;

        command[0]=0;
        command[1]=25;
        command[2]=0;
        command[3]=30;
        command[4]=0;
        command[5]=0;
        command[6]=0;
        //hand down position?
        command[7]=0;
        command[8]=0;
        command[9]=11;
        command[10]=31;
        command[11]=7;
        command[12]=0;
        command[13]=7;
        command[14]=3;
        command[15]=0;
        posLeft->positionMove(command.data());
        
        done=false;

        while(!done)
        {
            posLeft->checkMotionDone(&done);
            Time::delay(0.1);
        }

        //back to finger down
        command[7]=38;
        command[8]=4;
        command[9]=48;
        command[10]=55;
        command[11]=2;
        command[12]=10;
        command[13]=48;
        command[14]=0;
        command[15]=14;
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

		/*
        iCubFinger finger("right_middle");
        int nEncs;
        moveEncs->getAxes(&nEncs);
        // Vector encs(nEncs);
        // moveEncs->getEncoders(encs.data());

        Vector joints;
        finger.getChainJoints(command, joints);
        Matrix tipFrame=finger.getH((M_PI/180.0)*joints);

        Vector tip_x=tipFrame.getCol(3);
        Vector tip_o=dcm2axis(tipFrame);
        icart->attachTipFrame(tip_x,tip_o);
        */

        xd.resize(3);
        od.resize(4);
        home.resize(3);
        home_od.resize(4);
        x_notes.resize(12);
        y_notes.resize(12);

        icart->getPose(home, home_od);
        fprintf(stdout,"home position = %s\n",home.toString().c_str());
        fprintf(stdout,"home angle = %s\n",home_od.toString().c_str());

        // home[0]=-0.25;
        // home[1]=0.2;
        // home[2]=+0.2;

        tableHeight = 0.16;

        // goHome();
        // icart->goToPoseSync(xd,od);
        // icart->waitMotionDone(0.04);

        //middle c caused last two notes to be unreachable?
        cout << "(Wait until finger movement is finished)" << endl;
        cout << "Line up A with middle finger in this position. Enter any character to continue." << endl;
        cout << "Table height is "  << tableHeight << "Z=0 around 65cm?" << endl;
        cin >> ack; 
        
        white_white_y=0.0225;
        small_white_black_y=0.00825; 
        big_white_black_y=0.01425;
        g_to_a_y=0.01125;
        black_white_x=0.035;

        //NOTE "y" is horizontal due to the setup. +y = move right from robot POV
        //-x = move forward from robot POV
        // -z = move down from robot POV
        //TODO: Change these values
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

        cout << "Run runmode 0(Cartesian) or 1(Motor)?" << endl;
        cin >> ack;
        run_mode = ack - '0';
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
        test[0]=9;
        test[1]=11;
        test[2]=0;
        test[3]=2;
        test[4]=4;
        test[5]=5;
        test[6]=7;

        t=Time::now();

        if(run_mode == 0)
        {
            generateTarget(test[index]);

            // go to the target 
            cout << "Going to this note: " << test[index] << endl;
            icart->goToPoseSync(xd,od);
            icart->waitMotionDone(0.04);
            icart->askForPose(xd,od, xdhat, odhat, armPos);
            fprintf(stdout,"armPos = %s\n",armPos.toString().c_str());
            cout << "Continue?" << endl;
            cin >> ack;

            //go down
            xd[2] = tableHeight;
            icart->goToPoseSync(xd,od);
            icart->waitMotionDone(0.04);
            icart->askForPose(xd,od, xdhat, odhat, armPos);
            fprintf(stdout,"armPos = %s\n",armPos.toString().c_str());
            cout << "Continue?" << endl;
            cin >> ack;

            //back up
            xd[2] = home[2];
            icart->goToPoseSync(xd,od);
            icart->waitMotionDone(0.04);
            icart->askForPose(xd,od, xdhat, odhat, armPos);
            fprintf(stdout,"armPos = %s\n",armPos.toString().c_str());
            cout << "Continue?" << endl;
            cin >> ack;
        }
        else
        {
            generateTarget(test[index], "up");
            cout << "Going to this note: " << test[index] << endl;
            posRight->positionMove(command.data());
            
            bool done=false;

            while(!done)
            {
                posRight->checkMotionDone(&done);
                Time::delay(0.1);
            }
            cout << "Continue?" << endl;
            cin >> ack;

            generateTarget(test[index], "down");
            posRight->positionMove(command.data());
            
            done=false;

            while(!done)
            {
                posRight->checkMotionDone(&done);
                Time::delay(0.1);
            }
            cout << "Continue?" << endl;
            cin >> ack;

            generateTarget(test[index], "up");
            posRight->positionMove(command.data());
            
            done=false;

            while(!done)
            {
                posRight->checkMotionDone(&done);
                Time::delay(0.1);
            }
            cout << "Continue?" << endl;
            cin >> ack;
        }
        index++;
        if(index == test.size)
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

    void generateTarget(int i, string s)
    {
		int up = -9;
		int down = -4;
        switch(i)
        {
            //DONT USE FLATS
            case 0:
                if(s == "up")
                {
                    command[0]=up;
                    command[1]=80;
                    command[2]=0;
                    command[3]=67.5;
                    command[4]=20;
                    command[5]=0;
                    command[6]=0;
                }

                if(s == "down")
                {
                    command[0]=down;
                    command[1]=80;
                    command[2]=0;
                    command[3]=67.5;
                    command[4]=20;
                    command[5]=0;
                    command[6]=0;
                }
                break;
            case 1:
                if(s == "up")
                {
                    command[0]=-62;
                    command[1]=72;
                    command[2]=58;
                    command[3]=78;
                    command[4]=22;
                    command[5]=-2;
                    command[6]=17;
                }

                if(s == "down")
                {
                    command[0]=-52;
                    command[1]=73;
                    command[2]=59;
                    command[3]=78;
                    command[4]=22;
                    command[5]=-13;
                    command[6]=19;
                }
                break;
            case 2:
                if(s == "up")
                {
                    command[0]=up;
                    command[1]=80;
                    command[2]=0;
                    command[3]=63;
                    command[4]=20;
                    command[5]=0;
                    command[6]=0;
                }

                if(s == "down")
                {
                    command[0]=down;
                    command[1]=80;
                    command[2]=0;
                    command[3]=63;
                    command[4]=20;
                    command[5]=0;
                    command[6]=0;
                }
                break;
            case 3:
                if(s == "up")
                {
                    command[0]=-67;
                    command[1]=74;
                    command[2]=63;
                    command[3]=67;
                    command[4]=19;
                    command[5]=-6;
                    command[6]=7;
                }

                if(s == "down")
                {
                    command[0]=-56;
                    command[1]=75;
                    command[2]=63;
                    command[3]=68;
                    command[4]=17;
                    command[5]=-16;
                    command[6]=8;   
                }
                break;
            case 4:
                if(s == "up")
                {
                    command[0]=up;
                    command[1]=80;
                    command[2]=0;
                    command[3]=57;
                    command[4]=20;
                    command[5]=0;
                    command[6]=0;
                }

                if(s == "down")
                {
                    command[0]=down;
                    command[1]=80;
                    command[2]=0;
                    command[3]=57;
                    command[4]=20;
                    command[5]=0;
                    command[6]=0;
                }
                break;
            case 5:
                if(s == "up")
                {
                    command[0]=up;
                    command[1]=80;
                    command[2]=0;
                    command[3]=52;
                    command[4]=20;
                    command[5]=0;
                    command[6]=0;
                }

                if(s == "down")
                {
                    command[0]=down;
                    command[1]=80;
                    command[2]=0;
                    command[3]=52;
                    command[4]=20;
                    command[5]=0;
                    command[6]=0;
                }
                break;
            case 6:
                if(s == "up")
                {
                    command[0]=-75;
                    command[1]=71;
                    command[2]=64;
                    command[3]=44;
                    command[4]=21;
                    command[5]=-7;
                    command[6]=-13;
                }

                if(s == "down")
                {
                    command[0]=-63;
                    command[1]=71;
                    command[2]=63;
                    command[3]=45;
                    command[4]=15;
                    command[5]=-17;
                    command[6]=-12;
                }
                break;
            case 7:
                if(s == "up")
                {
                    command[0]=up;
                    command[1]=80;
                    command[2]=0;
                    command[3]=47;
                    command[4]=20;
                    command[5]=0;
                    command[6]=0;
                }

                if(s == "down")
                {
                    command[0]=down;
                    command[1]=80;
                    command[2]=0;
                    command[3]=47;
                    command[4]=20;
                    command[5]=0;
                    command[6]=0;
                }
                break;
            case 8:
                if(s == "up")
                {
                    command[0]=-84;
                    command[1]=65;
                    command[2]=65;
                    command[3]=26;
                    command[4]=27;
                    command[5]=-10;
                    command[6]=-20;
                }

                if(s == "down")
                {
                    command[0]=-73;
                    command[1]=66;
                    command[2]=68;
                    command[3]=26;
                    command[4]=18;
                    command[5]=-18;
                    command[6]=-20;
                }
                break;
            case 9:
                if(s == "up")
                {
                    command[0]=up;
                    command[1]=80;
                    command[2]=0;
                    command[3]=75;
                    command[4]=20;
                    command[5]=0;
                    command[6]=0;
                }

                if(s == "down")
                {
                    command[0]=down;
                    command[1]=80;
                    command[2]=0;
                    command[3]=75;
                    command[4]=20;
                    command[5]=0;
                    command[6]=0;
                }
                break;
            case 10:
                if(s == "up")
                {
                    command[0]=-87;
                    command[1]=59;
                    command[2]=60;
                    command[3]=16;
                    command[4]=39;
                    command[5]=-16;
                    command[6]=-12;
                }

                if(s == "down")
                {
                    command[0]=-77;
                    command[1]=59;
                    command[2]=57;
                    command[3]=16;
                    command[4]=36;
                    command[5]=-19;
                    command[6]=-12;
                }
                break;
            case 11:
                if(s == "up")
                {
                    command[0]=up;
                    command[1]=80;
                    command[2]=0;
                    command[3]=71.5;
                    command[4]=20;
                    command[5]=0;
                    command[6]=0;
                }

                if(s == "down")
                {
                    command[0]=down;
                    command[1]=80;
                    command[2]=0;
                    command[3]=71.5;
                    command[4]=20;
                    command[5]=0;
                    command[6]=0;
                }
                break;
        }
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
