
#pragma config(Sensor, dgtl1,  ER,             sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  EL,             sensorQuadEncoder)
#pragma config(Motor,  port2,           shooterR1,     tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           shooterR2,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           intakeF,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           driveR,        tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           driveL,        tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           intakeB,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           shooterL1,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           shooterL2,     tmotorVex393_MC29, openLoop, reversed)

#pragma platform(VEX)
//Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(15)
#pragma userControlDuration(105)

#include "Vex_Competition_Includes.c"

static int L_fullCourt_speed = 1500;
static int L_halfCourt_speed = 1000;
const int L_speedInc = 25;

static int R_fullCourt_speed = 1500;
static int R_halfCourt_speed = 1000;
const int R_speedInc = 25;

/*

kp until it oscilates then use 1/2 that
ki until it reaches target in acceptable time
kd to smooth it out

*/


//left flywheel variables
static float L_kP = 0.2;
static float L_kI = 0.002;
static float L_kD = 0.1;

//right flywheel variables
static float R_kP = 0.2;
static float R_kI = 0.002;
static float R_kD = 0.1;


int threshold = 15;

void pre_auton()
{
    // Set bStopTasksBetweenModes to false if you want to keep user created tasks running between
    // Autonomous and Tele-Op modes. You will need to manage all user created tasks if set to false.
    bStopTasksBetweenModes = true;

    //clear encoders
    SensorValue[ER] = 0;
    SensorValue[EL] = 0;
}

task auton_intakeControl()
{
    int count = 0;

    while (true)
    {
        //turn on intake for 1.5 sec
        while (count < 75)
        {
            motor[intakeF] = 127;
            motor[intakeB] = 127;
            ++count;
            wait1Msec(20);
        }
        //wait 1 sec
        wait1Msec(1000);
    }
}

task autonomous()
{
    startTask(auton_intakeControl);

    //clear encoders
    SensorValue[ER] = 0;
    SensorValue[EL] = 0;

    //target speed of flywheels in rpm
    int L_target = 0;
    int R_target = 0;
    //target converted into tics/100ms
    int L_trueTarget = 0;
    int R_trueTarget = 0;

    // in tics per second
    int integralActiveZone = 5;

    int L_error = 0;
    int L_lastError = 0;
    int L_errorT = 0;

    int L_vel = 0;

    float L_proportional = 0;
    float L_integral = 0;
    float L_derivative = 0;

    int R_error = 0;
    int R_lastError = 0;
    int R_errorT = 0;

    int R_vel = 0;

    float R_proportional = 0;
    float R_integral = 0;
    float R_derivative = 0;

    int L_sensVal = 0;
    int R_sensVal = 0;

    int L_prevSensVal = 0;
    int R_prevSensVal = 0;

    while (true)
    {
        L_target = L_fullCourt_speed;
        R_target = R_fullCourt_speed;

        //in tics/100ms
        L_trueTarget = L_target * 0.024;
        R_trueTarget = R_target * 0.024;

        //calc speed of flywheels
        L_prevSensVal = -SensorValue[EL];
        R_prevSensVal = SensorValue[ER];
        wait1Msec(100);
        L_sensVal = -SensorValue[EL];
        R_sensVal = SensorValue[ER];
        L_vel = (L_sensVal - L_prevSensVal);
        R_vel = (R_sensVal - R_prevSensVal);

        L_error = L_trueTarget - L_vel;
        R_error = R_trueTarget - R_vel;

        if (L_error < integralActiveZone && L_error != 0)
            L_errorT += L_error;
        else
            L_errorT = 0;

        if (R_error < integralActiveZone && R_error != 0)
            R_errorT += R_error;
        else
            R_errorT = 0;

        if (L_error == 0)
            L_derivative = 0;
        if (R_error == 0)
            R_derivative = 0;

        L_proportional = L_error * L_kP;
        L_integral = L_errorT * L_kI;
        L_derivative = (L_error - L_lastError) * L_kD;

        R_proportional = R_error * R_kP;
        R_integral = R_errorT * R_kI;
        R_derivative = (R_error - R_lastError) * R_kD;

        if (L_integral > 50)
            L_integral = 50;
        if (R_integral > 50)
            R_integral = 50;

        L_lastError = L_error;
        R_lastError = R_error;

        motor[shooterL1] = L_proportional + L_integral + L_derivative;
        motor[shooterL2] = L_proportional + L_integral + L_derivative;
        motor[shooterR1] = R_proportional + R_integral + R_derivative;
        motor[shooterR2] = R_proportional + R_integral + R_derivative;

        wait1Msec(20);
    }
}

task updateFlywheels()
{
    //clear encoders
    SensorValue[ER] = 0;
    SensorValue[EL] = 0;

    //target speed of flywheels in rpm
    static int L_target = 0;
    static int R_target = 0;
    //target converted into tics/100ms
    static int L_trueTarget = 0;
    static int R_trueTarget = 0;

    // in tics per second
    int integralActiveZone = 5;

    int L_error = 0;
    int L_lastError = 0;
    int L_errorT = 0;

    static int L_vel = 0;

    float L_proportional = 0;
    float L_integral = 0;
    float L_derivative = 0;

    int R_error = 0;
    int R_lastError = 0;
    int R_errorT = 0;

    static int R_vel = 0;

    float R_proportional = 0;
    float R_integral = 0;
    float R_derivative = 0;

    int L_sensVal = 0;
    int R_sensVal = 0;

    int L_prevSensVal = 0;
    int R_prevSensVal = 0;

    while (true)
    {
        if (vexRT[Btn8D] == 1)
        {
            L_target = L_fullCourt_speed;
            R_target = R_fullCourt_speed;
        }
        else if (vexRT[Btn8R] == 1)
        {
            L_target = L_halfCourt_speed;
            R_target = R_halfCourt_speed;
        }
        else
        {
            //L_target = 0;
            //R_target = 0;
        }
        if (vexRT[Btn7D] == 1)
        {
            L_fullCourt_speed -= L_speedInc;
            R_fullCourt_speed -= R_speedInc;
        }
        if (vexRT[Btn7U] == 1)
        {
            L_fullCourt_speed += L_speedInc;
            R_fullCourt_speed += R_speedInc;
        }

        if(L_fullCourt_speed < 0)
            L_fullCourt_speed = 0;
        if(R_fullCourt_speed < 0)
            R_fullCourt_speed = 0;

        if(L_target != 0 && R_target != 0){
        //in tics/100ms
        L_trueTarget = L_target * 0.024;
        R_trueTarget = R_target * 0.024;

        //calc speed of flywheels
        L_prevSensVal = -SensorValue[EL];
        R_prevSensVal = SensorValue[ER];
        wait1Msec(100);
        L_sensVal = -SensorValue[EL];
        R_sensVal = SensorValue[ER];
        L_vel = (L_sensVal - L_prevSensVal);
        R_vel = (R_sensVal - R_prevSensVal);

        L_error = L_trueTarget - L_vel;
        R_error = R_trueTarget - R_vel;

        if (L_error < integralActiveZone && L_error != 0)
        {
            L_errorT += L_error;
        }
        else
        {
            L_errorT = 0;
        }

        if (R_error < integralActiveZone && R_error != 0)
        {
            R_errorT += R_error;
        }
        else
        {
            R_errorT = 0;
        }

        if (L_error == 0)
        {
            L_derivative = 0;
        }
        if (R_error == 0)
        {
            R_derivative = 0;
        }

        L_proportional = L_error * L_kP;
        L_integral = L_errorT * L_kI;
        L_derivative = (L_error - L_lastError) * L_kD;

        R_proportional = R_error * R_kP;
        R_integral = R_errorT * R_kI;
        R_derivative = (R_error - R_lastError) * R_kD;

        if (L_integral > 50)
        {
            L_integral = 50;
        }
        if (R_integral > 50)
        {
            R_integral = 50;
        }

        L_lastError = L_error;
        R_lastError = R_error;

        motor[shooterL1] = L_proportional + L_integral + L_derivative;
        motor[shooterL2] = L_proportional + L_integral + L_derivative;
        motor[shooterR1] = R_proportional + R_integral + R_derivative;
        motor[shooterR2] = R_proportional + R_integral + R_derivative;

        wait1Msec(20);
    }
else
{
        motor[shooterL1] = 0;
        motor[shooterL2] = 0;
        motor[shooterR1] = 0;
        motor[shooterR2] = 0;
}
}
}

task updateDrive()
{

    int X2 = 0;
    int Y1 = 0;
    int X1 = 0;

    while (true)
    {
        if (abs(vexRT[Ch3]) > threshold)
            Y1 = vexRT[Ch3];
        else
            Y1 = 0;

        if (abs(vexRT[Ch4]) > threshold)
            X1 = vexRT[Ch4];
        else
            X1 = 0;

        if (abs(vexRT[Ch1]) > threshold)
            X2 = vexRT[Ch1];
        else
            X2 = 0;

        //if only one (or no) joystick axis is greater than the threshold
        if (!((Y1 != 0 && X1 != 0) || (Y1 != 0 && X2 != 0) || (X1 != 0 && X2 != 0)))
        {
            motor[driveR] = Y1 + X1;
            motor[driveL] = -Y1 + X1;
        }
        wait1Msec(20);
    }
}

task updateIntake()
{
    while (true)
    {
        //forward
        if (vexRT[Btn6U] == 1)
            motor[intakeB] = 127;
        else if (vexRT[Btn6D] == 1)
            motor[intakeF] = 127;

        //reverse
        else if (vexRT[Btn5U] == 1)
            motor[intakeB] = -127;
        else if (vexRT[Btn5D] == 1)
            motor[intakeF] = -127;

        wait1Msec(20);
    }
}

task usercontrol()
{
    startTask(updateFlywheels);
    startTask(updateDrive);
    startTask(updateIntake);
}
