//////////////////////////////////////////////////////////////////////////
//////////////////            lts.cxx        /////////////////////////////
//////////////////////////////////////////////////////////////////////////
////////////////           PSOPT  Example             ////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////// Title: Linear tangent steering problem           ////////////////
//////// Last modified:         16 February 2009          ////////////////
//////// Reference:             Betts (2001)              ////////////////
//////// (See PSOPT handbook for full reference)          ////////////////
//////////////////////////////////////////////////////////////////////////
////////     Copyright (c) Victor M. Becerra, 2009        ////////////////
//////////////////////////////////////////////////////////////////////////
//////// This is part of the PSOPT software library, which ///////////////
//////// is distributed under the terms of the GNU Lesser ////////////////
//////// General Public License (LGPL)                    ////////////////
//////////////////////////////////////////////////////////////////////////

#include "psopt.h"
#include <GLFW/glfw3.h>
#include "include/Field.hpp"
#include <fstream>

#define RED_ZONE
#define USE_OPENGL

adouble path_circle(adouble x, adouble y, double center_x, double center_y, double radius)
{
    return (pow(x - center_x, 2.0) + pow(y - center_y, 2.0)) / (radius * radius);
}

adouble path_rectangle(adouble x, adouble y, double center_x, double center_y, double base, double height)
{
    return fabs((x - center_x) / base + (y - center_y) / height) + fabs((x - center_x) / base - (y - center_y) / height);
}

//////////////////////////////////////////////////////////////////////////
///////////////////  Define the end point (Mayer) cost function //////////
//////////////////////////////////////////////////////////////////////////

adouble endpoint_cost(adouble *initial_states, adouble *final_states, adouble *parameters, adouble &t0, adouble &tf,
                      adouble *xad, int iphase, Workspace *workspace)
{
    return tf;
}

//////////////////////////////////////////////////////////////////////////
///////////////////  Define the integrand (Lagrange) cost function  //////
//////////////////////////////////////////////////////////////////////////

adouble integrand_cost(adouble *states, adouble *controls, adouble *parameters, adouble &time, adouble *xad, int iphase,
                       Workspace *workspace)
{
    double w = 0.5;

    return w * (pow(controls[CINDEX(1)], 2) + pow(controls[CINDEX(2)], 2) + pow(controls[CINDEX(3)], 2) + pow(controls[CINDEX(4)], 2));
}

//////////////////////////////////////////////////////////////////////////
///////////////////  Define the DAE's ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

void dae(adouble *derivatives, adouble *path, adouble *states, adouble *controls, adouble *parameters, adouble &time,
         adouble *xad, int iphase, Workspace *workspace)
{

    adouble x1 = states[CINDEX(1)]; // world x
    adouble x2 = states[CINDEX(2)]; // world y
    adouble x3 = states[CINDEX(3)]; // theta
    adouble x4 = states[CINDEX(4)]; // d/dt phi_1
    adouble x5 = states[CINDEX(5)]; // d/dt phi_2
    adouble x6 = states[CINDEX(6)]; // d/dt phi_3
    adouble x7 = states[CINDEX(7)]; // d/dt phi_4

    adouble u1 = controls[CINDEX(1)]; // u_1, torque on wheel 1
    adouble u2 = controls[CINDEX(2)]; // u_2, torque on wheel 2
    adouble u3 = controls[CINDEX(3)]; // u_3, torque on wheel 3
    adouble u4 = controls[CINDEX(4)]; // u_4, torque on wheel 4

    double M = 25.0;
    double m = 0.5;
    double R = 0.636;
    double D = 0.450;
    double r = 0.050; // radius of the wheels
    double i = M * D * D / 6;

    derivatives[CINDEX(1)] = (-(x4 * sin(x3 + M_PI / 4)) - (x5 * cos(x3 + M_PI / 4)) + (x6 * sin(x3 + M_PI / 4)) + (x7 * cos(x3 + M_PI / 4))) * r / 2;
    derivatives[CINDEX(2)] = ((x4 * cos(x3 + M_PI / 4)) - (x5 * sin(x3 + M_PI / 4)) - (x6 * cos(x3 + M_PI / 4)) + (x7 * sin(x3 + M_PI / 4))) * r / 2;
    derivatives[CINDEX(3)] = (x4 + x5 + x6 + x7) * r / (4 * R);

    double a = (M * i + 4 * M * m * R * R + 3 * m * i + 8 * m * m * R * R) / (M + m);
    double b = -i;
    double c = (M * i + 4 * M * m * R * R - m * i) / (M + m);
    double d = 1 / (2 * m * r * r * (i + 2 * m * R * R));

    derivatives[CINDEX(4)] = (a * u1 + b * u2 + c * u3 + b * u4) * d;
    derivatives[CINDEX(5)] = (b * u1 + a * u2 + b * u3 + c * u4) * d;
    derivatives[CINDEX(6)] = (c * u1 + b * u2 + a * u3 + b * u4) * d;
    derivatives[CINDEX(7)] = (b * u1 + c * u2 + b * u3 + a * u4) * d;

    path[CINDEX(1)] = (x4 + x5 + x6 + x7) * r / (4 * R);       // d/dt theta
    path[CINDEX(2)] = (a * u1 + b * u2 + c * u3 + b * u4) * d; // d^2/dt^2 phi_1
    path[CINDEX(3)] = (b * u1 + a * u2 + b * u3 + c * u4) * d; // d^2/dt^2 phi_2
    path[CINDEX(4)] = (c * u1 + b * u2 + a * u3 + b * u4) * d; // d^2/dt^2 phi_3
    path[CINDEX(5)] = (b * u1 + c * u2 + b * u3 + a * u4) * d; // d^2/dt^2 phi_4

// ステージ上でロボットが通れない障害物
#ifdef RED_ZONE
    // obstacle
    path[CINDEX(6)] = path_circle(x1, x2, 4.125, 5.800, 0.107 + 0.700);
    path[CINDEX(7)] = path_circle(x1, x2, 4.125, 8.460, 0.107 + 0.900);
    path[CINDEX(7)] = path_circle(x1, x2, 2.610, 7.130, 0.107 + 0.700);
    path[CINDEX(8)] = path_circle(x1, x2, 2.610, 4.470, 0.107 + 0.800);
    path[CINDEX(9)] = path_circle(x1, x2, 4.125, 3.140, 0.107 + 0.700);

    // fense
    path[CINDEX(10)] = path_rectangle(x1, x2, 4.125, 1.57, 0.95, 4.50);
    path[CINDEX(11)] = path_rectangle(x1, x2, 4.125, 9.28, 0.95, 1.64);

    // kickball
    path[CINDEX(12)] = path_circle(x1, x2, 3.092, 2.469, 0.085 + 0.500);

#else
    // obstacle
    path[CINDEX(6)] = path_circle(x1, x2, 2.575, 3.140, 0.107 + 0.700);
    path[CINDEX(7)] = path_circle(x1, x2, 2.575, 5.800, 0.107 + 0.700);
    path[CINDEX(8)] = path_circle(x1, x2, 2.575, 8.460, 0.107 + 0.700);
    path[CINDEX(9)] = path_circle(x1, x2, 4.090, 7.130, 0.107 + 0.700);
    path[CINDEX(10)] = path_circle(x1, x2, 4.090, 4.470, 0.107 + 0.700);
    // path[CINDEX(11)] = path_circle(x1, x2, 3.170, 2.180, 0.085 + 0.600);

    // fense
    path[CINDEX(11)] = path_rectangle(x1, x2, 2.575, 1.57, 0.95, 4.50);
    path[CINDEX(12)] = path_rectangle(x1, x2, 2.575, 9.28, 0.95, 1.64);

#endif
}

////////////////////////////////////////////////////////////////////////////
///////////////////  Define the events function ////////////////////////////
////////////////////////////////////////////////////////////////////////////

void events(adouble *e, adouble *initial_states, adouble *final_states, adouble *parameters, adouble &t0, adouble &tf,
            adouble *xad, int iphase, Workspace *workspace)
{
    adouble x10 = initial_states[CINDEX(1)];
    adouble x20 = initial_states[CINDEX(2)];
    adouble x30 = initial_states[CINDEX(3)];
    adouble x40 = initial_states[CINDEX(4)];
    adouble x50 = initial_states[CINDEX(5)];
    adouble x60 = initial_states[CINDEX(6)];
    adouble x70 = initial_states[CINDEX(7)];
    adouble x1f = final_states[CINDEX(1)];
    adouble x2f = final_states[CINDEX(2)];
    adouble x3f = final_states[CINDEX(3)];
    adouble x4f = final_states[CINDEX(4)];
    adouble x5f = final_states[CINDEX(5)];
    adouble x6f = final_states[CINDEX(6)];
    adouble x7f = final_states[CINDEX(7)];

    e[CINDEX(1)] = x10;
    e[CINDEX(2)] = x20;
    e[CINDEX(3)] = x30;
    e[CINDEX(4)] = x40;
    e[CINDEX(5)] = x50;
    e[CINDEX(6)] = x60;
    e[CINDEX(7)] = x70;
    e[CINDEX(8)] = x1f;
    e[CINDEX(9)] = x2f;
    e[CINDEX(10)] = x3f;
    e[CINDEX(11)] = x4f;
    e[CINDEX(12)] = x5f;
    e[CINDEX(13)] = x6f;
    e[CINDEX(14)] = x7f;
}

///////////////////////////////////////////////////////////////////////////
///////////////////  Define the phase linkages function ///////////////////
///////////////////////////////////////////////////////////////////////////

void linkages(adouble *linkages, adouble *xad, Workspace *workspace)
{
    // No linkages as this is a single phase problem
}

////////////////////////////////////////////////////////////////////////////
///////////////////  Define the main routine ///////////////////////////////
////////////////////////////////////////////////////////////////////////////

int main(void)
{

    ////////////////////////////////////////////////////////////////////////////
    ///////////////////  Declare key structures ////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    Alg algorithm;
    Sol solution;
    Prob problem;

    ////////////////////////////////////////////////////////////////////////////
    ///////////////////  Register problem name  ////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    problem.name = "Time-Optimal Trajectory Generation Problem";

    problem.outfilename = "traj.txt";

    ////////////////////////////////////////////////////////////////////////////
    ////////////  Define problem level constants & do level 1 setup ////////////
    ////////////////////////////////////////////////////////////////////////////

    problem.nphases = 1;
    problem.nlinkages = 0;

    psopt_level1_setup(problem);

    /////////////////////////////////////////////////////////////////////////////
    /////////   Define phase related information & do level 2 setup /////////////
    /////////////////////////////////////////////////////////////////////////////

    problem.phases(1).nstates = 7;
    problem.phases(1).ncontrols = 4;
    problem.phases(1).nevents = 14;
    problem.phases(1).npath = 11;
    problem.phases(1).nodes = "[50]";

    psopt_level2_setup(problem, algorithm);

    ////////////////////////////////////////////////////////////////////////////
    ///////////////////  Declare DMatrix objects to store results //////////////
    ////////////////////////////////////////////////////////////////////////////

    DMatrix x, u, t;
    DMatrix H;

    ////////////////////////////////////////////////////////////////////////////
    ///////////////////  Enter problem bounds information //////////////////////
    ////////////////////////////////////////////////////////////////////////////

    /*
                RED                     BLUE
                x     y     theta       x     y     theta
        trsz    0.550 9.550             6.150 9.550
        rz      1.000 2.000 M_PI/4      5.700 2.000
        ts1     5.770 3.140 M_PI/4      1.000 3.140
        ts2     5.770 4.470 M_PI/4      1.000 4.470
        ts3     5.770 5.800 M_PI/4      1.000 5.800
        ts4     5.770 7.130 M_PI/4      1.000 7.130
        ts5     5.770 8.460 M_PI/4      1.000 8.460
        kz      2.712 2.189 -2.76
        wp      4.650 4.200 -2.76
        rz2     1.000 3.805
        rz3     1.000 5.135

        prsz    4.650 0.550             prsz    2.050 0.550
        pp1     5.702 1.340             pp1     2.050 0.550
        pp2     5.702 1.070             pp2     2.050 0.550
        pp3     5.702 0.800             pp3     2.050 0.550
        pp4     5.702 0.530             pp4     2.050 0.550
        pp5     5.754 0.462 -2*M_PI/15  pp5     2.050 0.550
    
        kz      2.695 2.145 -2.768
                2.706 2.156 -2.740
        kz2     2.808 1.850 -2.722
        kz3     3.043 1.631 -2.689
    */

    double first_x      = 1.000;
    double first_y      = 2.000;
    double first_theta  = M_PI/4;
    double final_x      = 5.770;
    double final_y      = 3.140;
    double final_theta  = M_PI/4;

#ifdef RED_ZONE
    problem.phases(1).bounds.lower.states(1) = 0.55;
    problem.phases(1).bounds.lower.states(2) = 0.55;
    problem.phases(1).bounds.upper.states(1) = 5.80;
    problem.phases(1).bounds.upper.states(2) = 9.55;
#else
    problem.phases(1).bounds.lower.states(1) = 0.95;
    problem.phases(1).bounds.lower.states(2) = 0.55;
    problem.phases(1).bounds.upper.states(1) = 6.20;
    problem.phases(1).bounds.upper.states(2) = 9.55;
#endif

    problem.phases(1).bounds.lower.states(3) = -2 * M_PI;
    problem.phases(1).bounds.lower.states(4) = -10 * M_PI;
    problem.phases(1).bounds.lower.states(5) = -10 * M_PI;
    problem.phases(1).bounds.lower.states(6) = -10 * M_PI;
    problem.phases(1).bounds.lower.states(7) = -10 * M_PI;

    
    problem.phases(1).bounds.upper.states(3) = 2 * M_PI;
    problem.phases(1).bounds.upper.states(4) = 10 * M_PI;
    problem.phases(1).bounds.upper.states(5) = 10 * M_PI;
    problem.phases(1).bounds.upper.states(6) = 10 * M_PI;
    problem.phases(1).bounds.upper.states(7) = 10 * M_PI;

    problem.phases(1).bounds.lower.controls(1) = -1.0;
    problem.phases(1).bounds.lower.controls(2) = -1.0;
    problem.phases(1).bounds.lower.controls(3) = -1.0;
    problem.phases(1).bounds.lower.controls(4) = -1.0;

    problem.phases(1).bounds.upper.controls(1) = 1.0;
    problem.phases(1).bounds.upper.controls(2) = 1.0;
    problem.phases(1).bounds.upper.controls(3) = 1.0;
    problem.phases(1).bounds.upper.controls(4) = 1.0;

    problem.phases(1).bounds.lower.events(1) = first_x;
    problem.phases(1).bounds.lower.events(2) = first_y;
    problem.phases(1).bounds.lower.events(3) = first_theta;
    problem.phases(1).bounds.lower.events(4) = 0.0;
    problem.phases(1).bounds.lower.events(5) = 0.0;
    problem.phases(1).bounds.lower.events(6) = 0.0;
    problem.phases(1).bounds.lower.events(7) = 0.0;
    problem.phases(1).bounds.lower.events(8) = final_x; // final x
    problem.phases(1).bounds.lower.events(9) = final_y; // final y
    problem.phases(1).bounds.lower.events(10) = final_theta; // final theta
    problem.phases(1).bounds.lower.events(11) = 0.0;  // final d/dt phi_1
    problem.phases(1).bounds.lower.events(12) = 0.0;  // final d/dt phi_2
    problem.phases(1).bounds.lower.events(13) = 0.0;  // final d/dt phi_3
    problem.phases(1).bounds.lower.events(14) = 0.0;  // final d/dt phi_4

    problem.phases(1).bounds.upper.events = problem.phases(1).bounds.lower.events;

    problem.phases(1).bounds.lower.path(1) = -M_PI / 4;
    problem.phases(1).bounds.upper.path(1) = M_PI / 4;
    problem.phases(1).bounds.lower.path(2) = -100.0;
    problem.phases(1).bounds.upper.path(2) = 100.0;
    problem.phases(1).bounds.lower.path(3) = -100.0;
    problem.phases(1).bounds.upper.path(3) = 100.0;
    problem.phases(1).bounds.lower.path(4) = -100.0;
    problem.phases(1).bounds.upper.path(4) = 100.0;
    problem.phases(1).bounds.lower.path(5) = -100.0;
    problem.phases(1).bounds.upper.path(5) = 100.0;

    problem.phases(1).bounds.lower.path(6) = 1.0;
    problem.phases(1).bounds.upper.path(6) = 1000.0;
    problem.phases(1).bounds.lower.path(7) = 1.0;
    problem.phases(1).bounds.upper.path(7) = 1000.0;
    problem.phases(1).bounds.lower.path(8) = 1.0;
    problem.phases(1).bounds.upper.path(8) = 1000.0;
    problem.phases(1).bounds.lower.path(9) = 1.0;
    problem.phases(1).bounds.upper.path(9) = 1000.0;
    problem.phases(1).bounds.lower.path(10) = 1.0;
    problem.phases(1).bounds.upper.path(10) = 1000.0;
    problem.phases(1).bounds.lower.path(11) = 1.0;
    problem.phases(1).bounds.upper.path(11) = 1000.0;
    problem.phases(1).bounds.lower.path(12) = 1.0;
    problem.phases(1).bounds.upper.path(12) = 1000.0;
    // problem.phases(1).bounds.lower.path(13) = 1.0;
    // problem.phases(1).bounds.upper.path(13) = 1000.0;
    // problem.phases(1).bounds.lower.path(14) = 1.0;
    // problem.phases(1).bounds.upper.path(14) = 1000.0;
    // problem.phases(1).bounds.lower.path(15) = 1.0;
    // problem.phases(1).bounds.upper.path(15) = 1000.0;
    // problem.phases(1).bounds.lower.path(16) = 1.0;
    // problem.phases(1).bounds.upper.path(16) = 1000.0;

    problem.phases(1).bounds.lower.StartTime = 0.0;
    problem.phases(1).bounds.upper.StartTime = 0.0;

    problem.phases(1).bounds.lower.EndTime = 0.0;
    problem.phases(1).bounds.upper.EndTime = 100.0;

    ////////////////////////////////////////////////////////////////////////////
    ///////////////////  Register problem functions  ///////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    problem.integrand_cost = &integrand_cost;
    problem.endpoint_cost = &endpoint_cost;
    problem.dae = &dae;
    problem.events = &events;
    problem.linkages = &linkages;

    ////////////////////////////////////////////////////////////////////////////
    ///////////////////  Define & register initial guess ///////////////////////
    ////////////////////////////////////////////////////////////////////////////

    int nnodes = (int)problem.phases(1).nodes(1);

    DMatrix x0(7, nnodes);
    x0(1, colon()) = linspace(first_x, final_x, nnodes); // x
    x0(2, colon()) = linspace(first_y, final_y, nnodes); // y
    x0(3, colon()) = linspace(first_theta, final_theta, nnodes); // theta
    x0(4, colon()) = linspace(0.0, 0.0, nnodes); // d/dt phi_1
    x0(5, colon()) = linspace(0.0, 0.0, nnodes); // d/dt phi_2
    x0(6, colon()) = linspace(0.0, 0.0, nnodes); // d/dt phi_3
    x0(7, colon()) = linspace(0.0, 0.0, nnodes); // d/dt phi_4

    DMatrix u0(4, nnodes);
    u0(1, colon()) = linspace(0.0, 0.0, nnodes);
    u0(2, colon()) = linspace(0.0, 0.0, nnodes);
    u0(3, colon()) = linspace(0.0, 0.0, nnodes);
    u0(4, colon()) = linspace(0.0, 0.0, nnodes);

    problem.phases(1).guess.controls = u0;
    problem.phases(1).guess.states = x0;
    problem.phases(1).guess.time = linspace(0.0, 100.0, nnodes);

    ////////////////////////////////////////////////////////////////////////////
    ///////////////////  Enter algorithm options  //////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    algorithm.nlp_method = "IPOPT";
    algorithm.scaling = "automatic";
    algorithm.derivatives = "automatic";
    algorithm.nlp_iter_max = 5000;
    algorithm.nlp_tolerance = 1.e-6;

    ////////////////////////////////////////////////////////////////////////////
    ///////////////////  Now call PSOPT to solve the problem   /////////////////
    ////////////////////////////////////////////////////////////////////////////

    psopt(solution, problem, algorithm);

    if (solution.error_flag)
        exit(0);

    ////////////////////////////////////////////////////////////////////////////
    ///////////  Extract relevant variables from solution structure   //////////
    ////////////////////////////////////////////////////////////////////////////

    x = solution.get_states_in_phase(1);
    u = solution.get_controls_in_phase(1);
    t = solution.get_time_in_phase(1);

    // liner interpolation
    double dt = 0.001;
    vector<double> a(nnodes - 1, 0.0);
    vector<double> b(nnodes - 1, 0.0);
    vector<vector<double>> fine_path;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < nnodes - 1; j++)
        {
            a[j] = (x.elem(i + 1, j + 2) - x.elem(i + 1, j + 1)) / (t.elem(1, j + 2) - t.elem(1, j + 1));
            b[j] = x.elem(i + 1, j + 1);
        }

        fine_path.emplace_back();
        fine_path[i].push_back(x.elem(i + 1, 1));

        for (double t_ = dt; t_ < t.elem(1, nnodes); t_ += dt)
        {
            for (int j = 1; j < nnodes - 1; j++)
            {
                if (t_ < t.elem(1, j + 1))
                {
                    fine_path[i].push_back(a[j] * (t_ - t.elem(1, j)) + b[j]);
                    break;
                }
            }
        }
        fine_path[i].push_back(x.elem(i + 1, nnodes));
    }

    for (int i = 0; i < fine_path[2].size(); i++)
    {
        if(fine_path[2][i] > M_PI){
            fine_path[2][i] -= 2 * M_PI;
        }else if(fine_path[2][i] < -M_PI){
            fine_path[2][i] += 2 * M_PI;
        }
    }

    cout << "" << endl;
    cout << "path has " << fine_path[0].size() << " poses" << endl;

    ofstream ofs("path.csv");
    for (unsigned int i = 0; i < fine_path[0].size(); i++)
    {
        for (unsigned int j = 0; j < fine_path.size(); j++)
        {
            ofs << fine_path[j][i];
            ofs << ",";
        }
        ofs << '\n';
    }
    ofs.close();

    ////////////////////////////////////////////////////////////////////////////
    ///////////  Save solution data to files if desired ////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    x.Save("x.dat");
    u.Save("u.dat");
    t.Save("t.dat");

    ////////////////////////////////////////////////////////////////////////////
    ///////////  Plot some results if desired (requires gnuplot) ///////////////
    ////////////////////////////////////////////////////////////////////////////
#ifndef USE_OPENGL
    DMatrix pos_x = x(1, colon());
    DMatrix pos_y = x(2, colon());
    DMatrix pos = x(colon(1, 3), colon());
    DMatrix vel = x(colon(4, 7), colon());

    plot(pos_x, pos_y, problem.name + ": x-y trajectory", "x", "y", "pos");

    plot(t, pos, problem.name + ": pose", "time (s)", "pose[m, rad]", "x1 x2 x3");

    plot(t, vel, problem.name + ": wheel velocity", "time (s)", "angular velocity[rad/s]", "x4 x5 x6 x7");

    plot(t, u, problem.name + ": control", "time (s)", "control", "u1 u2 u3 u4");

    plot(pos_x, pos_y, problem.name + ": x-y trajectory", "x", "y", "pos", "pdf", "xy.pdf");

    plot(t, pos, problem.name + ": pose", "time (s)", "pose[m, rad]", "x1 x2 x3", "pdf", "status_pos.pdf");

    plot(t, vel, problem.name + ": wheel velocity", "time (s)", "angular velocity[rad/s]", "x4 x5 x6 x7", "pdf", "states_wheelVel.pdf");

    plot(t, u, problem.name + ": control", "time (s)", "control", "u1 u2 u3 u4", "pdf", "control.pdf");

#else 
    ////////////////////////////////////////////////////////////////////////////////
    // GLFW の初期化 (GLFW)
    if (glfwInit() == GL_FALSE)
    {
        // 初期化に失敗した処理
        std::cerr << "Can't initialize GLFW" << std::endl;
        return 1;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // 終了時の処理登録 (GLFW)
    atexit(glfwTerminate);

    ////////////////////////////////////////////////////////////////////////////////
    // ウィンドウを作成 (GLFW)
    GLFWwindow *const window(glfwCreateWindow(/* int           width   = */ 670,
                                              /* int           height  = */ 1010,
                                              /* const char  * title   = */ "ぱすぷらくん",
                                              /* GLFWmonitor * monitor = */ NULL,
                                              /* GLFWwindow  * share   = */ NULL));
    if (window == NULL)
    {
        // ウィンドウ作成に失敗した処理
        std::cerr << "Can't create GLFW window." << std::endl;
        return 1;
    }

    // 作成したウィンドウを処理対象とする (GLFW)
    glfwMakeContextCurrent(/* GLFWwindow *  window = */ window);

    // 描画範囲の指定
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0f, 670.0f, 0.0f, 1010.0f, -1.0f, 1.0f);

    ////////////////////////////////////////////////////////////////////////////////

    // ウィンドウが開いている間繰り返す
    while (glfwWindowShouldClose(/* GLFWwindow * window = */ window) == GL_FALSE)
    {
        // ループ処理
        // 画面の初期化
        glClearColor(0.2f, 0.2f, 0.2f, 0.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // 描画処理
#ifdef RED_ZONE
        drawRedField();
#else
        drawBlueField();
#endif

        // ロボットの描画
        for (unsigned int i = 1; i < fine_path[0].size(); i += 150)
        {
            float robot_width = 70; // [cm]
            float x0 = (float)fine_path[0][i - 1] * 100;
            float y0 = (float)fine_path[1][i - 1] * 100;
            float theta0 = (float)fine_path[2][i - 1];
            float x1 = x0 + cos(theta0) * robot_width / 2;
            float y1 = y0 + sin(theta0) * robot_width / 2;
            float x2 = x0 + cos(theta0) * (robot_width / 2)         - sin(theta0) * (robot_width / 2 - 10);
            float y2 = y0 + sin(theta0) * (robot_width / 2)         + cos(theta0) * (robot_width / 2 - 10);
            float x3 = x0 + cos(theta0) * (robot_width / 2 - 10)    - sin(theta0) * (robot_width / 2);
            float y3 = y0 + sin(theta0) * (robot_width / 2 - 10)    + cos(theta0) * (robot_width / 2);
            float x4 = x0 + cos(theta0) * (-robot_width / 2 + 10)   - sin(theta0) * (robot_width / 2);
            float y4 = y0 + sin(theta0) * (-robot_width / 2 + 10)   + cos(theta0) * (robot_width / 2);
            float x5 = x0 + cos(theta0) * (-robot_width / 2)        - sin(theta0) * (robot_width / 2 - 10);
            float y5 = y0 + sin(theta0) * (-robot_width / 2)        + cos(theta0) * (robot_width / 2 - 10);
            float x6 = x0 + cos(theta0) * (-robot_width / 2)        - sin(theta0) * (-robot_width / 2 + 10);
            float y6 = y0 + sin(theta0) * (-robot_width / 2)        + cos(theta0) * (-robot_width / 2 + 10);
            float x7 = x0 + cos(theta0) * (-robot_width / 2 + 10)   - sin(theta0) * (-robot_width / 2);
            float y7 = y0 + sin(theta0) * (-robot_width / 2 + 10)   + cos(theta0) * (-robot_width / 2);
            float x8 = x0 + cos(theta0) * (robot_width / 2 - 10)    - sin(theta0) * (-robot_width / 2);
            float y8 = y0 + sin(theta0) * (robot_width / 2 - 10)    + cos(theta0) * (-robot_width / 2);
            float x9 = x0 + cos(theta0) * (robot_width / 2)         - sin(theta0) * (-robot_width / 2 + 10);
            float y9 = y0 + sin(theta0) * (robot_width / 2)         + cos(theta0) * (-robot_width / 2 + 10);

            GLfloat vtx[] = {
                x0,
                y0,
                x1,
                y1,
                x2,
                y2,
                x3,
                y3,
                x4,
                y4,
                x5,
                y5,
                x6,
                y6,
                x7,
                y7,
                x8,
                y8,
                x9,
                y9,
                x1,
                y1,
            };

            glVertexPointer(2, GL_FLOAT, 0, vtx);
            glLineWidth(1.0f);
            glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

            glEnableClientState(GL_VERTEX_ARRAY);
            glDrawArrays(GL_LINE_LOOP, 0, 11);
            glDisableClientState(GL_VERTEX_ARRAY);
        }

        // カラーバッファ入れ替え <= ダブルバッファリング (GLFW)
        glfwSwapBuffers(window);

        // イベント待ち (GLFW)
        glfwWaitEvents();
    }
#endif
}

////////////////////////////////////////////////////////////////////////////
///////////////////////      END OF FILE     ///////////////////////////////
////////////////////////////////////////////////////////////////////////////
