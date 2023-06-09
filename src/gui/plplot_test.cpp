// Simple demo of a 2D line plot.
//
// Copyright (C) 2011-2014 Alan W. Irwin
// Copyright (C) 2012  Andrew Ross
//
// This file is part of PLplot.
//
// PLplot is free software; you can redistribute it and/or modify
// it under the terms of the GNU Library General Public License as published
// by the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// PLplot is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Library General Public License for more details.
//
// You should have received a copy of the GNU Library General Public License
// along with PLplot; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
//
//

#include "plc++demos.h"

#include "message/common_message.h"
#include "common/suspend.h"
#include "math/geometry.h"
#include "math/BezierGenerator.h"
#include "control/ControlSimulator.h"
#include "control/MobileRobotController.h"

#include <vector>



namespace control{
    struct ControlBase{


    };

    struct DoubleControlBaseSim{
        common::Time time;
       
        struct SteerStatus{
            float vel = 0.0;
            float angle = 0.0;
        };
        struct SteerWheelConfig{
            float pose_x = 0.0;
            float pose_y = 0.0;
            float vel_acc = 0.5; //m/s2
            float rot_acc = 0.5; // ras/s2
            SteerStatus actual;

        };
        struct Pose{
            float x = 0.0;
            float y = 0.0;
            float yaw = 0.0;
        };
        std::array<SteerWheelConfig,2> steer_wheel_config;
        
        Pose current_pose;

        void init(){
            steer_wheel_config[0].pose_x = 1.2;
            steer_wheel_config[0].pose_y = 0.5;

            steer_wheel_config[1].pose_x = - 1.2;
            steer_wheel_config[1].pose_y = - 0.5;


            time = common::FromUnixNow();
        }
        
        void getRotateCenter(){};
        
        
        void steer(const SteerStatus& cmd_1, const SteerStatus& cmd_2){

            common::Time now = common::FromUnixNow();
            
            float all_us = common::ToMicroSeconds(now - time);
            float all_s =  all_us * 1e-6;
            int N_step_us = 100;
            int N = all_us/N_step_us;
            float N_step_s = N_step_us * 1e-6;
            float step_us = 0.0;
            for(int i = 0 ; i < N;i++){
                
            }

            
        }

    };
}




class Plot {
public:
    Plot( int, char ** );
    void Arrow(PLFLT x, PLFLT y, PLFLT yaw,PLFLT len);
    ~Plot();
private:
    // Class data
    plstream         *pls = nullptr;

    static const int NSIZE;
};

const int Plot::NSIZE = 101;

Plot::~Plot() {
    if(pls){
        delete pls;
    }
}
void
pltrw( PLFLT x, PLFLT y, PLFLT *tx, PLFLT *ty, PLPointer pltr_data )
{


    PLINT   ul, ur, vl, vr;
    PLFLT   du, dv;
    PLFLT   xl, xr, yl, yr;

    PLcGrid2 *grid = (PLcGrid2 *) pltr_data;
    PLFLT_NC_MATRIX&   xg   = grid->xg;
    PLFLT_NC_MATRIX&   yg   = grid->yg;
    PLINT   nx    = grid->nx;
    PLINT   ny    = grid->ny;

    ul = (PLINT) x;
    ur = ul + 1;
    du = x - ul;

    vl = (PLINT) y;
    vr = vl + 1;
    dv = y - vl;

    xl = xg[ul][vl];
    yl = yg[ul][vl];
    std::cout << "ul: " << ul << ", vl: " << vl << std::endl;

    *tx = vl;
    *ty = 5;
    return;

    std::cout << "xl: " << xl << ", yl: " << yl << std::endl;

    std::cout << "1 *tx: " << *tx << ", *ty: " << *ty << std::endl;

// Look up coordinates in row-dominant array.
// Have to handle right boundary specially -- if at the edge, we'd better
// not reference the out of bounds point.
//



    *tx += xl;
    *ty += yl;
    std::cout << "2 *tx: " << *tx << ", *ty: " << *ty << std::endl;

}

void Plot::Arrow(PLFLT x, PLFLT y, PLFLT yaw,PLFLT len) {
    PLFLT x1 = x + len*cos(yaw), y1 = y + len* sin(yaw) , len2 = 0.2*len;
    PLFLT yaw2 = yaw + 1.7*M_PI_2, yaw3 = yaw - 1.7*M_PI_2;
    PLFLT x2 = x1 + len2*cos(yaw2), y2 = y1 + len2* sin(yaw2);
    PLFLT x3 = x1 + len2*cos(yaw3), y3 = y1 + len2* sin(yaw3);

    pls->join(x,y,x1,y1);


    pls->join(x1,y1,x2,y2);
    pls->join(x1,y1,x3,y3);


}
Plot::Plot( int argc, char **argv )
{
    PLFLT x[NSIZE], y[NSIZE];
    PLFLT xmin = -1.5, xmax = 1.5, ymin = -1.5, ymax = 1.5;
    const char      *legline[4];


    PLINT           colbox, collab, colline[4], styline[4];
    PLINT           id1;


    //pen
    colbox     = 1;
    collab     = 3;
    styline[0] = colline[0] = 2;      // pens color and line style
    styline[1] = colline[1] = 3;
    styline[2] = colline[2] = 4;
    styline[3] = colline[3] = 5;

    legline[0] = "sum";                       // pens legend
    legline[1] = "sin";
    legline[2] = "sin*noi";
    legline[3] = "sin+noi";


    // Prepare data to be plotted.
    for ( int i = 0; i < NSIZE; i++ )
    {
        x[i] = (PLFLT) ( i ) / (PLFLT) ( NSIZE - 1 );
        y[i] = ymax * x[i] * x[i];
    }

    pls = new plstream();

    // Parse and process command line arguments
    pls->parseopts( &argc, argv, PL_PARSE_FULL );

    std::cout << "init" << std::endl;
    // Initialize plplot
    pls->sdev("qtwidget");

    pls->init();
    std::cout << "init done" << std::endl;

    PLINT r0, g0, b0;
    PLFLT a0;
    PLINT r, g, b;
    r0 = 255;
    g0 = 255;
    b0 = 255;
    // they work without any obvious error messages.

    // Create a labelled box to hold the plot.
    pls->env( xmin, xmax, ymin, ymax, 1, 1 );
    pls->scol0( 0, 255, 255, 255 );
    pls->scol0( 1, 0, 0, 255 );
    pls->scol0( 2, 0, 255, 0 );

    pls->scolor( 2);

    pls->lab( "x", "y=100 x#u2#d", "Simple PLplot demo of a 2D line plot" );



    pls->scolbg( r0, g0, b0 );
    pls->gcolbg( r, g, b );
    std::cout << "background colour parameters: r, g, b = " << r << " " << g << " " << b << std::endl;

    {

        {
            float x1 = 0.0 ;
            float y1 = 0.5;  //Line 1 start
            float x2 = 1.0;
            float y2 = 1.6;  //Line 1 end
            float x3 = 0.0;
            float y3 = 1.0; //Line 2 start
            float x4 = 1.0;
            float y4 = 1.0;

            float xc, yc;
            bool ok = math::LineLineIntersect(x1,y1,x2,y2,x3,y3,x4,y4,xc,yc);
            std::cout << "1 check LineLineIntersect : " << ok << std::endl;
            pls->join(x1,y1,x2,y2);
            pls->join(x3,y3,x4,y4);
            if(ok){
                pls->join(0.0,0.0,xc,yc);
            }

        }
        {
            float x1 = -0.5 ;
            float y1 = 0.5;  //Line 1 start
            float x2 = -1.0;
            float y2 = 1.6;  //Line 1 end
            float x3 = x1;
            float y3 = y1; //Line 2 start
            float x4 = x2;
            float y4 = y2;

            float xc, yc;
            bool ok = math::LineLineIntersect(x1,y1,x2,y2,x3,y3,x4,y4,xc,yc);
            std::cout << "2 check LineLineIntersect : " << ok << std::endl;

            pls->join(x1,y1,x2,y2);
            pls->join(x3,y3,x4,y4);
            if(ok){
                pls->join(0.0,0.0,xc,yc);
            }

        }


        common::Suspend s;
        int i = 0;

        {

            control::DoubleSteerController controller;
            control::SteerWheelBase wheel_1;
            wheel_1.enable_rot = true;
            wheel_1.mount_position_x = -0.4075;
            wheel_1.mount_position_y = -0.1675;
            wheel_1.mount_position_yaw = 0.0;

            control::SteerWheelBase wheel_2;
            wheel_2.enable_rot = true;
            wheel_2.mount_position_x = 0.4075;
            wheel_2.mount_position_y = 0.1675;
            wheel_2.mount_position_yaw = 0.0;

            controller.set_wheel(wheel_1,wheel_2);

            controller.cmd_vel(0.8, 0.5);
            std::cout << "get cmd_vel " << controller.m_steer_wheel_1.target_rot_angle << ", "
            << controller.m_steer_wheel_1.target_forward_vel << ", "
            << controller.m_steer_wheel_2.target_rot_angle << ", "
            << controller.m_steer_wheel_2.target_forward_vel<< std::endl;

            float axis_range = 2.5;
            pls->env( -axis_range, axis_range, -axis_range, axis_range, 1, 0 );

            pls->lab  ( "x", "y", "Robot" );

            control::DoubleSteerBase robot;


            robot.steer_1.position.set(wheel_1.mount_position_x ,wheel_1.mount_position_y ,wheel_1.mount_position_yaw );
            robot.steer_2.position.set(wheel_2.mount_position_x ,wheel_2.mount_position_y ,wheel_2.mount_position_yaw );

            robot.update(controller.m_steer_wheel_1.target_forward_vel,controller.m_steer_wheel_1.target_rot_angle ,
                         controller.m_steer_wheel_2.target_forward_vel,controller.m_steer_wheel_2.target_rot_angle );
            Arrow(robot.position.x(),robot.position.y(),robot.position.yaw(),1.0);


            std::cout << "robot .steer_1.position: " << robot .steer_1.position << "\n"
            <<"robot .steer_2.position: " << robot .steer_2.position << "\n";
            Arrow(robot .steer_1.position.x(),robot.steer_1.position.y(),robot.steer_1.position.yaw() + (robot.steer_1.actual_forward_vel > 0.0 ? 0.0 : M_PI )   ,0.2);
            Arrow(robot .steer_2.position.x(),robot.steer_2.position.y(),robot.steer_2.position.yaw()+ (robot.steer_2.actual_forward_vel > 0.0 ? 0.0 : M_PI ),0.2);
            Arrow(robot .steer_1.position.x(),robot.steer_1.position.y(),robot.steer_1.position.yaw() -M_PI_2,10.2);
            Arrow(robot .steer_2.position.x(),robot.steer_2.position.y(),robot.steer_2.position.yaw() -M_PI_2,10.2);
            Arrow(robot .steer_1.position.x(),robot.steer_1.position.y(),robot.steer_1.position.yaw() + M_PI_2,10.2);
            Arrow(robot .steer_2.position.x(),robot.steer_2.position.y(),robot.steer_2.position.yaw() + M_PI_2,10.2);

            if(robot.is_rotate){

                pls->join(robot.position.x(),robot.position.y(),robot.rotate_center_x,robot.rotate_center_y);
//                pls->join(robot.steer_1.position.x(),robot.steer_1.position.y(),robot.rotate_center_x,robot.rotate_center_y);
//                pls->join(robot.steer_1.position.x(),robot.steer_1.position.y(),robot.rotate_center_x,robot.rotate_center_y);

            }


        }

//        pls->bop();

        std::cout << __LINE__ <<  " wait" << std::endl;


//        std::cout << __LINE__ <<  " wait" << std::endl;
//        pls->adv(0);
        std::cout << __LINE__ <<  " wait" << std::endl;

        pls->env( xmin, xmax, ymin, ymax, 1, 1 );
        std::cout << __LINE__ <<  " wait" << std::endl;

        pls->lab  ( "x", "y", "Arrow" );

        std::cout << __LINE__ <<  " wait" << std::endl;

        pls->width(0.4);
        static PLINT mark  = 50;
        static PLINT space = 500;
        pls->styl( 1, &mark, &space );
        while ( i < 100){
            pls->width(i%3);

            Arrow(0.0,0.0,0.0628*i,1);
            pls->flush();

//            pls->replot();
            s.sleep(1);
            i++;
        }

//        pls->eop();

    }

    // Plot the data that was prepared above.
//    pls->line( NSIZE, x, y );

    PLINT narr;
    bool  fill;

    {
//        pls->bop();


        pls->env( xmin, xmax, ymin, ymax, 1, 1 );
        pls->lab  ( "x", "y", "Bezier" );

        float PA[2] = {-1.2,0.5};
        float PB[2] = {-1.0,0.0};
        float PC[2] = {-0.8,0.0};
        float PD[2] = {0.0,0.0};
        float PE[3] = {0.0,0.0,0.0};

        float t_step = 0.01;
        size_t t_num = 1.0/t_step;
        t_step = 1.0/t_num;


        std::vector<PLFLT> Bezier_x(t_num);
        std::vector<PLFLT> Bezier_y(t_num);

        std::vector<std::array<float,2>> path;
        math::buildBezier(PA,PB,PC,PD,0.01,path);
        for(size_t i = 0 ; i < path.size() ; i++){


            Bezier_x[i] = path[i][0];
            Bezier_y[i] = path[i][1];

        }
        std::cout << "t_num " << t_num << std::endl;

        float arc_len = 0.0;
//        arc_len += sqrt((Bezier_x[1] - Bezier_x[0])*(Bezier_x[1] - Bezier_x[0]) + (Bezier_y[1] - Bezier_y[0])*(Bezier_y[1] - Bezier_y[0]) );
        for(size_t i = 1 ; i < t_num ; i++){
            arc_len += sqrt((Bezier_x[i] - Bezier_x[i-1])*(Bezier_x[i] - Bezier_x[i-1]) + (Bezier_y[i] - Bezier_y[i-1])*(Bezier_y[i] - Bezier_y[i-1]) );
        }

        char buffer[100];
        sprintf(buffer,"arc_len=%.3f",arc_len);
//        pls->wind( 0.0, 1.0, 0.0, 1.0 );
        pls->sfci( 0 );
        pls->schr( 0, 0.5 );
        pls->sfont(0,0,1);
        pls->font( 1 );
        pls->ptex(0,1,1,0,0.1,buffer);

//        pls->mtex( "t", 1.5, 0.5, 0.5, "PLplot Example 6 - plpoin symbols (compact)" );
        pls->lsty(2);

        pls->line( t_num, Bezier_x.data(), Bezier_y.data() );
//        pls->eop();

    }
    if(0){
        pls->col0( 1 );

        pls->join(0.1,0.1,-2.5,-2.5);
        pls->col0( 1 );

        Arrow(-4.2,4.3,1.4,1.5);
        pls->col0( 2 );

        for(int i = 0 ; i< 10;i++){
            pls->join(0.0,0.0,4.0,i - 5);

        }


        for(int j = 0 ; j < 0; j++){

            for ( int i = 0; i < NSIZE; i++ )
            {
                x[i] = (PLFLT) ( i ) / (PLFLT) ( NSIZE - 1 );
                y[i] = ymax * x[i] * x[i] + j * 1;
            }
            pls->line( NSIZE, x, y );

            if(j == 5){
                pls->clear();
            }
        }

        pls->scolbg( r0, g0, b0 );
        pls->gcolbg( r, g, b );
        std::cout << "background colour parameters: r, g, b = " << r << " " << g << " " << b << std::endl;

    }

    // In C++ we don't call plend() to close PLplot library
    // this is handled by the destructor
    delete pls;
    pls = nullptr;
}

int main( int argc, char ** argv )
{
    Plot x ( argc, argv );
}


//--------------------------------------------------------------------------
//                              End of Plot.cc
//--------------------------------------------------------------------------
