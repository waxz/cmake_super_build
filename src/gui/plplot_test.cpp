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

#include <vector>

//https://gist.github.com/TimSC/47203a0f5f15293d2099507ba5da44e6#file-linelineintersect-cpp
/** Calculate determinant of matrix:
	[a b]
	[c d]
*/
inline float Det(float a, float b, float c, float d)
{
    return a*d - b*c;
}
///Calculate intersection of two lines.
///\return true if found, false if not found or error
bool LineLineIntersect(float x1, float y1, //Line 1 start
                       float x2, float y2, //Line 1 end
                       float x3, float y3, //Line 2 start
                       float x4, float y4, //Line 2 end
                       float &ixOut, float &iyOut) //Output
{
    //http://mathworld.wolfram.com/Line-LineIntersection.html

    float detL1 = Det(x1, y1, x2, y2);
    float detL2 = Det(x3, y3, x4, y4);
    float x1mx2 = x1 - x2;
    float x3mx4 = x3 - x4;
    float y1my2 = y1 - y2;
    float y3my4 = y3 - y4;

    float xnom = Det(detL1, x1mx2, detL2, x3mx4);
    float ynom = Det(detL1, y1my2, detL2, y3my4);
    float denom = Det(x1mx2, y1my2, x3mx4, y3my4);
    if(denom == 0.0)//Lines don't seem to cross
    {
        ixOut = NAN;
        iyOut = NAN;
        return false;
    }

    ixOut = xnom / denom;
    iyOut = ynom / denom;
    if(!std::isfinite(ixOut) || !std::isfinite(iyOut)) //Probably a numerical issue
        return false;

    return true; //All OK
}


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
            bool ok = LineLineIntersect(x1,y1,x2,y2,x3,y3,x4,y4,xc,yc);
            std::cout << "1 check LineLineIntersect : " << ok << std::endl;
            pls->join(x1,y1,x2,y2);
            pls->join(x3,y3,x4,y4);
            pls->join(0.0,0.0,xc,yc);

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
            bool ok = LineLineIntersect(x1,y1,x2,y2,x3,y3,x4,y4,xc,yc);
            std::cout << "2 check LineLineIntersect : " << ok << std::endl;

            pls->join(x1,y1,x2,y2);
            pls->join(x3,y3,x4,y4);
            pls->join(0.0,0.0,xc,yc);

        }

        common_message::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = 0.1;

        common::Suspend s;
        int i = 0;
        std::cout << __LINE__ <<  " wait" << std::endl;

//        pls->bop();

        std::cout << __LINE__ <<  " wait" << std::endl;


//        std::cout << __LINE__ <<  " wait" << std::endl;
//        pls->adv(0);
        std::cout << __LINE__ <<  " wait" << std::endl;

        pls->env( xmin, xmax, ymin, ymax, 1, 1 );
        std::cout << __LINE__ <<  " wait" << std::endl;

        pls->lab  ( "x", "y", "Arrow" );

        std::cout << __LINE__ <<  " wait" << std::endl;

        while ( i < 100){
            Arrow(0.0,0.0,0.0628*i,1);
            pls->flush();

//            pls->replot();
            s.sleep(10);
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

        PLFLT PA[2] = {-1.2,0.5};
        PLFLT PB[2] = {-1.0,0.0};
        PLFLT PC[2] = {-0.8,0.0};
        PLFLT PD[2] = {0.0,0.0};

        float t_step = 0.01;
        size_t t_num = 1.0/t_step;

        std::vector<PLFLT> Bezier_x(t_num);
        std::vector<PLFLT> Bezier_y(t_num);
        for(size_t i = 0 ; i < t_num ; i++){
            PLFLT t = i*t_step;

            PLFLT PAB[2];
            PAB[0] = (1-t) *PA[0] + t*PB[0];
            PAB[1] = (1-t) *PA[1] + t*PB[1];

            PLFLT PBC[2];
            PBC[0] = (1-t) *PB[0] + t*PC[0];
            PBC[1] = (1-t) *PB[1] + t*PC[1];

            PLFLT PCD[2];
            PCD[0] = (1-t) *PC[0] + t*PD[0];
            PCD[1] = (1-t) *PC[1] + t*PD[1];


            PLFLT PABC[2];
            PABC[0] = (1-t) *PAB[0] + t*PBC[0];
            PABC[1] = (1-t) *PAB[1] + t*PBC[1];

            PLFLT PBCD[2];
            PBCD[0] = (1-t) *PBC[0] + t*PCD[0];
            PBCD[1] = (1-t) *PBC[1] + t*PCD[1];


            Bezier_x[i] = (1-t) *PABC[0] + t*PBCD[0];
            Bezier_y[i] =  (1-t) *PABC[1] + t*PBCD[1];

        }
        std::cout << "t_num " << t_num << std::endl;

        float arc_len = 0.0;
//        arc_len += sqrt((Bezier_x[1] - Bezier_x[0])*(Bezier_x[1] - Bezier_x[0]) + (Bezier_y[1] - Bezier_y[0])*(Bezier_y[1] - Bezier_y[0]) );
        for(size_t i = 1 ; i < t_num ; i++){
            arc_len += sqrt((Bezier_x[i] - Bezier_x[i-1])*(Bezier_x[i] - Bezier_x[i-1]) + (Bezier_y[i] - Bezier_y[i-1])*(Bezier_y[i] - Bezier_y[i-1]) );
        }

        char buffer[100];
        sprintf(buffer,"arc_len=%.3f",arc_len);
        pls->sfont(0,0,1);
        pls->ptex(0,1,1,0,1,buffer);



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
