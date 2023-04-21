//
//
// This software is provided under the LGPL in March 2009 by the
// Cluster Science Centre
// QSAS team,
// Imperial College, London
//
// Copyright (C) 2009  Imperial College, London
//
// This is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Lesser Public License as published
// by the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// To received a copy of the GNU Library General Public License
// write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
//
//

#include "qt_PlotWindow.h"
#include "plc++demos.h"

int main( int argc, char** argv )
{



    if(0){
        const int NSIZE = 101;
        PLFLT x[NSIZE], y[NSIZE];
        PLFLT xmin = 0., xmax = 1., ymin = 0., ymax = 100.;
        int   i;

        // Prepare data to be plotted.
        for ( i = 0; i < NSIZE; i++ )
        {
            x[i] = (PLFLT) ( i ) / (PLFLT) ( NSIZE - 1 );
            y[i] = ymax * x[i] * x[i];
        }



        plstream         *pls;
        PLINT       strm;

        pls = new plstream();
        QtExtWidget * plot;
        QMainWindow w;

        plot = new QtExtWidget( QT_DEFAULT_X, QT_DEFAULT_Y,&w );

        // Parse and process command line arguments
        pls->parseopts( &argc, argv, PL_PARSE_FULL );

        std::cout << "init" << std::endl;

        // Initialize plplot
//        pls->init();

        plmkstrm( &strm );
        plsdev( "extqt" );
        plsetqtdev( plot );
        plinit();

        std::cout << "init done" << std::endl;

        // Create a labelled box to hold the plot.
        pls->env( xmin, xmax, ymin, ymax, 0, 0 );
        pls->lab( "x", "y=100 x#u2#d", "Simple PLplot demo of a 2D line plot" );

        // Plot the data that was prepared above.
        pls->line( NSIZE, x, y );


        // In C++ we don't call plend() to close PLplot library
        // this is handled by the destructor
        delete pls;
        delete plot;

        return 0;
    }




    int res;

    // Command-line options are only to be interpreted by PLplot.  Thus,
    // make a deep copy of the arguments for PLplot use before QApplication
    // has a chance to alter them.
    int Argc = argc;
    char** Argv;

    Argv = new char*[argc];
    for ( int i = 0; i < Argc; ++i )
    {
        int len = strlen( argv[i] ) + 1;
        Argv[i] = new char[len];
        strncpy( Argv[i], argv[i], len );
    }

    // Limit QApplication's interpretation of argv to just the first
    // argument (the application name) so that all command-line
    // options such as the PLplot -bg command-line option (which would
    // generate a warning message when misinterpreted by QApplication)
    // are completely ignored.
    argc = 1;
    QApplication a( argc, argv );
    PlotWindow   * win = new PlotWindow( Argc, Argv );
    a.setActiveWindow( win );
    win->setVisible( true );
    std::cout << "exec" << std::endl;
    QMainWindow w;


    w.show();

    res = a.exec();


    std::cout << "exec done" << std::endl;

    for ( int i = 0; i < Argc; ++i )
    {
        delete[] Argv[i];
    }
    delete[] Argv;

    return res;
}
