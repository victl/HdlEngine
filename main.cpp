#include <iostream>
#include "HdlEngine.h"
#include <sys/time.h>

using namespace std;

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        cout << "Error: too few arguments'" << endl
             << "./Usage: HdlEngine 'hdl_file' 'output_dir'" << endl;
        return -1;
    }
        struct timeval start, end;
        gettimeofday(&start, NULL);
////    for(int i = 0; i < 101; ++i){
////       victl:: hdlEngine.processNextFrame();
////    }
    int count = 0;
//#ifdef OFFLINE
    victl::HdlEngine hdlEngine(argv[1]);
    while(hdlEngine.processNextFrame()){++count;}
    hdlEngine.write3bPng("map_tmp/final-3b.png");
    hdlEngine.visualLocalMap("map_tmp/visualized-final.png");
//#else
    //victl::HdlEngine hdlEngine;
    //hdlEngine.processNextFrame();
//#endif
        gettimeofday(&end, NULL);
        long useconds = end.tv_usec - start.tv_usec;
        long seconds  = end.tv_sec  - start.tv_sec;
        DLOG(INFO) << "Time to process all frame: " << seconds<<"s, "<<useconds<<"microseconds";
    return 0;
}

