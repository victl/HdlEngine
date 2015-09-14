#include "HdlEngine.h"
#include <sys/time.h>
#include <iomanip>
//if not offline
#ifndef OFFLINE
#include <pthread.h>
#include <cstring>
#include <cstdlib>
#include <module/shm.h>
#include <module/module.h>
#include <module/types.h>
#endif

namespace victl {

HdlEngine::HdlEngine()
    : frameProcessedNum(0)
    , correction(params.Ugv.CorrectionFile)
    , dynamicMapRange(params)
    , accumMapRange(params)
    , localMapRange(params)
    , localMap(params.LocalMap.initialHeight, params.LocalMap.initialWidth, CV_8UC3, cv::Scalar(0,0,0))
{
    rawHdlPoints = new RawHdlPoint[MAX_CLOUD_NUM];
    hdlPointXYZs = new HdlPointXYZ[MAX_CLOUD_NUM];
    hdlPointCloud = new HdlPoint[MAX_CLOUD_NUM];
    lpts = new SimpleCarpose[params.Hdl.MaxCP];
    rpts = new SimpleCarpose[params.Hdl.MaxCP];
    spts = new SimpleCarpose[params.Hdl.MaxCP];
}

HdlEngine::HdlEngine(const std::string hdlFileName)
    : frameProcessedNum(0)
    , correction(params.Ugv.CorrectionFile)
    , dynamicMapRange(params)
    , accumMapRange(params)
    , localMapRange(params)
    , localMap(params.LocalMap.initialHeight, params.LocalMap.initialWidth, CV_8UC3, cv::Scalar(0,0,0))
{
    rawHdlPoints = new RawHdlPoint[MAX_CLOUD_NUM];
    hdlPointXYZs = new HdlPointXYZ[MAX_CLOUD_NUM];
    hdlPointCloud = new HdlPoint[MAX_CLOUD_NUM];
    lpts = new SimpleCarpose[params.Hdl.MaxCP];
    rpts = new SimpleCarpose[params.Hdl.MaxCP];
    spts = new SimpleCarpose[params.Hdl.MaxCP];
    initialize(hdlFileName);
}

HdlEngine::~HdlEngine()
{
    hdlReader.is_open() ? hdlReader.close(),NULL:NULL;
    carposeReader.is_open() ? carposeReader.close(),NULL:NULL;
}

bool HdlEngine::initialize(const std::string hdlFileName)
{
    frameProcessedNum = 0;
    if(hdlFileName.substr(hdlFileName.size() - 3,hdlFileName.size()) != "hdl"){
        DLOG(FATAL) << "Error reading HDL file: " << hdlFileName << "\nNot a HDL file.";
        return false;
    }
    baseFileName = hdlFileName.substr(0,hdlFileName.size() - 4);
    hdlReader.is_open() ? hdlReader.close(), hdlReader.open(hdlFileName.c_str(), std::ios::binary) : hdlReader.open(hdlFileName.c_str(), std::ios::binary);
    carposeReader.is_open() ? carposeReader.close(), carposeReader.open((baseFileName+".carposes").c_str()) : carposeReader.open((baseFileName+".carposes").c_str());
    cameraPointReader.is_open() ? cameraPointReader.close(), cameraPointReader.open((baseFileName+".camerapoints").c_str(), std::ios::binary) : cameraPointReader.open((baseFileName+".camerapoints").c_str());
    if(!hdlReader){
        DLOG(FATAL)  << "Error reading HDL file: " << hdlFileName << "\nFile not exist or you don't have "
                                                                     "permission to access it.";
    }
    if(!carposeReader){
        DLOG(FATAL)  << "Error reading car poses file: " << baseFileName + ".carposes" << "\nFile not exist or you don't have "
                                                                          "permission to access it.";
    }
    if(!cameraPointReader){
        DLOG(FATAL)  << "Error reading camera points file: " << baseFileName + ".camerapoints" << "\nFile not exist or you don't have "
                                                                          "permission to access it.";
    }
    //we resize those maps to 0 first, so each map is guaranteed to be re-initialized to default value (0s)
    dynamicMap.resize(0);dynamicMap.resize(params.Scale.Width*params.Scale.Width);
    accumMap.resize(0);accumMap.resize(params.Scale.Width*params.Scale.Width);
    //hdlInstream && carposInstream must be opened properly, so set them as return value
    return hdlReader && carposeReader && cameraPointReader;
}

bool HdlEngine::processNextFrame()
{
    ++frameProcessedNum;
#ifdef DEBUG
    DLOG(INFO) << "Processing frame No." << frameProcessedNum << "...";
#endif

//read points from hdl
#ifdef OFFLINE
    if(!readPointsFromFile())
    {
        return false;
    }
#else
    if(!readPointsFromShm())
    {
        DLOG(FATAL) << "Error(s) occurred during reading hdl points from shared memory." ;
    }
#endif

    //Process the dynamic map
//    Range dynamicMapRange(currentPose, params);
    dynamicMapRange = Range(currentPose, params);
    dynamicMap.resize(0);
    dynamicMap.resize(dynamicMapRange.maxX * dynamicMapRange.maxY);
    for (int i = 0; i < totalPointsNum; ++i){
        //for convinence, define some tmp variables to represent current point's features:
        unsigned int distance = hdlPointCloud[i].distance;
        unsigned short rotAngle = hdlPointCloud[i].rotAngle;
//        unsigned char intensity = hdlPointCloud[i].intensity;
//        unsigned char beamId = hdlPointCloud[i].beamId;

        if( distance < 3000 //3 meters
                && rotAngle > 18000 - correction.blockedByHipAngle  //135 degree
                && rotAngle < 18000 + correction.blockedByHipAngle //225 degree
                && hdlPointXYZs[i].z > 0 ) //point height is greater than LiDAR
        {
            //means current point is not useful
            continue;
        }
        int x = hdlPointCloud[i].x;
        int y = hdlPointCloud[i].y;
        int z = hdlPointCloud[i].z;

        //the following codes translate hdlpoint's xyz into global coordinates.
        //NOTE: CURRENT ALGORITHM DIDN'T TAKE PITCH AND ROLL VALUE INTO CONSIDERATION
        //THIS MIGHT NOT BE RIGHT. WILL BE CONSIDERED IN NEAR FUTURE - 2015-9-10
        double eulr = currentPose.eulr;
        //It seemed that x, y, z are relative coordinate where the origin is the LiDAR, and were measured in millimeter
        //Here, they are translated into meters.
        double dx =(double) x/1000.0;
        double dy=(double) y/1000.0;

        //And here, they were transformed into North-East astronomical coordinates
        double cx =dx * cos(eulr) + dy * sin(eulr) + currentPose.x;
        double cy =dy* cos(eulr) - dx* sin(eulr) + currentPose.y;

        //change to local coordinate. (this will change cx, cy. because they are pass as reference)
        //if the point falls out of the detecting range (toLocal returns false), we ignore current point
        int col, row;
        if(!dynamicMapRange.toLocal(cx, cy, col, row)){
            continue;
        }


        int id = row * dynamicMapRange.maxX + col;
        /*Following commented out functionalities are to be implimented in future*/
//        if (ic>200&&z>0 &&abs(x)<7000)
//            if (d_map->TrafficSignGrid.count(id))
//                d_map->TrafficSignGrid[id]++;
//            else
//                d_map->TrafficSignGrid[id] = 1;

//            if(z>0&&id<GRID_NUM*GRID_NUM&&cc>=0&&cc<GRID_NUM&&rr>=0&&rr<GRID_NUM&&abs(x)<10000&&ic>50)//&&abs(PointsInGrid[id].first - PointsInGrid[id].second)>500)//cc<1500&&rr>0&&rr<1500)
//                Railgridsvec.push_back(id);
//            if(id<GRID_NUM*GRID_NUM&&cc>=0&&cc<GRID_NUM&&rr>=0&&rr<GRID_NUM&&abs(x)<10000)//&&abs(PointsInGrid[id].first - PointsInGrid[id].second)>500)//cc<1500&&rr>0&&rr<1500)
//                NumOfPointsInGrid[id] ++;//	Railgridsvec
        /*Following three lines' intention is unknown, and currently useless*/
//        double R = sqrt(pow((col+0.5)*gridsize-width/2,2)+pow(height/2-(row+0.5)*gridsize,2));
//        double a =(acos(((col+0.5)*gridsize-width/2)/R ));
//        int line = int(a/M_PI*angle);
        ++dynamicMap.at(id).pointNum;
//        dynamicMap.at(id).average = (dynamicMap.at(id).average *dynamicMap.at(id).pointNum + z) / (dynamicMap.at(id).pointNum + 1);//testing
        if (!dynamicMap.at(id).highest &&!dynamicMap.at(id).lowest)
        {
            dynamicMap.at(id).highest = z;
            dynamicMap.at(id).lowest = z;
        }
        else
        {
            if (dynamicMap.at(id).highest < z)
            {
                dynamicMap.at(id).highest = z;
            } else if (dynamicMap.at(id).lowest > z)
            {
                dynamicMap.at(id).lowest = z;
            }
        }
    }
    calcProbability();
//    rayTracing(currentPose);
    updateAccumMap();
    if(params.Hdl.RecordLocalMap)
    {
        updateLocalMap();
    }

#ifdef DEBUG
    if( (params.LocalMap.SaveInterval != 0 && frameProcessedNum % params.LocalMap.SaveInterval == 0) ){
        DLOG(INFO) << "Saving  map...";
        std::string  /*dynamicMapFileName("dynamicmap-"), */accumMapFileName("accummap-");
//        dynamicMapFileName += std::to_string(frameProcessedNum) + ".png";
        accumMapFileName += /*std::*/to_string(frameProcessedNum) + ".png";
//        saveFrame(dynamicMap, dynamicMapRange.maxX, dynamicMapRange.maxY, dynamicMapFileName);
        saveFrame(accumMap, accumMapRange.maxX, accumMapRange.maxY, accumMapFileName);
        if(params.Hdl.RecordLocalMap)
        {
            if(params.LocalMap.SaveNeeded.count(frameProcessedNum)){
                std::string localMapFileName("localmap-"), localMap3bFileName("localmap-3b-");
                localMapFileName += to_string(frameProcessedNum) + ".png";
                localMap3bFileName += to_string(frameProcessedNum) + ".png";
                saveLocalMap(localMapFileName);
//                write3bPng(localMap3bFileName);//NOTE: After re-designing of the local map storage, write3bpng() now has the same effect of saveLocalMap()
                visualLocalMap(localMapFileName);
            }
        }
    }
#endif

    return true;
}

//This function is very IN-MATURE, seemed to have some minor bugs, but I couldn't find where the problem is. So, pls use it with caution!
bool HdlEngine::saveFrame(const std::vector<Grid> &frame, int width, int height,const std::string& name)
{
    cv::Mat img(width, height, CV_8UC1, cv::Scalar(127));
    for(uint i = 0; i < frame.size(); ++i){
        int row = height - i / width - 1;
        int col = i % width;
//        if (frame.at(i).p < 0.5)
//        {
//            img.at<uchar>(row, col) = 255;
////            DLOG(INFO) << "Clear: " <<frame.at(i).pointNum;
//        } else if (frame.at(i).p > 0.5)
//        {
//            img.at<uchar>(row, col) = 0;
////            DLOG(INFO) << "Occupied: " <<frame.at(i).pointNum;
//        }
        switch (frame.at(i).o) {
        case OCCUPIED:
            img.at<uchar>(row, col) = 0;
            break;
        case CLEAR:
            img.at<uchar>(row, col) = 255;
            break;
        default:
            break;
        }
    }
    cv::imshow("Current Map State", img);
    cv::waitKey(25);
//    cv::imwrite(name, img);
    return true;
}

bool HdlEngine::visualLocalMap(const std::string &name)
{
    cv::Mat img(localMap.rows, localMap.cols, CV_8UC3, cv::Scalar(127,127,127));
    for(int col = 0; col < img.cols; ++col)
    {
        for(int row = 0; row < img.rows; ++row)
        {
            unsigned char base = localMap.at<cv::Vec3b>(row, col)[0];
            if(isPresent(base, ROADEDGE_OCCUPIED))
            {
                img.at<cv::Vec3b>(row, col) = cv::Vec3b(0,0,0);
            }
            else if(isPresent(base, ROADEDGE_CLEAR)) {
                img.at<cv::Vec3b>(row, col) = cv::Vec3b(255,255,255);
            }
            if (isPresent(base, LANELINE_CAMERA))
            {
                img.at<cv::Vec3b>(row, col) = cv::Vec3b(0,255,0);
            }
            if (isPresent(base, STOPLINE_YES))
            {
                img.at<cv::Vec3b>(row, col) = cv::Vec3b(0,0,255);
            }
        }
    }
    for( size_t i = 0; i < carposes.size(); ++i)
    {
        int x,y;
        if( localMapRange.toLocal(carposes[i].x, carposes[i].y, x, y) )
        {
            img.at<cv::Vec3b>(img.rows - y -1, x) = cv::Vec3b(244,18,220) ;
        }
    }
    cv::imwrite(name, img);
    return true;
}

void HdlEngine::saveLocalMap(const std::string name)
{
    cv::imwrite(name, localMap);
}

bool HdlEngine::updateAccumMap()
{
    std::vector<Grid> newAccumMap(dynamicMapRange.maxX * dynamicMapRange.maxY);
    for(int x = 0; x < dynamicMapRange.maxX; ++x)
    {
        for(int y = 0; y < dynamicMapRange.maxY; ++y)
        {
            int id = y * dynamicMapRange.maxX + x;
            int accumX, accumY;

            if(dynamicMapRange.translate(x, y, accumMapRange, accumX, accumY))
            {
                //Firstly, copy old accumulated map value to new map
                int accumId = accumY * accumMapRange.maxX + accumX;
                newAccumMap[id] = accumMap[accumId];
            }

                //then merge new accumulated map with dynamic map
                newAccumMap[id] += dynamicMap[id];
//                if (newAccumMap[id].p < 0.5)
//                {
//                    if (dynamicMap[id].p < newAccumMap[id].p)
//                    {
//                        newAccumMap[id].p = dynamicMap[id].p;
//                    }
//                    else if (dynamicMap[id].p > 0.5/*0.99*/)
//                    {
//                        float S = (dynamicMap[id].p / (1 - dynamicMap[id].p))
//                            * (newAccumMap[id].p / (1 - newAccumMap[id].p));
//                        newAccumMap[id].p = S / (1 + S);
//                        if (newAccumMap[id].p > 0.99)
//                        {
//                            newAccumMap[id].p = 0.99;
//                        }
//                        else if (newAccumMap[id].p <= 0.01)
//                        {
//                            newAccumMap[id].p = 0.01;
//                        }
//                    }
//                }
//                else if (newAccumMap[id].p > 0.5)
//                {
//                    if (dynamicMap[id].p > newAccumMap[id].p)
//                    {
//                        newAccumMap[id].p = dynamicMap[id].p;
//                    }
//                    else if (dynamicMap[id].p < 0.5/*0.01*/)
//                    {
//                        float S = (dynamicMap[id].p / (1 - dynamicMap[id].p))
//                            * (newAccumMap[id].p / (1 - newAccumMap[id].p));
//                        newAccumMap[id].p = S / (1 + S);
//                        if (newAccumMap[id].p > 0.99)
//                        {
//                            newAccumMap[id].p = 0.99;
//                        }
//                        else if (newAccumMap[id].p <= 0.01)
//                        {
//                            newAccumMap[id].p = 0.01;
//                        }
//                    }
//                }
//                else if (newAccumMap[id].p == 0.5 && dynamicMap[id].p != 0.5)
//                {
//                    newAccumMap[id].p = dynamicMap[id].p;
//                }
//            }else
//            {
//                newAccumMap[id] = dynamicMap[id];
//            }//END ALL IF

//            if(newAccumMap[id].p > params.ProbMap.OccupiedThreshold)
//            {
//                newAccumMap[id].type = OCCUPIED;
//            }
//            else if (newAccumMap[id].p < params.ProbMap.ClearThreshold){
//                newAccumMap[id].type = CLEAR;
//            }
//            short mid = (newAccumMap.at(id).highest + newAccumMap.at(id).lowest) / 2;
//            short interval = newAccumMap.at(id).highest - newAccumMap.at(id).lowest;
//            if( interval > params.ProbMap.unitHeight
//                    && abs(mid - newAccumMap.at(id).average) < interval * params.ProbMap.MaxAvgMidDiff
//                    && newAccumMap.at(id).average > params.ProbMap.MaxGroundHeight )
//            {
//                newAccumMap[id].type = OCCUPIED;
//            }
//            else if( newAccumMap.at(id).pointNum &&
//                    newAccumMap.at(id).lowest < params.ProbMap.MaxGroundHeight)
//            {
//                newAccumMap[id].type = CLEAR;
//            }
            if(newAccumMap.at(id).HitCount >= params.ProbMap.OccupiedThreshold)
            {
                newAccumMap.at(id).o = OCCUPIED;
            }
            else if(newAccumMap[id].o != OCCUPIED && newAccumMap[id].pointNum)
            {
                newAccumMap[id].o = CLEAR;
            }
        }
    }
    //following codes are for debugging
//#ifdef DEBUG
//    if(frameProcessedNum == 300){
//        std::ofstream pointStatics("pointStatics.txt");
//        for(unsigned short x = 0; x < accumMapRange.maxX; ++x)
//        {
//            for(unsigned short y = 0; y < accumMapRange.maxY; ++y)
//            {
//                unsigned short dynamX, dynamY;

//                if(accumMapRange.translate(x, y, dynamicMapRange, dynamX, dynamY))
//                {
//                    int id = y * accumMapRange.maxX + x;
//                    pointStatics << /*std::*/to_string(accumMap.at(id).HitCount) << '\t'
//                                 << /*std::*/to_string(accumMap.at(id).pointNum) << std::endl;
//                }
//            }
//        }
//        pointStatics.close();
//    }
//#endif
    //end debug codes
    accumMap = newAccumMap;
    accumMapRange = dynamicMapRange;
    return true;
}

bool HdlEngine::updateLocalMap()
{
    //if the first time to update
    if(frameProcessedNum == 1)
    {
        //initialize localMapRange
        localMapRange = accumMapRange;
        localMapRange.right = localMapRange.left + params.LocalMap.initialWidth/params.Scale.xScale;
        localMapRange.top = localMapRange.bottom + params.LocalMap.initialHeight/params.Scale.yScale;
        localMapRange.update();
    }
    adjustLocalMapSize();
    int localX, localY;
    if(!accumMapRange.translate(0, accumMapRange.maxY - 1, localMapRange, localX, localY))
    {
        DLOG(INFO) << "Out of range: (localX, localY) : " << localX << '\t' << localY
                   << "\n";
    }
    cv::Mat region(localMap,cv::Rect(localX,localMap.rows - localY -1,accumMapRange.maxX, accumMapRange.maxY));
    updateRegion(region, accumMap);

    return true;
}

bool HdlEngine::adjustLocalMapSize()
{
    if(localMapRange.left <= accumMapRange.left
            && localMapRange.right >= accumMapRange.right
            && localMapRange.bottom <= accumMapRange.bottom
            && localMapRange.top >= accumMapRange.top)
    {
        return false;//means no adjustment needed
    }

    //Or else expanding is needed
    Range before(params);
    before = localMapRange;
    cv::Rect rect(0, 0, before.maxX, before.maxY);
    if(localMapRange.left > accumMapRange.left)
    {
        localMapRange.left -= params.LocalMap.ExpandUnit;
        rect.x = params.LocalMap.ExpandUnit * params.Scale.xScale;
    }
    else if (localMapRange.right < accumMapRange.right)
    {
        localMapRange.right += params.LocalMap.ExpandUnit;
    }
    if(localMapRange.bottom > accumMapRange.bottom)
    {
        localMapRange.bottom -= params.LocalMap.ExpandUnit;
    }
    else if(localMapRange.top < accumMapRange.top)
    {
        localMapRange.top += params.LocalMap.ExpandUnit;
        rect.y = params.LocalMap.ExpandUnit * params.Scale.yScale;
    }
    localMapRange.update();
    cv::Mat newLocalMap(localMapRange.maxY, localMapRange.maxX, CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat oldRegion(newLocalMap, rect);
    localMap.copyTo(oldRegion);
    localMap = newLocalMap;
    return true;
}

bool HdlEngine::updateRegion(cv::Mat region, const std::vector<Grid> &accumMap)
{
    for(unsigned short x = 0; x < accumMapRange.maxX; ++x)
    {
        for(unsigned short y = 0; y < accumMapRange.maxY; ++y)
        {
            int id = y * dynamicMapRange.maxX + x;
            writeOnMat(region, x, y, accumMap.at(id).a);
            writeOnMat(region, x, y, accumMap.at(id).o);
        }
    }
    return true;
}

bool HdlEngine::writeOnMat(cv::Mat mat, int x, int y, unsigned char value)
{
//    mat.at<unsigned char>(mat.rows - y -1, x) = value;
    switch (value) {
    case OCCUPIED:
        mat.at<cv::Vec3b>(mat.rows - y -1, x)[0] |= ROADEDGE_OCCUPIED;
        break;
    case CLEAR:
        if( (mat.at<cv::Vec3b>(mat.rows - y -1, x)[0] & ROADEDGE_OCCUPIED) != ROADEDGE_OCCUPIED)
            mat.at<cv::Vec3b>(mat.rows - y -1, x)[0] |= ROADEDGE_CLEAR;
        break;
    case CAMERALANELINE:
        mat.at<cv::Vec3b>(mat.rows - y -1, x)[0] |= LANELINE_CAMERA;
        mat.at<cv::Vec3b>(mat.rows - y -1, x)[1] |= 255;//this line is for visualize, do not use in production env
        break;
    case CAMERASTOPLINE:
        mat.at<cv::Vec3b>(mat.rows - y -1, x)[0] |= STOPLINE_YES;
        mat.at<cv::Vec3b>(mat.rows - y -1, x)[2] |= 255;//this line is for visualize, do not use in production env
        break;
    case CAMERALSINTERSECT:
        mat.at<cv::Vec3b>(mat.rows - y -1, x)[0] |= LANELINE_CAMERA;
        mat.at<cv::Vec3b>(mat.rows - y -1, x)[0] |= STOPLINE_YES;
        break;
    default:
        break;
    }
    return true;
}

Point3B HdlEngine::get3b(unsigned short xx, unsigned short yy, MapType type)
{
    Point3B point;
    unsigned char a;
    unsigned char o;
    //for dynamic map and accumulated map, we use vector access. For local map, cv::Mat access method is used
    switch (type) {
    case DYNAMICMAP:
        o = dynamicMap.at( yy * dynamicMapRange.maxX + xx ).o;
        a = dynamicMap.at( yy * dynamicMapRange.maxX + xx ).a;
        break;
    case ACCUMMAP:
        o = accumMap.at( yy * accumMapRange.maxX + xx ).o;
        a = accumMap.at( yy * accumMapRange.maxX + xx ).a;
        break;
        //after the local map had switched from 1channel to 3channel, the content of local map grid is already of type 'Point3B'
//#ifdef OFFLINE
//    case LOCALMAP:
//        value = localMap.at<unsigned char>( localMap.rows - yy - 1, xx);
//        break;
//#endif
    case LOCALMAP:
    {
        cv::Vec3b v3 = localMap.at<cv::Vec3b>(localMap.rows - yy -1, xx);
        point.base = v3[0];
        point.road = v3[1];
        point.sig = v3[2];
        break;
    }
    default:
        DLOG(FATAL) << "Wrong map type: " << type
                    << ".(inside get3b()) Please check out defines.h (enum MapType section) to see valid types.";
        break;
    }

    if(LOCALMAP == type)
    {
        return point;
    }
    switch (a) {
    case LANELINE:
        point.base |= (ROADEDGE_CLEAR | LANELINE_HDL);
        break;
    case ZEBRA:
        point.base |= ROADEDGE_CLEAR;
        break;
    case INTERSECTION:
        point.base |= ROADEDGE_CLEAR;
        point.road |= REGION_INTERSECTION;
        break;
    case CURB:
        point.base |= ROADEDGE_OCCUPIED;
        point.road |= CURB_YES;
        break;
    case TREE:
        point.base |= ROADEDGE_OCCUPIED;
        break;
    case TRUNK:
        point.base |= ROADEDGE_OCCUPIED;
        break;
    case PIT:
        //NOTE: Pits should be treated as OCCUPIED, but I'm not sure for the moment
//        point.base |= ROADEDGE_OCCUPIED;
        break;
    case LANECENTER:
        point.base |= ROADEDGE_CLEAR;
        break;
    case TRAFFICSIGN:
        point.base |= ROADEDGE_OCCUPIED;
        //type of traffic signs will be added in the future
        break;
        //CAMERA... were added after 3b format definition version 2. might be called version 2.1
    case CAMERALANELINE:
        point.base |= LANELINE_CAMERA;
        break;
    case CAMERASTOPLINE:
        point.base |= STOPLINE_YES;
        break;
    case CAMERALSINTERSECT:
        point.base |= LANELINE_CAMERA;
        point.base |= STOPLINE_YES;
        break;
        //after the local map had switched from 1channel to 3channel, the content of local map grid is already of type 'Point3B'
//    case AUNKNOWN:
//        point.base |= ROADEDGE_UNKNOWN;
//        break;
//    case CLEAR:
//        point.base |= ROADEDGE_CLEAR;
//        break;
//    case OCCUPIED:
//        point.base |= ROADEDGE_OCCUPIED;
//        break;
    default:
        point.base |= ROADEDGE_UNKNOWN;
        break;
    }
//    if(point.base&ROADEDGE_CLEAR == ROADEDGE_CLEAR){
//        DLOG(INFO)<<"CLEAR POINT: ("<<xx<<", "<<yy<<")";
//    }
    switch (o) {
    case OCCUPIED:
        point.base |= ROADEDGE_OCCUPIED;
        break;
    case CLEAR:
        point.base |= ROADEDGE_CLEAR;
        break;
    default:
        break;
    }
    return point;
}

bool HdlEngine::write3bPng(const std::string fileName, MapType type)
{
    std::ofstream osHeader((fileName+"-header.txt").c_str());
    switch (type) {
    case DYNAMICMAP:
    {
        cv::Mat img_d(dynamicMapRange.maxY, dynamicMapRange.maxY, CV_8UC3);
        for(unsigned int i = 0; i < dynamicMap.size(); ++i)
        {
            unsigned short x = i % dynamicMapRange.maxX;
            unsigned short y = i / dynamicMapRange.maxX;
            Point3B pt = get3b(x, y, DYNAMICMAP);
            img_d.at<cv::Vec3b>(img_d.rows - y - 1, x) = cv::Vec3b(pt.base, pt.road, pt.sig);
        }
        cv::imwrite(fileName, img_d);
        osHeader << (dynamicMapRange.right + dynamicMapRange.left) / 2 << '\t'
                 << (dynamicMapRange.top + dynamicMapRange.bottom) / 2
                 << std::endl
                 << dynamicMapRange.maxX << '\t'
                 << dynamicMapRange.maxY;
        break;
    }
    case ACCUMMAP:
    {
        cv::Mat img_a(accumMapRange.maxY, accumMapRange.maxY, CV_8UC3);
        for(unsigned int i = 0; i < accumMap.size(); ++i)
        {
            unsigned short x = i % accumMapRange.maxX;
            unsigned short y = i / accumMapRange.maxX;
            Point3B pt = get3b(x, y, ACCUMMAP);
            img_a.at<cv::Vec3b>(img_a.rows - y - 1, x) = cv::Vec3b(pt.sig, pt.road, pt.base);
        }
        cv::imwrite(fileName, img_a);
        osHeader << (accumMapRange.right + accumMapRange.left) / 2 << '\t'
                 << (accumMapRange.top + accumMapRange.bottom) / 2
                 << std::endl
                 << accumMapRange.maxX << '\t'
                 << accumMapRange.maxY;
        break;
    }
    case LOCALMAP:
    {
//        cv::Mat img_l(localMap.rows, localMap.cols, CV_8UC3);
//        for(int x = 0; x < img_l.cols; ++x)
//        {
//            for(int y = 0; y < img_l.rows; ++y)
//            {
//                Point3B pt = get3b(x, y, LOCALMAP);
//                img_l.at<cv::Vec3b>(img_l.rows - y - 1, x) = cv::Vec3b(pt.base, pt.road, pt.sig);
//            }
//        }
        cv::imwrite(fileName, localMap);
        osHeader << (localMapRange.right + localMapRange.left) / 2 << '\t'
                 << (localMapRange.top + localMapRange.bottom) / 2
                 << std::endl
                 << localMapRange.maxX << '\t'
                 << localMapRange.maxY;
        break;
    }
    default:
        DLOG(FATAL) << "Wrong map type: " << type
                    << ".(inside write3bPng()) Please check out defines.h (enum MapType section) to see valid types.";
        break;
    }
    osHeader.close();
    return true;
}

const std::vector<Grid> &HdlEngine::getAccumMap()
{
    return accumMap;
}

const Range &HdlEngine::getAccumMapRange()
{
    return accumMapRange;
}

const Carpose &HdlEngine::getCurrentPose()
{
    return currentPose;
}

bool HdlEngine::readPointsFromFile()
{
    //Firstly, read num of points in current frame.
    if(!hdlReader.read((char*)&totalPointsNum, sizeof(totalPointsNum))){
        //because DLOG(FATAL) will terminate the program immediately. It is no good for timing the program. so
        //they are commented out during development.
//        DLOG(FATAL)  << "Error reading HDL file: " << baseFileName + ".hdl" << "\nReading total points number "
//                                                                     "of frame: " << frameProcessed;
        return false;
    }
#ifdef DEBUG
//    DLOG(INFO) << "Total point number: " << totalPointsNum;
#endif

    //Secondly, read all points into container of raw HDL points
    //After 2015-9-10, all '.hdl' file will include xyz info, no need to recalculate.
    //this is a critical file format change. IMPORTANT!!!
    switch(params.Hdl.HdlVersion)
    {
    case 1:
        if(!hdlReader.read((char*)rawHdlPoints, sizeof(RawHdlPoint)*totalPointsNum)){
    //        DLOG(FATAL)  << "Error reading HDL file: " << baseFileName + ".hdl" << "\nReading the" <<" points "
    //                                                                         "of frame: " << frameProcessed <<" error.";
            return false;
        }

        //traverse all points, calculate each point's XYZ coordinates
        populateXYZ(rawHdlPoints, hdlPointXYZs, totalPointsNum);
        //copy raw hdl point and their xyzs to hdlPointClout
        for(int i = 0; i < totalPointsNum; ++i)
        {
            hdlPointCloud[i].beamId = rawHdlPoints[i].beamId;
            hdlPointCloud[i].distance = rawHdlPoints[i].distance;
            hdlPointCloud[i].intensity = rawHdlPoints[i].intensity;
            hdlPointCloud[i].rotAngle = rawHdlPoints[i].rotAngle;
            hdlPointCloud[i].x = hdlPointXYZs[i].x;
            hdlPointCloud[i].y = hdlPointXYZs[i].y;
            hdlPointCloud[i].z = hdlPointXYZs[i].z;
        }

        //Thirdly, read in the carpos of current frame
        carposeReader >> currentPose.x >> currentPose.y >> currentPose.eulr;
        carposes.push_back(currentPose);

    case 2:
        if(!hdlReader.read((char*)hdlPointCloud, sizeof(HdlPoint)*totalPointsNum)){
    //        DLOG(FATAL)  << "Error reading HDL file: " << baseFileName + ".hdl" << "\nReading the" <<" points "
    //                                                                         "of frame: " << frameProcessed <<" error.";
            return false;
        }
        //Read in the carpose of current frame
        carposeReader >> currentPose.x >> currentPose.y >> currentPose.eulr >> currentPose.roll >> currentPose.pitch;
        carposes.push_back(currentPose);
    }

    return true;
}

#ifndef OFFLINE
bool HdlEngine::readPointsFromShm()
{
    //This function should not be used when running offline

    MetaData shm;
    //1. get carpose
    shm.type = module::MetaData::META_NAVIGATION;
    SHARED_OBJECTS.GetMetaData(&shm);
    currentPose.x = shm.value.v_navi.ENU[0];
    currentPose.y = shm.value.v_navi.ENU[1];
    currentPose.roll = shm.value.v_navi.Eulr[0];//俯仰角
    currentPose.pitch = shm.value.v_navi.Eulr[1];//滚转角
    currentPose.eulr = shm.value.v_navi.Eulr[2];//方位角

    //2. get hdl points
    shm.type = module::MetaData::META_LASER_HDL;
    SHARED_OBJECTS.GetMetaData(&shm);
    totalPointsNum = shm.value.v_laserHdl.pts_count;
    memcpy(hdlPointCloud, shm.value.v_laserHdl.pts, sizeof(HdlPoint) * totalPointsNum);

    //3.push currentPose into carposes
    carposes.push_back(currentPose);

    return true;
}
#endif


bool HdlEngine::populateXYZ(RawHdlPoint *rawHdlPoints , HdlPointXYZ *hdlPointXYZs, int totalPointsNum)
{
    for (int i = 0; i < totalPointsNum; ++i)
    {
//        unsigned char intensity = rawHdlPoints[i].intensity;
        unsigned int distance = rawHdlPoints[i].distance;
        unsigned char beamId = rawHdlPoints[i].beamId;
        unsigned short rotAngle = rawHdlPoints[i].rotAngle;

        //'phi' is the vertical angle. Because this angle is fixed for each laser beam, so the correction parameters is in fact
        // directly the angle, not a tiny correction of a certain value.
        float cos_phi = correction.cos_vertAngle[beamId];
        float sin_phi = correction.sin_vertAngle[beamId];
        //'theta' is the horizontal (rotation) angle. Because the LiDAR is revolving quickly, so - unlike the 'phi' - the real theta angle
        // is the rotation angle adjusted (plus) by correction.rotAngle.
        float cos_theta = correction.cos_raw[rotAngle] * correction.cos_rotAngle[beamId] + correction.sin_raw[rotAngle] * correction.sin_rotAngle[beamId];
        float sin_theta = correction.sin_raw[rotAngle] * correction.cos_rotAngle[beamId] - correction.cos_raw[rotAngle] * correction.sin_rotAngle[beamId];
        float r_measure = distance * 2.0f;//multiplying 2 is because the unit length of distance is 2mm instead of 1mm
        //Here, all 'r_...' refer to the direct distance between LiDAR and the reflect point
        //this distance need not be horizontal. Instead, it might have a vertical angle
        float r_adjusted = r_measure + correction.dist[beamId];
        float r_horizontal = r_adjusted * cos_phi;//r_horizontal is the horizontal branch of r_adjusted
        float xx = abs(r_horizontal * sin_theta - correction.horizOffset[beamId] * cos_theta);
        float yy = abs(r_horizontal * cos_theta + correction.horizOffset[beamId] * sin_theta);

        float rx = (correction.dist[beamId] - correction.distX[beamId]) * (xx/22640.0f - 0.106007f) + correction.distX[beamId];//what the fuck are those singular constant values???
        float ry = (correction.dist[beamId] - correction.distY[beamId]) * (yy/23110.0f - 0.083514f) + correction.distY[beamId];

        //The following calculation is confusing for Zou. Why recalculate all those values again and again?
        //Answers might reside in the LiDAR's manual pdf file. Will be referred to in future.
        //x:
        r_adjusted = r_measure + rx;
        r_horizontal = r_adjusted * cos_phi;
        int x = (int)(r_horizontal * sin_theta - correction.horizOffset[beamId] * cos_theta);

        //y:
        r_adjusted = r_measure + ry;
        r_horizontal = r_adjusted * cos_phi;
        int y = (int)(r_horizontal * cos_theta + correction.horizOffset[beamId] * sin_theta);

        //z:
        r_adjusted = r_measure + correction.dist[beamId];
        int z = (int)(r_adjusted * sin_phi + correction.vertOffset[beamId]);

        hdlPointXYZs[i].x = x;
        hdlPointXYZs[i].y = y;
        hdlPointXYZs[i].z = z;
    }   //end for(totalPointsNum)

    return true;
}

bool HdlEngine::calcProbability()
{
    //The following for-loop corresponse to XIAO KE's ProbabilityMap()
//    for(auto &g : dynamicMap){
//        unsigned char n = (g.highest - g.lowest) / params.ProbMap.unitHeight;
//        if(n) // current grid contain laser points and interval between highest and lowest is greater than unitHeight
//        {
//            g.p = 0.5 + n * params.ProbMap.incrementUnit;
//            g.p > 1 ? g.p = 1 : 0;
//            g.type = OCCUPIED;
//            g.HitCount = 1;
//#ifdef MOREDETAILS
//            if(g.p > 0.5)
//                g.type = OCCUPIED;
//#endif
//        }
//        //because the S/(1+S) formula is actually not very effective. Here I used a much simpler way to handle this problem
////        else if (g.pointNum){//current grid contain laser point(s) but interval is smaller
////            g.p = 0.5 - params.ProbMap.incrementUnit / 100;
////            g.p < 0 ? g.p = 0 : 0;
////#ifdef MOREDETAILS
////            if(g.p < 0.5)
////                g.type = CLEAR;
////#endif
////        }
//    }//end for(auto g:...)
    for(size_t i = 0; i < dynamicMap.size(); ++i){
        unsigned char n = (dynamicMap.at(i).highest - dynamicMap.at(i).lowest) / params.ProbMap.unitHeight;
        if(n) // current grid contain laser points and interval between highest and lowest is greater than unitHeight
        {
            dynamicMap.at(i).p = 0.5 + n * params.ProbMap.incrementUnit;
            dynamicMap.at(i).p > 1 ? dynamicMap.at(i).p = 1 : 0;
//            dynamicMap.at(i).o = OCCUPIED;
            dynamicMap.at(i).HitCount = 1;
#ifdef MOREDETAILS
            if(dynamicMap.at(i).p > 0.5)
                dynamicMap.at(i).type = OCCUPIED;
#endif
        }
    }//end for(sizt_t i)

    //Since version 2, camera points well added
    if(params.Hdl.HdlVersion > 1)
    {
        //Here, l..., r..., s... represent left lanemark, right lanemark, stopline
        bool lValid, rValid, sValid;
        int lnum, rnum, snum;
        cameraPointReader.read((char*)&lValid, sizeof(lValid));
        cameraPointReader.read((char*)&rValid, sizeof(rValid));
        cameraPointReader.read((char*)&sValid, sizeof(sValid));
        if(lValid)
        {
            cameraPointReader.read((char*)&lnum, sizeof(lnum));
            cameraPointReader.read((char*)lpts, sizeof(SimpleCarpose) * lnum);
            for(int i = 0; i < lnum; ++i)
            {
                int x,y;
                if(dynamicMapRange.toLocal(lpts[i].x, lpts[i].y, x, y))
                {
                    dynamicMap.at(y * dynamicMapRange.maxX + x).a = CAMERALANELINE;
                }
            }
        }
        if(rValid)
        {
            cameraPointReader.read((char*)&rnum, sizeof(rnum));
            cameraPointReader.read((char*)rpts, sizeof(SimpleCarpose) * rnum);
            for(int i = 0; i < rnum; ++i)
            {
                int x,y;
                if(dynamicMapRange.toLocal(rpts[i].x, rpts[i].y, x, y))
                {
                    dynamicMap.at(y * dynamicMapRange.maxX + x).a = CAMERALANELINE;
                }
            }
        }
        if(sValid)
        {
            cameraPointReader.read((char*)&snum, sizeof(snum));
            cameraPointReader.read((char*)spts, sizeof(SimpleCarpose) * snum);
            for(int i = 0; i < snum; ++i)
            {
                int x,y;
                if(dynamicMapRange.toLocal(spts[i].x, spts[i].y, x, y))
                {
                    dynamicMap.at(y * dynamicMapRange.maxX + x).a = CAMERASTOPLINE;
                }
            }
        }
    }

    return true;
}

//Following codes are from XIAO KE, but they are not needed for the moment
/*
bool HdlEngine::rayTracing(const Carpose& currentPose)
{
    //Following codes were inherited from XIAO KE
    float eulr = currentPose.eulr;
    float height = dynamicMapRange.maxY;
    float width = dynamicMapRange.maxX;
//	int grid_size = width / gridNum;
    int row, col;
//    int x, y;
    double R, r;
//	pointT tempPointT;

    int angle = 100;
    double ratio = 0.3;
    std::map<int, std::set<double> > anglemap;
    if (eulr > 0)
        eulr = eulr - 2 * M_PI;
    int MinAngle = (int((int((-eulr / M_PI + 0.5)*angle)) % (2 * angle) - ratio*angle)) % (2 * angle);
    int MaxAngle = (int((int((-eulr / M_PI + 0.5)*angle)) % (2 * angle) + ratio*angle)) % (2 * angle);
    if (MinAngle < 0)
        MinAngle = MinAngle + 2 * angle;
    int axis = (int((-eulr / M_PI + 0.5)*angle)) % (2 * angle);
    //for(map<long,pair<int,int> >::iterator i=PointsInGrid.begin();i!= PointsInGrid.end();i++)
    for (unsigned int i = 0; i < dynamicMap.size(); ++i)
    {
        if (dynamicMap[i].highest || dynamicMap[i].lowest)
        {
            row = i / dynamicMapRange.maxX;
            col = i % dynamicMapRange.maxX;
            if (dynamicMap[i].highest - dynamicMap[i].lowest < params.ProbMap.HeightThreshold)
                continue;
//            R = sqrt(pow((col + 0.5) - width / 2, 2) + pow(height / 2 - (row + 0.5), 2));
            R = sqrt(pow((col + 0.5) - width / 2, 2) + pow((row + 0.5), 2));
            double a = (acos(((col + 0.5) - width / 2) / R));
            int line = int(a / M_PI*angle);
            if (MinAngle < MaxAngle)
            {
                if (row <= dynamicMapRange.maxX / 2)
                {
                    if (abs(line - axis) < ratio*angle)
                        anglemap[line].insert(R);
                }
                else
                {
                    if (abs(2 * angle - 1 - line - axis) < ratio*angle)
                        anglemap[2 * angle - 1 - line].insert(R);
                }
            }
            else
            {
                if (row <= dynamicMapRange.maxX / 2)
                {
                    if (line<MaxAngle)
                        anglemap[line].insert(R);
                }
                else
                {
                    if (2 * angle - 1 - line>MinAngle)
                        anglemap[2 * angle - 1 - line].insert(R);
                }
            }
        }
    }
    frontPoints(angle, axis, MaxAngle, MinAngle, anglemap);

    for (unsigned int i = 0; i < dynamicMapRange.maxX; ++i)//���������Ե�bug����Ҫ����ȥ������grid�ı���Ӧ�ø������йأ��Ϸ��Ĳ�һ����
    {
        for (size_t j = 0; j < dynamicMapRange.maxY; ++j)
        {
            row = i;
            col = j;
//            tempPointT.x = col;
//            tempPointT.y = row;

//            r = sqrt(pow((col + 0.5) - width / 2, 2) + pow(height / 2 - (row + 0.5), 2));
            R = sqrt(pow((col + 0.5) - width / 2, 2) + pow((row + 0.5), 2));

            if (row <= dynamicMapRange.maxX / 2)
                R = *(anglemap[(acos(((col + 0.5) - width / 2) / r) / M_PI*angle)].begin());
            else
                R = *(anglemap[(2 * angle - 1 - int(acos(((col + 0.5) - width / 2) / r) / M_PI*angle))].begin());

            if (r <= R)
            {
                double p = likelihood(r);
                dynamicMap[i*dynamicMapRange.maxX + j].p = p;// p.second;
            }
        }
    }
    anglemap.clear();
    return true;
}

bool HdlEngine::frontPoints(unsigned char angle, int axis, int MaxAngle, int MinAngle, std::map<int, std::set<double> > &anglemap)
{
    double ratio = 0.4;
    int row, col;
    double R;
    float height = dynamicMapRange.maxY;
    float width = dynamicMapRange.maxX;
//	int grid_size = width / gridNum;

    for (unsigned short i = 0; i < dynamicMapRange.maxX; ++i)
    {
        row = 0;
        col = i;
//        R = sqrt(pow((col + 0.5) - width / 2, 2) + pow(height / 2 - (row + 0.5), 2));
        R = sqrt(pow((col + 0.5) - width / 2, 2) + pow((row + 0.5), 2));
        double a = (acos(((col + 0.5) - width / 2) / R));
        int line = int(a / M_PI*angle);
        //	if(abs(int(a/M_PI*angle)-axis)<0.25*angle)
        if (row <= dynamicMapRange.maxX / 2)
        {
            if (abs(line - axis) < ratio*angle)
                anglemap[line].insert(R);
        }
        else
        {
            if (abs(2 * angle - 1 - line - axis) < ratio*angle)
                anglemap[2 * angle - 1 - line].insert(R);
        }

        row = dynamicMapRange.maxY - 1;
        col = i;
//        R = sqrt(pow((col + 0.5) - width / 2, 2) + pow(height / 2 - (row + 0.5), 2));
        R = sqrt(pow((col + 0.5) - width / 2, 2) + pow((row + 0.5), 2));
        a = (acos(((col + 0.5) - width / 2) / R));
        line = int(a / M_PI*angle);
        //	if(abs(line-axis)<0.25*angle)
        if (row <= dynamicMapRange.maxX / 2)
        {
            if (abs(line - axis) < ratio*angle)
                anglemap[line].insert(R);
        }
        else
        {
            if (abs(2 * angle - 1 - line - axis) < ratio*angle)
                anglemap[2 * angle - 1 - line].insert(R);
        }

        row = i;
        col = 0;
//        R = sqrt(pow((col + 0.5) - width / 2, 2) + pow(height / 2 - (row + 0.5), 2));
        R = sqrt(pow((col + 0.5) - width / 2, 2) + pow((row + 0.5), 2));
        a = (acos(((col + 0.5) - width / 2) / R));
        line = int(a / M_PI*angle);
        //	if(abs(line-axis)<0.25*angle)
        if (row <= dynamicMapRange.maxX / 2)
        {
            if (abs(line - axis) < ratio*angle)
                anglemap[line].insert(R);
        }
        else
        {
            if (abs(2 * angle - 1 - line - axis) < ratio*angle)
                anglemap[2 * angle - 1 - line].insert(R);
        }

        row = i;
        col = dynamicMapRange.maxY - 1;
//        R = sqrt(pow((col + 0.5) - width / 2, 2) + pow(height / 2 - (row + 0.5), 2));
        R = sqrt(pow((col + 0.5) - width / 2, 2) + pow((row + 0.5), 2));
        a = (acos(((col + 0.5) - width / 2) / R));
        line = int(a / M_PI*angle);
        //		if(abs(line-axis)<0.25*angle)
        if (MinAngle < MaxAngle)
        {
            if (row <= dynamicMapRange.maxX / 2)
            {
                if (abs(line - axis) < ratio*angle)
                    anglemap[line].insert(R);
            }
            else
            {
                if (abs(2 * angle - 1 - line - axis) < ratio*angle)
                    anglemap[2 * angle - 1 - line].insert(R);
            }
        }
        else
        {
            if (row <= dynamicMapRange.maxX / 2)
            {
                if (line<MaxAngle)
                    anglemap[line].insert(R);
            }
            else
            {
                if (2 * angle - 1 - line>MinAngle)
                    anglemap[2 * angle - 1 - line].insert(R);
            }
        }
    }
    return true;
}

double HdlEngine::likelihood(double d)
{
    double Dmax = 1000;
    double Dmin = 2;
    if (d >= Dmax)
        return 0.5;

    if (d <= Dmin&&d >= 0)
        return  0.4*Dmin / Dmax;

    if (d > Dmin && d < Dmax)
        return		0.4*d / Dmax;

    return 0.5;
}

unsigned char HdlEngine::p2color(float p)
{
    int delta = 50;
    if(p<0.5&& p>=0)
        return 255*(1-p);

    if(p>0.5&&p<=1)
        return 127- (p-0.5)/0.05*delta >=0?127- (p-0.5)/0.05*delta:0;

    return 127;
}

*/
}//end namespace victl
