#include "HdlEngine.h"
#include <sys/time.h>

HdlEngine::HdlEngine()
    : frameProcessedNum(0)
    , correction(params.Ugv.CorrectionFile)
    , dynamicMapRange(params)
    , accumMapRange(params)
    #ifdef OFFLINE
    , localMapRange(params)
    , localMap(params.LocalMap.initialHeight, params.LocalMap.initialWidth, CV_8UC1, cv::Scalar(127))
    #endif
{

}

HdlEngine::HdlEngine(const std::string hdlFileName)
    : frameProcessedNum(0)
    , correction(params.Ugv.CorrectionFile)
    , dynamicMapRange(params)
    , accumMapRange(params)
    #ifdef OFFLINE
    , localMapRange(params)
    , localMap(params.LocalMap.initialHeight, params.LocalMap.initialWidth, CV_8UC1, cv::Scalar(127))
    #endif
{
    initialize(hdlFileName);
}

HdlEngine::~HdlEngine()
{
    hdlInstream.is_open() ? hdlInstream.close(),NULL:NULL;
    carposeInstream.is_open() ? carposeInstream.close(),NULL:NULL;
}

bool HdlEngine::initialize(const std::string hdlFileName)
{
    frameProcessedNum = 0;
    if(hdlFileName.substr(hdlFileName.size() - 3,hdlFileName.size()) != "hdl"){
        DLOG(FATAL) << "Error reading HDL file: " << hdlFileName << "\nNot a HDL file.";
        return false;
    }
    baseFileName = hdlFileName.substr(0,hdlFileName.size() - 4);
    hdlInstream.is_open() ? hdlInstream.close(), hdlInstream.open(hdlFileName.c_str(), std::ios::binary) : hdlInstream.open(hdlFileName.c_str(), std::ios::binary);
    carposeInstream.is_open() ? carposeInstream.close(), carposeInstream.open((baseFileName+".hdl_dgps").c_str()) : carposeInstream.open((baseFileName+".hdl_dgps").c_str());
    if(!hdlInstream){
        DLOG(FATAL)  << "Error reading HDL file: " << hdlFileName << "\nFile not exist or you don't have "
                                                                     "permission to access it.";
    }
    if(!carposeInstream){
        DLOG(FATAL)  << "Error reading hdl_dgps file: " << hdlFileName << "\nFile not exist or you don't have "
                                                                          "permission to access it.";
    }
    //we resize those maps to 0 first, so each map is guaranteed to be re-initialized to default value (0s)
    dynamicMap.resize(0);dynamicMap.resize(params.Scale.Width*params.Scale.Width);
    accumMap.resize(0);accumMap.resize(params.Scale.Width*params.Scale.Width);
    //hdlInstream && carposInstream must be opened properly, so set them as return value
    return hdlInstream && carposeInstream;
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

    //traverse all points, calculate each point's XYZ coordinates
    populateXYZ(rawHdlPoints, hdlPointXYZs, totalPointsNum);

    //Process the dynamic map
//    Range dynamicMapRange(currentPose, params);
    dynamicMapRange = Range(currentPose, params);
    dynamicMap.resize(0);
    dynamicMap.resize(dynamicMapRange.maxX * dynamicMapRange.maxY);
    for (int i = 0; i < totalPointsNum; ++i){
        //for convinence, define some tmp variables to represent current point's features:
        unsigned int distance = rawHdlPoints[i].distance;
        unsigned short rotAngle = rawHdlPoints[i].rotAngle;
//        unsigned char intensity = rawHdlPoints[i].intensity;
//        unsigned char beamId = rawHdlPoints[i].beamId;

        if( distance < 3000 //3 meters
                && rotAngle > 18000 - correction.blockedByHipAngle  //135 degree
                && rotAngle < 18000 + correction.blockedByHipAngle //225 degree
                && hdlPointXYZs[i].z > 0 ) //point height is greater than LiDAR
        {
            //this point is not useful
            continue;
        }
        int x = hdlPointXYZs[i].x;
        int y = hdlPointXYZs[i].y;
        int z = hdlPointXYZs[i].z;
        double eulr = currentPose.eulr;
        //It seemed that x, y, z are relative coordinate where the origin is the LiDAR, and were measured in millimeter
        //Here, they are translated into meters.
        float dx =(float) x/1000.0;
        float dy=(float) y/1000.0;

        //And here, they were transformed into North-East astronomical coordinates
        double cx =dx * cos(eulr) + dy * sin(eulr) + currentPose.x;
        double cy =dy* cos(eulr) - dx* sin(eulr) + currentPose.y;

        //change to local coordinate. (this will change cx, cy. because they are pass as reference)
        //if the point falls out of the detecting range (toLocal returns false), we ignore current point
        if(!dynamicMapRange.toLocal(cx, cy)){
            continue;
        }
        unsigned short col = cx;
        unsigned short row = cy;


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
#ifdef OFFLINE
    updateLocalMap();
#endif
#ifdef DEBUG
    if(params.LocalMap.SaveNeeded.count(frameProcessedNum)
            || (params.LocalMap.SaveInterval != 0 && frameProcessedNum % params.LocalMap.SaveInterval == 0) ){
        DLOG(INFO) << "Saving  map...";
        std::string  /*dynamicMapFileName("dynamicmap-"), */accumMapFileName("accummap-");
//        dynamicMapFileName += std::to_string(frameProcessedNum) + ".png";
        accumMapFileName += /*std::*/to_string(frameProcessedNum) + ".png";
//        saveFrame(dynamicMap, dynamicMapRange.maxX, dynamicMapRange.maxY, dynamicMapFileName);
        saveFrame(accumMap, accumMapRange.maxX, accumMapRange.maxY, accumMapFileName);
#ifdef OFFLINE
//        std::string localMapFileName("localmap-"), localMap3bFileName("localmap-3b-");
//        localMapFileName += std::to_string(frameProcessedNum) + ".png";
//        localMap3bFileName += std::to_string(frameProcessedNum) + ".png";
//        saveLocalMap(localMapFileName);
//        write3bPng(localMap3bFileName);
//        visualLocalMap("visualLocalMap.png");
#endif
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
        switch (frame.at(i).type) {
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
    cv::imshow("map of current frame", img);
    cv::waitKey(80);
//    cv::imwrite(name, img);
    return true;
}

#ifdef OFFLINE
bool HdlEngine::visualLocalMap(const std::string &name)
{
    cv::Mat img(localMap.rows, localMap.cols, CV_8UC1, cv::Scalar(127));
    for(int col = 0; col < img.cols; ++col)
    {
        for(int row = 0; row < img.rows; ++row)
        {
            if(localMap.at<unsigned char>(row, col) == OCCUPIED)
            {
                img.at<unsigned char>(row, col) = 0;
            }
            else if(localMap.at<unsigned char>(row, col) == CLEAR) {
                img.at<unsigned char>(row, col) = 255;
            }
        }
    }
    cv::imwrite(name, img);
    return true;
}

void HdlEngine::saveLocalMap(const std::string name)
{
    cv::imwrite(name, localMap);
}
#endif

bool HdlEngine::updateAccumMap()
{
    std::vector<Grid> newAccumMap(dynamicMapRange.maxX * dynamicMapRange.maxY);
    for(unsigned short x = 0; x < dynamicMapRange.maxX; ++x)
    {
        for(unsigned short y = 0; y < dynamicMapRange.maxY; ++y)
        {
            int id = y * dynamicMapRange.maxX + x;
            unsigned short accumX, accumY;

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
                newAccumMap.at(id).type = OCCUPIED;
            }
            else if(newAccumMap[id].type != OCCUPIED && newAccumMap[id].pointNum)
            {
                newAccumMap[id].type = CLEAR;
            }
        }
    }
    //following codes are for debugging
#ifdef DEBUG
    if(frameProcessedNum == 300){
        std::ofstream pointStatics("pointStatics.txt");
        for(unsigned short x = 0; x < accumMapRange.maxX; ++x)
        {
            for(unsigned short y = 0; y < accumMapRange.maxY; ++y)
            {
                unsigned short dynamX, dynamY;

                if(accumMapRange.translate(x, y, dynamicMapRange, dynamX, dynamY))
                {
                    int id = y * accumMapRange.maxX + x;
                    pointStatics << /*std::*/to_string(accumMap.at(id).HitCount) << '\t'
                                 << /*std::*/to_string(accumMap.at(id).pointNum) << std::endl;
                }
            }
        }
        pointStatics.close();
    }
#endif
    //end debug codes
    accumMap = newAccumMap;
    accumMapRange = dynamicMapRange;
    return true;
}

#ifdef OFFLINE
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
    unsigned short localX, localY;
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

    //need expanding
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
    cv::Mat newLocalMap(localMapRange.maxY, localMapRange.maxX, CV_8UC1, cv::Scalar(127));
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
            if(region.at<unsigned char>(region.rows - y -1, x) ==OCCUPIED)
                continue;
            int id = y * dynamicMapRange.maxX + x;
            if(accumMap.at(id).p > 0.5 || accumMap.at(id).type == OCCUPIED)
            {
                writeOnMat(region, x, y, OCCUPIED);
            }
            else if(accumMap.at(id).p < 0.5 || accumMap.at(id).type == CLEAR)
            {
                writeOnMat(region, x, y, CLEAR);
            }
//            else
//            {
//                writeOnMat(region, x, y, UNKNOWN);
//            }
        }
    }
    return true;
}
#endif

bool HdlEngine::writeOnMat(cv::Mat mat, int x, int y, unsigned char value)
{
    mat.at<unsigned char>(mat.rows - y -1, x) = value;
    return true;
}

Point3B HdlEngine::get3b(unsigned short xx, unsigned short yy, MapType type)
{
    Point3B point;
    unsigned char value;
    //for dynamic map and accumulated map, we use vector access. For local map, cv::Mat access method is used
    switch (type) {
    case DYNAMICMAP:
        value = dynamicMap.at( yy * dynamicMapRange.maxX + xx ).type;
        break;
    case ACCUMMAP:
        value = accumMap.at( yy * accumMapRange.maxX + xx ).type;
        break;
#ifdef OFFLINE
    case LOCALMAP:
        value = localMap.at<unsigned char>( localMap.rows - yy - 1, xx);
        break;
#endif
    default:
        DLOG(FATAL) << "Wrong map type: " << type
                    << ".(inside get3b()) Please check out defines.h (enum MapType section) to see valid types.";
        break;
    }

    switch (value) {
    case LANELINE:
        point.base |= (ROADEDGE_CLEAR | LANELINE_DOTTED);
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
    case UNKNOWN:
        point.base |= ROADEDGE_UNKNOWN;
        break;
    case CLEAR:
        point.base |= ROADEDGE_CLEAR;
        break;
    case OCCUPIED:
        point.base |= ROADEDGE_OCCUPIED;
        break;
    default:
        point.base |= ROADEDGE_UNKNOWN;
        break;
    }
//    if(point.base&ROADEDGE_CLEAR == ROADEDGE_CLEAR){
//        DLOG(INFO)<<"CLEAR POINT: ("<<xx<<", "<<yy<<")";
//    }
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
#ifdef OFFLINE
    case LOCALMAP:
    {
        cv::Mat img_l(localMap.rows, localMap.cols, CV_8UC3);
        for(int x = 0; x < img_l.cols; ++x)
        {
            for(int y = 0; y < img_l.rows; ++y)
            {
                Point3B pt = get3b(x, y, LOCALMAP);
                img_l.at<cv::Vec3b>(img_l.rows - y - 1, x) = cv::Vec3b(pt.base, pt.road, pt.sig);
            }
        }
        cv::imwrite(fileName, img_l);
        osHeader << (localMapRange.right + localMapRange.left) / 2 << '\t'
                 << (localMapRange.top + localMapRange.bottom) / 2
                 << std::endl
                 << localMapRange.maxX << '\t'
                 << localMapRange.maxY;
        break;
    }
#endif
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

bool HdlEngine::readPointsFromFile()
{
    //Firstly, read num of points in current frame.
    if(!hdlInstream.read((char*)&totalPointsNum, sizeof(totalPointsNum))){
        //because DLOG(FATAL) will terminate the program immediately. It is no good for timing the program. so
        //they are commented out during development.
//        DLOG(FATAL)  << "Error reading HDL file: " << baseFileName + ".hdl" << "\nReading total points number "
//                                                                     "of frame: " << frameProcessed;
        return false;
    }
    //Secondly, read all points into container of raw HDL points
    if(!hdlInstream.read((char*)rawHdlPoints, sizeof(RawHdlPoint)*totalPointsNum)){
//        DLOG(FATAL)  << "Error reading HDL file: " << baseFileName + ".hdl" << "\nReading the" <<" points "
//                                                                         "of frame: " << frameProcessed <<" error.";
        return false;
    }
    //Thirdly, read in the carpos of current frame
    carposeInstream >> currentPose.x >> currentPose.y >> currentPose.eulr;
    carposes.push_back(currentPose);
    return true;
}

bool HdlEngine::readPointsFromShm()
{
    //TODO: codes to read points from shared memory
#ifdef DEBUG
    return false;
#endif
    return true;
}


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
        float r_measure = distance * 2.0f;//why multiplying 2 is unclear for Zou
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
            dynamicMap.at(i).type = OCCUPIED;
            dynamicMap.at(i).HitCount = 1;
#ifdef MOREDETAILS
            if(dynamicMap.at(i).p > 0.5)
                dynamicMap.at(i).type = OCCUPIED;
#endif
        }
    }//end for(sizt_t i)

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
