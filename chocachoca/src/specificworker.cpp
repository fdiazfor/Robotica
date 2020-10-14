/*
 *    Copyright (C) 2020 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
	for(int i = 0 ; i < tam_tab ; i++)
	    for (int j = 0; j < tam_tab ; j++)
            this->pos[i][j] = false;
	this->est = Estado::avanzar;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }






	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
	}

}

void SpecificWorker::compute()
{
    const float threshold = 200; // millimeters
     // rads per second
    int i = 0, j = 0;
    float alpha = 0;
    RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
    std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });

    try
    {
        this->differentialrobot_proxy->getBasePose(i, j, alpha);
        int ti = k(i);
        int tj = k(j);
        this->pos[ti][tj] = true;

        switch (this->est){
            case Estado::avanzar:
                avanzar(threshold, ldata.front(), ti, tj);
                break;
            case Estado::pared:
// random wait between 1.5s and 0.1sec
                break;
        }
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

int SpecificWorker::k(int cord) {
    int result = cord + 2500;
    return (int)result/10;
}

bool SpecificWorker::siguienteOcupada(int i, int j) {
    int si, sj;
    float alpha;
    this->differentialrobot_proxy->getBasePose(si, sj, alpha);
    if( alpha < 0.5367 || alpha > 5.7727){
        sj = j + 1;
        si = i;
    } else if (alpha > 0.5367 && alpha < 1.0603){
        sj = j + 1;
        si = i + 1;
    } else if (alpha > 1.0603 && alpha < 2.1075){
        sj = j;
        si = i + 1;
    } else if (alpha > 2.1075 && alpha < 2.6311){
        sj = j - 1;
        si = i + 1;
    } else if (alpha > 2.6311 && alpha < 3.6783){
        sj = j - 1;
        si = i;
    } else if (alpha > 3.6783 && alpha < 4.2019){
        sj = j - 1;
        si = i - 1;
    } else if (alpha > 4.2019 && alpha < 5.2491){
        sj = j;
        si = i - 1;
    } else if (alpha > 5.2421 && alpha < 5.7727){
        sj = j + 1;
        si = i - 1;
    }

    return this->pos[si][sj];
}

void SpecificWorker::avanzar(float threshold, auto distActual, int i, int j) {
    //Condición de salida
    if (threshold > distActual){
        this->est = Estado::pared;
        differentialrobot_proxy->setSpeedBase(0, 0);
        return;
    } else if(siguienteOcupada(i, j)){
        this->est = Estado::rotar;
        differentialrobot_proxy->setSpeedBase(0, 0);
        return;
    }
    differentialrobot_proxy->setSpeedBase(1000, 0);
}

void SpecificWorker::pared(float threshold,  RoboCompLaser::TLaserData ldata , int i, int j) {
    float rot = 1.5708;
    if (threshold < ldata.front().dist){
        this->est = Estado::avanzar;
        return;
    }
    std::cout << ldata.front().dist << std::endl;
    differentialrobot_proxy->setSpeedBase(5, rot);
    usleep(rand()%(1500000-100000 + 1) + 100000);
}

void SpecificWorker::bloqueo() {

}




/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot_proxy->correctOdometer(...)
// this->differentialrobot_proxy->getBasePose(...)
// this->differentialrobot_proxy->getBaseState(...)
// this->differentialrobot_proxy->resetOdometer(...)
// this->differentialrobot_proxy->setOdometer(...)
// this->differentialrobot_proxy->setOdometerPose(...)
// this->differentialrobot_proxy->setSpeedBase(...)
// this->differentialrobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

