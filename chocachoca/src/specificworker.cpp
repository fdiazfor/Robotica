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
	this->rotando = false;
	this->bloqueado = false;
	this->rotaciones = 0;
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
        float target = 0;
        switch (this->est){
            case Estado::avanzar:
                this->avanzar(threshold, ldata, ti, tj);
                break;
            case Estado::pared:
                this->pared(threshold, ldata, ti, tj);
                break;
            case Estado::rotar:
                this->rotar(threshold, ldata, i, j, alpha, target);
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
    return (int)result/50;
}

bool SpecificWorker::siguienteOcupada(int i, int j, float sum) {
    int si, sj;
    float alpha;
    this->differentialrobot_proxy->getBasePose(si, sj, alpha);
    this->sumAng(alpha, sum);
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
    if(sj > 99 || sj < 0 || si > 99 || si < 0)
        return true;

    return this->pos[si][sj];
}

void SpecificWorker::avanzar(float threshold, RoboCompLaser::TLaserData ldata, int i, int j) {
    std::cout <<"________avanzar_______"<< std::endl;
    if (threshold > ldata.front().dist ){
        this->est = Estado::pared;
        differentialrobot_proxy->setSpeedBase(0, 0);
        return;
    } else if(siguienteOcupada(i, j, 0) && !this->bloqueado){
        this->est = Estado::rotar;
        differentialrobot_proxy->setSpeedBase(0, 0);
        return;
    }
    differentialrobot_proxy->setSpeedBase(1000, 0);
    this->rotaciones = 0;
    this->rotando = false;
    this->bloqueado = false;
}

void SpecificWorker::pared(float threshold,  RoboCompLaser::TLaserData ldata , int i, int j) {
    std::cout <<"________pared_______"<< std::endl;
    float rot = 2;
    if (threshold < ldata.front().dist){
        differentialrobot_proxy->setSpeedBase(0, 0);
        this->est = Estado::avanzar;
        return;
    }
    std::cout << ldata.front().dist << std::endl;
    differentialrobot_proxy->setSpeedBase(5, rot);
}

void SpecificWorker::rotar(float threshold,  RoboCompLaser::TLaserData ldata, int i, int j, float alpha, float &target) {
    std::cout <<"________rotar_______"<<"Rotando "<<this->rotando<<"Bloqueado "<<this->bloqueado<< std::endl;
    if (threshold > ldata.front().dist){
        std::cout <<"________rotar1_______"<< std::endl;
        differentialrobot_proxy->setSpeedBase(0, 0);
        this->est = Estado::pared;
        return;
    }else if(abs((alpha- target)) < 0.01 ){
        std::cout <<"________rotar2_______"<< std::endl;
        differentialrobot_proxy->setSpeedBase(0, 0);
        this->est = Estado::avanzar;
        this->rotando = false;
        return;
    }
    if(!this->rotando){
        target = anguloObjetivo(i, j);
        std::cout <<"________rotar3_______"<< std::endl;
        this->rotando = true;
        this->rotaciones++;
        if(target == 0 || rotaciones > 1) {
            this->bloqueado = true;
            this->rotando = false;
            std::cout <<"________bloqueado_______"<< std::endl;
            this->est = Estado::avanzar;
        }
    }
    differentialrobot_proxy->setSpeedBase(5, 2);
}

float SpecificWorker::sumAng(float ang, float add) {
    if ((6.28319 - ang) > add)
        return ang + add;
    else
        return add - (6.28319 - ang);
}

float SpecificWorker::anguloObjetivo(int i, int j) {
    float ang = 0.523599;
    while ( ang < 6.28319){
        if(!this->siguienteOcupada(i, j, ang)){
            return ang;
        }
        ang = ang + 0.523599;
    }
    return 0;
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

