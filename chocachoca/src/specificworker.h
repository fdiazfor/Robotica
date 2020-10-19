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

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

const int tam_tab = 100;
enum Estado{ avanzar, pared, rotar};

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);



public slots:
	void compute();
	int startup_check();
	void initialize(int period);
private:
	std::shared_ptr < InnerModel > innerModel;
	bool startup_check_flag;
	void avanzar(float treshold, RoboCompLaser::TLaserData ldata, int i, int j);
	void pared(float threshold,  RoboCompLaser::TLaserData ldata , int i, int j);
	void rotar(float threshold,  RoboCompLaser::TLaserData ldata, int i, int j, float alpha, float &target);
	float sumAng(float ang, float add);
	bool pos[tam_tab][tam_tab];
	bool siguienteOcupada(int i, int j, float sum);
	float anguloObjetivo(int i, int j);
	Estado est;
	bool rotando;
	bool bloqueado;
	int rotaciones;
	int k(int cord);

};

#endif
