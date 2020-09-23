#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include "ui_counterDlg.h"
#include <chrono>
#include "timer.h"
#include <time.h>
#include <stdio.h>


class ejemplo1 : public QWidget, public Ui_Counter
{
Q_OBJECT
    public:
        ejemplo1();
        virtual ~ejemplo1();
    
    public slots:
        //Stop the timer shown on the qt screen
	    void doButton();
        //Shows the time since the application were initialized throw qDebug
        void showTime();
        //Shows the timer's period throw qDebug
        void showPeriod();
        //Make the user able to change the timer's period
        void changePeriod(int nPeriod);

        
    private:
        Timer mytimer, mytimerLong;
        int cont = 0;
        bool stopped = false;
		
		// dos callbacks con diferente número de parámetros
        void count();
        time_t getTime();

		int trick = 5;
		time_t start;
};

#endif // ejemplo1_H
