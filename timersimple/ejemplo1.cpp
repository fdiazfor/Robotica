#include "ejemplo1.h"


ejemplo1::ejemplo1(): Ui_Counter()
{
    start = time(NULL);
    ctime(&start);
	setupUi(this);
	show();
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()));
	connect(GetTime, SIGNAL(clicked()), this, SLOT(showTime()));
	//connect(GetPeriod, SIGNAL(clicked()), this, SLOT(showPeriod()));
	//connect(SetPeriod, SIGNAL(clicked()), this, SLOT(changePeriod()));
	connect(setPeriod, SIGNAL(valueChanged(int)), this, SLOT(changePeriod(int)));

	mytimer.connect(std::bind(&ejemplo1::count, this));
    mytimer.start(500);    
}

ejemplo1::~ejemplo1()
{}

void ejemplo1::doButton()
{
	stopped = !stopped;
	if(stopped)
		mytimer.stop();
	else
		mytimer.start(mytimer.getPeriod());
	qDebug() << "click on button";
}

time_t ejemplo1::getTime()
{
    time_t currentTime = time(NULL);
    ctime(&currentTime);
    return (currentTime - start);
}

void ejemplo1::count()
{
    lcdNumber->display(++cont);
    trick++;
}

void ejemplo1::showTime()
{
    qDebug() << "Since the app was started" << getTime() << "seconds has passed";
}

void ejemplo1::showPeriod()
{
    qDebug() << "The current period is" << mytimer.getPeriod() << "miliseconds";
}

void ejemplo1::changePeriod(int nPeriod) {
    /*
    if (!stopped){
        qDebug() << "The timer should be stopped before changing the period!";
    }else {
        qDebug() << "Introduce the period you would like to set (miliseconds):";
        QTextStream in(stdin);
        QString data;
        in >> data;
        mytimer.setPeriod(data.toInt());
    }
     */
    mytimer.setPeriod(nPeriod);
}

