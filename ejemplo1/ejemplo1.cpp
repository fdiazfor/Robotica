#include "ejemplo1.h"

ejemplo1::ejemplo1(): Ui_Counter()
{
	setupUi(this);
	show();
	q.start(500);
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );
	connect( &q, SIGNAL(timeout()), this, SLOT( counter()) );
}

void ejemplo1::doButton()
{
	qDebug() << "click on button";
}

void ejemplo1::counter()
{
    qDebug() << "Hola";
}




