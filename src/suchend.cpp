#include "stdio.h"
#include "suchend.h"
#include "ausfuehrend.h"
#include "roboter.h"
#include "verabschiedend.h"

Suchend::Suchend(Bildanalysator_Proxy::ObjektTyp o,Koordinator* dlg)
    : Dialogzustand(dlg)
{
    //ctor
    printf("S U C H E N D\n");
    observerPos=0;
    m_suessigkeit = o;
    dlg->getBildAnalysator()->bildanalyseStarten(m_suessigkeit);
}

Suchend::~Suchend()
{
    //dtor

}

void Suchend::objektErkannt()
{
    dialog->setZustand(new Ausfuehrend(dialog));
    delete this;
    //dialog->getDialogzustand()->warte(30);
//    dialog->getDialogzustand()->warte(10);
}

void Suchend::objektNichtErkannt()
{
  Roboter* r=dialog->getRoboter();

  observerPos++;
  if(observerPos < r->getMaxObserverPos())
  {
    r->publish(observerPos);
    Timer* t = dialog->getTimer();
    t->stop();
    t->setStopZeit(10);
    t->start();
  }else
  {
    dialog->setZustand(new Verabschiedend(dialog,0));
    delete this;
  }
}

void Suchend::observerPosEingenommen()
{
  dialog->getBildAnalysator()->bildanalyseStarten(m_suessigkeit);
  Timer* t = dialog->getTimer();
  t->stop();
}

void Suchend::onTimer()
{
    std::cout << "5 Sekunden timeout von Roboter" << std::endl;
    dialog->setZustand(new Verabschiedend(dialog,0));
    delete this;
}

int Suchend::getSuessigkeit()
{
    return m_suessigkeit;
}


