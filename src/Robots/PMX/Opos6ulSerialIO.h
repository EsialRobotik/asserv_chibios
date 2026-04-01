#ifndef SRC_ROBOTS_PMX_OPOS6ULSERIALIO_H_
#define SRC_ROBOTS_PMX_OPOS6ULSERIALIO_H_

#include "Communication/SerialIO.h"

// SerialIO avec handshake "###" au démarrage.
// Empêche les commandes parasites quand l'OPOS6UL n'est pas connectée
// (bruit sur RX interprété comme des commandes z/s/q/d...).
class Opos6ulSerialIO : public SerialIO
{
public:
    using SerialIO::SerialIO;
    void commandInput() override;
};

#endif /* SRC_ROBOTS_PMX_OPOS6ULSERIALIO_H_ */
