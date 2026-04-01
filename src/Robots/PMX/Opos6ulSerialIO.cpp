#include "ch.h"
#include "hal.h"
#include "Opos6ulSerialIO.h"

void Opos6ulSerialIO::commandInput()
{
    // Handshake : attente de "###" avant d'accepter les commandes.
    // Sans cela, le bruit sur RX (pin flottante quand l'OPOS6UL n'est pas
    // connectée) est interprété comme des commandes parasites (z/s/q/d...).
    // L'OPOS6UL doit envoyer "###" pour débloquer la réception.
    // Un seul '#' pourrait être généré par le bruit (0x23), mais 3 consécutifs
    // ont une probabilité de 1/16 millions — quasi impossible par hasard.
    {
        uint8_t count = 0;
        while (count < 3)
        {
            char c = streamGet(m_serialDriver);
            if (c == '#')
                count++;
            else
                count = 0;
        }
    }

    // Handshake reçu, fonctionnement normal
    SerialIO::commandInput();
}
