#ifndef CMD_LIST
#define CMD_LIST

#define CAPACITY 32


// Internal Structures
enum typeCMD {
    CMD_NULL, // Erreur....
    CMD_GO, // Avancer ou reculer d'une certaine distance
    CMD_TURN, // Tourner d'un certain angle
    CMD_WAIT, // Attendre un certain temps ('fin, je suppose, c'est pas implémenté...)
    CMD_STOP, // Bon, là, je sais pas...
    CMD_GOTO, // Aller à un point précis
    CMD_GOTOANGLE, // Se tourner vers un point précis
    CMD_GOTOENCHAIN // Aller vers un point précis, mais si la commande suivante est un GOTO ou
    // un GOTOENCHAIN, on s'autorise à ne pas s'arrêter au point de consigne
};

typedef struct CMD_struct {
    typeCMD type;
    float value;
    float secValue;
} CMD;

class CMDList
{
public:
    CMDList();
    CMDList(int capacity); //GDI
    bool enqueue(typeCMD cmd , float val, float val2 = 0);
    CMD dequeue();
    int size();
    ~CMDList();

private:
    CMD list[CAPACITY];
    int nextFreePos;
    int headPos;
    bool full;
};

#endif
