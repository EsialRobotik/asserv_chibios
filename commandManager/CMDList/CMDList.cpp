#include "CMDList.h"

//GDI
//CMDList::CMDList(int capacity)
CMDList::CMDList()
{
    nextFreePos = 0;
    headPos = 0;
    full = false;
}

CMDList::~CMDList()
{
}

bool CMDList::enqueue(typeCMD cmd , float val, float val2)
{
    if (full) {
        return false;
    } else {
        list[nextFreePos].type = cmd;
        list[nextFreePos].value = val;
        list[nextFreePos].secValue = val2;
        nextFreePos = (nextFreePos + 1) % CAPACITY;

        if (nextFreePos == headPos) {
            full = true;
        }

        return true;
    }
}


int CMDList::size()
{
    if (full) {
        return CAPACITY;
    }

    return nextFreePos - headPos;
}


CMD CMDList::dequeue()
{
    CMD res;
    res.type = CMD_NULL;

    if (!full && nextFreePos == headPos) {
        return res; // List if empty !
    }

    if (nextFreePos == headPos) {
        full = false;
    }

    int returnIndex = headPos;
    //GDI
    headPos = (headPos + 1) % CAPACITY;
    //headPos = (headPos + 1) % capa;

    res.type = list[returnIndex].type;
    res.value = list[returnIndex].value;
    res.secValue = list[returnIndex].secValue;
    return res;
}
