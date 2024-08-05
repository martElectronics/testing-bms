#ifndef __Timer
#define __Timer
#include <Arduino.h>

class Timer
{
private:
    unsigned long int tAux, PT, ET;
    int stepindex;

public:
    Timer()
    {
        stepindex = 0;
    }

    bool update(bool start, unsigned long int PT, unsigned long int &ET)
    {
        bool act = false;
        switch (stepindex)
        {
        case 0:
            ET = 0;
            if (start)
            {
                stepindex += 10;
                tAux = millis();
            }

            break;

        case 10:
            if (!start)
                stepindex = 0;
            else
            {
                ET = millis() - tAux;
                if (ET >= PT)
                {
                    stepindex += 10;
                }
            }
            break;

        case 20:
            if (!start)
                stepindex = 0;
            ET = PT;
            act = true;
            break;

        default:
            break;
        }
        return act;
    }

    bool updateCyclic(bool start, unsigned long int PT)
    {
        bool act = false;
        switch (stepindex)
        {
        case 0:
            ET = 0;
            if (start)
            {
                stepindex += 10;
                tAux = millis();
            }

            break;

        case 10:
            if (!start)
                stepindex = 0;
            else
            {
                ET = millis() - tAux;
                if (ET >= PT)
                {
                    stepindex += 10;
                }
            }
            break;

        case 20:
            
            stepindex =0;
            ET = PT;
            act = true;
            break;

        default:
            break;
        }
        return act;
    }
};

#endif