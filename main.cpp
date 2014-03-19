#include <iostream>

#include "../include/Engine.h"
#include "../include/Analyser.h"

int main()
{
    bool isEngine = true;
    if (isEngine) {
        Engine engine;
        engine.run();
        //engine.runSeries();
    } else {
        Analyser analyser;
        analyser.run();
    }
    return 0;
}
