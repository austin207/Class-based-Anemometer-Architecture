#ifndef ROM_H
#define ROM_H

#include <Preferences.h>
#include "SystemVars.h"

class ROM {
public:
    static void initialize();
    static void update();
private:
    static Preferences preferences;
};

#endif // ROM_H
