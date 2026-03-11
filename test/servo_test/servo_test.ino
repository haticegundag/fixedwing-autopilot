#include <AtabeyAutopilot.h>

using namespace atabey::drivers;

ServoPWM<5,6> elevon;

void setup()
{
    elevon.init();
}

void loop()
{
    elevon.setPosition(0,0);
    delay(2000);

    elevon.setPosition(20,-20);
    delay(2000);

    elevon.setPosition(-20,20);
    delay(2000);
}