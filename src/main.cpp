#include <Arduino.h>
#include "AppContainer.h"

static AppContainer app;

void setup()
{
  app.begin();
}

void loop()
{
  app.loop();
}
