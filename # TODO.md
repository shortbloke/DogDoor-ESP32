# To Do List

## Basics

1. Ensure stepper works as desired.
1. Ensure stepper stops whenever limit it hit. Does not go past.
1. Review if critical actions need to be protected during loop to ensure smooth stepper operation and response to sensors being read.
1. If door already opening, then don't read distance sensors.
1. Check acceleration
1. Add keep open and keep closed options controlled by toggle switch

## Refactoring

1. Evaluate moving WiFi checking status code to main.cpp or a file, as a different concern from the dog door operation
1. Reduce serial logging, with debug flag?
1. Ensure display is only updated when actually needed

## Enhancements

1. Log top position when changing.
1. Only log sensor detection first time it's within threshold, don't log when opening
1. Add MQTT Support to publish state and be able to be controlled
1. Create README.md
1. Create tests?
1. Maybe move to hall effect sensors
1. Create new enclosure to support display, maybe just lid. Though box looks big now.
