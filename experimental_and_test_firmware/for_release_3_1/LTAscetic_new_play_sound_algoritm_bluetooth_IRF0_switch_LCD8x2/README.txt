In this version of the firmware, there are important changes and additions.

Firmware can work with both wired bandanna, so it is with wireless Bluetooth bandana, and at the same time with two bandanas (bluetooth and wired).
Anticheat controls both zones, therefore,

IMPORTANT !!!
If you plan to use only wired bandanna, it is necessary to conclude pin STATE on the Bluetooth-connector to connect through a resistor 270K to ground.
Otherwise anticheat will not work correctly!
If you are difficult to solder a resistor, there is another way - correct file isr.с : in function ISR(TIMER1_COMPA_vect) {...} get the line to read

if ((! (TSOP_IN & TSOP_PIN)) && (! (BT_STATE_IN & BT_STATE_PIN))) // if the input INT0 low level and there is no Bluetooth connection

and correct it so

if ((! (TSOP_IN & TSOP_PIN)) / * && (! (BT_STATE_IN & BT_STATE_PIN)) * /) // if the input INT0 low level

Rebuild the project. 
The new firmware file with the extension .hex will be in a subdirectory "defalt" of the project directory.



Rewritten algorithm play audio files.
In this firmware sound files are reproduced through the interruption, and not in the main program loop.
Therefore, the main loop of the program keeps track of game events at the time, when you hear the sound.
Therefore, it became possible to process all the hits, that happened during playback  "injured sound".
But to give the player a chance to survive when he is being fired gunfire, put such a thing as "time invulnerability."
During this time, after the hit, the player invulnerable, all hit ignored.
Duration of invulnerability is adjustable from 0.5 seconds to 3 seconds.

In the settings now made and the carrier frequency of the IR signal - 36 or 56 kHz.

You can also adjust the rate when shooting in auto mode.

Firmware understands the IR RC command "Start Game" and "kill the player" and commands to change the colors of the team.
You can use the remote control "ascet-pult".
http://github.com/PingvinOpenTag/LTAscetic/tree/master/Ascet_pult_3_1
But before you start using it you need to initialize it. 
Command to initialize on the menu.
