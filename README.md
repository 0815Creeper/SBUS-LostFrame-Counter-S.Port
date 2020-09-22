# SBUS-LostFrame-Counter-S.Port
(Deutsch unten, German below)

This Code is based on the openXsensor project. See https://github.com/openXsensor/openXsensor

SBUS LostFrame Counter, that sends its information via FrSky S.Port Telemetry. based on OpenXsensor

for wiring see circuit.png

When running Sensordiscovery 2 Fuel sensors will be found. Rename them as follows: 
The one with subID 0601 as "LF" (accumulated lost SBUS-frames since power-on). Set unit to none "-". 
The one with subID 0600 as "LF3s" (0-100%: percentege of lost SBUS-Frames within the last 3 seconds. (Under good reception conditions typically between 1 and 10%)
Hint: when the SBUS Frames are lost during a loss of signal, there obviosly is no telemetry-link either. Therefore monitor the accumulated LF: If it increases abruptly (>100 frames), there was something very wrong. (1 frame equals 9ms, therefore 111 frames equal 1 second)



# Eine ausführliche Schritt-für-Schritt Anleitung zur Herstellung des SBUS-LF-Zählers:
Die empfohlene Hardware:
- Servostecker weiblich (s.portkabel)
- Servostecker/Jumper-Pin-Kabel weiblich/männlich/V-Kabel je nach Bedarf (sbus kabel)
- Arduino Nano (bzw. ein klon aus fernost) 16MHz, 5V, Atmega328
- NPN Transistor (um den SBUS zu invertieren) [ich habe z.B. einen c1815 verwendet, weil der gerade da lag]
- 10kOhm Widerstand (Basiswiderstand für den NPN)
- 10kOhm Widerstand (Pullupwiderstand für den Seriellen -SBUS- Eingang am Arduino)
- evtl dünne Kabel um den Transistor mit dem Arduino zu verlöten

Dann ein paar Grundinfos zur Hardware:
Der Arduino hat einen Linearspannungswandler an bord, der eine Eingansspannung am VIN/RAW Pin (beschriftung varriert) von 7-12V in stabiele 5V umwandelt. Diese 5V gehen an den Prozessor und an zwei 5V-Pins einer ist als solcher beschriftet. Nun sind linearwandler dafür bekannt, immer eine höhere Eingans- als Ausgangsspannung zu brauchen. Meine persönlichen Erfahrungen sagen, dass man den Wandler trotzdem problemlos, statt mit 7V, auch nur mit 4,5V betereiben kann (also direkt empfängerspannung mit 4 leeren NimH), aber das muss man einfach probieren. Speißt man die Versorgungsspannung nicht via VIN/RAW, sondern direkt via USB oder 5V/VCC pin ein, führen Spannungen von >5,5V über lange Sicht auf jeden Fall zum Tod des Prozessors.

Dann zur Verkabelung (Schaltplan siehe circuit.png):
s.portkabel minus geht an GND des Arduino
s.portkabel plus geht an VIN/RAW (oder 5V für alle, die den empfänger IMMER mit weniger als 5,5V betreiben (4 volle NimH können mehr als 5,5V haben!!!)
s.portkabel Signal geht an Pin D2 oder D4 des Arduino (eigendlich egal, muss anschließend nur in der Software festgelegt werden)
SBUS Signal geht an einen 10kOhm Widerstand, dieser geht auf der anderen Seite dann an die Basis des NPN-Transistors (Vgl das Datenblatt des verwedeten NPNs)
Der Emitter des Transistors geht an GND des Arduinos
Der RX des Arduino wird über einen 10kOhm Widerstand mit VCC/5V verbunden (NICHT VIN/RAW !!!!)
Der Collector des Transistors geht an RX des Arduinos

Glückwunsch, löten abgeschlossen.

Nun zur Software (für Anfänger):
Zunächst brauchen wir die Arduino IDE. Das richtige Bertiebssytem suchen und die software mit allen Treibern installieren.

Dann den Code aus diesem Repository als ZIP herunterladen. (grüner Button "Clone or download")

Nun den openXsensor ordner entpacken (der ordner darf nicht umbenannt werden).

Im Odner interessieren wir uns jetzt für eine Datei: oXs_config_advanced.h 
diese in einem Texteditor öffnen. (ich nutze gerne notepad++ aber da geht alles beliebige. Der windowsinterne Editor hat allerdings Porbleme mit den Zeilenubrüchen, also lieber einen anderen verwenden...)
Zum Verständnis: alles mit "//" ist Kommentar und wird vom Computer ignoriert.
wir interessieren uns jetzt für "#define PIN_SERIALTX" und passen es an: 2 oder 4 ist erlaubt, je nachdem an welchen Pin man das S.Port Signalkabel gelötet hat.
Datei speichern!!! (besonders bei Notepad++ vergisst man das gerne)

So, fast geschafft.

Nun die Arduino ide geöffnet.
Unter Datei -> Öffnen zum openXsensor ordner navigieren und die openXsensor.ino öffnen.
Unter Werkzeuge -> Board den Arduino Nano auswählen.
Unter Werkzeuge -> Prozessor den 328 auswählen. (möglicherweise auch old-bootloader)
Unter Werkzeuge -> Port schauen, ob welche verfügbar sind und falls ja: nummern merken, die sind es nicht!

Nun den Nano mit USB am PC anstecken. Mindestens eine LED geht sofort an. Falls nicht, sofort wieder abstecken und auf Hitzeentwicklung prüfen (ihr habt irgendwo nen Kurzschluss gebaut...)
Unter Werkzeuge -> Port nun den neu dazugekommenen Port auswählen und den blauen pfeil(2. von links) "Hochladen" drücken. Kurz warten.(NICHT mehrfach drücken) Nun steht unten entweder in blau "Hochladen
abgeschlossen" oder in orange "Problem..." falls ersteres, Glückwunsch, falls letzteres, versucht mal unter Werkzeuge -> Prozessor den 328 mit/ohne old-bootloader.

SPort und SBUS kabel an den Empfänger und Sensordiscovery am Sender ausführen. 
2 Fuel Sensoren werden gefunden. Ich schlage vor sie umzubennen. 
Jender, der mit SubID 0601 zu "LF" (hier kommen die Gesamtzahl der Lost-SBUS-Frames seit Einschalten an). Hier die Einheit auf none "-" ändern. 
Den  mit SubID 0600 zu LF3s (hier kommen in 0-100% die in den letzten 3s verlorenen Frames an. (Bei guter Antennenverlegung ohne Störquellen typisch sind hier Werte zwischen 1 und 10% je nach Entfernung)
Hinweis: sollten bei einem Signalverlust SBUS Frames verloren gehen, kommt natürlich auch keine Telemetrie beim Sender mehr an beobachtet also unbedingt auch den gesamt LF-Wert. (Insb. bei Log-Auswertungen) Nimmt dieser schlagartig zu (>100 Frames) ist was sehr im Argen. (1 Frame entspricht 9ms, damit entsprechen 111 Frames einer Sekunde)
