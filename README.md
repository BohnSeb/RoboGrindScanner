# RoboGrindScanner
Tools to control a mobile platform MiR-100 and an UR5e for 3d scan ob big objects.

## Warnung! Versionen checken!

## Todos/Fragen
- Passwörter für MiR100 Wlan und so (Sylvan Hausmann als Ansprechpartner)
- Welche Daten liefert der Artec3d (Punktwolke? Rohdaten?)
- Braucht Artec3d einen Lizenzschlüssel?
- ROS Steuerung der MiR100
- ROS Steuerung des UR (Nicht offizielles Repo benutzen, sondern vom FZI (Mauch oderso ist der Name vom maintainer))

## Stände
- MiR100 ist im Labor, eine Aluminium Platte muss montiert werden.
- Aluminiumplatte befindet sich im Labor, bei der Fitness-Laufmaschine
- Artec3d Sensor wird von Oliver im Labor deponiert. Er stellt den Karton unten an das mittlere Fenster.
- UR-Arme gibt es zwei im Labor, welchen wir nehmen steht noch nicht fest.

## Empfehlungen
- für UR-Simulation sollte RViz und MoveIt verwendet werden.
- View-Path-Planning am Anfang zweitrangig.
- Lieber eine unvollendete Lösung, die Ergebnisse liefert, als Lösung die Professionell ist und kein Ergebnis produziert.
- UR sollte per LAN an die MiR100 verbunden werden. Dann kann beides über den ROS-Master der MiR100 gesteuert werden.