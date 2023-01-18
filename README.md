# :robot: OSCAR
Este repositorio se divide fundamentalmente en:
## [actions](actions)
Incluye todos los archivos referentes a "acciones" que hará el robot, desde moverse de un punto a otro hasta subir o bajar la garra. En esta carpeta encontraremos los siguientes archivos:
### [ball.py](actions/ball.py)
Aquí se encuentran todas las funciones que toman lugar a la hora de encontrar y agarrar la pelota (excepto las referentes a la identificación de la misma, que se encuentran en el archivo [vision.py](#vision.py))
### [map.py](acions/map.py)
Todo lo referente al movimiento dentro del mapa se encuentra aquí: tanto el desplazamiento entre baldosas como el recorrido total.
### [moves.py](actions/moves.py)
Los movimientos que no necesariamente están relacionados al recorrido de un mapa se pueden ver en este archivo. Funciones como `run`, `spin` o `arc` se hayan aquí, aunque también otras más complejas como `slalom`, `eight` u otras.
## [helpers](helpers)
Todas las funciones de ayuda las encontraremos aquí. Viene a ser una carpeta donde se guarda toda la lógica del proyecto.
### [location.py](helpers/location.py)
Incluye las funciones referentes al posicionamiento y localización del robot con su entorno: cálculos que devuelven el cuadrante al que está apuntando el robot, su distancia a otro punto, etc.
### [map.py](helpers/map.py)
Se declaran todas las funciones que hagan referencia al mapa y su recorrido, desde funciones que realizan el cambio de base entre referencias hasta funciones que calculan el grid y las mejores "siguientes celdas" hacia las que avanzar.
### [maths.py](helpers/maths.py)
Incluye las funciones que sean meras operaciones matemáticas, esto incluye normalizaciones, obtención de signos, geometría, etc,
### [plot.py](helpers/plot.py)
Aquí se encuentran todas las funciones que tienen como objetivo generar el plot del recorrido realizado por el robot.
### [vision.py](helpers/vision.py)
Este archivo incluye las funciones dirigidas al reconocimiento de objetos mediante visión por computador.
## [libs](libs)
Aquí se guardan las librerias externas necesarias para le ejecución de los programas. Únicamente encontramos [brickpi.py](libs/brickpi.py) puesto que el resto de ellas se instalaron con `pip`.
## [logs](logs)
En esta carpeta encontramos dos subcarpetas diferentes: [log](logs/log) y [odometry](logs/odometry), ambos con el nombre correspondiente a la fecha y hora de la ejecución del robot que representan.
La carpeta [log](logs/log) incluye los registros de ejecución del programa (mostrados por pantalla durante la misma), mientras que la carpeta [odometry](logs/odometry) incluye archivos `.csv` con la posición del robot en cada momento (cada actualización de odometría) que se pueden emplear para realizar un plot con el recorrido realizado.
## [res](res)
Esta carpeta incluye los recursos necesarios para el robot, ya sean imágenes o mapas.
## [tests](tests)
Aquí es donde se guardan los archivos empleados como prueba para la calibraci
## [Robot.py](Robot.py)
Este archivo alberga la clase Robot y todos sus atributos y funciones. Desde ella se gestionan todas las funciones directamente relacionadas con el robot, ya sea la lectura de sensores, posicionamiento de actuadores, modificación de los valores de odometría...
## [main.py](main.py)
Este es el archivo que el usuario ejecuta directamente, incluye una serie de funciones que se darán dependiendo del input que se escriba en los argumentos.
