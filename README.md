# :robot: OSCAR
Este repositorio se divide fundamentalmente en:
## [actions](actions)
Incluye todos los archivos referentes a "acciones" que hará el robot, desde moverse de un punto a otro hasta subir o bajar la garra. En esta carpeta encontraremos los siguientes archivos:
### [ball.py](actions/ball.py)
Aquí se encuentran todas las funciones que toman lugar a la hora de encontrar y agarrar la pelota (excepto las referentes a la identificación de la misma, que se encuentran en el archivo [vision.py](#vision.py))
### [map.py](map.py)
Todo lo referente al movimiento dentro del mapa se encuentra aquí: tanto el desplazamiento entre baldosas como el recorrido total.
### [moves.py](moves.py)
Los movimientos que no necesariamente están relacionados al recorrido de un mapa se pueden ver en este archivo. Funciones como `run`, `spin` o `arc` se hayan aquí, aunque también otras más complejas como `slalom`, `eight` u otras.
## [helpers](helpers)
Todas las funciones de ayuda las encontraremos aquí. Viene a ser una carpeta donde se guarda toda la lógica del proyecto.
### [location.py](location.py)
Incluye las funciones referentes al posicionamiento y localización del robot con su entorno: cálculos que devuelven el cuadrante al que está apuntando el robot, su distancia a otro punto, etc.
### [map.py](map.py)
Se declaran todas las funciones que hagan referencia al mapa y su recorrido, desde funciones que realizan el cambio de base entre referencias hasta funciones que calculan el grid y las mejores "siguientes celdas" hacia las que avanzar.
### [maths.py](maths.py)
