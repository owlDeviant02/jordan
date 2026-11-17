# Jordan — robot de competición VEX U *High Stakes*

Programa del robot **Jordan**, desarrollado por el equipo **UCSA Prime** de la Universidad del Cono Sur de las Américas (Asunción, Paraguay) para el desafío VEX U *High Stakes*, temporada 2024-2025. Jordan es el robot de 24″ del equipo y compitió en el VEX Robotics World Championship 2025.

Este repositorio contiene el **programa de partido**: la rutina autónoma de 30 segundos y el control manual de 1:30. **Un mismo programa sirve para las dos alianzas, azul y roja**: se elige una u otra cambiando una sola línea (ver [Cómo elegir la alianza](#cómo-elegir-la-alianza)).

---

## Contenido

- [Características](#características)
- [Plataforma](#plataforma)
- [Estructura del repositorio](#estructura-del-repositorio)
- [Cómo elegir la alianza](#cómo-elegir-la-alianza)
- [Compilar y cargar el programa](#compilar-y-cargar-el-programa)
- [Configuración del robot (puertos)](#configuración-del-robot-puertos)
- [Controles del piloto](#controles-del-piloto)
- [Rutina autónoma](#rutina-autónoma)
- [Clasificación de aros por color](#clasificación-de-aros-por-color)
- [Odometría y control PID](#odometría-y-control-pid)
- [Modo Jordan y brazo Fishmech](#modo-jordan-y-brazo-fishmech)
- [Notas](#notas)

---

## Características

- **Tracción híbrida de seis motores**: cuatro ruedas omnidireccionales en las esquinas y dos de tracción en el centro.
- **Odometría** con dos ruedas de seguimiento y el sensor inercial, a través de LemLib.
- **Dos lazos PID** (avance y giro), con ganancias ajustadas sobre el robot.
- **Clasificación de aros por color** con el sensor de visión: una compuerta neumática descarta los aros de la alianza rival.
- **Sujeción neumática** de la estaca móvil.
- **Brazo Fishmech** para anotar en las estacas de pared y de alianza, con automatización mediante un final de carrera y un sensor de línea.

## Plataforma

| Componente | Versión |
|---|---|
| Cerebro | VEX V5 |
| Sistema operativo del robot | [PROS](https://pros.cs.purdue.edu/) 4.1.2 |
| Biblioteca de odometría y control | [LemLib](https://github.com/LemLib/LemLib) 0.5.5 |
| Lenguaje | C++ |

## Estructura del repositorio

```
├── src/
│   └── main.cpp        ← todo el programa del robot, comentado por secciones
├── include/            ← cabeceras de PROS, LemLib y LVGL (no se editan)
├── firmware/           ← bibliotecas precompiladas de PROS y LemLib (no se editan)
├── Makefile, common.mk ← reglas de compilación de PROS
└── project.pros        ← configuración del proyecto PROS
```

`src/main.cpp` está dividido en seis secciones, en este orden:

1. **Selección de la alianza**: la única línea que hay que cambiar entre partidos.
2. **Chasis, odometría y control PID**: motores de tracción, ruedas de seguimiento, IMU y LemLib.
3. **Mecanismos, sensores y neumática**: intake, cadena, Fishmech, sensores y electroválvulas.
4. **Funciones auxiliares**: clasificación por color, cadena hasta el final de carrera, reinicio del Fishmech y movimientos simples.
5. **Rutina autónoma**: una para cada alianza.
6. **Control manual**.

## Cómo elegir la alianza

Al principio de `src/main.cpp` está esta línea:

```cpp
constexpr Alianza ALIANZA = Alianza::AZUL; // ← CAMBIAR AQUÍ: Alianza::AZUL o Alianza::ROJA
```

| Valor | Aros que conserva | Aros que descarta | Rutina autónoma |
|---|---|---|---|
| `Alianza::AZUL` | azules | rojos | lado azul |
| `Alianza::ROJA` | rojos | azules | lado rojo |

No hace falta cambiar nada más: a partir de esa constante, el programa elige solo el color que se descarta (`COLOR_RIVAL`), la rutina autónoma y la velocidad de la cadena en el autónomo. Después de cambiarla, hay que **compilar y cargar** el programa de nuevo.

> **Sugerencia:** para no tener que recompilar en la competencia, se puede compilar una vez con cada alianza y cargar cada versión en una ranura distinta del cerebro (por ejemplo, la 1 para azul y la 2 para roja), cambiando `"slot"` en `project.pros` o usando `pros upload --slot N`.

## Compilar y cargar el programa

Requisitos: la [extensión de PROS para Visual Studio Code](https://pros.cs.purdue.edu/v5/getting-started/index.html) o PROS CLI.

```bash
pros make      # compila el programa
pros upload    # lo carga en el cerebro V5, conectado por USB o por el control
```

Desde Visual Studio Code también pueden usarse los botones **Build** y **Upload** de la extensión de PROS.

> **Importante:** antes de iniciar el programa, el robot debe estar **quieto**, porque `initialize()` calibra el sensor inercial durante unos segundos.

## Configuración del robot (puertos)

### Motores (puertos inteligentes)

| Elemento | Puertos | Notas |
|---|---|---|
| Tracción izquierda | 11, 12, 13 | cartucho azul (600 RPM); 11 y 13 invertidos |
| Tracción derecha | 14, 15, 16 | cartucho azul (600 RPM); 15 invertido |
| Intake | 5, 7 | |
| Cadena | 3, 8 | 8 invertido |
| Brazo Fishmech | 2 | invertido; freno en modo *hold* |

Un número negativo en el código (por ejemplo, `-11`) indica que ese motor gira en sentido inverso.

### Sensores

| Sensor | Puerto | Función |
|---|---|---|
| Sensor inercial (IMU) | 21 | rumbo del robot |
| Sensor de visión | 17 | color de los aros (firma 1: azul; firma 2: rojo) |
| Encoder de la rueda de seguimiento vertical | ADI E-F | odometría (avance) |
| Encoder de la rueda de seguimiento horizontal | ADI G-H | odometría (desplazamiento lateral) |
| Sensor de línea | ADI A | posición cero del Fishmech |
| Final de carrera | ADI D | detecta el aro arriba de la cadena |

### Neumática

| Actuador | Puerto | Función |
|---|---|---|
| Pinza trasera (dos cilindros) | ADI B | sujeta la estaca móvil |
| Compuerta de clasificación (un cilindro) | ADI C | descarta los aros de la alianza rival |

## Controles del piloto

| Mando | Acción |
|---|---|
| Joystick izquierdo (vertical) | avanzar / retroceder |
| Joystick derecho (horizontal) | girar (a media sensibilidad) |
| **R1** | recoger: intake y cadena hacia arriba, con clasificación por color |
| **R2** | expulsar: intake y cadena en reversa |
| **B** | cerrar la pinza sobre la estaca móvil |
| **Y** | abrir la pinza |
| **X** | activar el modo Jordan (ver más abajo) |
| **A** | volver al modo normal |
| **L1** | subir el Fishmech (anotar) |
| **L2** | bajar el Fishmech |
| **ABAJO** | retroceder un poco para quedar a distancia de anotar en la estaca de alianza |

Durante el control manual, la pantalla del cerebro muestra la pose estimada por la odometría (X, Y y ángulo), las lecturas de los dos encoders y el rumbo del IMU.

## Rutina autónoma

Las dos rutinas, la azul y la roja, siguen la misma secuencia, reflejada para cada lado del campo y con tiempos ajustados en la práctica:

1. El robot retrocede hasta la **estaca móvil** y la sujeta con la pinza trasera.
2. Enciende el intake y la cadena, y sube un aro hasta el **final de carrera**.
3. Se apoya en la **estaca de alianza** y anota ese aro con el **Fishmech**.
4. Devuelve el Fishmech a su posición cero con el **sensor de línea**, mientras se mueve de lado a lado para soltarse de la estaca.
5. Recoge más aros del campo y de la **esquina** y los anota en la estaca móvil.

Durante todo el autónomo, la clasificación por color corre en segundo plano (`colorSorting()`).

Los **giros** usan el PID angular de LemLib (`turnToHeading` y `swingToHeading`); los **traslados** son a lazo abierto, por tiempo y voltaje (`moveRobot`).

## Clasificación de aros por color

El sensor de visión tiene cargadas dos firmas de color: **1 = aro azul** y **2 = aro rojo**. En cada lectura, `actualizarCompuerta()`:

1. Toma el objeto más grande de cada firma.
2. Lo considera cercano si su ancho supera `ANCHO_MINIMO_ARO` (100 unidades del sensor).
3. Si es del **color rival**, extiende el pistón de la compuerta y el aro sale del robot; si es del **color propio**, lo retrae y el aro sigue hacia la estaca.

El pistón queda en esa posición hasta que aparece un aro del otro color.

El color rival (`COLOR_RIVAL`) se deduce de la constante `ALIANZA`. Si alguna vez se quisiera descartar otro color, basta con cambiar esa línea. Si cambian las condiciones de luz, las firmas pueden volver a generarse con la utilidad del sensor de visión de VEX y reemplazarse en `initialize()`.

## Odometría y control PID

- **Ruedas de seguimiento**: dos ruedas de 2,75″ que no reciben tracción, así que no patinan como las ruedas motrices. La vertical está a 5,5″ del centro de giro y la horizontal, a −2″.
- **Sensor inercial**: da el rumbo; se calibra en `initialize()`.
- **PID lateral** (avance): kP = 10, kI = 0, kD = 24.
- **PID angular** (giro): kP = 6, kI = 0, kD = 28.

Las ganancias se ajustaron a mano, por iteraciones sobre el robot.

## Modo Jordan y brazo Fishmech

El **modo Jordan** prepara un aro para anotarlo en una estaca de pared:

1. Con **X**, la cadena sube el aro.
2. Cuando el **final de carrera** lo detecta, la cadena se frena en modo *hold* y deja el aro en posición.
3. Con **L1**, el Fishmech lo toma y lo anota.

Al usar L1 o L2, el programa vuelve al modo normal.

El Fishmech usa el freno en modo *hold*: el motor mantiene activamente su posición, incluso con el aro cargado.

## Notas

- La tarea `colorSorting()` del autónomo consulta el sensor en un ciclo continuo, sin pausa.
- Antes de cada partido conviene recargar los depósitos de aire, porque la pinza y la compuerta consumen aire en cada actuación.
