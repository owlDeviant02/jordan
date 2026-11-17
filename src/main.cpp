/**
 * ============================================================================
 *  JORDAN — Programa de partido (VEX U High Stakes, temporada 2024-2025)
 *  Equipo UCSA Prime · Universidad del Cono Sur de las Américas
 * ============================================================================
 *
 *  Este archivo contiene todo el programa del robot:
 *
 *    1. Selección de la alianza (azul o roja)        ← lo único que hay que
 *                                                       cambiar antes de cada
 *                                                       partido.
 *    2. Configuración del chasis, la odometría y los controladores PID (LemLib).
 *    3. Configuración de los mecanismos: intake, cadena, brazo Fishmech,
 *       sensores y neumática.
 *    4. Funciones auxiliares: clasificación por color, cadena hasta el final
 *       de carrera, reinicio del Fishmech y movimientos a lazo abierto.
 *    5. Rutina autónoma (30 s): una para cada alianza.
 *    6. Control manual del piloto (1:30).
 *
 *  Plataforma: PROS 4.1.2 + LemLib 0.5.5, cerebro VEX V5.
 * ============================================================================
 */

#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// ============================================================================
//  1. SELECCIÓN DE LA ALIANZA
// ============================================================================
//
//  El mismo programa sirve para las dos alianzas. Basta con cambiar la línea
//  ALIANZA de más abajo:
//
//    - Alianza::AZUL  → el robot conserva los aros AZULES y descarta los ROJOS,
//                       y ejecuta la rutina autónoma del lado azul.
//    - Alianza::ROJA  → el robot conserva los aros ROJOS y descarta los AZULES,
//                       y ejecuta la rutina autónoma del lado rojo.
//
//  No hace falta tocar nada más: el color que se descarta, la rutina autónoma
//  y la velocidad de la cadena del autónomo se eligen solos a partir de esta
//  constante. Después de cambiarla, compilar y cargar el programa de nuevo
//  (ver README.md).
//
enum class Alianza { AZUL, ROJA };

constexpr Alianza ALIANZA = Alianza::AZUL; // ← CAMBIAR AQUÍ: Alianza::AZUL o Alianza::ROJA

// ============================================================================
//  2. CHASIS, ODOMETRÍA Y CONTROL PID
// ============================================================================

// Tracción: seis motores V5 con cartucho azul (600 RPM), tres por lado.
// Un número negativo invierte el sentido de giro de ese motor.
pros::MotorGroup left_mg({ -11, 12, -13 }, pros::MotorGearset::blue);  // Lado izquierdo
pros::MotorGroup right_mg({ 14, -15, 16 }, pros::MotorGearset::blue); // Lado derecho

// Parámetros geométricos de la tracción para LemLib.
lemlib::Drivetrain drivetrain(&left_mg,                 // grupo de motores izquierdo
                              &right_mg,                // grupo de motores derecho
                              15,                       // trocha (distancia entre ruedas): 15 pulgadas
                              lemlib::Omniwheel::OLD_4, // ruedas omnidireccionales de 4"
                              300,                      // RPM en la rueda (600 RPM con reducción 2:1)
                              2                         // deriva horizontal (ajuste de LemLib)
);

// Sensor inercial (IMU): da el rumbo del robot.
pros::Imu imu(21);

// Encoders ópticos de las ruedas de seguimiento (puertos de tres hilos).
// Estas ruedas no reciben tracción, así que no patinan como las motrices.
pros::adi::Encoder adi_ver_encoder('E', 'F', true); // Rueda de seguimiento vertical (sentido invertido)
pros::adi::Encoder adi_hor_encoder('G', 'H');       // Rueda de seguimiento horizontal

// Ruedas de seguimiento de 2,75": el último número es su distancia, en
// pulgadas, al centro de giro del robot.
lemlib::TrackingWheel vertical_tracking_wheel(&adi_ver_encoder, lemlib::Omniwheel::OLD_275, 5.5);
lemlib::TrackingWheel horizontal_tracking_wheel(&adi_hor_encoder, lemlib::Omniwheel::OLD_275, -2);

// Sensores de odometría: una rueda vertical, una horizontal y el IMU.
lemlib::OdomSensors sensors(&vertical_tracking_wheel,   // rueda vertical 1
                            nullptr,                    // rueda vertical 2 (no se usa)
                            &horizontal_tracking_wheel, // rueda horizontal 1
                            nullptr,                    // rueda horizontal 2 (no se usa)
                            &imu                        // sensor inercial
);

// Controlador PID lateral (avance). Ganancias ajustadas a mano sobre el robot.
lemlib::ControllerSettings lateral_controller(10,  // ganancia proporcional (kP)
                                              0,   // ganancia integral (kI)
                                              24,  // ganancia derivativa (kD)
                                              3,   // límite anti-windup del término integral
                                              1,   // rango de error pequeño, en pulgadas
                                              100, // tiempo dentro del rango pequeño para terminar, en ms
                                              3,   // rango de error grande, en pulgadas
                                              500, // tiempo dentro del rango grande para terminar, en ms
                                              20   // aceleración máxima (slew)
);

// Controlador PID angular (giro). Ganancias ajustadas a mano sobre el robot.
lemlib::ControllerSettings angular_controller(6,   // ganancia proporcional (kP)
                                              0,   // ganancia integral (kI)
                                              28,  // ganancia derivativa (kD)
                                              3,   // límite anti-windup del término integral
                                              1,   // rango de error pequeño, en grados
                                              100, // tiempo dentro del rango pequeño para terminar, en ms
                                              3,   // rango de error grande, en grados
                                              500, // tiempo dentro del rango grande para terminar, en ms
                                              0    // aceleración máxima (0 = sin límite)
);

// Chasis de LemLib: une la tracción, los dos PID y la odometría.
lemlib::Chassis chassis(drivetrain,         // tracción
                        lateral_controller, // PID lateral
                        angular_controller, // PID angular
                        sensors             // sensores de odometría
);

// ============================================================================
//  3. MECANISMOS, SENSORES Y NEUMÁTICA
// ============================================================================

// Control principal del piloto.
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Estructura superior.
pros::MotorGroup intake({ 5, 7 });  // Intake: recoge los aros del piso
pros::MotorGroup chains({ 3, -8 }); // Cadena: sube los aros hasta la estaca
pros::Motor fishMech(-2);           // Brazo Fishmech: anota en las estacas de pared y de alianza

// Sensores de los mecanismos.
pros::ADIButton scoreSwitch('D');  // Final de carrera: detecta el aro arriba de la cadena
pros::ADIAnalogIn lineSensor('A'); // Sensor de línea: marca la posición cero del Fishmech
pros::Vision visionSensor(17);     // Sensor de visión: distingue el color de los aros

// Neumática (dos electroválvulas, en puertos de tres hilos).
pros::adi::Pneumatics clamping('b', false);  // Pinza trasera que sujeta la estaca móvil
pros::adi::Pneumatics colorSort('c', false); // Compuerta de la cadena que descarta los aros rivales

// Firmas de color del sensor de visión (se cargan en initialize()).
// Los números son los índices de firma: 1 = aro azul, 2 = aro rojo.
int blue = 1;
int red = 2;

// Firma del color que se DESCARTA, deducida de la alianza elegida arriba.
// Si alguna vez se quisiera descartar un color distinto, basta con cambiar
// esta línea (o la constante ALIANZA).
const int COLOR_RIVAL = (ALIANZA == Alianza::AZUL) ? red : blue;

// Ancho mínimo (en píxeles del sensor) para considerar que un aro está cerca
// de la compuerta. Evita reaccionar a aros lejanos.
constexpr int ANCHO_MINIMO_ARO = 100;

// ============================================================================
//  INICIALIZACIÓN
// ============================================================================

/**
 * Se ejecuta una vez, apenas arranca el programa. Calibra el sensor inercial
 * (el robot debe estar quieto) y carga las firmas de color del sensor de visión.
 */
void initialize()
{
	chassis.calibrate(); // Calibra el IMU y pone la odometría en cero

	// Firmas generadas con la utilidad del sensor de visión de VEX.
	pros::vision_signature_s_t BLUE_RING = pros::Vision::signature_from_utility(1, -3837, -3555, -3696, 6421, 7263, 6842, 3, 0);

	pros::vision_signature_s_t RED_RING = pros::Vision::signature_from_utility(
		2, 7229, 10779, 9004, -2731, -1057, -1894, 3, 0);

	visionSensor.set_signature(1, &BLUE_RING);
	visionSensor.set_signature(2, &RED_RING);
}

/**
 * Se ejecuta mientras el robot está deshabilitado por el sistema de control
 * de la competencia. No se usa.
 */
void disabled() {}

/**
 * Se ejecuta antes del autónomo cuando el robot está conectado al sistema de
 * control de la competencia (por ejemplo, para un selector de rutinas). No se usa.
 */
void competition_initialize() {}

// ============================================================================
//  4. FUNCIONES AUXILIARES
// ============================================================================

// Velocidad de la cadena mientras busca un aro en el autónomo. Se ajustó por
// separado para cada lado del campo.
const int VELOCIDAD_CADENA_AUTONOMO = (ALIANZA == Alianza::AZUL) ? -50 : -60;

// Indica que el Fishmech ya volvió a su posición cero.
bool fishMechReset = false;

/**
 * Baja el Fishmech hasta que el sensor de línea detecta la marca de la
 * posición cero (lectura menor a 1800) y lo detiene allí.
 */
void resetFishMechPosition() {
	while (lineSensor.get_value() >= 1800) {
		fishMech.move(-100);
	}
	fishMechReset = true;
	fishMech.move(0);
}

/**
 * Hace funcionar el intake y la cadena hasta que el final de carrera detecta
 * un aro arriba de la cadena; entonces la detiene, con el aro listo para el
 * Fishmech.
 */
void chainUntilRing() {
	while (!scoreSwitch.get_value()) {
		chains.move(VELOCIDAD_CADENA_AUTONOMO);
		intake.move(-127);
	}
	chains.move(0);
}

/**
 * Indica si el sensor de visión ve, cerca de la compuerta, un aro con la
 * firma indicada. Se toma el objeto más grande de esa firma (índice 0) y se
 * exige que su ancho supere ANCHO_MINIMO_ARO.
 */
bool aroCercaDeColor(int firma) {
	return visionSensor.get_object_count() > 0
		&& visionSensor.get_by_sig(0, firma).signature == firma
		&& visionSensor.get_by_sig(0, firma).width > ANCHO_MINIMO_ARO;
}

/**
 * Mueve la compuerta según el último aro detectado:
 *   - aro del color RIVAL  → se extiende el pistón y el aro sale del robot;
 *   - aro del color PROPIO → se retrae el pistón y el aro sigue hacia la estaca.
 * El pistón queda en esa posición hasta que aparece un aro del otro color.
 */
void actualizarCompuerta() {
	if (aroCercaDeColor(blue)) {
		(COLOR_RIVAL == blue) ? colorSort.extend() : colorSort.retract();
	}
	else if (aroCercaDeColor(red)) {
		(COLOR_RIVAL == red) ? colorSort.extend() : colorSort.retract();
	}
}

/**
 * Clasificación por color continua, para correr como tarea durante el autónomo.
 */
void colorSorting() {
	while (true) {
		actualizarCompuerta();
	}
}

/** Mueve los dos lados de la tracción con el mismo voltaje (−127 a 127). */
void moveRobot(int voltage) {
	left_mg.move(voltage);
	right_mg.move(voltage);
}

/** Detiene la tracción. */
void stopRobot() {
	left_mg.move(0);
	right_mg.move(0);
}

// ============================================================================
//  5. RUTINA AUTÓNOMA (30 s)
// ============================================================================
//
//  Las dos rutinas hacen la misma secuencia, reflejada para cada lado del
//  campo, con tiempos ajustados en la práctica:
//
//    1. Retroceden hasta la estaca móvil y la sujetan con la pinza trasera.
//    2. Encienden el intake y la cadena, y cargan un aro hasta el final de carrera.
//    3. Se apoyan en la estaca de alianza y anotan ese aro con el Fishmech.
//    4. Devuelven el Fishmech a su posición cero (sensor de línea) mientras
//       el robot se mueve de lado a lado para soltarse de la estaca.
//    5. Recogen más aros del campo y de la esquina, y los anotan en la estaca móvil.
//
//  Los giros usan el PID angular de LemLib (turnToHeading / swingToHeading);
//  los traslados son a lazo abierto, por tiempo y voltaje.
//

/** Rutina autónoma del lado AZUL. */
void autonomoAzul() {
	// Toma la estaca móvil
	chassis.turnToHeading(15, 5000, { .maxSpeed = 80 }, false);
	pros::delay(100);
	moveRobot(-80);
	pros::delay(200);
	left_mg.move(-60);
	right_mg.move(-40);
	pros::delay(900);
	moveRobot(-60);
	pros::delay(400);
	clamping.extend(); // Sujeta la estaca

	// Carga un aro y se acerca a la estaca de alianza
	chains.move(-90);
	intake.move(-127);
	pros::delay(300);
	stopRobot();
	chassis.turnToHeading(28, 5000, { .maxSpeed = 80 }, false);
	pros::delay(500);
	pros::Task chainTask(chainUntilRing); // Sube el aro hasta el final de carrera
	moveRobot(80);
	pros::delay(600);
	moveRobot(40);
	pros::delay(400);
	stopRobot();

	pros::delay(300);
	stopRobot();
	pros::delay(50);
	chassis.swingToHeading(90, DriveSide::LEFT, 5000, { .maxSpeed = 80 }, false);
	pros::delay(100);
	moveRobot(50); // Se apoya en la estaca de alianza para ubicarse
	pros::delay(500);
	stopRobot();
	pros::delay(200);
	moveRobot(-35); // Toma un poco de distancia para anotar
	pros::delay(400);
	stopRobot();
	pros::delay(100);

	// Anota con el Fishmech (el aro ya está arriba de la cadena)
	fishMech.move(127);
	pros::delay(800);
	fishMech.move(0);
	pros::delay(300);

	// Devuelve el Fishmech a cero mientras el robot se suelta de la estaca
	pros::Task resetFishMech(resetFishMechPosition);
	left_mg.move(40);
	right_mg.move(-40);
	pros::delay(150);
	while (!fishMechReset) {
		left_mg.move(-80);
		right_mg.move(80);
		pros::delay(200);
		left_mg.move(80);
		right_mg.move(-80);
		pros::delay(200);
	}
	pros::delay(100);

	// Segundo aro
	chassis.turnToHeading(-32, 5000, { .maxSpeed = 80 }, false);
	chains.move(-90);
	pros::delay(100);
	moveRobot(127);
	pros::delay(670);
	moveRobot(40);
	pros::delay(2500);
	stopRobot();
	pros::delay(500);
	moveRobot(-50);
	pros::delay(300);
	stopRobot();

	// Tercer aro
	chassis.turnToHeading(90, 5000, { .maxSpeed = 80 }, false);
	moveRobot(127);
	pros::delay(550);
	moveRobot(40);
	pros::delay(2000);
	stopRobot();
	pros::delay(500);
	moveRobot(-40);
	pros::delay(350);

	// Esquina: golpea la pila de aros para sacarlos
	chassis.turnToHeading(45, 5000, { .maxSpeed = 80 }, false);
	pros::delay(500);
	moveRobot(127);
	pros::delay(600);
	stopRobot();
	pros::delay(250);
	moveRobot(-80);
	pros::delay(500);
	stopRobot();
	pros::delay(100);

	// Último traslado
	chassis.turnToHeading(-150, 5000, { .maxSpeed = 80 }, false);
	chains.move(-90);
	pros::delay(50);
	moveRobot(127);
	pros::delay(800);
	moveRobot(30);
	pros::delay(1000);
	chains.move(0);
	intake.move(0);
}

/** Rutina autónoma del lado ROJO (reflejo de la azul, con tiempos propios). */
void autonomoRojo() {
	// Toma la estaca móvil
	chassis.turnToHeading(-25, 5000, { .maxSpeed = 80 }, false);
	pros::delay(100);
	moveRobot(-80);
	pros::delay(200);
	right_mg.move(-70);
	left_mg.move(-40);
	pros::delay(500);
	moveRobot(-60);
	pros::delay(500);
	clamping.extend(); // Sujeta la estaca

	pros::delay(400);
	stopRobot();

	// Carga un aro y se acerca a la estaca de alianza
	chassis.turnToHeading(-25, 5000, { .maxSpeed = 80 }, false);
	chains.move(-95);
	intake.move(-127);
	pros::delay(500);
	moveRobot(80);
	pros::delay(300);
	pros::Task chainTask(chainUntilRing); // Sube el aro hasta el final de carrera
	pros::delay(300);
	moveRobot(40);
	pros::delay(300);
	stopRobot();

	pros::delay(300);
	stopRobot();
	pros::delay(50);

	// Giro sobre una rueda, en sentido opuesto al de la rutina azul
	chassis.swingToHeading(-90, DriveSide::RIGHT, 5000, { .maxSpeed = 80 }, false);
	pros::delay(100);
	moveRobot(50); // Se apoya en la estaca de alianza para ubicarse
	pros::delay(500);
	stopRobot();
	pros::delay(200);

	moveRobot(-35); // Toma un poco de distancia para anotar
	pros::delay(400);
	stopRobot();
	pros::delay(100);

	// Anota con el Fishmech
	fishMech.move(127);
	pros::delay(800);
	fishMech.move(0);
	pros::delay(300);

	// Devuelve el Fishmech a cero mientras el robot se suelta de la estaca
	pros::Task resetFishMech(resetFishMechPosition);
	left_mg.move(40);
	right_mg.move(-40);
	pros::delay(150);
	while (!fishMechReset) {
		left_mg.move(-80);
		right_mg.move(80);
		pros::delay(200);
		left_mg.move(80);
		right_mg.move(-80);
		pros::delay(200);
	}
	pros::delay(100);

	// Segundo aro
	chassis.turnToHeading(35, 5000, { .maxSpeed = 80 }, false);
	chains.move(-95);
	pros::delay(100);
	moveRobot(127);
	pros::delay(670);
	moveRobot(40);
	pros::delay(2500);
	stopRobot();
	pros::delay(500);
	moveRobot(-50);
	pros::delay(350);
	stopRobot();

	// Tercer aro
	chassis.turnToHeading(-90, 5000, { .maxSpeed = 80 }, false);
	moveRobot(127);
	pros::delay(550);
	moveRobot(40);
	pros::delay(2000);
	stopRobot();
	pros::delay(500);
	moveRobot(-40);
	pros::delay(350);

	// Esquina: golpea la pila de aros para sacarlos
	chassis.turnToHeading(-45, 5000, { .maxSpeed = 80 }, false);
	pros::delay(500);
	moveRobot(127);
	pros::delay(600);
	stopRobot();
	pros::delay(250);
	moveRobot(-80);
	pros::delay(500);
	stopRobot();
	pros::delay(100);

	// Último traslado
	chassis.turnToHeading(150, 5000, { .maxSpeed = 80 }, false);
	chains.move(-95);
	pros::delay(50);
	moveRobot(127);
	pros::delay(800);
	moveRobot(30);
	pros::delay(1000);
	chains.move(0);
	intake.move(0);
}

/**
 * Período autónomo. Lo inicia el sistema de control de la competencia.
 * Prepara el robot, lanza la clasificación por color en segundo plano y
 * ejecuta la rutina de la alianza elegida.
 */
void autonomous()
{
	chassis.setPose(0, 0, 0);                        // La posición inicial es el origen
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE); // La tracción frena al soltarla
	fishMech.set_brake_mode(pros::MotorBrake::hold); // El Fishmech mantiene su posición
	pros::Task colorSortAutonomous(colorSorting);    // Clasificación por color en segundo plano

	if (ALIANZA == Alianza::AZUL) {
		autonomoAzul();
	}
	else {
		autonomoRojo();
	}
}

// ============================================================================
//  6. CONTROL MANUAL (1:30)
// ============================================================================
//
//  Mandos del control:
//
//    Joystick izquierdo (vertical)   avanzar / retroceder
//    Joystick derecho (horizontal)   girar (a media sensibilidad)
//    R1                              recoger: intake y cadena hacia arriba, con
//                                    clasificación por color
//    R2                              expulsar: intake y cadena en reversa
//    B / Y                           cerrar / abrir la pinza de la estaca móvil
//    X                               «modo Jordan»: la cadena sube un aro hasta
//                                    el final de carrera y lo deja listo para el Fishmech
//    A                               volver al modo normal
//    L1 / L2                         subir / bajar el Fishmech
//    ABAJO                           retroceder un poco para anotar en la estaca de alianza
//

/**
 * Control manual. Lo inicia el sistema de control de la competencia (o se
 * ejecuta directamente después de initialize() si no hay uno conectado).
 */
void opcontrol()
{
	fishMech.set_brake_mode(pros::MotorBrake::hold);  // El Fishmech mantiene su posición
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE); // La tracción frena al soltarla

	// Pantalla del cerebro: fondo azul con el nombre del equipo y del robot
	pros::screen::set_pen(pros::Color::dark_blue);
	pros::delay(10);
	pros::screen::set_eraser(pros::Color::dark_blue);
	pros::delay(10);
	pros::screen::fill_rect(0, 0, 480, 256);
	pros::delay(10);
	pros::screen::set_pen(pros::Color::ghost_white);
	pros::delay(10);
	pros::screen::print(pros::E_TEXT_LARGE_CENTER, 3, "UCSA PRIME");
	pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 5, "JORDAN");

	// Pantalla del control
	controller.print(0, 1, "UCSA PRIME");

	// Estado del piloto
	bool piston = false; // true = pinza cerrada sobre la estaca móvil
	int jordanMode = 0;  // 0 = normal · 1 = subiendo un aro al Fishmech · 2 = aro listo (cadena frenada)

	while (true)
	{
		// Datos en la pantalla del cerebro: pose de la odometría, encoders y rumbo del IMU
		pros::screen::print(pros::E_TEXT_SMALL, 7, "X: %f, Y: %f, Theta: %f", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
		pros::screen::print(pros::E_TEXT_MEDIUM, 9, "Vertical: %d", adi_ver_encoder.get_value());
		pros::screen::print(pros::E_TEXT_MEDIUM, 10, "Horizontal: %d", adi_hor_encoder.get_value());
		pros::screen::print(pros::E_TEXT_MEDIUM, 11, "IMU: %f", imu.get_heading());

		// Tracción: avance con el joystick izquierdo y giro con el derecho
		int dir = controller.get_analog(ANALOG_LEFT_Y);
		double turn = controller.get_analog(ANALOG_RIGHT_X) * 0.5;
		left_mg.move(dir + turn);
		right_mg.move(dir - turn);

		// Pinza de la estaca móvil: B cierra, Y abre
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B))
		{
			piston = true;
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
		{
			piston = false;
		}
		piston ? clamping.extend() : clamping.retract();

		// Intake y cadena
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			// Recoger: en modo normal la cadena sube los aros hasta la estaca
			intake.move(-127);
			chains.move(jordanMode == 0 ? -90 : 0);
			actualizarCompuerta(); // Clasificación por color mientras se recoge
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			// Expulsar: todo en reversa
			chains.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			intake.move(127);
			chains.move(jordanMode == 0 ? 75 : 0);
		}
		else
		{
			// Sin botón: en modo Jordan sigue subiendo el aro; si no, todo quieto
			intake.move(jordanMode == 1 ? -127 : 0);
			chains.move(jordanMode == 1 ? -75 : 0);
		}

		// Modo Jordan: X lo activa, A vuelve al modo normal
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X))
		{
			jordanMode = 1;
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A))
		{
			jordanMode = 0;
		}
		// Cuando el final de carrera detecta el aro, la cadena se frena y lo
		// deja listo para el Fishmech
		if (scoreSwitch.get_value() && jordanMode == 1)
		{
			chains.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			jordanMode = 2;
		}

		// Fishmech: L1 sube (anota), L2 baja
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		{
			jordanMode = 0;
			fishMech.move(127);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
		{
			jordanMode = 0;
			fishMech.move(-50);
		}
		else
		{
			fishMech.move(0);
		}

		// ABAJO: retrocede un poco para quedar a distancia de anotar en la estaca de alianza
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
		{
			moveRobot(-30);
			pros::delay(380);
			stopRobot();
		}

		pros::delay(10); // Pausa del ciclo: libera el procesador para las demás tareas. NO QUITAR.
	}
}
