/*
 * bellalui_algorithm.c
 *
 *  Created on: 26 Jun 2020
 *      Author: Anas Jnini
 */

#include <airbrakes/bellalui_algorithm.h>
#include <stdbool.h>
#include <math.h>


const double interpolation_table[61][2] = { { 0.0009, 0.0048 }, { 0.0010, 0.0052 }, {
					0.0010, 0.0054 }, { 0.0010, 0.0056 }, { 0.0011, 0.0057 }, {
					0.0011, 0.0057 }, { 0.0011, 0.0058 }, { 0.0011, 0.0058 }, {
					0.0011, 0.0059 }, { 0.0011, 0.0059 }, { 0.0011, 0.0060 }, {
					0.0011, 0.0060 }, { 0.0011, 0.0060 }, { 0.0011, 0.0061 }, {
					0.0011, 0.0061 }, { 0.0011, 0.0061 }, { 0.0011, 0.0061 }, {
					0.0011, 0.0062 }, { 0.0012, 0.0062 }, { 0.0012, 0.0062 }, {
					0.0012, 0.0063 }, { 0.0012, 0.0063 }, { 0.0012, 0.0063 }, {
					0.0012, 0.0063 }, { 0.0012, 0.0064 }, { 0.0012, 0.0064 }, {
					0.0012, 0.0064 }, { 0.0012, 0.0064 }, { 0.0012, 0.0064 }, {
					0.0012, 0.0065 }, { 0.0012, 0.0065 }, { 0.0012, 0.0065 }, {
					0.0012, 0.0065 }, { 0.0012, 0.0066 }, { 0.0012, 0.0066 }, {
					0.0012, 0.0066 }, { 0.0012, 0.0066 }, { 0.0012, 0.0066 }, {
					0.0012, 0.0067 }, { 0.0012, 0.0067 }, { 0.0012, 0.0067 }, {
					0.0012, 0.0067 }, { 0.0012, 0.0067 }, { 0.0012, 0.0068 }, {
					0.0012, 0.0068 }, { 0.0012, 0.0068 }, { 0.0012, 0.0068 }, {
					0.0012, 0.0068 }, { 0.0012, 0.0068 }, { 0.0012, 0.0069 }, {
					0.0012, 0.0069 }, { 0.0012, 0.0069 }, { 0.0012, 0.0069 }, {
					0.0012, 0.0069 }, { 0.0012, 0.0069 }, { 0.0012, 0.0070 }, {
					0.0012, 0.0070 }, { 0.0012, 0.0070 }, { 0.0012, 0.0070 }, {
					0.0012, 0.0070 }, { 0.0012, 0.0070 } }; // DR E

struct Rocket {
	double abx; // DR
	double abn; // DR
	double Sm; // DR

};

const struct Rocket Eiger = { 2.05, 3, 0.0191 }; // DR

const double angle_tab[15] = { 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55,
		60, 65, 70 }; // DR

const double surface_tab[15] = { 0, 0.00011, 0.000223, 0.000338, 0.000456,
		0.000576, 0.000698, 0.000822, 0.000948, 0.001075, 0.001203,
		0.001332, 0.001461, 0.00159, 0.001719 }; // DR

const double h_tab[15] = { 0, 0.00376, 0.00745, 0.01104, 0.0145, 0.01781,
		0.02095, 0.02388, 0.02658, 0.02903, 0.0312, 0.03309, 0.03464,
		0.03587, 0.03676 }; // DR

const double CD0 = 1.17;


const double target_altitude = 5000; //altitude that we are targeting
const double target_speed = 1; // speed we are targeting at that altitude
const double upper_bound = -1.3090E-04; // DR E (0.5*ro*S*Cmax)/M (AB completely closed)// upper bound for the research algorithm function of CD/air density -
const double lower_bound = -2.1194E-04; // DR E (0.5*ro*S*Cmax)/M (AB completely open) // lower bound for the research algorithm
const double dry_mass = 38; //dry mass of rocket // DR
const double acceleration = -9.81; //g // E :)



int invdrag(double angle_of_attack, double velocity, double viscosity, double CD_target_drag) {
	// CD target drag (Rocket+AB)

	double U = abs(velocity * cos(angle_of_attack));
	double Rex = Eiger.abx * U / viscosity;
	double delta = 0.37 * Eiger.abx / (pow(Rex, 0.2));

	double coeff1 = (angle_tab[14] / surface_tab[14]);
	double coeff2 = (h_tab[14] / angle_tab[14]);
	double sol[3] = { 0, 0, 0 };
	double P = CD_target_drag * Eiger.Sm / (Eiger.abn * CD0);

	double q = 0.001 * delta;
	double qr = 60 * delta;

	while (q < delta) {
		double S = P / q;
		double the1 = coeff1 * S;

		if (the1 > 0 && the1 < 66.1) {
			double h = coeff2 * the1;

			if (h > 0 && h < 36.76 * 1E-2) {
				qr = (49 / 72) * (pow((h / delta), 0.28571));

				if (fabs(qr - q) < 0.001) {
					double sol[3] = { the1, h, abs(q - qr) };
				}

				double theta = ((the1 * 217.7) / 70) - 217.7;

				return theta;
			}
		}

		q = q + 0.05 * delta;
	}

	if (sol[0] == 0) {
		double q2 = delta;
		double qr = 60 * delta;
		double m = 30 * delta;

		while (q2 < m) {
			double S2 = P / q2;
			double the2 = (coeff1) * S2;

			if (the2 > 0 && the2 < 66.1) {
				double h2 = (coeff2) * the2;

				if (h2 > 0 && h2 < (36.76 * 1E-3)) {
					qr = 1 - (0.4444) * (delta / h2)
							+ (0.125) * ((delta / h2) * (delta / h2));
				}

				if (fabs(qr - q2) < 1E-3) {
					double sol[3] = { the2, h2, abs(q2 - qr) };
					double theta = ((the2 * 217.7) / 70) - 217.7; // DR (check for max and min angles on motor!) if the min angle (closed?) is different than 217.7, change it, (not 70))!

					return theta;
				}

			}
			q2 = q2 + 0.01 * delta;
		}
	}

	return 10000;
}

float bellalui_angle_tab(float altitude, float speed) {
	//int16_t data1[10];

	double upper_bound = -1.3090E-04; // DR E (0.5*ro*S*Cmax)/M (AB completely closed)// upper bound for the research algorithm function of CD/air density -
	double lower_bound = -2.1194E-04; // DR E (0.5*ro*S*Cmax)/M (AB completely open) // lower bound for the research algorithm

	double k = 0.5 / (target_altitude - altitude);
	double target_speed_sq = target_speed * target_speed;
	double current_speed_sq = speed * speed;
	double image_of_a = k * (log((1 + (target_speed_sq) * (lower_bound / acceleration)) / (1 + (current_speed_sq) * (lower_bound / acceleration)))); //f(a)
	double image_of_b = k * (log((1 + (target_speed_sq) * (upper_bound / acceleration)) / (1 + (current_speed_sq) * (upper_bound / acceleration)))); //f(b)
	double rho = -9.9814e-05 * altitude + 1.1037; //linear interp of rho (ENV linear interp between rho and altitude)// E (check simulator to build a linear model between rho and x depending on env)
	/*
	 double xma = -0.5*(log((1+(x222)*(a/A))/(1+(x2220)*(a/A))))/(fa)+altitude;
	 double xmi = -0.5*(log((1+(x222)*(b/A))/(1+(x2220)*(b/A))))/(fb)+altitude;
	 */
	// Bisection algorithm to solve the non linear equation f(x)=x look into the theory for more info

	double solution = 0.5 * (lower_bound + upper_bound); // initialisation of the m, the solution of the dichotomy algorithm
	int found_solution = true;
	int iterations = 0;

	if ((image_of_a - lower_bound) * (image_of_b - upper_bound) > 0) {

		found_solution = false;
		solution = 10000;


		double theta = -190; //keep old angle (en mÈmoire)
		// break;
		return theta;
	}

	if ((image_of_a - lower_bound) * (image_of_b - upper_bound) < 0) {
		// printf("SOL EXISTE");

		while ((upper_bound - lower_bound) > 1E-6) {

			image_of_a = k * (log((1 + (target_speed_sq) * (lower_bound / acceleration)) / (1 + (current_speed_sq) * (lower_bound / acceleration))));
			image_of_b = k * (log((1 + (target_speed_sq) * (upper_bound / acceleration)) / (1 + (current_speed_sq) * (upper_bound / acceleration))));
			solution = (lower_bound + upper_bound) * 0.5;

			double image_of_solution = k
					* (log((1 + (target_speed_sq) * (solution / acceleration)) / (1 + (current_speed_sq) * (solution / acceleration))));

			if (image_of_a - lower_bound > 0 && image_of_b - upper_bound < 0) {
				if (image_of_solution - solution < 0)
					upper_bound = solution;

				if (image_of_solution - solution > 0)
					lower_bound = solution;
			}
			if (image_of_b - upper_bound > 0 && image_of_a - lower_bound < 0) {
				if (image_of_solution - solution < 0)
					lower_bound = solution;

				if (image_of_solution - solution > 0)
					upper_bound = solution;
			}
		}
	}

	//	int j=floor(speed/5);
	//double consta=drM[j][1];
	//double theta=-190.5-(1/consta)*((m*Mdry)/(0.5*rho*0.0189)-0.3225-consta);//interpline
	//double thetainvdrag=
	// double test = invdrag(0, speed, 1.3850E-05, solution); //  invdrag  (angle d'attaque/vitesse/1.3850e-05=nu variable env,m solution drag)

	int j = floor(speed / 5);
	double consta = interpolation_table[j][1];
	//double theta = -190.5-(1/consta)*((solution*dry_mass)/(0.5*rho*0.0189)-0.3225-consta);//interpline
	double theta = 12.4-(1/consta)*((solution*dry_mass)/(0.5*rho*0.0189)-0.3225-consta);//interpline

	return (float) theta;
}
